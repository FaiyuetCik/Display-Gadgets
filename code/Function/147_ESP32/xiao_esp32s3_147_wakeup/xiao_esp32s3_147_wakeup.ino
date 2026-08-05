/*
  XIAO ESP32-S3 Plus + 1.47 Inch Display IMU wake demo.

  USR1 forces screen sleep. USR2 wakes manually.
  LSM6-compatible IMU wake/motion INT1 on D14 wakes the screen.

  Adapted from:
    - xiao_esp32s3_114_wakeup (ESP32-S3 sleep / battery patterns)
    - xiao_nrf52840_147_wakeup (1.47-inch UI layout)
    - xiao_esp32s3_147_electronic_quicksand (1.47 panel init + IMU detection)

  Required Arduino libraries:
    - Seeed_GFX / TFT_eSPI
*/

#include <Arduino.h>
#include "driver.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// ========================= Pin map =========================

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t I2C_SDA_PIN  = D4;
static constexpr uint8_t I2C_SCL_PIN  = D5;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

static constexpr uint8_t IMU_INT_PIN  = D14;
static constexpr uint8_t USR1_PIN     = D15;   // force sleep
static constexpr uint8_t USR2_PIN     = D19;   // force wake
static constexpr uint8_t BAT_ADC_PIN  = D16;

// ========================= IMU registers =========================

// LSM6-series registers (also used by some QMI8658-compatible wake config).
static constexpr uint8_t REG_WHO_AM_I    = 0x0F;
static constexpr uint8_t REG_CTRL1_XL    = 0x10;
static constexpr uint8_t REG_CTRL2_G     = 0x11;
static constexpr uint8_t REG_CTRL3_C     = 0x12;
static constexpr uint8_t REG_TAP_CFG     = 0x58;
static constexpr uint8_t REG_WAKE_UP_THS = 0x5B;
static constexpr uint8_t REG_WAKE_UP_DUR = 0x5C;
static constexpr uint8_t REG_MD1_CFG     = 0x5E;
static constexpr uint8_t REG_WAKE_UP_SRC = 0x1B;
static constexpr uint8_t REG_OUTX_L_G    = 0x22;
static constexpr uint8_t REG_OUTX_L_A    = 0x28;

// ========================= Timing =========================

static constexpr uint32_t AUTO_SLEEP_MS     = 8000;
static constexpr uint32_t UI_REFRESH_MS     = 250;
static constexpr uint32_t BAT_REFRESH_MS    = 1000;
static constexpr uint32_t WAKE_LOCK_MS      = 1200;
static constexpr uint8_t  BACKLIGHT_AWAKE   = 160;
static constexpr uint8_t  BACKLIGHT_SLEEP   = 0;
static constexpr float    BAT_DIVIDER_RATIO = (1000.0f + 510.0f) / 510.0f;

// ========================= Display =========================

TFT_eSPI tft;

// ========================= Colours =========================

static constexpr uint16_t C_BLACK  = TFT_BLACK;
static constexpr uint16_t C_WHITE  = TFT_WHITE;
static constexpr uint16_t C_GREEN  = TFT_GREEN;
static constexpr uint16_t C_CYAN   = TFT_CYAN;
static constexpr uint16_t C_YELLOW = TFT_YELLOW;
static constexpr uint16_t C_RED    = TFT_RED;
static constexpr uint16_t C_GRAY   = 0x8410;

// ========================= Runtime state =========================

uint8_t       imuAddr        = 0;
volatile bool imuWakeFlag    = false;
bool          screenAwake    = true;
uint32_t      lastActivityMs = 0;
uint32_t      lastUiMs       = 0;
uint32_t      lastBatMs      = 0;
uint32_t      lastWakeMs     = 0;
uint32_t      wakeCount      = 0;
uint32_t      intCount       = 0;
uint8_t       lastWakeSrc    = 0;
float         ax = 0, ay = 0, az = 0;
float         gx = 0, gy = 0, gz = 0;
uint32_t      batMv          = 0;
int           batPercent     = 0;

// ========================= I2C helpers =========================

static int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool write8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

static bool read8(uint8_t addr, uint8_t reg, uint8_t *val) {
  return readBytes(addr, reg, val, 1);
}

// ========================= IMU detection & wake config =========================

static bool findLsm(uint8_t addr) {
  uint8_t who = 0;
  if (!read8(addr, REG_WHO_AM_I, &who)) return false;
  if (who != 0x69 && who != 0x6A && who != 0x6C) return false;
  imuAddr = addr;
  Serial.printf("[IMU] LSM6-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

static bool findQmi(uint8_t addr) {
  uint8_t who = 0;
  if (!read8(addr, 0x00, &who)) return false;
  if (who == 0x00 || who == 0xFF) return false;
  imuAddr = addr;
  Serial.printf("[IMU] QMI8658-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

static bool initImuWake() {
  // Try LSM6 first (wake-up feature is LSM6-specific), then QMI8658.
  bool ok = findLsm(0x6A) || findLsm(0x6B) || findQmi(0x6B) || findQmi(0x6A);
  if (!ok) {
    Serial.println("[IMU] not found");
    return false;
  }

  // Configure LSM6-style wake-up registers.
  // For QMI8658 the WAKE_UP registers don't exist and writes will just
  // be ignored; the demo will still show IMU data but without wake-up.
  ok = true;
  ok &= write8(imuAddr, REG_CTRL3_C, 0x44);   // BDU + auto-increment
  ok &= write8(imuAddr, REG_CTRL1_XL, 0x40);  // 104 Hz, ±2g
  ok &= write8(imuAddr, REG_CTRL2_G,  0x40);  // 104 Hz, 2000 dps
  ok &= write8(imuAddr, REG_TAP_CFG,  0x80);  // enable embedded interrupts
  ok &= write8(imuAddr, REG_WAKE_UP_THS, 0x05);
  ok &= write8(imuAddr, REG_WAKE_UP_DUR, 0x00);
  ok &= write8(imuAddr, REG_MD1_CFG, 0x20);   // route wake-up → INT1

  // Clear stale wake source.
  uint8_t dummy = 0;
  (void)read8(imuAddr, REG_WAKE_UP_SRC, &dummy);

  Serial.print("[IMU] wake config ");
  Serial.println(ok ? "OK" : "partial");
  return true;
}

static bool readImu() {
  uint8_t g[6] = {}, a[6] = {};
  if (!readBytes(imuAddr, REG_OUTX_L_G, g, sizeof(g))) return false;
  if (!readBytes(imuAddr, REG_OUTX_L_A, a, sizeof(a))) return false;

  gx = le16(&g[0]) * 0.00875f;
  gy = le16(&g[2]) * 0.00875f;
  gz = le16(&g[4]) * 0.00875f;
  ax = le16(&a[0]) * 0.000061f;
  ay = le16(&a[2]) * 0.000061f;
  az = le16(&a[4]) * 0.000061f;
  return true;
}

// ========================= Interrupt handler =========================

void IRAM_ATTR onImuWake() {
  imuWakeFlag = true;
  intCount++;
}

// ========================= LCD helpers =========================

static void prepareLcdPins() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_SCK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_SCK_PIN, LOW);
  digitalWrite(LCD_MOSI_PIN, LOW);
}

static void hardResetPanel() {
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(20);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(80);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(180);
}

static void applyXIAO147PanelFix() {
  tft.writecommand(0x36);
  tft.writedata(0x48);
  delay(10);
}

static void setBacklight(uint8_t pwm) {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, pwm ? HIGH : LOW);
  analogWrite(LCD_BL_PIN, pwm);
}

static void initLcd() {
  prepareLcdPins();
  setBacklight(BACKLIGHT_AWAKE);
  hardResetPanel();
  tft.init();
  tft.setRotation(0);
  applyXIAO147PanelFix();
  tft.invertDisplay(false);
  tft.fillScreen(C_BLACK);
}

// ========================= Battery =========================

static int lipoPercent(float v) {
  if (v >= 4.20f) return 100;
  if (v <= 3.30f) return 0;
  return constrain((int)((v - 3.30f) * 100.0f / 0.90f + 0.5f), 0, 100);
}

static void updateBattery() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    sum += analogReadMilliVolts(BAT_ADC_PIN);
    delay(2);
  }
  batMv = (uint32_t)((sum / 16.0f) * BAT_DIVIDER_RATIO);
  batPercent = lipoPercent(batMv / 1000.0f);
}

static uint16_t batteryColor() {
  return batPercent <= 15 ? C_RED : (batPercent <= 35 ? C_YELLOW : C_GREEN);
}

// ========================= UI =========================

static void printFixed(int x, int y, uint16_t color, const String &text, int width) {
  String out = text;
  while ((int)out.length() < width) out += ' ';
  if ((int)out.length() > width) out = out.substring(0, width);
  tft.setTextSize(1);
  tft.setTextColor(color, C_BLACK);
  tft.setCursor(x, y);
  tft.print(out);
}

static void drawLayout() {
  tft.fillScreen(C_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(C_GREEN, C_BLACK);
  tft.setCursor(8, 10);
  tft.print("Hello,XIAO!");

  tft.setTextSize(1);
  tft.setTextColor(C_CYAN, C_BLACK);
  tft.setCursor(10, 36);
  tft.print("IMU D14 Wake Demo");

  tft.drawFastHLine(8, 52, 156, 0x39E7);

  // Power block
  tft.drawRoundRect(8, 66, 156, 82, 6, C_CYAN);
  tft.setTextColor(C_CYAN, C_BLACK);
  tft.setCursor(18, 78);
  tft.print("POWER STATE");
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(18, 98);
  tft.print("Screen");
  tft.setCursor(18, 116);
  tft.print("WakeCnt");
  tft.setCursor(18, 134);
  tft.print("BAT");

  // Motion block
  tft.drawRoundRect(8, 156, 156, 90, 6, C_YELLOW);
  tft.setTextColor(C_YELLOW, C_BLACK);
  tft.setCursor(18, 168);
  tft.print("MOTION");
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(18, 190);
  tft.print("Acc");
  tft.setCursor(18, 210);
  tft.print("Gyr");
  tft.setCursor(18, 230);
  tft.print("INT");

  // Test block
  tft.drawRoundRect(8, 258, 156, 42, 6, C_GREEN);
  tft.setTextColor(C_GREEN, C_BLACK);
  tft.setCursor(18, 270);
  tft.print("TEST");
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(18, 288);
  tft.print("USR1 sleep  USR2 wake");
}

static void updateUi() {
  char buf[48];

  printFixed(78, 98, C_GREEN, "AWAKE", 12);

  snprintf(buf, sizeof(buf), "%lu", (unsigned long)wakeCount);
  printFixed(78, 116, C_YELLOW, buf, 12);

  snprintf(buf, sizeof(buf), "%.2fV %d%%", batMv / 1000.0f, batPercent);
  printFixed(48, 134, batteryColor(), buf, 12);

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", ax, ay, az);
  printFixed(48, 190, C_WHITE, buf, 16);

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", gx, gy, gz);
  printFixed(48, 210, C_WHITE, buf, 16);

  snprintf(buf, sizeof(buf), "D14:%lu src:0x%02X", (unsigned long)intCount, lastWakeSrc);
  printFixed(48, 230, C_CYAN, buf, 16);

  uint32_t now = millis();
  uint32_t remain = now - lastActivityMs < AUTO_SLEEP_MS
                      ? (AUTO_SLEEP_MS - (now - lastActivityMs)) / 1000
                      : 0;
  snprintf(buf, sizeof(buf), "Auto sleep in %lus", (unsigned long)remain);
  printFixed(18, 306, C_GRAY, buf, 20);
}

// ========================= Sleep / wake =========================

static void screenWake(const char *reason) {
  if (screenAwake && millis() - lastWakeMs < WAKE_LOCK_MS) return;

  screenAwake = true;
  lastActivityMs = lastWakeMs = millis();
  wakeCount++;

  setBacklight(BACKLIGHT_AWAKE);
  drawLayout();
  updateUi();

  Serial.print("[WAKE] ");
  Serial.print(reason);
  Serial.print("  count=");
  Serial.println((unsigned long)wakeCount);
}

static void screenSleep() {
  if (!screenAwake) return;

  tft.fillScreen(C_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(C_CYAN, C_BLACK);
  tft.setCursor(18, 120);
  tft.print("Sleeping...");
  tft.setTextSize(1);
  tft.setCursor(18, 150);
  tft.print("Pick up device to wake");
  delay(500);

  setBacklight(BACKLIGHT_SLEEP);
  screenAwake = imuWakeFlag = false;

  uint8_t dummy = 0;
  (void)read8(imuAddr, REG_WAKE_UP_SRC, &dummy);
}

static void lightSleepWait() {
  gpio_wakeup_enable((gpio_num_t)IMU_INT_PIN, GPIO_INTR_HIGH_LEVEL);
  gpio_wakeup_enable((gpio_num_t)USR2_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_sleep_enable_timer_wakeup(250000);
  esp_light_sleep_start();
}

static void handleWakeEvents() {
  bool pending = false;
  noInterrupts();
  if (imuWakeFlag) {
    imuWakeFlag = false;
    pending = true;
  }
  interrupts();

  if (digitalRead(IMU_INT_PIN) == HIGH) pending = true;

  if (digitalRead(USR2_PIN) == LOW) {
    screenWake("USR2");
    while (digitalRead(USR2_PIN) == LOW) delay(5);
    return;
  }

  if (!pending) return;

  uint8_t src = 0;
  if (read8(imuAddr, REG_WAKE_UP_SRC, &src)) lastWakeSrc = src;
  if ((src & 0x0F) || !screenAwake) screenWake("IMU_D14");
}

// ========================= Setup / loop =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), onImuWake, RISING);

  analogReadResolution(12);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus 1.47 IMU Wake Demo ===");

  initLcd();

  if (!initImuWake()) {
    tft.setCursor(12, 104);
    tft.setTextColor(C_RED, C_BLACK);
    tft.print("IMU init failed");
    while (1) delay(1000);
  }

  updateBattery();
  readImu();
  lastActivityMs = lastBatMs = millis();
  drawLayout();
  updateUi();
}

void loop() {
  handleWakeEvents();

  if (!screenAwake) {
    lightSleepWait();
    handleWakeEvents();
    return;
  }

  if (digitalRead(USR1_PIN) == LOW) {
    delay(30);
    if (digitalRead(USR1_PIN) == LOW) {
      screenSleep();
      while (digitalRead(USR1_PIN) == LOW) delay(5);
    }
  }

  uint32_t now = millis();
  if (now - lastBatMs >= BAT_REFRESH_MS) {
    lastBatMs = now;
    updateBattery();
  }
  if (now - lastUiMs >= UI_REFRESH_MS) {
    lastUiMs = now;
    readImu();
    updateUi();
  }
  if (now - lastActivityMs >= AUTO_SLEEP_MS) screenSleep();
  delay(5);
}
