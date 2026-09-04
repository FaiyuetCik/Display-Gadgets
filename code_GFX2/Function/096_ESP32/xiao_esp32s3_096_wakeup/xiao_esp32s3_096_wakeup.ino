/*
  XIAO ESP32-S3 Plus + 0.96 Inch Display IMU wake demo.

  USR1 forces screen sleep. USR2 wakes manually.
  LSM6-compatible IMU wake/motion INT1 on D14 wakes the screen.
  Uses ESP32 light sleep with GPIO wakeup.

  0.96 board pin map:
    LCD CS  = D2     LCD DC  = D3     I2C SDA = D4
    I2C SCL = D5     USR1    = D6     USR2    = D7
    LCD SCK = D8     IMU INT = D14    LCD MOSI= D10
    BAT ADC = D16    LCD RST = D17    LCD BL  = D18

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  Arduino_GFX bus + panel construction, manual pins, and
  tft.begin()/setRotation()/invertDisplay(). Config_Seeed_0inch96_LCD_ST7789
  bakes 80x160 BGR rot2. Backlight PWM stays app-owned via
  analogWrite(LCD_BL_PIN, pwm); LSM6 IMU wake, battery ADC, and light-sleep
  peripheral logic unchanged.

  Required libraries:
    - Seeed_GFX2
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>
#include <Wire.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

Seeed_GFX display;

static constexpr int8_t LCD_RST_PIN = 13;  // XIAO ESP32-S3 Plus
static constexpr int8_t LCD_BL_PIN  = 12;

// ── Pins ───────────────────────────────────────────────────────────

static constexpr uint8_t I2C_SDA_PIN  = D4;
static constexpr uint8_t I2C_SCL_PIN  = D5;
static constexpr uint8_t USR1_PIN     = D6;
static constexpr uint8_t USR2_PIN     = D7;
static constexpr uint8_t IMU_INT_PIN  = 41;  // D14 -> GPIO41
static constexpr uint8_t BAT_ADC_PIN  = 10;  // D16 -> GPIO10

// ── IMU register map (LSM6DS3 / ISM330DHCX compatible) ────────────

static constexpr uint8_t REG_CTRL1_XL    = 0x10;
static constexpr uint8_t REG_CTRL3_C     = 0x12;
static constexpr uint8_t REG_TAP_CFG     = 0x58;
static constexpr uint8_t REG_WAKE_UP_THS = 0x5B;
static constexpr uint8_t REG_WAKE_UP_DUR = 0x5C;
static constexpr uint8_t REG_MD1_CFG     = 0x5E;
static constexpr uint8_t REG_WAKE_UP_SRC = 0x1B;
static constexpr uint8_t REG_OUTX_L_G    = 0x22;
static constexpr uint8_t REG_OUTX_L_A    = 0x28;

// ── Timing ─────────────────────────────────────────────────────────

static constexpr uint32_t AUTO_SLEEP_MS       = 8000;
static constexpr uint32_t UI_REFRESH_MS       = 250;
static constexpr uint32_t BAT_REFRESH_MS      = 1000;
static constexpr uint32_t WAKE_LOCK_MS        = 1200;
// Native USB CDC can disconnect while the ESP32-S3 is in light sleep.
// Keep this false for serial-monitor testing; set true for low-power testing.
static constexpr bool     ENABLE_LIGHT_SLEEP  = false;
static constexpr uint8_t  BACKLIGHT_AWAKE_PWM = 160;
static constexpr uint8_t  BACKLIGHT_SLEEP_PWM = 0;
static constexpr float    BAT_DIVIDER_RATIO   = (316.0f + 160.0f) / 160.0f;

// ── Runtime state ──────────────────────────────────────────────────

uint8_t       imuAddr          = 0;
volatile bool imuWakeFlag      = false;
bool          screenAwake      = true;
uint32_t      lastActivityMs   = 0;
uint32_t      lastUiMs         = 0;
uint32_t      lastBatMs        = 0;
uint32_t      lastWakeMs       = 0;
uint32_t      wakeCount        = 0;
uint32_t      intCount         = 0;
uint8_t       lastWakeSrc      = 0;
float         ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
uint32_t      batMv            = 0;
int           batPercent       = 0;

// ── I2C helpers ────────────────────────────────────────────────────

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

// ── IMU ────────────────────────────────────────────────────────────

static bool findLsm(uint8_t addr) {
  uint8_t who = 0;
  if (!read8(addr, 0x0F, &who)) return false;
  if (who != 0x69 && who != 0x6A && who != 0x6C) return false;
  imuAddr = addr;
  Serial.printf("[IMU] LSM6-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

static bool initImuWake() {
  bool ok = findLsm(0x6A) || findLsm(0x6B);
  if (!ok) {
    Serial.println("[IMU] LSM6-compatible IMU not found");
    return false;
  }

  ok &= write8(imuAddr, REG_CTRL3_C,     0x44);  // BDU=1, auto-increment
  ok &= write8(imuAddr, REG_CTRL1_XL,    0x40);  // 104 Hz, +/-2g
  ok &= write8(imuAddr, 0x11,            0x40);  // 104 Hz gyro
  ok &= write8(imuAddr, REG_TAP_CFG,     0x80);  // enable embedded functions
  ok &= write8(imuAddr, REG_WAKE_UP_THS, 0x05);
  ok &= write8(imuAddr, REG_WAKE_UP_DUR, 0x00);
  ok &= write8(imuAddr, REG_MD1_CFG,     0x20);  // route wake-up to INT1

  uint8_t dummy = 0;
  (void)read8(imuAddr, REG_WAKE_UP_SRC, &dummy);
  return ok;
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

void IRAM_ATTR onImuWake() {
  imuWakeFlag = true;
  intCount++;
}

// ── LCD init ───────────────────────────────────────────────────────

static void setBacklight(uint8_t pwm) {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, pwm ? HIGH : LOW);
  analogWrite(LCD_BL_PIN, pwm);
}

// ── Battery ────────────────────────────────────────────────────────

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
  return batPercent <= 15 ? TFT_RED : (batPercent <= 35 ? TFT_YELLOW : TFT_GREEN);
}

// ── UI (compact layout for 80x160) ─────────────────────────────────

static void printFixed(int x, int y, uint16_t color, const String &text, int width) {
  String out = text;
  while ((int)out.length() < width) out += ' ';
  if ((int)out.length() > width) out = out.substring(0, width);
  display.setTextColor(color, TFT_BLACK);
  display.setTextSize(1);
  display.setCursor(x, y);
  display.print(out);
}

static void drawLayout() {
  display.fillScreen(TFT_BLACK);

  // Title
  display.setTextColor(TFT_GREEN, TFT_BLACK);
  display.setTextSize(2);
  display.setCursor(4, 3);
  display.print("Wake");

  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.setTextSize(1);
  display.setCursor(1, 23);
  display.print("0.96 ESP32-S3");

  display.drawFastHLine(4, 36, 72, TFT_WHITE);

  // POWER section
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.setCursor(4, 42);
  display.print("POWER");
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(4, 56);
  display.print("State");
  display.setCursor(4, 68);
  display.print("BAT");

  display.drawFastHLine(4, 82, 72, TFT_YELLOW);

  // MOTION section
  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  display.setCursor(4, 88);
  display.print("MOTION");
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(4, 102);
  display.print("Acc");
  display.setCursor(4, 114);
  display.print("Gyr");
  display.setCursor(4, 126);
  display.print("INT");

  display.drawFastHLine(4, 140, 72, TFT_WHITE);

  // Footer is updated with the sleep countdown in updateUi().
}

static void updateUi() {
  char buf[32];

  // State
  printFixed(36, 56, TFT_GREEN, "ON", 7);

  // Battery
  snprintf(buf, sizeof(buf), "%.2f %d", batMv / 1000.0f, batPercent);
  printFixed(28, 68, batteryColor(), buf, 8);

  // Acc
  snprintf(buf, sizeof(buf), "%.1f %.1f", ax, ay);
  printFixed(22, 102, TFT_WHITE, buf, 9);

  // Gyr
  snprintf(buf, sizeof(buf), "%.0f %.0f", gx, gy);
  printFixed(22, 114, TFT_WHITE, buf, 9);

  // INT count + wake src
  snprintf(buf, sizeof(buf), "%lu/%02X", (unsigned long)intCount, lastWakeSrc);
  printFixed(22, 126, TFT_CYAN, buf, 9);

  // Sleep countdown
  uint32_t now = millis();
  uint32_t remain = now - lastActivityMs < AUTO_SLEEP_MS
                      ? (AUTO_SLEEP_MS - (now - lastActivityMs)) / 1000 : 0;
  snprintf(buf, sizeof(buf), "U1S U2W %lus", (unsigned long)remain);
  printFixed(4, 148, TFT_DARKGREY, buf, 12);
}

// ── Sleep / Wake ───────────────────────────────────────────────────

static void screenWake(const char *reason) {
  if (screenAwake && millis() - lastWakeMs < WAKE_LOCK_MS) return;
  screenAwake = true;
  lastActivityMs = lastWakeMs = millis();
  wakeCount++;
  setBacklight(BACKLIGHT_AWAKE_PWM);
  drawLayout();
  updateUi();
  Serial.printf("[WAKE] %s  count=%lu\n", reason, (unsigned long)wakeCount);
}

static void screenSleep(const char *reason) {
  if (!screenAwake) return;

  Serial.printf("[SLEEP] %s\n", reason);
  Serial.flush();

  display.fillScreen(TFT_BLACK);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.setTextSize(2);
  display.setCursor(12, 52);
  display.print("Sleep");
  display.setTextSize(1);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(4, 82);
  display.print("Move to wake");
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.setCursor(1, 102);
  display.print("IMU INT = D14");

  delay(450);
  setBacklight(BACKLIGHT_SLEEP_PWM);

  screenAwake = false;
  imuWakeFlag = false;
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
  if (imuWakeFlag) { imuWakeFlag = false; pending = true; }
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

// ── Setup / Loop ───────────────────────────────────────────────────

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
  Serial.println("=== XIAO ESP32-S3 Plus 0.96 IMU Wake Demo ===");

  if (!display.begin<Board_XIAO_0inch96_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_Seeed_0inch96_LCD_ST7789>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  setBacklight(BACKLIGHT_AWAKE_PWM);
  display.setTextWrap(false);
  display.fillScreen(TFT_BLACK);

  if (!initImuWake()) {
    display.setCursor(8, 72);
    display.setTextColor(TFT_RED, TFT_BLACK);
    display.print("IMU init failed");
    while (1) delay(1000);
  }

  updateBattery();
  readImu();
  lastActivityMs = lastBatMs = millis();
  drawLayout();
  updateUi();
  Serial.println("[READY] awake; USR1=sleep, USR2=manual wake, motion=IMU wake");
  Serial.printf("[READY] auto sleep in %lu seconds\n",
                (unsigned long)(AUTO_SLEEP_MS / 1000));
  Serial.printf("[READY] sleep mode: %s\n",
                ENABLE_LIGHT_SLEEP ? "ESP32 light sleep (USB CDC may disconnect)"
                                   : "display only (USB CDC stays connected)");
}

void loop() {
  handleWakeEvents();

  if (!screenAwake) {
    if (ENABLE_LIGHT_SLEEP) {
      lightSleepWait();
    } else {
      delay(10);
    }
    handleWakeEvents();
    return;
  }

  // USR1: force sleep
  if (digitalRead(USR1_PIN) == LOW) {
    delay(30);
    if (digitalRead(USR1_PIN) == LOW) {
      screenSleep("USR1");
      while (digitalRead(USR1_PIN) == LOW) delay(5);
    }
  }

  uint32_t now = millis();
  if (now - lastBatMs >= BAT_REFRESH_MS) { lastBatMs = now; updateBattery(); }
  if (now - lastUiMs >= UI_REFRESH_MS)   { lastUiMs = now; readImu(); updateUi(); }
  if (now - lastActivityMs >= AUTO_SLEEP_MS) screenSleep("AUTO_TIMEOUT");
  delay(5);
}
