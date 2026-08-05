/*
  XIAO ESP32-S3 Plus + 1.14 Inch Display IMU wake demo.

  USR1 forces screen sleep. USR2 wakes manually.
  LSM6-compatible IMU wake/motion INT1 on D14 wakes the screen.
*/

#include <Arduino.h>
#include "driver.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

static constexpr uint8_t LCD_CS_PIN = D2;
static constexpr uint8_t LCD_DC_PIN = D3;
static constexpr uint8_t LCD_SCK_PIN = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN = D17;
static constexpr uint8_t LCD_BL_PIN = D18;

static constexpr uint8_t I2C_SDA_PIN = D4;
static constexpr uint8_t I2C_SCL_PIN = D5;
static constexpr uint8_t USR1_PIN = D6;
static constexpr uint8_t USR2_PIN = D7;
static constexpr uint8_t IMU_INT_PIN = D14;
static constexpr uint8_t BAT_ADC_PIN = D16;

static constexpr uint8_t REG_WAKE_UP_SRC = 0x1B;
static constexpr uint8_t REG_CTRL1_XL = 0x10;
static constexpr uint8_t REG_CTRL3_C = 0x12;
static constexpr uint8_t REG_TAP_CFG = 0x58;
static constexpr uint8_t REG_WAKE_UP_THS = 0x5B;
static constexpr uint8_t REG_WAKE_UP_DUR = 0x5C;
static constexpr uint8_t REG_MD1_CFG = 0x5E;
static constexpr uint8_t REG_OUTX_L_G = 0x22;
static constexpr uint8_t REG_OUTX_L_A = 0x28;

static constexpr uint32_t AUTO_SLEEP_MS = 8000;
static constexpr uint32_t UI_REFRESH_MS = 250;
static constexpr uint32_t BAT_REFRESH_MS = 1000;
static constexpr uint32_t WAKE_LOCK_MS = 1200;
static constexpr uint8_t BACKLIGHT_AWAKE_PWM = 160;
static constexpr uint8_t BACKLIGHT_SLEEP_PWM = 0;
static constexpr float BAT_DIVIDER_RATIO = (316.0f + 160.0f) / 160.0f;

TFT_eSPI tft(135, 240);

uint8_t imuAddr = 0;
volatile bool imuWakeFlag = false;
bool screenAwake = true;
uint32_t lastActivityMs = 0;
uint32_t lastUiMs = 0;
uint32_t lastBatMs = 0;
uint32_t lastWakeMs = 0;
uint32_t wakeCount = 0;
uint32_t intCount = 0;
uint8_t lastWakeSrc = 0;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
uint32_t batMv = 0;
int batPercent = 0;

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

  ok &= write8(imuAddr, REG_CTRL3_C, 0x44);
  ok &= write8(imuAddr, REG_CTRL1_XL, 0x40);
  ok &= write8(imuAddr, 0x11, 0x40);
  ok &= write8(imuAddr, REG_TAP_CFG, 0x80);
  ok &= write8(imuAddr, REG_WAKE_UP_THS, 0x05);
  ok &= write8(imuAddr, REG_WAKE_UP_DUR, 0x00);
  ok &= write8(imuAddr, REG_MD1_CFG, 0x20);

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

static void setBacklight(uint8_t pwm) {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, pwm ? HIGH : LOW);
  analogWrite(LCD_BL_PIN, pwm);
}

static void initLcd() {
  prepareLcdPins();
  setBacklight(BACKLIGHT_AWAKE_PWM);
  hardResetPanel();
  tft.init();
  tft.setRotation(0);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
}

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

static void printFixed(int x, int y, uint16_t color, const String &text, int width) {
  String out = text;
  while ((int)out.length() < width) out += ' ';
  if ((int)out.length() > width) out = out.substring(0, width);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(x, y);
  tft.print(out);
}

static void drawLayout() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.print("Wake Demo");

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 32);
  tft.print("XIAO ESP32-S3 1.14");

  tft.drawRoundRect(6, 48, 123, 66, 6, TFT_CYAN);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(14, 58);
  tft.print("POWER");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(14, 76);
  tft.print("State");
  tft.setCursor(14, 92);
  tft.print("BAT");

  tft.drawRoundRect(6, 118, 123, 68, 6, TFT_YELLOW);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(14, 124);
  tft.print("MOTION");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(14, 142);
  tft.print("Acc");
  tft.setCursor(14, 158);
  tft.print("Gyr");
  tft.setCursor(14, 174);
  tft.print("INT");

  tft.drawRoundRect(6, 194, 123, 24, 6, TFT_GREEN);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(18, 202);
  tft.print("U1 sleep  U2 wake");
}

static void updateUi() {
  char buf[36];
  printFixed(58, 76, TFT_GREEN, "AWAKE", 10);
  snprintf(buf, sizeof(buf), "%.2fV %d%%", batMv / 1000.0f, batPercent);
  printFixed(42, 92, batteryColor(), buf, 13);

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", ax, ay, az);
  printFixed(42, 142, TFT_WHITE, buf, 14);
  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", gx, gy, gz);
  printFixed(42, 158, TFT_WHITE, buf, 14);
  snprintf(buf, sizeof(buf), "%lu 0x%02X", (unsigned long)intCount, lastWakeSrc);
  printFixed(42, 174, TFT_CYAN, buf, 14);

  uint32_t now = millis();
  uint32_t remain = now - lastActivityMs < AUTO_SLEEP_MS ? (AUTO_SLEEP_MS - (now - lastActivityMs)) / 1000 : 0;
  snprintf(buf, sizeof(buf), "sleep in %lus", (unsigned long)remain);
  printFixed(20, 224, 0x8410, buf, 15);
}

static void screenWake(const char *reason) {
  if (screenAwake && millis() - lastWakeMs < WAKE_LOCK_MS) return;
  screenAwake = true;
  lastActivityMs = lastWakeMs = millis();
  wakeCount++;
  setBacklight(BACKLIGHT_AWAKE_PWM);
  drawLayout();
  updateUi();
  Serial.printf("[WAKE] %s count=%lu\n", reason, (unsigned long)wakeCount);
}

static void screenSleep() {
  if (!screenAwake) return;
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(20, 92);
  tft.print("Sleeping");
  tft.setTextSize(1);
  tft.setCursor(14, 124);
  tft.print("Move board to wake");
  delay(450);
  setBacklight(BACKLIGHT_SLEEP_PWM);
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
  Serial.println("=== XIAO ESP32-S3 Plus 1.14 IMU Wake Demo ===");

  initLcd();
  if (!initImuWake()) {
    tft.setCursor(12, 104);
    tft.setTextColor(TFT_RED, TFT_BLACK);
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
