/*
  XIAO nRF52840 Plus + 1.14 Display
  IMU D14 Wake Screen Sleep Demo - nRF System ON Sleep

  Based on the proven-working 1.14 graphictest skeleton.
  Wake/sleep logic ported from 1.47 nRF52840 wakeup.

  1.14 board pin map:
    LCD CS  = D2     LCD DC  = D3     I2C SDA = D4
    I2C SCL = D5     USR1    = D6     USR2    = D7
    LCD SCK = D8     LCD MOSI= D10    IMU INT = D14
    LCD RST = D17    LCD BL  = D18

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  driver.h + manual pin setup + tft.init()/setRotation(0)/invertDisplay(true).
  DROP driver.h + prepareLcdPins/hardResetPanel. RST/BL are now raw GPIO 38/37.
  Config_Seeed_1inch14_LCD_ST7789 bakes 135x240 RGB invert=true rot0.
  LSM6DS3 wake interrupt, USR1/USR2 buttons and nRF System ON sleep unchanged.

  Required libraries:
    - Seeed_GFX2
    - Seeed_Arduino_LSM6DS3
    - Adafruit_TinyUSB
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include "LSM6DS3.h"

// ========================= Pins =========================

static constexpr int8_t LCD_RST_PIN   = 38;  // XIAO nRF52840 Plus
static constexpr int8_t LCD_BL_PIN    = 37;
static constexpr uint8_t IMU_INT_PIN  = D14;
static constexpr uint8_t USR1_PIN     = D6;
static constexpr uint8_t USR2_PIN     = D7;

// ========================= LCD =========================

Seeed_GFX display;

// ========================= IMU =========================

LSM6DS3 myIMU(I2C_MODE, 0x6A);

static constexpr uint8_t LSM6DS3_ADDR    = 0x6A;
static constexpr uint8_t REG_CTRL1_XL    = 0x10;
static constexpr uint8_t REG_CTRL3_C     = 0x12;
static constexpr uint8_t REG_TAP_CFG     = 0x58;
static constexpr uint8_t REG_WAKE_UP_THS = 0x5B;
static constexpr uint8_t REG_WAKE_UP_DUR = 0x5C;
static constexpr uint8_t REG_MD1_CFG     = 0x5E;
static constexpr uint8_t REG_WAKE_UP_SRC = 0x1B;

// ========================= Timing =========================

static constexpr uint32_t AUTO_SLEEP_MS   = 8000;
static constexpr uint32_t UI_REFRESH_MS   = 250;
static constexpr uint32_t WAKE_LOCK_MS    = 1200;
static constexpr uint32_t SLEEP_STATUS_MS = 2000;

static constexpr uint8_t IMU_WAKE_THRESHOLD = 0x05;

// ========================= Colors =========================

static constexpr uint16_t C_BLACK  = TFT_BLACK;
static constexpr uint16_t C_WHITE  = TFT_WHITE;
static constexpr uint16_t C_GREEN  = TFT_GREEN;
static constexpr uint16_t C_CYAN   = TFT_CYAN;
static constexpr uint16_t C_YELLOW = TFT_YELLOW;
static constexpr uint16_t C_RED    = TFT_RED;
static constexpr uint16_t C_GRAY   = 0x8410;
static constexpr uint16_t C_LINE   = 0x39E7;

// ========================= Runtime state =========================

volatile bool g_imuWakeFlag = false;
volatile bool g_usrWakeFlag = false;

bool     g_screenAwake       = true;
uint32_t g_lastActivityMs    = 0;
uint32_t g_lastUiMs          = 0;
uint32_t g_wakeCount         = 0;
uint32_t g_intCount          = 0;
uint32_t g_lastWakeMs        = 0;
uint32_t g_sleepEnterMs      = 0;
uint32_t g_lastSleepStatusMs = 0;
uint32_t g_sleepLoopCount    = 0;

float   g_ax = 0, g_ay = 0, g_az = 0;
float   g_gx = 0, g_gy = 0, g_gz = 0;
uint8_t g_lastWakeSrc = 0;

// ========================= Low-level helpers =========================

static bool imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool imuReadReg(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)LSM6DS3_ADDR, 1) != 1) return false;
  val = Wire.read();
  return true;
}

void imuWakeIsr() { g_imuWakeFlag = true; g_intCount++; }
void usrWakeIsr() { g_usrWakeFlag = true; }

// ========================= LCD init =========================

static void backlightOn() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
}

static void backlightOff() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, LOW);
}

static void initDisplay() {
  if (!display.begin<Board_XIAO_1inch14_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_Seeed_1inch14_LCD_ST7789>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  backlightOn();
  display.fillScreen(C_BLACK);
}

// ========================= nRF System ON sleep =========================

static void systemOnSleepOnce() {
  __SEV();
  __WFE();
  __WFE();
}

static void enterSystemOnSleepLoop() {
  g_sleepLoopCount++;
  uint32_t now = millis();
  if (now - g_lastSleepStatusMs >= SLEEP_STATUS_MS) {
    g_lastSleepStatusMs = now;
    Serial.print("[SLEEP] loops=");
    Serial.print((unsigned long)g_sleepLoopCount);
    Serial.print(" D14=");
    Serial.print(digitalRead(IMU_INT_PIN));
    Serial.print(" awake=");
    Serial.println(g_screenAwake ? "Y" : "N");
  }
  systemOnSleepOnce();
}

// ========================= UI helpers =========================

static void printFixed(int x, int y, uint16_t color, const String &text, int w) {
  String out = text;
  while ((int)out.length() < w) out += ' ';
  if ((int)out.length() > w) out = out.substring(0, w);
  display.setTextSize(1);
  display.setTextColor(color, C_BLACK);
  display.setCursor(x, y);
  display.print(out);
}

// ========================= UI layout =========================

static void drawAwakeLayout() {
  display.fillScreen(C_BLACK);

  display.setTextSize(2);
  display.setTextColor(C_GREEN, C_BLACK);
  display.setCursor(4, 4);
  display.print("Hello,XIAO!");

  display.setTextSize(1);
  display.setTextColor(C_CYAN, C_BLACK);
  display.setCursor(6, 24);
  display.print("IMU D14 Wake Demo");

  display.drawFastHLine(4, 38, 127, C_LINE);

  // STATE card
  display.drawRoundRect(4, 46, 127, 60, 5, C_CYAN);
  display.setTextColor(C_CYAN, C_BLACK);
  display.setCursor(14, 56);  display.print("POWER STATE");
  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(14, 72);  display.print("Screen");
  display.setCursor(14, 86);  display.print("WakeCnt");

  // MOTION card
  display.drawRoundRect(4, 112, 127, 70, 5, C_YELLOW);
  display.setTextColor(C_YELLOW, C_BLACK);
  display.setCursor(14, 122); display.print("MOTION");
  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(14, 138); display.print("Acc");
  display.setCursor(14, 152); display.print("Gyr");
  display.setCursor(14, 166); display.print("INT");

  // TEST card
  display.drawRoundRect(4, 188, 127, 34, 5, C_GREEN);
  display.setTextColor(C_GREEN, C_BLACK);
  display.setCursor(12, 198); display.print("TEST");
  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(12, 212); display.print("U1 sleep  U2 wake");
}

static void updateAwakeUi() {
  char buf[48];

  printFixed(58, 72, C_GREEN,  "AWAKE", 10);

  snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_wakeCount);
  printFixed(58, 86, C_YELLOW, buf, 10);

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", g_ax, g_ay, g_az);
  printFixed(38, 138, C_WHITE, buf, 14);

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", g_gx, g_gy, g_gz);
  printFixed(38, 152, C_WHITE, buf, 14);

  snprintf(buf, sizeof(buf), "D14:%lu src:0x%02X", (unsigned long)g_intCount, g_lastWakeSrc);
  printFixed(38, 166, C_CYAN, buf, 14);

  uint32_t remain = 0;
  uint32_t now = millis();
  if (now - g_lastActivityMs < AUTO_SLEEP_MS) {
    remain = (AUTO_SLEEP_MS - (now - g_lastActivityMs)) / 1000;
  }
  snprintf(buf, sizeof(buf), "Auto sleep %lus", (unsigned long)remain);
  printFixed(12, 228, C_GRAY, buf, 18);
}

// ========================= Sleep / Wake =========================

static void screenWake(const char *reason) {
  if (g_screenAwake && (millis() - g_lastWakeMs < WAKE_LOCK_MS)) return;

  g_screenAwake = true;
  g_lastActivityMs = millis();
  g_lastWakeMs = millis();
  g_wakeCount++;

  display.initFromSleep();
  display.writecommand(ST7789_DISPON);
  delay(20);
  backlightOn();
  drawAwakeLayout();
  updateAwakeUi();

  Serial.print("[WAKE] reason=");
  Serial.print(reason);
  Serial.print(" wakeCount=");
  Serial.print(g_wakeCount);
  Serial.print(" sleptMs=");
  Serial.print((unsigned long)(millis() - g_sleepEnterMs));
  Serial.print(" sleepLoops=");
  Serial.println((unsigned long)g_sleepLoopCount);
}

static void screenSleep() {
  if (!g_screenAwake) return;

  Serial.println("[SLEEP] screen off, entering System ON sleep");

  display.fillScreen(C_BLACK);
  display.setTextSize(2);
  display.setTextColor(C_CYAN, C_BLACK);
  display.setCursor(16, 80);
  display.print("Sleeping...");
  display.setTextSize(1);
  display.setCursor(12, 112);
  display.print("Pick up device to wake");
  delay(500);

  // Turn off the ST7789 controller as well as its backlight. The previous
  // version only drove the BL pin low, which is not sufficient on every
  // 1.14-inch display board.
  display.writecommand(ST7789_DISPOFF);
  delay(5);
  display.writecommand(ST7789_SLPIN);
  delay(5);
  backlightOff();
  g_screenAwake = false;
  g_sleepEnterMs = millis();
  g_lastSleepStatusMs = 0;
  g_sleepLoopCount = 0;

  uint8_t dummy = 0;
  imuReadReg(REG_WAKE_UP_SRC, dummy);
  g_imuWakeFlag = false;
  g_usrWakeFlag = false;
}

// ========================= IMU init =========================

static bool initImuWakeInterrupt() {
  Wire.begin();

  int imuOk = myIMU.begin();
  Serial.print("[IMU] Seeed LSM6DS3 begin=");
  Serial.println(imuOk);

  if (imuOk != 0) {
    Serial.println("[IMU] WARNING: begin() returned non-zero");
  }

  bool ok = true;
  ok &= imuWriteReg(REG_CTRL3_C,     0x44);  // BDU=1, auto-increment
  ok &= imuWriteReg(REG_CTRL1_XL,    0x40);  // 104Hz, +/-2g
  ok &= imuWriteReg(REG_TAP_CFG,     0x80);  // enable embedded interrupts
  ok &= imuWriteReg(REG_WAKE_UP_THS, IMU_WAKE_THRESHOLD);
  ok &= imuWriteReg(REG_WAKE_UP_DUR, 0x00);
  ok &= imuWriteReg(REG_MD1_CFG,     0x20);  // route wake-up to INT1

  uint8_t dummy = 0;
  imuReadReg(REG_WAKE_UP_SRC, dummy);

  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuWakeIsr, RISING);

  Serial.print("[IMU] D14 wake interrupt ");
  Serial.println(ok ? "OK" : "FAILED");
  return ok;
}

static void updateImuData() {
  g_ax = myIMU.readFloatAccelX();
  g_ay = myIMU.readFloatAccelY();
  g_az = myIMU.readFloatAccelZ();
  g_gx = myIMU.readFloatGyroX();
  g_gy = myIMU.readFloatGyroY();
  g_gz = myIMU.readFloatGyroZ();
}

static void handleWakeEvents() {
  bool imuPending = false;
  bool usrPending = false;

  noInterrupts();
  if (g_imuWakeFlag) { g_imuWakeFlag = false; imuPending = true; }
  if (g_usrWakeFlag) { g_usrWakeFlag = false; usrPending = true; }
  interrupts();

  if (usrPending) { screenWake("USR2_INT"); return; }

  if (digitalRead(IMU_INT_PIN) == HIGH) imuPending = true;
  if (!imuPending) return;

  uint8_t src = 0;
  if (imuReadReg(REG_WAKE_UP_SRC, src)) g_lastWakeSrc = src;

  if ((src & 0x08) || (src & 0x07) || !g_screenAwake) screenWake("IMU_D14");
}

// ========================= Setup =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 1.14 IMU Wake Demo ===");

  // 1. LCD first (proven working order from graphictest)
  initDisplay();
  Serial.print("LCD: ");
  Serial.print(display.width());
  Serial.print("x");
  Serial.println(display.height());

  // 2. Buttons
  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(USR2_PIN), usrWakeIsr, FALLING);

  // 3. IMU
  initImuWakeInterrupt();

  // 4. Draw UI
  g_lastActivityMs = millis();
  drawAwakeLayout();
  updateAwakeUi();

  Serial.println("[BOOT] done. Screen should be on.");
}

// ========================= Loop =========================

void loop() {
  handleWakeEvents();

  if (!g_screenAwake) {
    enterSystemOnSleepLoop();
    handleWakeEvents();
    delay(1);
    return;
  }

  // USR1: force sleep
  if (digitalRead(USR1_PIN) == LOW) {
    delay(30);
    if (digitalRead(USR1_PIN) == LOW) {
      screenSleep();
      while (digitalRead(USR1_PIN) == LOW) delay(5);
    }
  }

  // USR2: force wake
  if (digitalRead(USR2_PIN) == LOW) {
    delay(30);
    if (digitalRead(USR2_PIN) == LOW) {
      screenWake("USR2");
      while (digitalRead(USR2_PIN) == LOW) delay(5);
    }
  }

  uint32_t now = millis();

  if (g_screenAwake) {
    if (now - g_lastUiMs >= UI_REFRESH_MS) {
      g_lastUiMs = now;
      updateImuData();
      updateAwakeUi();
    }

    if (now - g_lastActivityMs >= AUTO_SLEEP_MS) {
      screenSleep();
    }
  }

  delay(5);
}
