/*
  XIAO nRF52840 Plus + 1.47 Touch Display
  IMU D14 Wake Screen Sleep Demo v0.2 - nRF System ON Sleep

  Target behavior:
    Device screen sleeps -> nRF52840 enters System ON sleep -> user picks up/moves device
    -> LSM6DS3 INT1 triggers D14 -> nRF wakes -> screen lights up.

  This is still a standalone validation firmware.
  Compared with v0.1:
    - LCD backlight off = screen sleep
    - UI refresh paused while sleeping
    - nRF52840 executes WFE-based System ON sleep while screen is off
    - IMU wake-up/motion interrupt on D14 wakes CPU and screen

  It does NOT use nRF System OFF deep sleep. State is preserved and wake is fast.
  After this behavior is verified, merge the same state machine into the factory dashboard.

  Hardware assumptions from the current verified 1.47 nRF52840 Plus firmware:
    LCD CS  = D2
    LCD DC  = D3
    I2C SDA = D4
    I2C SCL = D5
    LCD SCK = D8
    LCD MOSI= D10
    IMU INT = D14
    LCD RST = D19   // current BSP workaround used in previous working firmware
    LCD BL  = D18

  Required Arduino libraries:
    - Arduino_GFX_Library
    - SparkFun LSM6DS3
*/

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include "SparkFunLSM6DS3.h"
#include <nrf.h>

// ========================= Pins =========================

static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t I2C_SDA_PIN   = D4;
static constexpr uint8_t I2C_SCL_PIN   = D5;
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t IMU_INT_PIN   = D14;
static constexpr uint8_t LCD_BL_PIN    = D18;
static constexpr uint8_t LCD_RST_PIN   = D19;

// Optional manual test buttons.
static constexpr uint8_t USR1_PIN = D17; // force sleep
static constexpr uint8_t USR2_PIN = D15; // force wake

// ========================= LCD =========================

Arduino_DataBus *lcdBus = new Arduino_SWSPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  LCD_SCK_PIN,
  LCD_MOSI_PIN,
  GFX_NOT_DEFINED
);

Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus,
  LCD_RST_PIN,
  0,
  false,
  172,
  320,
  34,
  0,
  34,
  0
);

// ========================= IMU =========================

LSM6DS3 myIMU(I2C_MODE, 0x6A);

static constexpr uint8_t LSM6DS3_ADDR      = 0x6A;
static constexpr uint8_t REG_WAKE_UP_SRC   = 0x1B;
static constexpr uint8_t REG_CTRL1_XL      = 0x10;
static constexpr uint8_t REG_CTRL3_C       = 0x12;
static constexpr uint8_t REG_TAP_CFG       = 0x58;
static constexpr uint8_t REG_WAKE_UP_THS   = 0x5B;
static constexpr uint8_t REG_WAKE_UP_DUR   = 0x5C;
static constexpr uint8_t REG_MD1_CFG       = 0x5E;

// ========================= UI / timing =========================

static constexpr uint32_t AUTO_SLEEP_MS = 8000;
static constexpr uint32_t UI_REFRESH_MS = 250;
static constexpr uint32_t WAKE_LOCK_MS  = 1200;
static constexpr uint32_t SLEEP_STATUS_MS = 2000;

// Wake-up sensitivity.
// Lower threshold: easier to wake, but more false triggers.
// Higher threshold: harder to wake.
// Suggested tuning range: 0x03 ~ 0x0A
static constexpr uint8_t IMU_WAKE_THRESHOLD = 0x05;

static constexpr uint8_t BACKLIGHT_AWAKE_PWM = 255;
static constexpr uint8_t BACKLIGHT_SLEEP_PWM = 0;

static constexpr uint16_t C_BLACK  = RGB565_BLACK;
static constexpr uint16_t C_WHITE  = RGB565_WHITE;
static constexpr uint16_t C_GREEN  = RGB565_LIGHTGREEN;
static constexpr uint16_t C_CYAN   = RGB565_CYAN;
static constexpr uint16_t C_YELLOW = RGB565_YELLOW;
static constexpr uint16_t C_RED    = RGB565_RED;
static constexpr uint16_t C_GRAY   = 0x8410;
static constexpr uint16_t C_LINE   = 0x39E7;

// ========================= Runtime state =========================

volatile bool g_imuWakeFlag = false;
volatile bool g_usrWakeFlag = false;

bool g_screenAwake = true;
uint32_t g_lastActivityMs = 0;
uint32_t g_lastUiMs = 0;
uint32_t g_wakeCount = 0;
uint32_t g_intCount = 0;
uint32_t g_lastWakeMs = 0;
uint32_t g_sleepEnterMs = 0;
uint32_t g_lastSleepStatusMs = 0;
uint32_t g_sleepLoopCount = 0;

float g_ax = 0.0f;
float g_ay = 0.0f;
float g_az = 0.0f;
float g_gx = 0.0f;
float g_gy = 0.0f;
float g_gz = 0.0f;
uint8_t g_lastWakeSrc = 0;

// ========================= Helpers =========================

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

void imuWakeIsr() {
  g_imuWakeFlag = true;
  g_intCount++;
}

void usrWakeIsr() {
  g_usrWakeFlag = true;
}

static void acquireForLcd() {
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void lcdHardReset() {
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(10);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(30);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(150);
}

static void lcdWriteMadctlFix() {
  acquireForLcd();
  lcdBus->beginWrite();
  lcdBus->writeC8D8(0x36, 0x48);
  lcdBus->endWrite();
}

static void setBacklight(uint8_t pwm) {
  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, pwm);
}

// ========================= nRF System ON sleep =========================
//
// WFE = Wait For Event. In System ON sleep, RAM/state are preserved.
// Any enabled GPIO interrupt, including D14 from IMU, wakes the CPU and loop continues.
//
// The SEV/WFE/WFE pattern clears any stale event first, then actually sleeps.
// This avoids immediately falling through because of a previous event flag.
static void systemOnSleepOnce() {
#if defined(NRF52840_XXAA) || defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  __SEV();
  __WFE();
  __WFE();
#else
  delay(10);
#endif
}

static void enterSystemOnSleepLoop() {
  // Keep Serial alive for debug, but note USB itself will dominate current if connected.
  g_sleepLoopCount++;

  // Avoid spamming serial while asleep.
  uint32_t now = millis();
  if (now - g_lastSleepStatusMs >= SLEEP_STATUS_MS) {
    g_lastSleepStatusMs = now;
    Serial.print("[SYS_ON_SLEEP] waiting, sleepLoops=");
    Serial.print((unsigned long)g_sleepLoopCount);
    Serial.print(" D14=");
    Serial.print(digitalRead(IMU_INT_PIN));
    Serial.print(" awake=");
    Serial.println(g_screenAwake ? "Y" : "N");
  }

  // Enter nRF System ON sleep until an interrupt/event arrives.
  systemOnSleepOnce();
}

static void printFixed(int x, int y, uint16_t color, const String &text, int width) {
  String out = text;
  while ((int)out.length() < width) out += ' ';
  if ((int)out.length() > width) out = out.substring(0, width);

  acquireForLcd();
  gfx->setTextSize(1);
  gfx->setTextColor(color, C_BLACK);
  gfx->setCursor(x, y);
  gfx->print(out);
}

// ========================= LCD UI =========================

static bool initLcd() {
  setBacklight(BACKLIGHT_AWAKE_PWM);
  lcdHardReset();

  if (!gfx->begin()) {
    Serial.println("[LCD] begin failed");
    return false;
  }

  lcdWriteMadctlFix();
  gfx->fillScreen(C_BLACK);
  return true;
}

static void drawAwakeLayout() {
  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(8, 10);
  gfx->print("Hello,XIAO!");

  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(10, 36);
  gfx->print("IMU D14 Wake Demo");

  gfx->drawFastHLine(8, 52, 156, C_LINE);

  gfx->drawRoundRect(8, 66, 156, 76, 6, C_CYAN);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(18, 78);
  gfx->print("POWER STATE");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(18, 98);
  gfx->print("Screen");
  gfx->setCursor(18, 116);
  gfx->print("WakeCnt");

  gfx->drawRoundRect(8, 154, 156, 92, 6, C_YELLOW);
  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(18, 166);
  gfx->print("MOTION");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(18, 188);
  gfx->print("Acc");
  gfx->setCursor(18, 208);
  gfx->print("Gyr");
  gfx->setCursor(18, 228);
  gfx->print("INT");

  gfx->drawRoundRect(8, 258, 156, 42, 6, C_GREEN);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(18, 270);
  gfx->print("TEST");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(18, 288);
  gfx->print("USR1 sleep  USR2 wake");
}

static void updateAwakeUi() {
  char buf[48];

  printFixed(78, 98, C_GREEN, "AWAKE", 12);

  snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_wakeCount);
  printFixed(78, 116, C_YELLOW, buf, 12);

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", g_ax, g_ay, g_az);
  printFixed(48, 188, C_WHITE, buf, 16);

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", g_gx, g_gy, g_gz);
  printFixed(48, 208, C_WHITE, buf, 16);

  snprintf(buf, sizeof(buf), "D14:%lu src:0x%02X", (unsigned long)g_intCount, g_lastWakeSrc);
  printFixed(48, 228, C_CYAN, buf, 16);

  uint32_t remain = 0;
  uint32_t now = millis();
  if (now - g_lastActivityMs < AUTO_SLEEP_MS) {
    remain = (AUTO_SLEEP_MS - (now - g_lastActivityMs)) / 1000;
  }
  snprintf(buf, sizeof(buf), "Auto sleep in %lus", (unsigned long)remain);
  printFixed(18, 306, C_GRAY, buf, 20);
}

static void screenWake(const char *reason) {
  if (g_screenAwake && (millis() - g_lastWakeMs < WAKE_LOCK_MS)) return;

  g_screenAwake = true;
  g_lastActivityMs = millis();
  g_lastWakeMs = millis();
  g_wakeCount++;

  setBacklight(BACKLIGHT_AWAKE_PWM);
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

  Serial.println("[SLEEP] screen backlight off, waiting for IMU D14 wake");

  acquireForLcd();
  gfx->fillScreen(C_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(18, 120);
  gfx->print("Sleeping...");
  gfx->setTextSize(1);
  gfx->setCursor(18, 150);
  gfx->print("Pick up device to wake");
  delay(500);

  setBacklight(BACKLIGHT_SLEEP_PWM);
  g_screenAwake = false;
  g_sleepEnterMs = millis();
  g_lastSleepStatusMs = 0;
  g_sleepLoopCount = 0;

  // Clear stale GPIO/source states before entering WFE loop.
  uint8_t dummy = 0;
  imuReadReg(REG_WAKE_UP_SRC, dummy);
  g_imuWakeFlag = false;
  g_usrWakeFlag = false;
}

// ========================= IMU wake config =========================

static bool initImuWakeInterrupt() {
  Wire.begin();

  int imuOk = myIMU.begin();
  Serial.print("[IMU] SparkFun begin=");
  Serial.println(imuOk);

  bool ok = true;

  // BDU=1 and register auto-increment enabled.
  ok &= imuWriteReg(REG_CTRL3_C, 0x44);

  // Accelerometer: 104Hz, +/-2g.
  // Wake-up event uses accelerometer.
  ok &= imuWriteReg(REG_CTRL1_XL, 0x40);

  // Enable embedded interrupts.
  ok &= imuWriteReg(REG_TAP_CFG, 0x80);

  // Wake-up threshold.
  // 0x05 is a medium-low threshold for "pick up / move device".
  ok &= imuWriteReg(REG_WAKE_UP_THS, IMU_WAKE_THRESHOLD);

  // No extra wake duration, responsive wake.
  ok &= imuWriteReg(REG_WAKE_UP_DUR, 0x00);

  // Route wake-up interrupt to INT1.
  // MD1_CFG bit5 = INT1_WU.
  ok &= imuWriteReg(REG_MD1_CFG, 0x20);

  uint8_t dummy = 0;
  imuReadReg(REG_WAKE_UP_SRC, dummy); // clear stale event.

  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuWakeIsr, RISING);

  Serial.print("[IMU] D14 wake interrupt ");
  Serial.println(ok ? "OK" : "FAILED");
  Serial.print("[IMU] wake threshold=0x");
  Serial.println(IMU_WAKE_THRESHOLD, HEX);

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
  if (g_imuWakeFlag) {
    g_imuWakeFlag = false;
    imuPending = true;
  }
  if (g_usrWakeFlag) {
    g_usrWakeFlag = false;
    usrPending = true;
  }
  interrupts();

  if (usrPending) {
    screenWake("USR2_INT");
    return;
  }

  // Polling fallback: if the edge is missed but INT is held high briefly, still wake.
  if (digitalRead(IMU_INT_PIN) == HIGH) {
    imuPending = true;
  }

  if (!imuPending) return;

  uint8_t src = 0;
  if (imuReadReg(REG_WAKE_UP_SRC, src)) {
    g_lastWakeSrc = src;
  }

  // WAKE_UP_SRC bit3 WU_IA indicates wake-up event latched.
  // But some boards may only expose axis bits, so wake on any non-zero source too.
  if ((src & 0x08) || (src & 0x07) || !g_screenAwake) {
    screenWake("IMU_D14");
  }
}

// ========================= Setup / loop =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(USR2_PIN), usrWakeIsr, FALLING);

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 1.47 IMU D14 Wake Demo v0.2 ===");
  Serial.println("Screen sleeps by backlight off, then nRF enters System ON sleep.");
  Serial.println("D14 IMU wake-up interrupt wakes CPU and turns screen on.");

  initImuWakeInterrupt();

  if (!initLcd()) {
    Serial.println("[FAIL] LCD init failed");
    return;
  }

  g_lastActivityMs = millis();
  drawAwakeLayout();
  updateAwakeUi();
}

void loop() {
  handleWakeEvents();

  if (!g_screenAwake) {
    // In sleep state, do not refresh UI or poll sensors.
    // CPU sleeps here and wakes on IMU D14 or USR2 interrupt.
    enterSystemOnSleepLoop();
    handleWakeEvents();
    delay(1);
    return;
  }

  // Manual test while awake:
  // USR1: force sleep.
  // USR2: force wake.
  if (digitalRead(USR1_PIN) == LOW) {
    delay(30);
    if (digitalRead(USR1_PIN) == LOW) {
      screenSleep();
      while (digitalRead(USR1_PIN) == LOW) delay(5);
    }
  }

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
