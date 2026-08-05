/*
  XIAO nRF52840 Plus + 0.96 Inch Display
  IMU D9 Wake / nRF System ON Sleep Test v0.1

  Target behavior:
    screen on -> idle timeout -> screen off -> nRF52840 System ON sleep
    -> user picks up / moves device
    -> LSM6DS3 INT1 triggers D9
    -> MCU wakes -> screen turns on and redraws UI

  Pin map for 0.96 nRF52840 Plus display board:
    LCD CS   = D2
    LCD DC   = D3
    I2C SDA  = D4
    I2C SCL  = D5
    USR1     = D6   force sleep
    USR2     = D7   force wake / reset wake counter
    LCD SCK  = D8
    IMU INT  = D9
    LCD MOSI = D10
    LCD RST  = D17
    LCD BL   = D18

  Notes:
    - This is System ON sleep, not System OFF.
    - RAM/state are preserved.
    - USB connection will dominate current measurement. For current testing, use battery power.
    - 0.96 panel color order is red/blue swapped in our current init, so this test uses visual color aliases.
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
static constexpr uint8_t USR1_PIN      = D6;   // force sleep
static constexpr uint8_t USR2_PIN      = D7;   // force wake / reset counter
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t IMU_INT_PIN   = D9;   // 0.96 version: IMU INT
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t LCD_RST_PIN   = D17;
static constexpr uint8_t LCD_BL_PIN    = D18;

// ========================= LCD =========================

static constexpr int LCD_W = 80;
static constexpr int LCD_H = 160;
static constexpr int LCD_ROTATION = 2;
static constexpr bool LCD_IPS = true;
static constexpr bool LCD_INVERT_COLORS = true;

// Known-good 0.96 ST7789 offset from Dashboard bring-up.
// If image shifts, test 25/0/25/0 or 26/0/26/0.
static constexpr int LCD_COL_OFFSET_1 = 24;
static constexpr int LCD_ROW_OFFSET_1 = 0;
static constexpr int LCD_COL_OFFSET_2 = 24;
static constexpr int LCD_ROW_OFFSET_2 = 0;

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
  LCD_ROTATION,
  LCD_IPS,
  LCD_W,
  LCD_H,
  LCD_COL_OFFSET_1,
  LCD_ROW_OFFSET_1,
  LCD_COL_OFFSET_2,
  LCD_ROW_OFFSET_2
);

// ========================= Colors =========================

static constexpr uint16_t C_BLACK  = RGB565_BLACK;
static constexpr uint16_t C_WHITE  = RGB565_WHITE;
static constexpr uint16_t C_GREEN  = RGB565_LIGHTGREEN;
static constexpr uint16_t C_RED    = RGB565_RED;
static constexpr uint16_t C_BLUE   = RGB565_BLUE;
static constexpr uint16_t C_CYAN   = RGB565_CYAN;
static constexpr uint16_t C_YELLOW = RGB565_YELLOW;
static constexpr uint16_t C_GRAY   = 0x8410;
static constexpr uint16_t C_DIM    = 0x2104;
static constexpr uint16_t C_LINE   = 0x39E7;

// Visual aliases for this 0.96 screen init.
// RED/BLUE and YELLOW/CYAN appear swapped on the actual panel.
static constexpr uint16_t V_RED     = C_BLUE;
static constexpr uint16_t V_BLUE    = C_RED;
static constexpr uint16_t V_YELLOW  = C_CYAN;
static constexpr uint16_t V_CYAN    = C_YELLOW;
static constexpr uint16_t V_GREEN   = C_GREEN;
static constexpr uint16_t V_WHITE   = C_WHITE;

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

// Motion wake threshold.
// Lower = easier to wake. Suggested range: 0x03 ~ 0x0A.
// 0x05 is a good starting point for "pick up device".
static constexpr uint8_t IMU_WAKE_THRESHOLD = 0x05;

// ========================= Timing =========================

static constexpr uint32_t AUTO_SLEEP_MS     = 8000;
static constexpr uint32_t UI_REFRESH_MS     = 250;
static constexpr uint32_t WAKE_LOCK_MS      = 1200;
static constexpr uint32_t SLEEP_LOG_MS      = 2000;
static constexpr uint32_t BTN_DEBOUNCE_MS   = 35;

// ========================= Runtime =========================

volatile bool g_imuWakeFlag = false;
volatile bool g_usrWakeFlag = false;

bool g_lcdOk = false;
bool g_screenAwake = true;
bool g_imuOk = false;

uint32_t g_lastActivityMs = 0;
uint32_t g_lastUiMs = 0;
uint32_t g_lastWakeMs = 0;
uint32_t g_sleepEnterMs = 0;
uint32_t g_lastSleepLogMs = 0;
uint32_t g_sleepLoopCount = 0;

uint32_t g_wakeCount = 0;
uint32_t g_imuIntCount = 0;
uint32_t g_usrWakeCount = 0;

float g_ax = 0.0f;
float g_ay = 0.0f;
float g_az = 0.0f;
float g_gx = 0.0f;
float g_gy = 0.0f;
float g_gz = 0.0f;

uint8_t g_lastWakeSrc = 0;

// ========================= Low-level helpers =========================

static void acquireForLcd() {
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void backlightOn() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
}

static void backlightOff() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, LOW);
}

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

static String padRight(const String &s, int width) {
  String out = s;
  while ((int)out.length() < width) out += ' ';
  if ((int)out.length() > width) out = out.substring(0, width);
  return out;
}

static void printTextFixed(int x, int y, uint16_t color, const String &s, int widthChars) {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->setTextSize(1);
  gfx->setTextColor(color, C_BLACK);
  gfx->setCursor(x, y);
  gfx->print(padRight(s, widthChars));
}

// ========================= Interrupts =========================

void imuWakeIsr() {
  g_imuWakeFlag = true;
  g_imuIntCount++;
}

void usrWakeIsr() {
  g_usrWakeFlag = true;
  g_usrWakeCount++;
}

// ========================= nRF System ON sleep =========================
//
// WFE = Wait For Event.
// SEV/WFE/WFE clears stale events first, then enters System ON sleep.

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
  g_sleepLoopCount++;

  uint32_t now = millis();
  if (now - g_lastSleepLogMs >= SLEEP_LOG_MS) {
    g_lastSleepLogMs = now;

    Serial.print("[SYS_ON_SLEEP] loops=");
    Serial.print((unsigned long)g_sleepLoopCount);
    Serial.print(" D9=");
    Serial.print(digitalRead(IMU_INT_PIN));
    Serial.print(" screen=");
    Serial.println(g_screenAwake ? "AWAKE" : "SLEEP");
  }

  systemOnSleepOnce();
}

// ========================= LCD UI =========================

static bool initLcd() {
  backlightOn();

  if (!gfx->begin()) {
    g_lcdOk = false;
    Serial.println("[LCD] begin failed");
    return false;
  }

  if (LCD_INVERT_COLORS) {
    gfx->invertDisplay(true);
  }

  gfx->fillScreen(C_BLACK);
  gfx->setTextWrap(false);
  g_lcdOk = true;

  Serial.println("[LCD] OK 0.96 ST7789 80x160");
  return true;
}

static void drawStaticAwakeUi() {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(V_GREEN, C_BLACK);
  gfx->setCursor(6, 3);
  gfx->print("Hello");

  gfx->setTextSize(1);
  gfx->setTextColor(V_CYAN, C_BLACK);
  gfx->setCursor(4, 23);
  gfx->print("0.96 IMU Wake");

  gfx->drawFastHLine(5, 32, 70, V_WHITE);

  gfx->setTextColor(V_CYAN, C_BLACK);
  gfx->setCursor(4, 39);
  gfx->print("STATE");

  gfx->setTextColor(V_WHITE, C_BLACK);
  gfx->setCursor(4, 54);
  gfx->print("Screen");
  gfx->setCursor(4, 67);
  gfx->print("Wake");
  gfx->setCursor(4, 80);
  gfx->print("Src");

  gfx->drawFastHLine(5, 92, 70, C_LINE);

  gfx->setTextColor(V_YELLOW, C_BLACK);
  gfx->setCursor(4, 100);
  gfx->print("MOTION");

  gfx->setTextColor(V_WHITE, C_BLACK);
  gfx->setCursor(4, 115);
  gfx->print("A");
  gfx->setCursor(4, 128);
  gfx->print("G");

  gfx->drawFastHLine(5, 141, 70, C_LINE);

  gfx->setTextColor(V_YELLOW, C_BLACK);
  gfx->setCursor(4, 150);
  gfx->print("USR1 Sleep");
}

static String fmtAxis(float v) {
  int iv = (int)roundf(v);
  if (iv > 99) iv = 99;
  if (iv < -99) iv = -99;

  char buf[6];
  snprintf(buf, sizeof(buf), "%+d", iv);
  return String(buf);
}

static void updateAwakeUi() {
  if (!g_lcdOk || !g_screenAwake) return;

  printTextFixed(46, 54, V_GREEN, "ON", 5);
  printTextFixed(46, 67, V_YELLOW, String((unsigned long)g_wakeCount), 5);

  char srcBuf[8];
  snprintf(srcBuf, sizeof(srcBuf), "0x%02X", g_lastWakeSrc);
  printTextFixed(46, 80, V_CYAN, srcBuf, 5);

  String acc = fmtAxis(g_ax) + " " + fmtAxis(g_ay) + " " + fmtAxis(g_az);
  printTextFixed(16, 115, V_WHITE, acc, 10);

  String gyr = fmtAxis(g_gx / 10.0f) + " " + fmtAxis(g_gy / 10.0f) + " " + fmtAxis(g_gz / 10.0f);
  printTextFixed(16, 128, V_WHITE, gyr, 10);
}

static void drawSleepHintThenOff() {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(V_CYAN, C_BLACK);
  gfx->setCursor(7, 55);
  gfx->print("Sleep");

  gfx->setTextSize(1);
  gfx->setTextColor(V_WHITE, C_BLACK);
  gfx->setCursor(8, 85);
  gfx->print("System ON");

  gfx->setTextColor(V_YELLOW, C_BLACK);
  gfx->setCursor(8, 104);
  gfx->print("Move to wake");

  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(8, 124);
  gfx->print("IMU INT = D9");
}

static void screenWake(const char *reason) {
  if (g_screenAwake && (millis() - g_lastWakeMs < WAKE_LOCK_MS)) return;

  g_screenAwake = true;
  g_lastActivityMs = millis();
  g_lastWakeMs = millis();
  g_wakeCount++;

  backlightOn();
  drawStaticAwakeUi();
  updateAwakeUi();

  Serial.print("[WAKE] reason=");
  Serial.print(reason);
  Serial.print(" wakeCount=");
  Serial.print((unsigned long)g_wakeCount);
  Serial.print(" sleptMs=");
  Serial.print((unsigned long)(millis() - g_sleepEnterMs));
  Serial.print(" sleepLoops=");
  Serial.print((unsigned long)g_sleepLoopCount);
  Serial.print(" wakeSrc=0x");
  Serial.println(g_lastWakeSrc, HEX);
}

static void screenSleep() {
  if (!g_screenAwake) return;

  Serial.println("[SLEEP] screen off, entering System ON sleep; wake source IMU D9");

  drawSleepHintThenOff();
  delay(450);

  uint8_t dummy = 0;
  imuReadReg(REG_WAKE_UP_SRC, dummy);

  noInterrupts();
  g_imuWakeFlag = false;
  g_usrWakeFlag = false;
  interrupts();

  backlightOff();

  g_screenAwake = false;
  g_sleepEnterMs = millis();
  g_lastSleepLogMs = 0;
  g_sleepLoopCount = 0;
}

// ========================= IMU =========================

static bool initImuWakeInterrupt() {
  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);

  int imuBegin = myIMU.begin();
  bool ok = (imuBegin == 0);

  // BDU=1 and register auto-increment enabled.
  ok &= imuWriteReg(REG_CTRL3_C, 0x44);

  // Accelerometer: 104Hz, +/-2g.
  ok &= imuWriteReg(REG_CTRL1_XL, 0x40);

  // Enable embedded functions / interrupt block.
  ok &= imuWriteReg(REG_TAP_CFG, 0x80);

  // Wake threshold and duration.
  ok &= imuWriteReg(REG_WAKE_UP_THS, IMU_WAKE_THRESHOLD);
  ok &= imuWriteReg(REG_WAKE_UP_DUR, 0x00);

  // Route wake-up interrupt to INT1.
  // MD1_CFG bit5 = INT1_WU.
  ok &= imuWriteReg(REG_MD1_CFG, 0x20);

  uint8_t dummy = 0;
  imuReadReg(REG_WAKE_UP_SRC, dummy);

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuWakeIsr, RISING);

  g_imuOk = ok;

  Serial.print("[IMU] LSM6DS3 wake interrupt on D9 ");
  Serial.println(ok ? "OK" : "FAILED");
  Serial.print("[IMU] wake threshold=0x");
  Serial.println(IMU_WAKE_THRESHOLD, HEX);

  return ok;
}

static void updateImuData() {
  if (!g_imuOk) return;

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
    screenWake("USR2");
    return;
  }

  // Fallback: if edge was missed but INT remains high briefly, still wake/check.
  if (digitalRead(IMU_INT_PIN) == HIGH) {
    imuPending = true;
  }

  if (!imuPending) return;

  uint8_t src = 0;
  if (imuReadReg(REG_WAKE_UP_SRC, src)) {
    g_lastWakeSrc = src;
  }

  // WAKE_UP_SRC bit3 WU_IA indicates wake event latched.
  // Axis bits can also appear, so accept non-zero source as wake evidence.
  if ((src & 0x08) || (src & 0x07) || !g_screenAwake) {
    screenWake("IMU_D9");
  }
}

static void resetCounters() {
  g_wakeCount = 0;
  g_imuIntCount = 0;
  g_usrWakeCount = 0;
  g_lastWakeSrc = 0;
  g_lastActivityMs = millis();

  Serial.println("[BTN] USR2 reset counters / wake");

  if (g_screenAwake) {
    drawStaticAwakeUi();
    updateAwakeUi();
  }
}

// ========================= Arduino =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
  pinMode(LCD_BL_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(USR2_PIN), usrWakeIsr, FALLING);

  Wire.begin();

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 0.96 IMU D9 System ON Wake Test v0.1 ===");
  Serial.println("[TEST] screen off -> System ON sleep -> move/pick up -> IMU D9 wakes screen");
  Serial.println("[BTN] USR1 force sleep, USR2 wake/reset counters");
  Serial.println("[NOTE] Use battery power for current measurement; USB current will dominate.");

  initLcd();
  initImuWakeInterrupt();

  g_lastActivityMs = millis();

  drawStaticAwakeUi();
  updateImuData();
  updateAwakeUi();
}

void loop() {
  handleWakeEvents();

  if (!g_screenAwake) {
    // Sleep state: no UI refresh, no sensor polling.
    enterSystemOnSleepLoop();
    handleWakeEvents();
    delay(1);
    return;
  }

  // USR1: force sleep.
  if (digitalRead(USR1_PIN) == LOW) {
    delay(BTN_DEBOUNCE_MS);
    if (digitalRead(USR1_PIN) == LOW) {
      while (digitalRead(USR1_PIN) == LOW) delay(5);
      screenSleep();
      return;
    }
  }

  // USR2: wake / reset counter while awake.
  if (digitalRead(USR2_PIN) == LOW) {
    delay(BTN_DEBOUNCE_MS);
    if (digitalRead(USR2_PIN) == LOW) {
      while (digitalRead(USR2_PIN) == LOW) delay(5);
      resetCounters();
    }
  }

  uint32_t now = millis();

  if (now - g_lastUiMs >= UI_REFRESH_MS) {
    g_lastUiMs = now;
    updateImuData();
    updateAwakeUi();
  }

  if (now - g_lastActivityMs >= AUTO_SLEEP_MS) {
    screenSleep();
  }

  delay(5);
}
