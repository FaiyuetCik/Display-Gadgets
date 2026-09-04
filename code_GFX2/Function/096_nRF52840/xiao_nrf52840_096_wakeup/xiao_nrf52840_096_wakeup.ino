/*
  XIAO nRF52840 Plus + 0.96 Display
  IMU D14 Wake Screen Sleep Demo - nRF System ON Sleep

  Uses Arduino_GFX with software SPI — hardware SPI on nRF52840
  is NOT compatible with this 0.96" ST7789 panel.

  0.96 board pin map:
    LCD CS  = D2     LCD DC  = D3     I2C SDA = D4
    I2C SCL = D5     USR1    = D6     USR2    = D7
    LCD SCK = D8     IMU INT = D14     LCD MOSI= D10
    LCD RST = D17    LCD BL  = D18

  0.96 differences from 1.14:
    - 80x160 resolution (vs 135x240)
    - IMU INT on D14
    - Compact UI layout for small screen
    - ST7789 IPS panel, invertDisplay(true)
    - BGR pixel format (R/B channels swapped in hardware)

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  Arduino_SWSPI + Arduino_ST7789 bus/panel setup, manual RST reset,
  acquireForLcd() (CS toggling) and gfx->begin()/invertDisplay(true).
  Board_XIAO_0inch96_LCD<RST=38,BL=37> + Config_Seeed_0inch96_LCD_ST7789 bake
  80x160 BGR rot2 invert=false. The V_RED/V_BLUE/V_CYAN/V_YELLOW colour-swap
  aliases (and unused C_RED/C_BLUE/C_CYAN/C_YELLOW raw constants) are dropped
  — the config's ST7789 BGR bit now produces correct colours. backlightOn/Off
  are runtime GPIO for the sleep demo, kept. LSM6DS3 wake-register config,
  __SEV()/__WFE() System ON sleep and GPIO ISRs unchanged.

  Required libraries:
    - Seeed_GFX2
    - SparkFun_LSM6DS3_Breakout (LSM6DS3 class; Seeed's LSM6DS3 lib is not installed)
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

static constexpr int8_t LCD_RST_PIN  = 38;  // XIAO nRF52840 Plus (raw GPIO)
static constexpr int8_t LCD_BL_PIN   = 37;
static constexpr uint8_t IMU_INT_PIN = D14;   // D14 in schematic, but D14 on actual board
static constexpr uint8_t USR1_PIN    = D6;
static constexpr uint8_t USR2_PIN    = D7;

// ========================= LCD =========================

Seeed_GFX display;
bool g_lcdOk = false;

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
static constexpr uint32_t SLEEP_LOG_MS    = 2000;
static constexpr uint32_t BTN_DEBOUNCE_MS = 35;

static constexpr uint8_t IMU_WAKE_THRESHOLD = 0x05;

// ========================= Colors =========================
// Config_Seeed_0inch96_LCD_ST7789 is BGR/rot2/invert=false and already produces
// correct colours, so the TFT_* palette is used directly. The original sketch's
// V_RED/V_BLUE/V_CYAN/V_YELLOW colour-swap aliases (and the now-unused
// C_RED/C_BLUE/C_CYAN/C_YELLOW raw constants) are dropped — they only
// compensated for the panel's R/B + Y/C swap under the old invertDisplay(true)
// init. Raw 0xRRGB literals (C_GRAY, C_LINE) stay unchanged.

static constexpr uint16_t C_BLACK = 0x0000;
static constexpr uint16_t C_WHITE = 0xFFFF;
static constexpr uint16_t C_GREEN = 0x07E0;
static constexpr uint16_t C_GRAY  = 0x8410;
static constexpr uint16_t C_LINE  = 0x8410;  // medium gray

// ========================= Runtime state =========================

volatile bool g_imuWakeFlag = false;
volatile bool g_usrWakeFlag = false;

bool     g_screenAwake       = true;
bool     g_imuOk              = false;
uint32_t g_lastActivityMs    = 0;
uint32_t g_lastUiMs          = 0;
uint32_t g_lastWakeMs        = 0;
uint32_t g_sleepEnterMs      = 0;
uint32_t g_lastSleepLogMs    = 0;
uint32_t g_sleepLoopCount    = 0;
uint32_t g_wakeCount         = 0;
uint32_t g_imuIntCount       = 0;

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

static String padRight(const String &s, int width) {
  String out = s;
  while ((int)out.length() < width) out += ' ';
  if ((int)out.length() > width) out = out.substring(0, width);
  return out;
}

void imuWakeIsr() {
  g_imuWakeFlag = true;
  g_imuIntCount++;
}
void usrWakeIsr() { g_usrWakeFlag = true; }

// ========================= LCD init =========================

// Backlight is runtime GPIO for the sleep demo (BL off while in System ON
// sleep, BL on when a wake redraws). display.begin<>() drives BL high at init;
// these helpers only toggle it during the run.
static void backlightOn() {
  pinMode(LCD_BL_PIN, OUTPUT);
  if (g_lcdOk && display.panelPtr()) {
    // GFX2 owns the backlight PWM after display.begin().
    display.panel().setBacklight(255);
  } else {
    digitalWrite(LCD_BL_PIN, HIGH);
  }
}

static void backlightOff() {
  pinMode(LCD_BL_PIN, OUTPUT);
  if (g_lcdOk && display.panelPtr()) {
    // Stop the GFX2 PWM output instead of only changing the GPIO latch.
    display.panel().setBacklight(0);
  } else {
    digitalWrite(LCD_BL_PIN, LOW);
  }
}

static bool initDisplay() {
  backlightOn();

  if (!display.begin<Board_XIAO_0inch96_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_Seeed_0inch96_LCD_ST7789>()) {
    g_lcdOk = false;
    Serial.println("[LCD] begin failed");
    Serial.println(display.lastResult().message);
    return false;
  }

  display.fillScreen(C_BLACK);
  display.setTextWrap(false);
  g_lcdOk = true;

  Serial.println("[LCD] OK 0.96 ST7789 80x160");
  return true;
}

// ========================= UI helpers =========================

static void printTextFixed(int x, int y, uint16_t color, const String &s, int widthChars) {
  if (!g_lcdOk) return;
  display.setTextSize(1);
  display.setTextColor(color, C_BLACK);
  display.setCursor(x, y);
  display.print(padRight(s, widthChars));
}

static String fmtAxis(float v) {
  int iv = (int)roundf(v);
  if (iv > 99) iv = 99;
  if (iv < -99) iv = -99;
  char buf[6];
  snprintf(buf, sizeof(buf), "%+d", iv);
  return String(buf);
}

// ========================= UI layout =========================

static void drawStaticAwakeUi() {
  if (!g_lcdOk) return;

  display.fillScreen(C_BLACK);

  // Title
  display.setTextSize(2);
  display.setTextColor(C_GREEN, C_BLACK);
  display.setCursor(6, 3);
  display.print("Hello");

  display.setTextSize(1);
  display.setTextColor(TFT_CYAN, C_BLACK);
  display.setCursor(4, 23);
  display.print("0.96 IMU Wake");

  display.drawFastHLine(5, 32, 70, C_WHITE);

  // STATE section
  display.setTextColor(TFT_CYAN, C_BLACK);
  display.setCursor(4, 39);
  display.print("STATE");

  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(4, 54);
  display.print("Screen");
  display.setCursor(4, 67);
  display.print("Wake");
  display.setCursor(4, 80);
  display.print("Src");

  display.drawFastHLine(5, 92, 70, C_WHITE);

  // MOTION section
  display.setTextColor(TFT_YELLOW, C_BLACK);
  display.setCursor(4, 100);
  display.print("MOTION");

  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(4, 115);
  display.print("A");
  display.setCursor(4, 128);
  display.print("G");

  display.drawFastHLine(5, 141, 70, C_WHITE);

  // Footer hint
  display.setTextColor(TFT_YELLOW, C_BLACK);
  display.setCursor(4, 150);
  display.print("USR1 Sleep");
}

static void updateAwakeUi() {
  if (!g_lcdOk || !g_screenAwake) return;

  // STATE values (right column, x=46)
  printTextFixed(46, 54, C_GREEN,  "ON", 5);
  printTextFixed(46, 67, TFT_YELLOW, String((unsigned long)g_wakeCount), 5);

  char srcBuf[8];
  snprintf(srcBuf, sizeof(srcBuf), "0x%02X", g_lastWakeSrc);
  printTextFixed(46, 80, TFT_CYAN, srcBuf, 5);

  // MOTION values
  String acc = fmtAxis(g_ax) + " " + fmtAxis(g_ay) + " " + fmtAxis(g_az);
  printTextFixed(16, 115, C_WHITE, acc, 15);

  String gyr = fmtAxis(g_gx) + " " + fmtAxis(g_gy) + " " + fmtAxis(g_gz);
  printTextFixed(16, 128, C_WHITE, gyr, 15);
}

static void drawSleepHintThenOff() {
  if (!g_lcdOk) return;

  display.fillScreen(C_BLACK);

  display.setTextSize(2);
  display.setTextColor(TFT_CYAN, C_BLACK);
  display.setCursor(7, 55);
  display.print("Sleep");

  display.setTextSize(1);
  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(8, 85);
  display.print("System ON");

  display.setTextColor(TFT_YELLOW, C_BLACK);
  display.setCursor(8, 104);
  display.print("Move to wake");

  display.setTextColor(C_GRAY, C_BLACK);
  display.setCursor(8, 124);
  display.print("IMU INT = D14");
}

// ========================= nRF System ON sleep =========================

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
    Serial.print("[SLEEP] loops=");
    Serial.print((unsigned long)g_sleepLoopCount);
    Serial.print(" D14=");
    Serial.print(digitalRead(IMU_INT_PIN));
    Serial.print(" awake=");
    Serial.println(g_screenAwake ? "Y" : "N");
  }

  systemOnSleepOnce();
}

// ========================= Sleep / Wake =========================

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

  Serial.println("[SLEEP] screen off, entering System ON sleep");

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
  Wire.begin();

  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);

  int imuBegin = myIMU.begin();
  Serial.print("[IMU] LSM6DS3 begin=");
  Serial.println(imuBegin);

  bool ok = true;

  // Write config registers
  ok &= imuWriteReg(REG_CTRL3_C,     0x44);  // BDU=1, auto-increment
  ok &= imuWriteReg(REG_CTRL1_XL,    0x40);  // 104Hz, +/-2g
  ok &= imuWriteReg(REG_TAP_CFG,     0x80);  // enable embedded interrupts
  ok &= imuWriteReg(REG_WAKE_UP_THS, IMU_WAKE_THRESHOLD);
  ok &= imuWriteReg(REG_WAKE_UP_DUR, 0x00);
  ok &= imuWriteReg(REG_MD1_CFG,     0x20);  // route wake-up to INT1

  // Verify registers were written correctly
  uint8_t v_ctrl1_xl = 0, v_tap_cfg = 0, v_md1_cfg = 0, v_wake_ths = 0;
  imuReadReg(REG_CTRL1_XL,    v_ctrl1_xl);
  imuReadReg(REG_TAP_CFG,     v_tap_cfg);
  imuReadReg(REG_MD1_CFG,     v_md1_cfg);
  imuReadReg(REG_WAKE_UP_THS, v_wake_ths);

  Serial.print("[IMU] CTRL1_XL    = 0x"); Serial.println(v_ctrl1_xl, HEX);
  Serial.print("[IMU] TAP_CFG     = 0x"); Serial.println(v_tap_cfg, HEX);
  Serial.print("[IMU] MD1_CFG     = 0x"); Serial.println(v_md1_cfg, HEX);
  Serial.print("[IMU] WAKE_UP_THS = 0x"); Serial.println(v_wake_ths, HEX);

  uint8_t dummy = 0;
  imuReadReg(REG_WAKE_UP_SRC, dummy);

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuWakeIsr, RISING);

  g_imuOk = ok;

  Serial.print("[IMU] D14 wake interrupt ");
  Serial.println(ok ? "OK" : "FAILED");
  Serial.print("[IMU] D14 pin state = ");
  Serial.println(digitalRead(IMU_INT_PIN));
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
  if (g_imuWakeFlag) { g_imuWakeFlag = false; imuPending = true; }
  if (g_usrWakeFlag) { g_usrWakeFlag = false; usrPending = true; }
  interrupts();

  if (usrPending) { screenWake("USR2"); return; }

  if (digitalRead(IMU_INT_PIN) == HIGH) {
    imuPending = true;
    Serial.println("[WAKE] D14 pin HIGH (polled)");
  }
  if (!imuPending) return;

  uint8_t src = 0;
  if (imuReadReg(REG_WAKE_UP_SRC, src)) g_lastWakeSrc = src;
  Serial.print("[WAKE] src=0x"); Serial.println(src, HEX);

  if ((src & 0x08) || (src & 0x07) || !g_screenAwake) screenWake("IMU_D14");
}

// ========================= Setup =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
  pinMode(LCD_BL_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(USR2_PIN), usrWakeIsr, FALLING);

  Wire.begin();

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 0.96 IMU Wake Demo ===");

  initDisplay();
  initImuWakeInterrupt();

  g_lastActivityMs = millis();

  drawStaticAwakeUi();
  updateImuData();
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
    delay(BTN_DEBOUNCE_MS);
    if (digitalRead(USR1_PIN) == LOW) {
      while (digitalRead(USR1_PIN) == LOW) delay(5);
      screenSleep();
      return;
    }
  }

  // USR2: wake / reset counters
  if (digitalRead(USR2_PIN) == LOW) {
    delay(BTN_DEBOUNCE_MS);
    if (digitalRead(USR2_PIN) == LOW) {
      while (digitalRead(USR2_PIN) == LOW) delay(5);
      screenWake("USR2");
      g_lastActivityMs = millis();
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
