/*
  XIAO nRF52840 Plus + 1.47 Touch Display
  Factory Dashboard v1.5 - Charge Status Latch

  Based on user-provided:
    xiao_147_nrf52840plus_all_v0.2.ino

  Goals:
    - Rebuild UI to match the newer ESP32-S3 Plus visual direction.
    - Keep existing LCD + Touch + IMU + Mic + SD + USR1/USR2 logic.
    - Add nRF52840 Plus battery status:
        READ_BAT = P0.14 active-low sink enable
        VBAT ADC = PIN_VBAT
        CHG status = P0.17 / ~CHG, active-low charging indication

  Notes:
    - Current BSP pin workaround is preserved:
        LCD_RST_PIN = D19
        BTN_A_PIN   = D17
      Comments in v0.2 said schematic expected D17/D19, but BSP mapping is currently swapped.
    - PDM external mic still requires BSP PIN_PDM_CLK/PIN_PDM_DIN fixed to D0/D1,
      same as the previous verified nRF52840 Plus baseline.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SdFat.h>
#include <PDM.h>
#include <Arduino_GFX_Library.h>
#include "SparkFunLSM6DS3.h"
#include "axs5106l_device.h"
#include <nrf.h>
#include <nrf_gpio.h>
#include <stdarg.h>
#include <math.h>

// ========================= Pin map =========================

static constexpr uint8_t PDM_CLK_PIN   = D0;
static constexpr uint8_t PDM_DATA_PIN  = D1;
static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t I2C_SDA_PIN   = D4;
static constexpr uint8_t I2C_SCL_PIN   = D5;
static constexpr uint8_t SD_CS_PIN     = D6;
static constexpr uint8_t TOUCH_INT_PIN = D7;
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t IMU_INT_PIN   = D14; // LSM6DS3 INT1, used for double-tap wake/count.
static constexpr uint8_t LCD_MOSI_PIN  = D10;

// Keep the currently verified workaround from v0.2.
static constexpr uint8_t BTN_B_PIN     = D15;
static constexpr uint8_t LCD_RST_PIN   = D19; // Schematic expected D17; current BSP workaround.
static constexpr uint8_t LCD_BL_PIN    = D18;
static constexpr uint8_t BTN_A_PIN     = D17; // Schematic expected D19; current BSP workaround.

// nRF52840 Plus internal battery measurement pins from schematic.
static constexpr uint8_t READ_BAT_P0_PIN = 14; // P0.14 / ~READ_BAT, active-low sink enable.
static constexpr uint8_t CHG_P0_PIN      = 17; // P0.17 / ~CHG, active-low charging indication.

#ifndef PIN_VBAT
// In the tested Seeeduino nRF52 BSP, PIN_VBAT was printed as 35.
// Keep fallback so the file still compiles if the macro is missing.
#define PIN_VBAT 35
#endif

// ========================= Timing =========================

static constexpr uint32_t UI_TEXT_FAST_MS   = 130;
static constexpr uint32_t UI_TEXT_SLOW_MS   = 220;
static constexpr uint32_t UI_VU_MS          = 75;
static constexpr uint32_t UI_MIC_TEXT_MS    = 360;
static constexpr uint32_t SERIAL_MS         = 500;
static constexpr uint32_t SD_REFRESH_MS     = 1200;
static constexpr uint32_t BAT_REFRESH_MS    = 1000;
static constexpr uint32_t TAP_DEBOUNCE_MS   = 220;
// Button debounce:
 // - BTN_DEBOUNCE_MS filters contact bounce.
 // - BTN_ACTION_LOCKOUT_MS prevents accidental double action from one long/noisy press.
static constexpr uint32_t BTN_DEBOUNCE_MS = 35;
static constexpr uint32_t BTN_ACTION_LOCKOUT_MS = 150;

// Backlight control:
//   USR1 short press: cycle brightness levels: 100% -> 75% -> 50% -> 25% -> 0% -> 100%
//   USR2 short press: toggle screen off / restore previous brightness
static const uint8_t BL_LEVELS[] = {255, 191, 128, 64, 0};
static constexpr uint8_t BL_LEVEL_COUNT = sizeof(BL_LEVELS) / sizeof(BL_LEVELS[0]);
static constexpr uint32_t MIC_DECAY_MS      = 60;

// ========================= Mic =========================

static constexpr int MIC_SAMPLE_RATE_HZ = 16000;
static constexpr int MIC_CHANNELS       = 1;
static constexpr int MIC_GAIN           = 30;

// ========================= Buttons =========================

static constexpr int BTN_A_ACTIVE_LEVEL = LOW;
static constexpr int BTN_B_ACTIVE_LEVEL = LOW;

// ========================= Battery =========================
//
// XIAO nRF52840 Plus schematic:
//   R16 = 1M, R17 = 499K.
//   ratio = (1000K + 499K) / 499K = 3.004.
//
// We verified in diagnostics:
//   PIN_VBAT pin=35, VADC around 1.28V, VBAT around 3.85V.

static constexpr float BAT_DIVIDER_RATIO = (1000.0f + 499.0f) / 499.0f;
static constexpr float BAT_CAL_FACTOR    = 1.000f;
static constexpr int ADC_BITS            = 12;
static constexpr int ADC_MAX             = (1 << ADC_BITS) - 1;
static constexpr float ADC_FULL_SCALE_V  = 3.600f;

// A real Li-ion cell is a low-impedance source, so samples are usually stable.
// With no battery inserted but USB plugged in, the charger/VBAT node can float near
// 4V and still produce a fake battery voltage. Detect that case by sample spread.
static constexpr uint16_t BAT_PRESENT_MIN_RAW = 80;
static constexpr uint16_t BAT_FLOAT_RANGE_RAW = 80;

// Avoid high-impedance ADC noise causing visible NO BAT flicker.
// Once a real battery has been detected, noisy-but-plausible readings are held/filtered.
// Missing battery is declared only after many consecutive bad batches.
static constexpr uint8_t BAT_INVALID_CONFIRM_COUNT = 3;
static constexpr uint8_t BAT_NOISY_HOLD_CONFIRM_COUNT = 20;
static constexpr float BAT_VALID_MIN_V = 2.80f;
static constexpr float BAT_VALID_MAX_V = 4.60f;

// Charger status filter.
// ~CHG can flicker or release briefly around charge regulation/termination.
// Set charging immediately when a LOW is observed, but clear it only after
// repeated HIGH samples and a latch timeout.
static constexpr uint8_t CHG_SAMPLE_COUNT = 9;
static constexpr uint8_t CHG_HIGH_CLEAR_COUNT = 8;
static constexpr uint32_t CHG_LATCH_HOLD_MS = 30000;
static constexpr float CHG_FULL_CLEAR_V = 4.18f;

// ========================= Colors =========================

static constexpr uint16_t C_BLACK   = RGB565_BLACK;
static constexpr uint16_t C_WHITE   = RGB565_WHITE;
static constexpr uint16_t C_GREEN   = RGB565_LIGHTGREEN;
static constexpr uint16_t C_RED     = RGB565_RED;
static constexpr uint16_t C_CYAN    = RGB565_CYAN;
static constexpr uint16_t C_YELLOW  = RGB565_YELLOW;
static constexpr uint16_t C_GRAY    = 0x8410;
static constexpr uint16_t C_ORANGE  = 0xFD20;
static constexpr uint16_t C_DIM     = 0x2104;
static constexpr uint16_t C_PANEL   = 0x0841;
static constexpr uint16_t C_LINE    = 0x39E7;
static constexpr uint16_t C_BLUE    = RGB565_BLUE;

// ========================= LCD =========================
//
// Preserve the nRF-proven LCD path:
//   ST7789 + Arduino_SWSPI + 172x320 + offset 34,0,34,0 + MADCTL 0x48.

Arduino_DataBus *lcdBus = new Arduino_SWSPI(
  LCD_DC_PIN, LCD_CS_PIN, LCD_SCK_PIN, LCD_MOSI_PIN, GFX_NOT_DEFINED
);

Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus, LCD_RST_PIN, 0, false, 172, 320, 34, 0, 34, 0
);

// ========================= Devices =========================

SdFat SD;
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// ========================= Runtime state =========================

volatile uint16_t g_micPeak = 0;
volatile uint32_t g_micLastUpdateMs = 0;
int16_t g_pdmBuf[256];

bool g_sdMounted = false;
uint32_t g_sdOkFreq = 0;

float g_ax = 0, g_ay = 0, g_az = 0;
float g_gx = 0, g_gy = 0, g_gz = 0;

bool g_touchValid = false;
int g_touchX = -1;
int g_touchY = -1;

bool g_btnA = false;
bool g_btnB = false;
int g_btnARaw = HIGH;
int g_btnBRaw = HIGH;

struct BatteryState {
  uint16_t raw = 0;
  uint16_t rawMin = 0;
  uint16_t rawMax = 0;
  float vadc = 0.0f;
  float vbat = 0.0f;
  int percent = 0;
  bool charging = false;
  bool valid = false;
};
BatteryState g_bat;
BatteryState g_lastGoodBat;
bool g_haveLastGoodBat = false;
uint8_t g_batInvalidStreak = 0;
uint8_t g_batValidStreak = 0;
uint8_t g_batNoisyStreak = 0;
const char *g_batFilterState = "BOOT";

bool g_chgRawLow = false;
bool g_chgState = false;
uint8_t g_chgHighStreak = 0;
uint32_t g_lastChgLowMs = 0;

uint32_t g_lastFastUiMs = 0;
uint32_t g_lastSlowUiMs = 0;
uint32_t g_lastVuMs = 0;
uint32_t g_lastMicTextMs = 0;
uint32_t g_lastSerialMs = 0;
uint32_t g_lastSdMs = 0;
uint32_t g_lastBatMs = 0;

String cache_sd = "";
String cache_touch = "";
String cache_bat = "";
int cache_chargeIcon = -1;
String cache_acc = "";
String cache_gyr = "";
String cache_btn1 = "";
String cache_btn2 = "";
String cache_tap = "";
String cache_bl = "";
String cache_mic = "";
String cache_footer = "";

float g_vuSmooth = 0.0f;
int cache_vuSegments = -1;

volatile bool g_imuIntFlag = false;
uint32_t g_doubleTapCount = 0;
uint32_t g_lastTapMs = 0;

uint8_t g_blIndex = 0;          // 0 = 255, full brightness
uint8_t g_blRestoreIndex = 0;   // brightness restored after screen-off

bool g_btnALastRawPressed = false;
bool g_btnBLastRawPressed = false;
uint32_t g_btnALastChangeMs = 0;
uint32_t g_btnBLastChangeMs = 0;
bool g_btnAPressEvent = false;
bool g_btnBPressEvent = false;
uint32_t g_lastBtnActionMs = 0;

// ========================= UI layout =========================

static constexpr int CARD_X = 7;
static constexpr int CARD_W = 158;

// Header
static constexpr int Y_TITLE = 8;
static constexpr int Y_SUB1  = 32;

// Cards
static constexpr int Y_SYS  = 54;
static constexpr int H_SYS  = 66;

static constexpr int Y_MOTION = 125;
static constexpr int H_MOTION = 58;

static constexpr int Y_MIC  = 189;
static constexpr int H_MIC  = 62;

static constexpr int Y_BTN  = 258;
static constexpr int H_BTN  = 44;

static constexpr int Y_PRODUCT = 308;

// Dynamic text rows
static constexpr int ROW_SD     = Y_SYS + 23;
static constexpr int ROW_TOUCH  = Y_SYS + 38;
static constexpr int ROW_BAT    = Y_SYS + 53;

// Small charging icon on the BAT row.
// It is drawn as pixels/triangles instead of a Unicode character because
// the built-in GFX font does not reliably support the ⚡ glyph.
static constexpr int CHG_ICON_X = 148;
static constexpr int CHG_ICON_Y = ROW_BAT - 2;
static constexpr int CHG_ICON_W = 12;
static constexpr int CHG_ICON_H = 14;

static constexpr int ROW_ACC    = Y_MOTION + 23;
static constexpr int ROW_GYR    = Y_MOTION + 39;

static constexpr int ROW_MIC_RAW = Y_MIC + 42;

static constexpr int ROW_BTN1   = Y_BTN + 16;
static constexpr int ROW_BTN2   = Y_BTN + 16;
static constexpr int ROW_BL     = Y_BTN + 29;
static constexpr int ROW_TAP_MOTION = Y_MOTION + 8;

// VU meter
static constexpr int VU_X = CARD_X + 16;
static constexpr int VU_Y = Y_MIC + 27;
static constexpr int VU_W = 124;
static constexpr int VU_H = 14;
static constexpr int VU_SEG_COUNT = 12;
static constexpr int VU_GAP = 2;
static constexpr int VU_SEG_W = (VU_W - 4 - (VU_SEG_COUNT - 1) * VU_GAP) / VU_SEG_COUNT;

// ========================= Basic helpers =========================

static void logf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
}

static String padRight(const String &s, int width) {
  String out = s;
  while ((int)out.length() < width) out += ' ';
  if ((int)out.length() > width) out = out.substring(0, width);
  return out;
}

static uint16_t colorByPercent(int pct) {
  if (pct <= 15) return C_RED;
  if (pct <= 35) return C_YELLOW;
  return C_GREEN;
}

static int lipoPercent(float v) {
  struct Point { float v; int p; };
  static const Point table[] = {
    {4.20f, 100},
    {4.10f, 90},
    {4.00f, 80},
    {3.92f, 70},
    {3.85f, 60},
    {3.79f, 50},
    {3.72f, 40},
    {3.66f, 30},
    {3.58f, 20},
    {3.50f, 10},
    {3.30f, 0}
  };

  if (v >= table[0].v) return 100;
  if (v <= table[10].v) return 0;

  for (int i = 0; i < 10; i++) {
    if (v <= table[i].v && v >= table[i + 1].v) {
      float t = (v - table[i + 1].v) / (table[i].v - table[i + 1].v);
      return table[i + 1].p + (int)roundf(t * (table[i].p - table[i + 1].p));
    }
  }

  return 0;
}

// ========================= Shared SPI helpers =========================

static void acquireForLcd() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void acquireForSd() {
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

// ========================= LCD low-level =========================

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

static bool initLcd() {
  applyBacklight();
  lcdHardReset();

  if (!gfx->begin()) {
    Serial.println("[LCD] gfx->begin() failed");
    return false;
  }

  lcdWriteMadctlFix();
  gfx->fillScreen(C_BLACK);
  return true;
}

static void printTextFixed(int x, int y, uint16_t color, const String &text, int widthChars) {
  acquireForLcd();
  gfx->setCursor(x, y);
  gfx->setTextSize(1);
  gfx->setTextColor(color, C_BLACK);
  gfx->print(padRight(text, widthChars));
}

// ========================= Backlight =========================

static uint8_t currentBacklightPwm() {
  return BL_LEVELS[g_blIndex];
}

static int currentBacklightPercent() {
  return (int)roundf((float)currentBacklightPwm() * 100.0f / 255.0f);
}

static void applyBacklight() {
  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, currentBacklightPwm());
}

static void cycleBacklightLevel() {
  g_blIndex = (g_blIndex + 1) % BL_LEVEL_COUNT;
  if (currentBacklightPwm() > 0) {
    g_blRestoreIndex = g_blIndex;
  }
  applyBacklight();
}

static void toggleBacklightOffRestore() {
  if (currentBacklightPwm() == 0) {
    if (BL_LEVELS[g_blRestoreIndex] == 0) g_blRestoreIndex = 0;
    g_blIndex = g_blRestoreIndex;
  } else {
    g_blRestoreIndex = g_blIndex;
    g_blIndex = BL_LEVEL_COUNT - 1; // last level is 0
  }
  applyBacklight();
}

static void handleBacklightButtons() {
  bool doA = false;
  bool doB = false;

  // Consume press events generated by updateButtons().
  if (g_btnAPressEvent) {
    g_btnAPressEvent = false;
    doA = true;
  }

  if (g_btnBPressEvent) {
    g_btnBPressEvent = false;
    doB = true;
  }

  if (!doA && !doB) return;

  uint32_t now = millis();
  if (now - g_lastBtnActionMs < BTN_ACTION_LOCKOUT_MS) return;
  g_lastBtnActionMs = now;

  // If both somehow arrive together, prefer USR2 power toggle.
  if (doB) {
    toggleBacklightOffRestore();
    Serial.print("[BL] USR2 toggle, pwm=");
    Serial.print(currentBacklightPwm());
    Serial.print(" pct=");
    Serial.println(currentBacklightPercent());
    return;
  }

  if (doA) {
    cycleBacklightLevel();
    Serial.print("[BL] USR1 cycle, pwm=");
    Serial.print(currentBacklightPwm());
    Serial.print(" pct=");
    Serial.println(currentBacklightPercent());
  }
}

// ========================= UI drawing =========================

static void drawCard(int x, int y, int w, int h, uint16_t accent, const char *title) {
  acquireForLcd();
  gfx->drawRoundRect(x, y, w, h, 6, accent);
  gfx->drawRoundRect(x + 1, y + 1, w - 2, h - 2, 6, C_LINE);
  gfx->fillRect(x + 5, y + 12, 4, h - 24, accent);
  gfx->setTextSize(1);
  gfx->setTextColor(accent, C_BLACK);
  gfx->setCursor(x + 14, y + 8);
  gfx->print(title);
}

static void drawVuFrame() {
  acquireForLcd();
  gfx->drawRoundRect(VU_X, VU_Y, VU_W, VU_H, 3, C_GREEN);
  for (int i = 0; i < VU_SEG_COUNT; ++i) {
    int x = VU_X + 2 + i * (VU_SEG_W + VU_GAP);
    gfx->fillRect(x, VU_Y + 2, VU_SEG_W, VU_H - 4, C_BLACK);
  }
}

static void drawChargeIcon(bool charging) {
  acquireForLcd();

  // Clear icon area first. This also removes the icon when charging stops.
  gfx->fillRect(CHG_ICON_X, CHG_ICON_Y, CHG_ICON_W, CHG_ICON_H, C_BLACK);

  if (!charging) return;

  // Lightning symbol, compact 12x14 px.
  // Shape:
  //    /|
  //   / |
  //  /__|
  //     /
  //    /
  const int x = CHG_ICON_X;
  const int y = CHG_ICON_Y;

  gfx->fillTriangle(x + 6, y + 0,  x + 1, y + 7,  x + 6, y + 7,  C_YELLOW);
  gfx->fillTriangle(x + 5, y + 6,  x + 11, y + 6, x + 4, y + 13, C_YELLOW);

  // Thin orange edge makes it visible on bright blue/cyan BAT text.
  gfx->drawLine(x + 6, y + 0, x + 1, y + 7, C_ORANGE);
  gfx->drawLine(x + 1, y + 7, x + 6, y + 7, C_ORANGE);
  gfx->drawLine(x + 11, y + 6, x + 4, y + 13, C_ORANGE);
}

static void drawStaticLayout() {
  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  gfx->setCursor(8, Y_TITLE);
  gfx->setTextSize(2);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->println("Hello,XIAO!");

  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(10, Y_SUB1);
  gfx->print("1.47 Inch Touch Display");

  gfx->drawFastHLine(8, 49, 156, C_LINE);

  drawCard(CARD_X, Y_SYS, CARD_W, H_SYS, C_CYAN, "SYSTEM");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(20, ROW_SD);    gfx->print("SD");
  gfx->setCursor(20, ROW_TOUCH); gfx->print("Touch");
  gfx->setCursor(20, ROW_BAT);   gfx->print("BAT");
  drawChargeIcon(false);

  drawCard(CARD_X, Y_MOTION, CARD_W, H_MOTION, C_YELLOW, "MOTION");
  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(110, ROW_TAP_MOTION); gfx->print("Tap 0");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(20, ROW_ACC); gfx->print("Acc");
  gfx->setCursor(20, ROW_GYR); gfx->print("Gyr");

  drawCard(CARD_X, Y_MIC, CARD_W, H_MIC, C_GREEN, "MIC LEVEL");
  drawVuFrame();

  drawCard(CARD_X, Y_BTN, CARD_W, H_BTN, C_BLUE, "BACKLIGHT / BUTTON");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(20, ROW_BTN1); gfx->print("BL");

  gfx->setTextSize(1);
  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(31, Y_PRODUCT);
  gfx->print("XIAO nRF52840 Plus");

  cache_sd = "";
  cache_touch = "";
  cache_bat = "";
  cache_chargeIcon = -1;
  cache_acc = "";
  cache_gyr = "";
  cache_btn1 = "";
  cache_btn2 = "";
  cache_tap = "";
  cache_bl = "";
  cache_mic = "";
  cache_footer = "";
  cache_vuSegments = -1;
}

// ========================= Touch + IMU =========================

static bool initTouchAndImu() {
  Wire.begin();
  touch_init(&Wire, LCD_RST_PIN, TOUCH_INT_PIN);
  myIMU.begin();
  return true;
}

static void updateTouch() {
  touch_data_t touch_data;
  if (get_touch_data(&touch_data)) {
    g_touchValid = true;
    g_touchX = touch_data.coords[0].x;
    g_touchY = touch_data.coords[0].y;
  } else {
    g_touchValid = false;
    g_touchX = -1;
    g_touchY = -1;
  }
}

static void updateImu() {
  g_ax = myIMU.readFloatAccelX();
  g_ay = myIMU.readFloatAccelY();
  g_az = myIMU.readFloatAccelZ();
  g_gx = myIMU.readFloatGyroX();
  g_gy = myIMU.readFloatGyroY();
  g_gz = myIMU.readFloatGyroZ();
}

// ========================= IMU double-tap interrupt =========================
//
// LSM6DS3 key registers used here:
//   TAP_SRC       0x1C: read to clear tap event; bit4 is DOUBLE_TAP on LSM6DS3 family.
//   TAP_CFG       0x58: enable tap recognition on X/Y/Z and embedded interrupts.
//   TAP_THS_6D    0x59: tap threshold.
//   INT_DUR2      0x5A: shock/quiet/duration timing.
//   WAKE_UP_THS   0x5B: bit7 enables single/double-tap mode.
//   MD1_CFG       0x5E: route double-tap event to INT1.
//
// If sensitivity is too high/low, tune TAP_THS_6D first.

static constexpr uint8_t LSM6DS3_ADDR        = 0x6A;
static constexpr uint8_t REG_TAP_SRC        = 0x1C;
static constexpr uint8_t REG_CTRL1_XL       = 0x10;
static constexpr uint8_t REG_TAP_CFG        = 0x58;
static constexpr uint8_t REG_TAP_THS_6D     = 0x59;
static constexpr uint8_t REG_INT_DUR2       = 0x5A;
static constexpr uint8_t REG_WAKE_UP_THS    = 0x5B;
static constexpr uint8_t REG_MD1_CFG        = 0x5E;

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

void imuIntIsr() {
  g_imuIntFlag = true;
}

static bool initImuDoubleTap() {
  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);

  // Accelerometer on, 416Hz, +/-2g. SparkFun begin() also configures the IMU,
  // but set it again here so tap detection has a known ODR.
  bool ok = true;
  ok &= imuWriteReg(REG_CTRL1_XL, 0x60);

  // Enable embedded interrupt recognition + tap on X/Y/Z.
  // 0x8E is a common LSM6DS3 tap config: interrupts enabled, X/Y/Z tap axes enabled.
  ok &= imuWriteReg(REG_TAP_CFG, 0x8E);

  // Threshold and timing. These are deliberately medium values for a small handheld board.
  // Lower TAP_THS_6D if double taps are hard to trigger; raise it if false triggers occur.
  ok &= imuWriteReg(REG_TAP_THS_6D, 0x0C);
  ok &= imuWriteReg(REG_INT_DUR2, 0x7F);

  // Enable single/double-tap recognition mode.
  ok &= imuWriteReg(REG_WAKE_UP_THS, 0x80);

  // Route double-tap event to INT1.
  ok &= imuWriteReg(REG_MD1_CFG, 0x08);

  uint8_t dummy = 0;
  imuReadReg(REG_TAP_SRC, dummy); // clear stale event.

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIntIsr, RISING);

  Serial.print("[IMU] double-tap INT1 on D14 init ");
  Serial.println(ok ? "OK" : "FAILED");

  return ok;
}

static void handleImuTapEvent() {
  bool shouldCheck = false;

  noInterrupts();
  if (g_imuIntFlag) {
    g_imuIntFlag = false;
    shouldCheck = true;
  }
  interrupts();

  // Polling fallback: if the interrupt edge was missed, read source while INT is high.
  if (digitalRead(IMU_INT_PIN) == HIGH) {
    shouldCheck = true;
  }

  if (!shouldCheck) return;

  uint8_t src = 0;
  if (!imuReadReg(REG_TAP_SRC, src)) return;

  // TAP_SRC bit4 = double tap. Count one event for one double-tap gesture.
  if (src & 0x10) {
    uint32_t now = millis();
    if (now - g_lastTapMs > TAP_DEBOUNCE_MS) {
      g_lastTapMs = now;
      g_doubleTapCount++;
      Serial.print("[TAP] double tap count=");
      Serial.print(g_doubleTapCount);
      Serial.print(" src=0x");
      Serial.println(src, HEX);
    }
  }
}

// ========================= Mic =========================

static bool initMic() {
  PDM.onReceive([]() {
    int bytesAvailable = PDM.available();
    if (bytesAvailable <= 0) return;
    if (bytesAvailable > (int)sizeof(g_pdmBuf)) bytesAvailable = sizeof(g_pdmBuf);
    int bytesRead = PDM.read((void *)g_pdmBuf, bytesAvailable);
    if (bytesRead <= 0) return;

    uint16_t peak = 0;
    int samples = bytesRead / 2;
    for (int i = 0; i < samples; ++i) {
      int32_t a = abs((int32_t)g_pdmBuf[i]);
      if (a > peak) peak = (uint16_t)((a > 65535) ? 65535 : a);
    }

    g_micPeak = peak;
    g_micLastUpdateMs = millis();
  });

  PDM.setBufferSize(sizeof(g_pdmBuf));
  PDM.setGain(MIC_GAIN);

  if (!PDM.begin(MIC_CHANNELS, MIC_SAMPLE_RATE_HZ)) {
    Serial.println("[MIC] PDM.begin failed");
    return false;
  }

  return true;
}

static uint16_t currentMicPeak() {
  uint32_t now = millis();
  if (now - g_micLastUpdateMs > MIC_DECAY_MS) {
    g_micPeak = (uint16_t)(g_micPeak * 0.75f);
  }
  return g_micPeak;
}

static uint16_t vuColorForIndex(int idx) {
  if (idx >= 10) return C_RED;
  if (idx >= 7) return C_ORANGE;
  return C_GREEN;
}

static void updateVuMeter() {
  uint16_t micPeak = currentMicPeak();

  float target = (float)micPeak / 2200.0f;
  if (target < 0.0f) target = 0.0f;
  if (target > 1.0f) target = 1.0f;

  if (target > g_vuSmooth) g_vuSmooth = g_vuSmooth * 0.55f + target * 0.45f;
  else g_vuSmooth = g_vuSmooth * 0.86f + target * 0.14f;

  int activeSegs = (int)roundf(g_vuSmooth * VU_SEG_COUNT);
  if (activeSegs < 0) activeSegs = 0;
  if (activeSegs > VU_SEG_COUNT) activeSegs = VU_SEG_COUNT;

  if (activeSegs == cache_vuSegments) return;

  acquireForLcd();
  for (int i = 0; i < VU_SEG_COUNT; ++i) {
    int x = VU_X + 2 + i * (VU_SEG_W + VU_GAP);
    uint16_t color = (i < activeSegs) ? vuColorForIndex(i) : C_BLACK;
    gfx->fillRect(x, VU_Y + 2, VU_SEG_W, VU_H - 4, color);
  }

  cache_vuSegments = activeSegs;
}

static void updateMicText() {
  uint16_t micPeak = currentMicPeak();
  String micText = String("Raw ") + String((unsigned)micPeak);

  if (micText != cache_mic) {
    cache_mic = micText;
    printTextFixed(20, ROW_MIC_RAW, C_WHITE, micText, 17);
  }
}

// ========================= SD =========================

static bool probeSdMount(uint32_t &okFreq) {
  const uint32_t freqs[] = {400000, 1000000, 4000000, 8000000};

  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
    acquireForSd();
    SPI.begin();
    delay(5);

    SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, freqs[i], &SPI);
    if (SD.begin(cfg)) {
      okFreq = freqs[i];
      return true;
    }
  }

  return false;
}

static void updateSdStatus() {
  uint32_t freq = 0;
  g_sdMounted = probeSdMount(freq);
  if (g_sdMounted) g_sdOkFreq = freq;
}

// ========================= Buttons =========================

static void updateButtons() {
  uint32_t now = millis();

  g_btnARaw = digitalRead(BTN_A_PIN);
  g_btnBRaw = digitalRead(BTN_B_PIN);

  bool rawAPressed = (g_btnARaw == BTN_A_ACTIVE_LEVEL);
  bool rawBPressed = (g_btnBRaw == BTN_B_ACTIVE_LEVEL);

  if (rawAPressed != g_btnALastRawPressed) {
    g_btnALastRawPressed = rawAPressed;
    g_btnALastChangeMs = now;
  }

  if (rawBPressed != g_btnBLastRawPressed) {
    g_btnBLastRawPressed = rawBPressed;
    g_btnBLastChangeMs = now;
  }

  if ((now - g_btnALastChangeMs) >= BTN_DEBOUNCE_MS && rawAPressed != g_btnA) {
    bool old = g_btnA;
    g_btnA = rawAPressed;
    if (g_btnA && !old) {
      g_btnAPressEvent = true;
    }
  }

  if ((now - g_btnBLastChangeMs) >= BTN_DEBOUNCE_MS && rawBPressed != g_btnB) {
    bool old = g_btnB;
    g_btnB = rawBPressed;
    if (g_btnB && !old) {
      g_btnBPressEvent = true;
    }
  }
}

// ========================= Battery =========================

static void enableBatteryDivider() {
  // READ_BAT is P0.14. Enable by sinking it to GND.
  NRF_P0->OUTCLR = (1UL << READ_BAT_P0_PIN);
  NRF_P0->DIRSET = (1UL << READ_BAT_P0_PIN);
}

static void disableBatteryDivider() {
  // High impedance = divider off, reduce leakage.
  NRF_P0->DIRCLR = (1UL << READ_BAT_P0_PIN);
}

static bool sampleChargingRawLow() {
  uint8_t lowCount = 0;

  for (uint8_t i = 0; i < CHG_SAMPLE_COUNT; i++) {
    if ((NRF_P0->IN & (1UL << CHG_P0_PIN)) == 0) {
      lowCount++;
    }
    delayMicroseconds(400);
  }

  // Majority vote. A single noisy low should not trigger charging.
  g_chgRawLow = (lowCount >= ((CHG_SAMPLE_COUNT / 2) + 1));
  return g_chgRawLow;
}

static bool updateChargingState(bool batValid, float vbat) {
  uint32_t now = millis();
  bool rawLow = sampleChargingRawLow();

  if (!batValid) {
    g_chgState = false;
    g_chgHighStreak = 0;
    return false;
  }

  if (rawLow) {
    g_chgState = true;
    g_lastChgLowMs = now;
    g_chgHighStreak = 0;
    return true;
  }

  if (g_chgHighStreak < 255) g_chgHighStreak++;

  // Do not clear CHG immediately. Many charger STAT pins blink/release briefly
  // during regulation. Keep the lightning icon stable for product UI.
  bool latchExpired = (now - g_lastChgLowMs) > CHG_LATCH_HOLD_MS;
  bool nearFull = vbat >= CHG_FULL_CLEAR_V;

  if (g_chgState && g_chgHighStreak >= CHG_HIGH_CLEAR_COUNT && (latchExpired || nearFull)) {
    g_chgState = false;
  }

  return g_chgState;
}

static uint16_t readBatteryRawAvg(uint8_t samples, uint16_t &rawMin, uint16_t &rawMax) {
  if (samples > 32) samples = 32;

  uint16_t buf[32];

  for (uint8_t i = 0; i < 6; i++) {
    (void)analogRead(PIN_VBAT);
    delay(2);
  }

  for (uint8_t i = 0; i < samples; i++) {
    buf[i] = analogRead(PIN_VBAT);
    delay(2);
  }

  // Sort small buffer and use a trimmed average to reject charger/floating spikes.
  for (uint8_t i = 0; i < samples; i++) {
    for (uint8_t j = i + 1; j < samples; j++) {
      if (buf[j] < buf[i]) {
        uint16_t t = buf[i];
        buf[i] = buf[j];
        buf[j] = t;
      }
    }
  }

  uint8_t trim = samples >= 16 ? 4 : 1;
  uint32_t sum = 0;
  uint8_t count = 0;

  // Report a robust range, not the absolute min/max, so one spike does not
  // make a real battery look like NO BAT.
  rawMin = buf[trim];
  rawMax = buf[samples - 1 - trim];

  for (uint8_t i = trim; i < samples - trim; i++) {
    sum += buf[i];
    count++;
  }

  return count ? (uint16_t)(sum / count) : buf[samples / 2];
}

static void updateBattery() {
  BatteryState measured;

  enableBatteryDivider();
  delay(30);

  measured.raw = readBatteryRawAvg(28, measured.rawMin, measured.rawMax);
  measured.vadc = ((float)measured.raw * ADC_FULL_SCALE_V) / (float)ADC_MAX;
  measured.vbat = measured.vadc * BAT_DIVIDER_RATIO * BAT_CAL_FACTOR;
  measured.percent = lipoPercent(measured.vbat);

  uint16_t spread = measured.rawMax - measured.rawMin;
  bool voltagePlausible = (measured.raw > BAT_PRESENT_MIN_RAW &&
                           measured.vbat > BAT_VALID_MIN_V &&
                           measured.vbat < BAT_VALID_MAX_V);
  bool stableBatch = voltagePlausible && (spread <= BAT_FLOAT_RANGE_RAW);

  measured.valid = stableBatch;
  measured.charging = stableBatch && updateChargingState(stableBatch, measured.vbat);

  disableBatteryDivider();

  if (stableBatch) {
    g_batValidStreak++;
    g_batInvalidStreak = 0;
    g_batNoisyStreak = 0;
    g_batFilterState = "STABLE";

    g_bat = measured;
    g_lastGoodBat = measured;
    g_haveLastGoodBat = true;
    return;
  }

  // Key change:
  // A real battery can occasionally produce a large spread because PIN_VBAT is a
  // high-impedance 1M/499K ADC node and the dashboard is also driving LCD/SD/PDM/IMU.
  // If the average voltage is still plausible and we already saw a stable battery
  // before, do NOT show NO BAT immediately. Keep the value valid and slowly filter it.
  if (voltagePlausible && g_haveLastGoodBat) {
    if (g_batNoisyStreak < 255) g_batNoisyStreak++;
    g_batInvalidStreak = 0;
    g_batValidStreak = 0;

    if (g_batNoisyStreak < BAT_NOISY_HOLD_CONFIRM_COUNT) {
      BatteryState filtered = measured;
      filtered.valid = true;
      filtered.charging = updateChargingState(true, filtered.vbat);

      // Low-pass only the displayed voltage/percent. Keep raw/range from this batch
      // for serial diagnosis.
      filtered.vbat = g_lastGoodBat.vbat * 0.75f + measured.vbat * 0.25f;
      filtered.vadc = filtered.vbat / (BAT_DIVIDER_RATIO * BAT_CAL_FACTOR);
      filtered.percent = lipoPercent(filtered.vbat);

      g_bat = filtered;
      g_lastGoodBat = filtered;
      g_haveLastGoodBat = true;
      g_batFilterState = "NOISY";
      return;
    }

    // If it stays noisy for a long time, treat it as missing/floating.
    g_bat = measured;
    g_bat.valid = false;
    g_bat.charging = false;
    g_batFilterState = "FLOAT";
    return;
  }

  g_batValidStreak = 0;
  g_batNoisyStreak = 0;
  if (g_batInvalidStreak < 255) g_batInvalidStreak++;

  if (g_haveLastGoodBat && g_batInvalidStreak < BAT_INVALID_CONFIRM_COUNT) {
    g_bat = g_lastGoodBat;
    g_bat.valid = true;
    g_bat.charging = updateChargingState(true, g_bat.vbat);
    g_batFilterState = "HOLD";
    return;
  }

  g_bat = measured;
  g_bat.valid = false;
  g_bat.charging = false;
  g_batFilterState = "MISS";
}

static String batteryShortText() {
  if (!g_bat.valid) return "NO BAT";

  char buf[32];
  snprintf(buf, sizeof(buf), "%.2fV %d%%", g_bat.vbat, g_bat.percent);
  return String(buf);
}

// ========================= UI update =========================

static void updateUiFast() {
  String sdText = g_sdMounted ? (String("OK ") + (g_sdOkFreq / 1000) + "k") : "Unplugged";
  if (sdText != cache_sd) {
    cache_sd = sdText;
    printTextFixed(62, ROW_SD, g_sdMounted ? C_GREEN : C_RED, sdText, 15);
  }

  String touchText = g_touchValid ? (String("(") + g_touchX + "," + g_touchY + ")") : String("release");
  if (touchText != cache_touch) {
    cache_touch = touchText;
    printTextFixed(62, ROW_TOUCH, C_CYAN, touchText, 15);
  }

  String batText = batteryShortText();
  if (batText != cache_bat) {
    cache_bat = batText;
    uint16_t c = g_bat.valid ? colorByPercent(g_bat.percent) : C_RED;
    if (g_bat.charging) c = C_CYAN;
    printTextFixed(62, ROW_BAT, c, batText, 13);
  }

  int chargeIconState = (g_bat.valid && g_bat.charging) ? 1 : 0;
  if (chargeIconState != cache_chargeIcon) {
    cache_chargeIcon = chargeIconState;
    drawChargeIcon(chargeIconState == 1);
  }

  String blText = String(currentBacklightPercent()) + "%";
  if (blText != cache_bl) {
    cache_bl = blText;
    uint16_t c = currentBacklightPwm() == 0 ? C_RED : C_CYAN;
    printTextFixed(48, ROW_BTN1, c, blText, 7);
  }

  String tapText = String("Tap ") + String(g_doubleTapCount);
  if (tapText != cache_tap) {
    cache_tap = tapText;
    printTextFixed(110, ROW_TAP_MOTION, C_YELLOW, tapText, 7);
  }

}

static void updateUiSlow() {
  char buf[64];

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", g_ax, g_ay, g_az);
  String accText(buf);
  if (accText != cache_acc) {
    cache_acc = accText;
    printTextFixed(50, ROW_ACC, C_WHITE, accText, 16);
  }

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", g_gx, g_gy, g_gz);
  String gyrText(buf);
  if (gyrText != cache_gyr) {
    cache_gyr = gyrText;
    printTextFixed(50, ROW_GYR, C_WHITE, gyrText, 16);
  }
}

// ========================= Serial =========================

static void printSerialStatus() {
  logf("[DASH] sd=%s freq=%lu touch=%s x=%d y=%d bat=%.3fV pct=%d valid=%s chg=%s "
       "raw=%u range=%u-%u spread=%u inv=%u noisy=%u filt=%s tap=%lu bl=%u/%d%% acc=(%.2f,%.2f,%.2f) gyr=(%.2f,%.2f,%.2f) "
       "mic=%u usr1=%s raw1=%d usr2=%s raw2=%d\n",
       g_sdMounted ? "Inserted" : "Unplugged",
       (unsigned long)g_sdOkFreq,
       g_touchValid ? "Y" : "N",
       g_touchX, g_touchY,
       g_bat.vbat, g_bat.percent, g_bat.valid ? "Y" : "N", g_bat.charging ? "Y" : "N",
       g_chgRawLow ? "LOW" : "HIGH", (unsigned)g_chgHighStreak,
       g_bat.raw, g_bat.rawMin, g_bat.rawMax, (unsigned)(g_bat.rawMax - g_bat.rawMin),
       (unsigned)g_batInvalidStreak,
       (unsigned)g_batNoisyStreak,
       g_batFilterState,
       (unsigned long)g_doubleTapCount,
       (unsigned)currentBacklightPwm(), currentBacklightPercent(),
       g_ax, g_ay, g_az,
       g_gx, g_gy, g_gz,
       (unsigned)currentMicPeak(),
       g_btnA ? "Pressed" : "Released", g_btnARaw,
       g_btnB ? "Pressed" : "Released", g_btnBRaw);
}

// ========================= Setup / loop =========================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BTN_A_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);
  g_btnARaw = digitalRead(BTN_A_PIN);
  g_btnBRaw = digitalRead(BTN_B_PIN);
  g_btnALastRawPressed = (g_btnARaw == BTN_A_ACTIVE_LEVEL);
  g_btnBLastRawPressed = (g_btnBRaw == BTN_B_ACTIVE_LEVEL);
  g_btnA = g_btnALastRawPressed;
  g_btnB = g_btnBLastRawPressed;
  g_btnALastChangeMs = millis();
  g_btnBLastChangeMs = millis();
  g_btnAPressEvent = false;
  g_btnBPressEvent = false;

  // P0.17 / ~CHG is an open-drain style status line on the charger side.
  // Enable internal pull-up to avoid floating-low false charging reports.
  nrf_gpio_cfg_input(CHG_P0_PIN, NRF_GPIO_PIN_PULLUP);
  sampleChargingRawLow();
  if (g_chgRawLow) {
    g_chgState = true;
    g_lastChgLowMs = millis();
  }

  analogReadResolution(ADC_BITS);
  disableBatteryDivider();

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 1.47 Factory Dashboard v1.5 ===");
  Serial.println("UI + battery latch filter + charge status latch + tap counter + backlight UI");
  Serial.print("PIN_VBAT = "); Serial.println(PIN_VBAT);
#ifdef PIN_PDM_CLK
  Serial.print("PIN_PDM_CLK = "); Serial.println(PIN_PDM_CLK);
#endif
#ifdef PIN_PDM_DIN
  Serial.print("PIN_PDM_DIN = "); Serial.println(PIN_PDM_DIN);
#endif

  initTouchAndImu();
  initImuDoubleTap();

  if (!initLcd()) {
    Serial.println("[FAIL] LCD init failed");
    return;
  }

  if (!initMic()) {
    Serial.println("[WARN] MIC init failed, VU may stay zero");
  }

  updateSdStatus();
  updateTouch();
  updateImu();
  updateButtons();
  updateBattery();

  drawStaticLayout();
  updateUiFast();
  updateUiSlow();
  updateVuMeter();
  updateMicText();
  printSerialStatus();
}

void loop() {
  uint32_t now = millis();

  handleImuTapEvent();
  updateButtons();
  handleBacklightButtons();

  if (now - g_lastSdMs >= SD_REFRESH_MS) {
    g_lastSdMs = now;
    updateSdStatus();
  }

  if (now - g_lastBatMs >= BAT_REFRESH_MS) {
    g_lastBatMs = now;
    updateBattery();
  }

  if (now - g_lastSlowUiMs >= UI_TEXT_SLOW_MS) {
    g_lastSlowUiMs = now;
    updateImu();
    updateTouch();
    updateUiSlow();
  }

  if (now - g_lastFastUiMs >= UI_TEXT_FAST_MS) {
    g_lastFastUiMs = now;
    updateUiFast();
  }

  if (now - g_lastVuMs >= UI_VU_MS) {
    g_lastVuMs = now;
    updateVuMeter();
  }

  if (now - g_lastMicTextMs >= UI_MIC_TEXT_MS) {
    g_lastMicTextMs = now;
    updateMicText();
  }

  if (now - g_lastSerialMs >= SERIAL_MS) {
    g_lastSerialMs = now;
    printSerialStatus();
  }
}
