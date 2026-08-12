/*
  XIAO nRF52840 Plus + 1.14 Inch Display
  Factory Dashboard v1.0.8 - timeout-safe I2C hot-plug + battery stable UI

  Based on the verified 1.47 nRF52840 Plus Dashboard style, adapted for the 1.14" board.

  1.14 board differences from 1.47:
    - LCD: 1.14" IPS ST7789, 135x240
    - No touch
    - No SD slot
    - Adds Grove I2C connector
    - 3 user buttons: USR1 / USR2 / USR3

  PRD / corrected pin map:
    D0  = MIC_CLK
    D1  = MIC_DATA
    D2  = LCD_CS
    D3  = LCD_DC
    D4  = SDA
    D5  = SCL
    D6  = BTN_A / USR1
    D7  = BTN_B / USR2
    D8  = LCD_SCK
    D9  = NC
    D10 = LCD_MOSI
    D11 = I2S_SD test pad
    D12 = I2S_SCK test pad
    D13 = I2S_WS test pad
    D14 = IMU_INT
    D15 = NC
    D16 = BAT_ADC / reserved battery sense in PRD
    D17 = LCD_RST   // D17/D19 issue has been fixed; use the corrected mapping
    D18 = LCD_BL
    D19 = BTN_C / USR3

  Covered modules:
    - LCD display + backlight PWM
    - PDM digital microphone, with Sketch-layer PDM.setPins(D1, D0, -1)
    - LSM6DS3-compatible IMU, 6-axis read + double-tap counter on D14
    - 3 user buttons
    - Grove I2C scan
    - nRF52840 Plus battery voltage + charging status using the proven XIAO path:
        READ_BAT = P0.14 active-low divider enable
        VBAT ADC = PIN_VBAT
        CHG      = P0.17 / active-low charging status

  Required Arduino libraries:
    - Arduino_GFX_Library
    - SparkFun LSM6DS3
*/

#include <Arduino.h>
#include <Wire.h>
#include <PDM.h>
#include <Arduino_GFX_Library.h>
#include "SparkFunLSM6DS3.h"
#include <nrf.h>
#include <nrf_gpio.h>
#include <math.h>

// ========================= Pins =========================

static constexpr uint8_t PDM_CLK_PIN   = D0;
static constexpr uint8_t PDM_DATA_PIN  = D1;
static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t I2C_SDA_PIN   = D4;
static constexpr uint8_t I2C_SCL_PIN   = D5;
static constexpr uint8_t BTN_A_PIN     = D6;   // USR1
static constexpr uint8_t BTN_B_PIN     = D7;   // USR2
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t IMU_INT_PIN   = D14;
static constexpr uint8_t LCD_RST_PIN   = D17;  // corrected mapping
static constexpr uint8_t LCD_BL_PIN    = D18;
static constexpr uint8_t BTN_C_PIN     = D19;  // USR3

// nRF52840 Plus internal battery measurement path.
static constexpr uint8_t READ_BAT_P0_PIN = 14; // P0.14 / ~READ_BAT, active-low sink enable.
static constexpr uint8_t CHG_P0_PIN      = 17; // P0.17 / ~CHG, active-low charging indication.

#ifndef PIN_VBAT
#define PIN_VBAT 35
#endif

// ========================= Timing =========================

static constexpr uint32_t UI_FAST_MS      = 120;
static constexpr uint32_t UI_SLOW_MS      = 320;
static constexpr uint32_t UI_VU_MS        = 65;
static constexpr uint32_t SERIAL_MS       = 650;
static constexpr uint32_t I2C_SCAN_MS     = 3000;  // safer periodic scan; avoids stressing Grove hot-plug
static constexpr uint32_t I2C_CHANGE_FLASH_MS = 700;  // briefly highlight a confirmed address-list change
static constexpr uint32_t BAT_REFRESH_MS  = 500;
static constexpr uint32_t BTN_DEBOUNCE_MS = 35;
static constexpr uint32_t BTN_ACTION_LOCKOUT_MS = 150;
static constexpr uint32_t TAP_DEBOUNCE_MS = 220;

// Backlight control:
//   USR1 short press: 100% -> 75% -> 50% -> 25% -> 100%
//   USR2 short press: quick off / restore last brightness
//   USR3 short press: toggle header text Hello,XIAO! <-> Seeed Studio
//
// Note on the 1.14 hardware behavior:
// this panel's backlight path behaves much more like a BL enable / coarse PWM path
// than a smooth analog dimmer. Also, mixing digitalWrite() and analogWrite() on nRF52
// can leave the PWM block in an awkward state. So we now:
//   - keep USR1 away from 0%
//   - use a separate screen-off state for USR2
//   - drive all levels through analogWrite(), including 0 and 255
static const uint8_t BL_LEVELS[] = {255, 96, 32, 8};
static const uint8_t BL_LABELS[] = {100, 75, 50, 25};
static constexpr uint8_t BL_LEVEL_COUNT = sizeof(BL_LEVELS) / sizeof(BL_LEVELS[0]);

// ========================= LCD parameters =========================

static constexpr int LCD_W = 135;
static constexpr int LCD_H = 240;
static constexpr int LCD_ROTATION = 0;
static constexpr bool LCD_IPS = true;

// Typical ST7789 1.14" 135x240 offsets.
// If the display is shifted/clipped, tune these four values first.
static constexpr int LCD_COL_OFFSET_1 = 52;
static constexpr int LCD_ROW_OFFSET_1 = 40;
static constexpr int LCD_COL_OFFSET_2 = 53;
static constexpr int LCD_ROW_OFFSET_2 = 40;

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

static constexpr uint16_t C_BLACK   = RGB565_BLACK;
static constexpr uint16_t C_WHITE   = RGB565_WHITE;
static constexpr uint16_t C_GREEN   = RGB565_LIGHTGREEN;
static constexpr uint16_t C_RED     = RGB565_RED;
static constexpr uint16_t C_CYAN    = RGB565_CYAN;
static constexpr uint16_t C_YELLOW  = RGB565_YELLOW;
static constexpr uint16_t C_ORANGE  = 0xFD20;
static constexpr uint16_t C_GRAY    = 0x8410;
static constexpr uint16_t C_DIM     = 0x2104;
static constexpr uint16_t C_BLUE    = RGB565_BLUE;
static constexpr uint16_t C_LINE    = 0x39E7;

// ========================= Battery =========================

static constexpr float BAT_DIVIDER_RATIO = (1000.0f + 499.0f) / 499.0f;
static constexpr float BAT_CAL_FACTOR    = 1.000f;
static constexpr int ADC_BITS            = 12;
static constexpr int ADC_MAX             = (1 << ADC_BITS) - 1;
static constexpr float ADC_FULL_SCALE_V  = 3.600f;

static constexpr uint16_t BAT_PRESENT_MIN_RAW = 80;
static constexpr uint16_t BAT_FLOAT_RANGE_RAW = 80;
static constexpr float BAT_VALID_MIN_V = 2.80f;
static constexpr float BAT_VALID_MAX_V = 4.60f;
static constexpr uint16_t BAT_STABLE_PRESENT_SPREAD_RAW = 30;
static constexpr uint8_t BAT_INSERT_CONFIRM_COUNT = 2;
static constexpr uint8_t BAT_REMOVE_CONFIRM_COUNT = 4;
static constexpr float BAT_INSERT_DELTA_V = 0.10f;
static constexpr float BAT_REMOVE_DELTA_V = 0.14f;
static constexpr float BAT_NOISY_CLOSE_DELTA_V = 0.08f;
static constexpr uint8_t BAT_PRESENT_NOISY_HOLD_COUNT = 4;
static constexpr uint32_t BAT_CHG_TRANSIENT_HOLD_MS = 900;

// Display-side SOC policy. A charging battery's terminal voltage rises immediately
// when USB is connected, but the real state of charge does not. Keep the percentage
// stable at the USB edge, then allow only a slow rise. After USB removal, wait for
// voltage relaxation before gently converging back to the voltage-derived estimate.
static constexpr uint32_t BAT_UI_CHARGE_LOCK_MS = 10000UL;
static constexpr uint32_t BAT_UI_CHARGE_RISE_MS = 120000UL;
static constexpr uint32_t BAT_UI_UNPLUG_HOLD_MS = 8000UL;
static constexpr uint32_t BAT_UI_CORRECT_STEP_MS = 6000UL;
// When a battery is inserted while USB is already present, the first measurable
// terminal voltage is already lifted by charging current. Subtract a small,
// empirically tuned surface/charge-voltage offset before deriving the initial SOC.
static constexpr float BAT_UI_USB_FIRST_COMP_V = 0.040f;

static constexpr uint8_t CHG_SAMPLE_COUNT = 9;
static constexpr uint8_t CHG_HIGH_CLEAR_COUNT = 1;

// ========================= MIC =========================

static constexpr int MIC_SAMPLE_RATE_HZ = 16000;
static constexpr int MIC_CHANNELS       = 1;
static constexpr int MIC_GAIN           = 30;
static constexpr int MIC_BUF_SAMPLES    = 256;
static constexpr uint32_t MIC_DECAY_MS  = 120;

volatile uint16_t g_micPeak = 0;
volatile uint32_t g_micRms = 0;
volatile uint32_t g_micBlocks = 0;
volatile uint32_t g_micLastUpdateMs = 0;
int16_t g_pdmBuf[MIC_BUF_SAMPLES];
float g_vuSmooth = 0.0f;
int g_cachedVuSegments = -1;

// ========================= Devices / state =========================

LSM6DS3 myIMU(I2C_MODE, 0x6A);
bool g_lcdOk = false;
bool g_micOk = false;
bool g_imuOk = false;

float g_ax = 0, g_ay = 0, g_az = 0;
float g_gx = 0, g_gy = 0, g_gz = 0;

uint8_t g_i2cDevices[12];
uint8_t g_i2cCount = 0;
bool g_i2cScanFault = false;
uint8_t g_i2cScanFaultCount = 0;
uint32_t g_lastI2cRecoveryMs = 0;
bool g_i2cHaveCompletedScan = false;
uint32_t g_i2cChangeFlashUntilMs = 0;
uint16_t g_i2cTimeoutCount = 0;
uint16_t g_i2cRecoveryCount = 0;
uint32_t g_lastImuRetryMs = 0;

bool g_btnA = false;
bool g_btnB = false;
bool g_btnC = false;
int g_btnARaw = HIGH;
int g_btnBRaw = HIGH;
int g_btnCRaw = HIGH;

bool g_btnALastRawPressed = false;
bool g_btnBLastRawPressed = false;
bool g_btnCLastRawPressed = false;
uint32_t g_btnALastChangeMs = 0;
uint32_t g_btnBLastChangeMs = 0;
uint32_t g_btnCLastChangeMs = 0;
bool g_btnAPressEvent = false;
bool g_btnBPressEvent = false;
bool g_btnCPressEvent = false;
uint32_t g_lastBtnActionMs = 0;

uint8_t g_blIndex = 0;
uint8_t g_blRestoreIndex = 0;
bool g_blScreenOff = false;
bool g_headerShowSeeed = false;

volatile bool g_imuIntFlag = false;
uint32_t g_doubleTapCount = 0;
uint32_t g_lastTapMs = 0;

uint32_t g_lastFastUiMs = 0;
uint32_t g_lastSlowUiMs = 0;
uint32_t g_lastVuMs = 0;
uint32_t g_lastSerialMs = 0;
uint32_t g_lastI2cMs = 0;
uint32_t g_lastBatMs = 0;

// ========================= Battery state =========================

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
const char *g_batFilterState = "BOOT";

bool g_chgRawLow = false;
bool g_chgState = false;
uint8_t g_chgHighStreak = 0;
uint32_t g_lastChgLowMs = 0;
bool g_chgRawInitialized = false;
uint32_t g_lastChgRawChangeMs = 0;

// Presence state is conservative under USB-C to avoid fake battery percentage.
enum BatteryPresenceState {
  BAT_BOOT = 0,
  BAT_USB_ONLY,
  BAT_INSERT_CANDIDATE,
  BAT_PRESENT,
  BAT_REMOVE_CANDIDATE
};

BatteryPresenceState g_batState = BAT_BOOT;
bool g_batPhysicallyConfirmed = false;

bool g_usbBaselineValid = false;
float g_usbBaselineVbat = 0.0f;
uint16_t g_usbBaselineRaw = 0;
uint16_t g_usbBaselineSpread = 0;
bool g_prevChgRawLow = false;
uint8_t g_insertCandidateStreak = 0;
uint8_t g_removeCandidateStreak = 0;

// ========================= UI layout =========================

static constexpr int ROW_TITLE = 2;
static constexpr int ROW_SUB1  = 21;

static constexpr int CARD_X = 3;
static constexpr int CARD_W = 129;

static constexpr int Y_SYS = 36;
static constexpr int H_SYS = 38;
static constexpr int ROW_SYS_TITLE = Y_SYS + 9;
static constexpr int ROW_BAT = ROW_SYS_TITLE;
static constexpr int ROW_GROVE = Y_SYS + 25;

static constexpr int Y_IMU = 79;
static constexpr int H_IMU = 50;
static constexpr int ROW_IMU_TITLE = Y_IMU + 9;
static constexpr int ROW_TAP = Y_IMU + 9;
static constexpr int ROW_ACC = Y_IMU + 25;
static constexpr int ROW_GYR = Y_IMU + 39;

static constexpr int Y_MIC = 134;
static constexpr int H_MIC = 45;
static constexpr int ROW_MIC_TITLE = Y_MIC + 9;
static constexpr int ROW_MIC_RAW   = Y_MIC + 32;

// Copied from 1.47 dashboard behavior, resized for 135 px width.
static constexpr int VU_X = CARD_X + 14;
static constexpr int VU_Y = Y_MIC + 21;
static constexpr int VU_W = 110;
static constexpr int VU_H = 10;
static constexpr int VU_SEG_COUNT = 12;
static constexpr int VU_GAP = 2;
static constexpr int VU_SEG_W = (VU_W - 4 - (VU_SEG_COUNT - 1) * VU_GAP) / VU_SEG_COUNT;

static constexpr int Y_BL = 182;
static constexpr int H_BL = 31;
static constexpr int ROW_BL = Y_BL + 19;
static constexpr int ROW_PRODUCT = 222;

static constexpr int BAT_ICON_X = 62;
static constexpr int BAT_ICON_Y = ROW_BAT - 2;
static constexpr int BAT_ICON_W = 18;
static constexpr int BAT_ICON_H = 10;
static constexpr int CHG_ICON_X = 116;
static constexpr int CHG_ICON_Y = ROW_BAT - 2;

String cache_micRaw = "";
String cache_acc = "";
String cache_gyr = "";
String cache_tap = "";
String cache_bat = "";
String cache_grove = "";
uint16_t cache_groveColor = 0xFFFF;
String cache_key = "";
String cache_bl = "";
int cache_batIconState = -1;
int cache_chgIconState = -1;

// ========================= Helpers =========================

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

  // Voltage-only SOC estimate. The low-voltage tail is intentionally expanded:
  // a deeply discharged battery can still keep the board running briefly, so
  // 3.0-3.3 V should show a critical 1-5% instead of jumping straight to 0%.
  static const Point table[] = {
    {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.92f, 70}, {3.85f, 60},
    {3.79f, 50},  {3.72f, 40}, {3.66f, 30}, {3.58f, 20}, {3.50f, 10},
    {3.30f, 5},   {3.10f, 1},  {3.00f, 0}
  };

  static constexpr size_t TABLE_COUNT = sizeof(table) / sizeof(table[0]);

  if (v >= table[0].v) return 100;
  if (v <= table[TABLE_COUNT - 1].v) return 0;

  for (size_t i = 0; i < TABLE_COUNT - 1; i++) {
    if (v <= table[i].v && v >= table[i + 1].v) {
      float t = (v - table[i + 1].v) / (table[i].v - table[i + 1].v);
      int pct = table[i + 1].p + (int)roundf(t * (table[i].p - table[i + 1].p));
      // A valid battery above the 3.00 V critical cutoff should never show 0%
      // while it is still powering the product.
      if (v > 3.00f && pct < 1) pct = 1;
      return pct;
    }
  }

  return 0;
}

static const char *batteryPresenceStateName(BatteryPresenceState s) {
  switch (s) {
    case BAT_BOOT:             return "BOOT";
    case BAT_USB_ONLY:         return "USB_ONLY";
    case BAT_INSERT_CANDIDATE: return "INSERT?";
    case BAT_PRESENT:          return "PRESENT";
    case BAT_REMOVE_CANDIDATE: return "REMOVE?";
    default:                   return "UNKNOWN";
  }
}

static void acquireForLcd() {
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void printTextFixed(int x, int y, uint16_t color, const String &s, int widthChars) {
  if (!g_lcdOk) return;
  acquireForLcd();
  gfx->setTextSize(1);
  gfx->setTextColor(color, C_BLACK);
  gfx->setCursor(x, y);
  gfx->print(padRight(s, widthChars));
}

static uint8_t currentBacklightPwm() {
  return g_blScreenOff ? 0 : BL_LEVELS[g_blIndex];
}

static int currentBacklightPercent() {
  return g_blScreenOff ? 0 : BL_LABELS[g_blIndex];
}

static void applyBacklight() {
  pinMode(LCD_BL_PIN, OUTPUT);

#if defined(ARDUINO_ARCH_NRF52) || defined(NRF52840_XXAA)
  analogWriteResolution(8);
#endif

  // USR1 never writes 0%. Only USR2 uses the explicit screen-off state.
  // For OFF, force the pin low because this hardware behaves more like BL enable.
  // For ON levels, return to PWM output.
  if (g_blScreenOff) {
    analogWrite(LCD_BL_PIN, 0);
    delayMicroseconds(300);
    digitalWrite(LCD_BL_PIN, LOW);
    return;
  }

  analogWrite(LCD_BL_PIN, BL_LEVELS[g_blIndex]);
}

static void cycleBacklightLevel() {
  if (g_blScreenOff) {
    g_blScreenOff = false;
    g_blIndex = g_blRestoreIndex;
  }
  g_blIndex = (g_blIndex + 1) % BL_LEVEL_COUNT;
  g_blRestoreIndex = g_blIndex;
  applyBacklight();
}

static void toggleBacklightOffRestore() {
  if (g_blScreenOff) {
    g_blScreenOff = false;
    g_blIndex = g_blRestoreIndex;
  } else {
    g_blRestoreIndex = g_blIndex;
    g_blScreenOff = true;
  }
  applyBacklight();
}

// ========================= LCD =========================

static bool initLcd() {
  applyBacklight();

  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  pinMode(LCD_RST_PIN, OUTPUT);

  if (!gfx->begin()) {
    g_lcdOk = false;
    Serial.println("[LCD] begin failed");
    return false;
  }

  gfx->fillScreen(C_BLACK);
  gfx->setTextWrap(false);
  g_lcdOk = true;
  Serial.println("[LCD] OK 1.14 ST7789 135x240 RST=D17");
  return true;
}

static void drawCard(int x, int y, int w, int h, uint16_t accent, const char *title) {
  acquireForLcd();
  gfx->drawRoundRect(x, y, w, h, 5, accent);
  gfx->drawRoundRect(x + 1, y + 1, w - 2, h - 2, 5, C_LINE);
  gfx->fillRect(x + 4, y + 9, 3, h - 18, accent);
  gfx->setTextSize(1);
  gfx->setTextColor(accent, C_BLACK);
  gfx->setCursor(x + 12, y + 8);
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

static void drawBatteryIcon(bool valid, int percent, bool charging) {
  acquireForLcd();
  int x = BAT_ICON_X;
  int y = BAT_ICON_Y;
  gfx->fillRect(x - 1, y - 1, BAT_ICON_W + 5, BAT_ICON_H + 2, C_BLACK);

  uint16_t outline = valid ? C_WHITE : C_GRAY;
  uint16_t fillColor = valid ? colorByPercent(percent) : C_RED;
  if (valid && charging) fillColor = C_CYAN;

  gfx->drawRoundRect(x, y, BAT_ICON_W, BAT_ICON_H, 2, outline);
  gfx->fillRect(x + BAT_ICON_W, y + 3, 3, 4, outline);

  if (!valid) {
    gfx->drawLine(x + 3, y + 2, x + BAT_ICON_W - 3, y + BAT_ICON_H - 3, C_RED);
    gfx->drawLine(x + BAT_ICON_W - 3, y + 2, x + 3, y + BAT_ICON_H - 3, C_RED);
    return;
  }

  int fillW = map(percent, 0, 100, 0, BAT_ICON_W - 4);
  if (fillW > 0) gfx->fillRect(x + 2, y + 2, fillW, BAT_ICON_H - 4, fillColor);
}

static void drawChargeIcon(bool charging) {
  acquireForLcd();
  int x = CHG_ICON_X;
  int y = CHG_ICON_Y;
  gfx->fillRect(x - 1, y - 1, 13, 14, C_BLACK);
  if (!charging) return;

  // Compact 11x13 lightning icon. Keep it outside the battery percent refresh
  // area; otherwise percent updates can erase half of the icon.
  gfx->fillTriangle(x + 6, y + 0, x + 1, y + 7, x + 6, y + 7, C_YELLOW);
  gfx->fillTriangle(x + 5, y + 6, x + 10, y + 6, x + 4, y + 13, C_YELLOW);
  gfx->drawLine(x + 6, y + 0, x + 1, y + 7, C_ORANGE);
  gfx->drawLine(x + 10, y + 6, x + 4, y + 13, C_ORANGE);
}

static void drawHeaderTitle() {
  acquireForLcd();
  gfx->fillRect(0, 0, LCD_W, 20, C_BLACK);
  gfx->setTextColor(C_GREEN, C_BLACK);

  if (g_headerShowSeeed) {
    gfx->setTextSize(2);
    gfx->setCursor(37, ROW_TITLE);
    gfx->print("Seeed");
  } else {
    gfx->setTextSize(2);
    gfx->setCursor(1, ROW_TITLE);
    gfx->print("Hello,XIAO!");
  }
}

static void drawStaticLayout() {
  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  drawHeaderTitle();

  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(6, ROW_SUB1);
  gfx->print("1.14 Inch Display");
  gfx->drawFastHLine(6, 31, 123, C_LINE);

  // SYSTEM: keep SYS and BAT on the same baseline with a cleaner gap.
  drawCard(CARD_X, Y_SYS, CARD_W, H_SYS, C_CYAN, "SYS");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(42, ROW_SYS_TITLE);
  gfx->print("BAT");
  gfx->setCursor(14, ROW_GROVE);
  gfx->print("I2C");
  drawBatteryIcon(false, 0, false);
  drawChargeIcon(false);

  drawCard(CARD_X, Y_IMU, CARD_W, H_IMU, C_YELLOW, "MOTION");
  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(88, ROW_TAP);
  gfx->print("Tap 0");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(14, ROW_ACC);
  gfx->print("A");
  gfx->setCursor(14, ROW_GYR);
  gfx->print("G");

  drawCard(CARD_X, Y_MIC, CARD_W, H_MIC, C_GREEN, "MIC LEVEL");
  drawVuFrame();

  // Only keep backlight info. Button status removed to avoid crowding.
  drawCard(CARD_X, Y_BL, CARD_W, H_BL, C_BLUE, "BACKLIGHT");

  gfx->fillRect(0, ROW_PRODUCT - 1, LCD_W, 12, C_BLACK);
  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(15, ROW_PRODUCT);
  gfx->print("XIAO nRF52840 Plus");

  cache_micRaw = "";
  cache_acc = "";
  cache_gyr = "";
  cache_tap = "";
  cache_bat = "";
  cache_grove = "";
  cache_groveColor = 0xFFFF;
  cache_key = "";
  cache_bl = "";
  cache_batIconState = -1;
  cache_chgIconState = -1;
  g_cachedVuSegments = -1;
}

// ========================= I2C / IMU =========================

// Seeed nRF52 Wire uses TWIM0 and waits for hardware events without a timeout.
// A Grove hot-plug in the middle of a transaction can therefore block forever.
// Runtime I2C access below talks to TWIM0 directly and bounds every wait.
static constexpr uint32_t I2C_TRANSACTION_TIMEOUT_US = 3500;
static constexpr uint32_t I2C_STOP_TIMEOUT_US = 1000;
static constexpr uint32_t I2C_BUS_SETTLE_US = 2500;

enum SafeI2cResult : uint8_t {
  SAFE_I2C_OK = 0,
  SAFE_I2C_NACK,
  SAFE_I2C_TIMEOUT,
  SAFE_I2C_BUS_ERROR
};

static uint32_t i2cSdaPsel() {
  return g_ADigitalPinMap[I2C_SDA_PIN];
}

static uint32_t i2cSclPsel() {
  return g_ADigitalPinMap[I2C_SCL_PIN];
}

static bool i2cLinesReleased() {
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(8);
  return digitalRead(I2C_SDA_PIN) == HIGH && digitalRead(I2C_SCL_PIN) == HIGH;
}

static bool i2cLinesStableReleased(uint32_t stableUs = I2C_BUS_SETTLE_US) {
  uint32_t startUs = micros();
  do {
    if (digitalRead(I2C_SDA_PIN) == LOW || digitalRead(I2C_SCL_PIN) == LOW) {
      return false;
    }
    delayMicroseconds(40);
  } while ((uint32_t)(micros() - startUs) < stableUs);
  return true;
}

static void clearTwimEvents() {
  NRF_TWIM0->EVENTS_STOPPED = 0;
  NRF_TWIM0->EVENTS_RXSTARTED = 0;
  NRF_TWIM0->EVENTS_TXSTARTED = 0;
  NRF_TWIM0->EVENTS_LASTRX = 0;
  NRF_TWIM0->EVENTS_LASTTX = 0;
  NRF_TWIM0->EVENTS_ERROR = 0;
  NRF_TWIM0->EVENTS_SUSPENDED = 0;
  uint32_t errors = NRF_TWIM0->ERRORSRC;
  NRF_TWIM0->ERRORSRC = errors;
}

static void safeTwimDisable() {
  NRF_TWIM0->TASKS_STOP = 1;
  uint32_t startUs = micros();
  while (!NRF_TWIM0->EVENTS_STOPPED &&
         (uint32_t)(micros() - startUs) < I2C_STOP_TIMEOUT_US) {
    // bounded wait
  }
  NRF_TWIM0->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);
  delayMicroseconds(10);
  clearTwimEvents();
}

static void safeTwimBegin() {
  safeTwimDisable();

  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);

  NRF_TWIM0->PSEL.SDA = i2cSdaPsel();
  NRF_TWIM0->PSEL.SCL = i2cSclPsel();
  NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K100;
  clearTwimEvents();
  NRF_TWIM0->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}

static bool waitTwimEventOrError(volatile uint32_t *eventReg, uint32_t timeoutUs) {
  uint32_t startUs = micros();
  while (!(*eventReg) && !NRF_TWIM0->EVENTS_ERROR) {
    if ((uint32_t)(micros() - startUs) >= timeoutUs) return false;
  }
  return true;
}

static bool waitTwimStopped(uint32_t timeoutUs) {
  uint32_t startUs = micros();
  while (!NRF_TWIM0->EVENTS_STOPPED) {
    if ((uint32_t)(micros() - startUs) >= timeoutUs) return false;
  }
  return true;
}

static uint8_t twimFinishWithStop() {
  uint32_t errors = NRF_TWIM0->EVENTS_ERROR ? NRF_TWIM0->ERRORSRC : 0;

  NRF_TWIM0->TASKS_STOP = 1;
  if (!waitTwimStopped(I2C_STOP_TIMEOUT_US)) {
    if (g_i2cTimeoutCount < 0xFFFF) g_i2cTimeoutCount++;
    safeTwimDisable();
    return SAFE_I2C_TIMEOUT;
  }

  clearTwimEvents();

  if (errors & (TWIM_ERRORSRC_ANACK_Msk | TWIM_ERRORSRC_DNACK_Msk)) {
    return SAFE_I2C_NACK;
  }
  if (errors) return SAFE_I2C_BUS_ERROR;
  return SAFE_I2C_OK;
}

static uint8_t safeI2cWrite(uint8_t address, const uint8_t *data, size_t len) {
  if (!i2cLinesReleased()) return SAFE_I2C_BUS_ERROR;

  uint8_t dummy = 0;
  clearTwimEvents();
  NRF_TWIM0->ADDRESS = address;
  NRF_TWIM0->TXD.PTR = (uint32_t)(len ? data : &dummy);
  NRF_TWIM0->TXD.MAXCNT = len;
  NRF_TWIM0->TASKS_RESUME = 1;
  NRF_TWIM0->TASKS_STARTTX = 1;

  if (!waitTwimEventOrError(&NRF_TWIM0->EVENTS_TXSTARTED, I2C_TRANSACTION_TIMEOUT_US)) {
    if (g_i2cTimeoutCount < 0xFFFF) g_i2cTimeoutCount++;
    safeTwimDisable();
    return SAFE_I2C_TIMEOUT;
  }
  NRF_TWIM0->EVENTS_TXSTARTED = 0;

  if (len && !NRF_TWIM0->EVENTS_ERROR) {
    if (!waitTwimEventOrError(&NRF_TWIM0->EVENTS_LASTTX, I2C_TRANSACTION_TIMEOUT_US)) {
      if (g_i2cTimeoutCount < 0xFFFF) g_i2cTimeoutCount++;
      safeTwimDisable();
      return SAFE_I2C_TIMEOUT;
    }
  }
  NRF_TWIM0->EVENTS_LASTTX = 0;
  return twimFinishWithStop();
}

static uint8_t safeI2cReadRaw(uint8_t address, uint8_t *data, size_t len) {
  if (!len) return SAFE_I2C_OK;
  if (!i2cLinesReleased()) return SAFE_I2C_BUS_ERROR;

  clearTwimEvents();
  NRF_TWIM0->ADDRESS = address;
  NRF_TWIM0->RXD.PTR = (uint32_t)data;
  NRF_TWIM0->RXD.MAXCNT = len;
  NRF_TWIM0->TASKS_RESUME = 1;
  NRF_TWIM0->TASKS_STARTRX = 1;

  if (!waitTwimEventOrError(&NRF_TWIM0->EVENTS_RXSTARTED, I2C_TRANSACTION_TIMEOUT_US)) {
    if (g_i2cTimeoutCount < 0xFFFF) g_i2cTimeoutCount++;
    safeTwimDisable();
    return SAFE_I2C_TIMEOUT;
  }
  NRF_TWIM0->EVENTS_RXSTARTED = 0;

  if (!NRF_TWIM0->EVENTS_ERROR &&
      !waitTwimEventOrError(&NRF_TWIM0->EVENTS_LASTRX, I2C_TRANSACTION_TIMEOUT_US)) {
    if (g_i2cTimeoutCount < 0xFFFF) g_i2cTimeoutCount++;
    safeTwimDisable();
    return SAFE_I2C_TIMEOUT;
  }
  NRF_TWIM0->EVENTS_LASTRX = 0;

  uint8_t result = twimFinishWithStop();
  if (result == SAFE_I2C_OK && NRF_TWIM0->RXD.AMOUNT != len) {
    return SAFE_I2C_BUS_ERROR;
  }
  return result;
}

static bool recoverI2cBus() {
  uint32_t now = millis();
  if (now - g_lastI2cRecoveryMs < 250) return false;
  g_lastI2cRecoveryMs = now;

  safeTwimDisable();

  // Release a slave stopped in the middle of a byte, then generate STOP.
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(10);

  for (uint8_t i = 0; i < 9; i++) {
    if (digitalRead(I2C_SDA_PIN) == HIGH) break;
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(8);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(8);
  }

  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(8);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(8);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  delayMicroseconds(30);

  bool released = i2cLinesReleased();
  safeTwimBegin();
  if (g_i2cRecoveryCount < 0xFFFF) g_i2cRecoveryCount++;
  return released;
}

static uint8_t safeI2cProbe(uint8_t address) {
  // nRF52840 TWIM does not reliably issue an address phase when TXD.MAXCNT is 0.
  // Use a bounded one-byte read instead: the slave must ACK its read address, but
  // no register/data byte is written to the device. The received byte is discarded.
  uint8_t discard = 0;
  uint8_t result = safeI2cReadRaw(address, &discard, 1);
  if (result == SAFE_I2C_TIMEOUT || result == SAFE_I2C_BUS_ERROR) {
    recoverI2cBus();
  }
  return result;
}

static bool safeI2cWriteReg(uint8_t address, uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {reg, value};
  uint8_t result = safeI2cWrite(address, tx, sizeof(tx));
  if (result == SAFE_I2C_TIMEOUT || result == SAFE_I2C_BUS_ERROR) recoverI2cBus();
  return result == SAFE_I2C_OK;
}

static bool safeI2cReadRegs(uint8_t address, uint8_t reg, uint8_t *data, size_t len) {
  uint8_t writeResult = safeI2cWrite(address, &reg, 1);
  if (writeResult != SAFE_I2C_OK) {
    if (writeResult == SAFE_I2C_TIMEOUT || writeResult == SAFE_I2C_BUS_ERROR) recoverI2cBus();
    return false;
  }

  uint8_t readResult = safeI2cReadRaw(address, data, len);
  if (readResult == SAFE_I2C_TIMEOUT || readResult == SAFE_I2C_BUS_ERROR) recoverI2cBus();
  return readResult == SAFE_I2C_OK;
}

static void clearI2cScanResultOnFault(const char *reason) {
  g_i2cCount = 0;
  g_i2cScanFault = true;
  if (g_i2cScanFaultCount < 255) g_i2cScanFaultCount++;

  Serial.print("[I2C] scan skipped: ");
  Serial.print(reason);
  Serial.print(" sda=");
  Serial.print(digitalRead(I2C_SDA_PIN));
  Serial.print(" scl=");
  Serial.print(digitalRead(I2C_SCL_PIN));
  Serial.print(" timeout=");
  Serial.print(g_i2cTimeoutCount);
  Serial.print(" recovery=");
  Serial.println(g_i2cRecoveryCount);
}

static void scanI2cBus() {
  uint8_t previousCount = g_i2cCount;
  uint8_t previousDevices[sizeof(g_i2cDevices)] = {};
  for (uint8_t i = 0; i < previousCount; i++) previousDevices[i] = g_i2cDevices[i];

  // Do not start a scan while connector contacts are still bouncing.
  if (!i2cLinesReleased() || !i2cLinesStableReleased()) {
    recoverI2cBus();
    clearI2cScanResultOnFault("bus settling");
    return;
  }

  g_i2cCount = 0;
  g_i2cScanFault = false;

  for (uint8_t addr = 0x08; addr <= 0x77 && g_i2cCount < sizeof(g_i2cDevices); addr++) {
    if (!i2cLinesReleased()) {
      recoverI2cBus();
      clearI2cScanResultOnFault("line low during scan");
      return;
    }

    uint8_t result = safeI2cProbe(addr);
    if (result == SAFE_I2C_OK) {
      g_i2cDevices[g_i2cCount++] = addr;
    } else if (result == SAFE_I2C_TIMEOUT || result == SAFE_I2C_BUS_ERROR) {
      clearI2cScanResultOnFault(result == SAFE_I2C_TIMEOUT ? "transaction timeout" : "transaction error");
      return;
    }
    delayMicroseconds(120);
  }

  bool addressListChanged = (previousCount != g_i2cCount);
  if (!addressListChanged) {
    for (uint8_t i = 0; i < g_i2cCount; i++) {
      if (previousDevices[i] != g_i2cDevices[i]) {
        addressListChanged = true;
        break;
      }
    }
  }

  if (g_i2cHaveCompletedScan && addressListChanged) {
    g_i2cChangeFlashUntilMs = millis() + I2C_CHANGE_FLASH_MS;
  }
  g_i2cHaveCompletedScan = true;

  Serial.print("[I2C] scan:");
  if (!g_i2cCount) Serial.print(" none");
  for (uint8_t i = 0; i < g_i2cCount; i++) {
    Serial.print(" 0x");
    if (g_i2cDevices[i] < 16) Serial.print("0");
    Serial.print(g_i2cDevices[i], HEX);
  }
  Serial.print(" timeout=");
  Serial.print(g_i2cTimeoutCount);
  Serial.print(" recovery=");
  Serial.println(g_i2cRecoveryCount);
}

static constexpr uint8_t LSM6DS3_ADDR        = 0x6A;
static constexpr uint8_t REG_WHO_AM_I        = 0x0F;
static constexpr uint8_t REG_CTRL1_XL        = 0x10;
static constexpr uint8_t REG_CTRL2_G         = 0x11;
static constexpr uint8_t REG_CTRL3_C         = 0x12;
static constexpr uint8_t REG_TAP_SRC         = 0x1C;
static constexpr uint8_t REG_OUTX_L_G        = 0x22;
static constexpr uint8_t REG_TAP_CFG         = 0x58;
static constexpr uint8_t REG_TAP_THS_6D      = 0x59;
static constexpr uint8_t REG_INT_DUR2        = 0x5A;
static constexpr uint8_t REG_WAKE_UP_THS     = 0x5B;
static constexpr uint8_t REG_MD1_CFG         = 0x5E;

static bool imuWriteReg(uint8_t reg, uint8_t val) {
  return safeI2cWriteReg(LSM6DS3_ADDR, reg, val);
}

static bool imuReadReg(uint8_t reg, uint8_t &val) {
  return safeI2cReadRegs(LSM6DS3_ADDR, reg, &val, 1);
}

static void updateImu() {
  uint8_t raw[12] = {};
  if (!safeI2cReadRegs(LSM6DS3_ADDR, REG_OUTX_L_G, raw, sizeof(raw))) {
    g_imuOk = false;
    return;
  }

  int16_t gxRaw = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
  int16_t gyRaw = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
  int16_t gzRaw = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
  int16_t axRaw = (int16_t)((uint16_t)raw[7] << 8 | raw[6]);
  int16_t ayRaw = (int16_t)((uint16_t)raw[9] << 8 | raw[8]);
  int16_t azRaw = (int16_t)((uint16_t)raw[11] << 8 | raw[10]);

  // CTRL1_XL = +/-2 g: 0.061 mg/LSB. CTRL2_G = 245 dps: 8.75 mdps/LSB.
  g_ax = axRaw * 0.000061f;
  g_ay = ayRaw * 0.000061f;
  g_az = azRaw * 0.000061f;
  g_gx = gxRaw * 0.00875f;
  g_gy = gyRaw * 0.00875f;
  g_gz = gzRaw * 0.00875f;
  g_imuOk = true;
}

void imuIntIsr() {
  g_imuIntFlag = true;
}

static bool initImuDoubleTap() {
  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);
  detachInterrupt(digitalPinToInterrupt(IMU_INT_PIN));

  uint8_t who = 0;
  bool ok = imuReadReg(REG_WHO_AM_I, who) && (who == 0x69 || who == 0x6A);

  if (ok) {
    ok &= imuWriteReg(REG_CTRL3_C, 0x44);     // BDU + register auto-increment
    ok &= imuWriteReg(REG_CTRL1_XL, 0x60);   // accel 416Hz, +/-2g
    ok &= imuWriteReg(REG_CTRL2_G, 0x60);    // gyro 416Hz, 245dps
    ok &= imuWriteReg(REG_TAP_CFG, 0x8E);    // embedded interrupt + X/Y/Z tap
    ok &= imuWriteReg(REG_TAP_THS_6D, 0x0C); // threshold
    ok &= imuWriteReg(REG_INT_DUR2, 0x7F);   // timing
    ok &= imuWriteReg(REG_WAKE_UP_THS, 0x80);// single/double tap mode
    ok &= imuWriteReg(REG_MD1_CFG, 0x08);    // double tap to INT1
  }

  uint8_t dummy = 0;
  if (ok) imuReadReg(REG_TAP_SRC, dummy);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIntIsr, RISING);
  g_imuOk = ok;

  Serial.print("[IMU] LSM6DS3 safe-TWIM double tap D14 ");
  Serial.print(ok ? "OK" : "FAILED");
  Serial.print(" who=0x");
  Serial.println(who, HEX);
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

  if (digitalRead(IMU_INT_PIN) == HIGH) shouldCheck = true;
  if (!shouldCheck) return;

  uint8_t src = 0;
  if (!imuReadReg(REG_TAP_SRC, src)) {
    g_imuOk = false;
    return;
  }

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

// ========================= Buttons =========================

static void updateButtons() {
  uint32_t now = millis();

  g_btnARaw = digitalRead(BTN_A_PIN);
  g_btnBRaw = digitalRead(BTN_B_PIN);
  g_btnCRaw = digitalRead(BTN_C_PIN);

  bool rawA = (g_btnARaw == LOW);
  bool rawB = (g_btnBRaw == LOW);
  bool rawC = (g_btnCRaw == LOW);

  if (rawA != g_btnALastRawPressed) { g_btnALastRawPressed = rawA; g_btnALastChangeMs = now; }
  if (rawB != g_btnBLastRawPressed) { g_btnBLastRawPressed = rawB; g_btnBLastChangeMs = now; }
  if (rawC != g_btnCLastRawPressed) { g_btnCLastRawPressed = rawC; g_btnCLastChangeMs = now; }

  if ((now - g_btnALastChangeMs) >= BTN_DEBOUNCE_MS && rawA != g_btnA) {
    bool old = g_btnA;
    g_btnA = rawA;
    if (g_btnA && !old) g_btnAPressEvent = true;
  }
  if ((now - g_btnBLastChangeMs) >= BTN_DEBOUNCE_MS && rawB != g_btnB) {
    bool old = g_btnB;
    g_btnB = rawB;
    if (g_btnB && !old) g_btnBPressEvent = true;
  }
  if ((now - g_btnCLastChangeMs) >= BTN_DEBOUNCE_MS && rawC != g_btnC) {
    bool old = g_btnC;
    g_btnC = rawC;
    if (g_btnC && !old) g_btnCPressEvent = true;
  }
}

static void handleButtonActions() {
  bool doA = false;
  bool doB = false;
  bool doC = false;

  if (g_btnAPressEvent) { g_btnAPressEvent = false; doA = true; }
  if (g_btnBPressEvent) { g_btnBPressEvent = false; doB = true; }
  if (g_btnCPressEvent) { g_btnCPressEvent = false; doC = true; }

  if (!doA && !doB && !doC) return;

  uint32_t now = millis();
  if (now - g_lastBtnActionMs < BTN_ACTION_LOCKOUT_MS) return;
  g_lastBtnActionMs = now;

  if (doB) {
    toggleBacklightOffRestore();
    Serial.print("[BTN] USR2 toggle backlight pwm="); Serial.print((unsigned)currentBacklightPwm()); Serial.print(" label="); Serial.print(currentBacklightPercent()); Serial.println("%");
    return;
  }

  if (doA) {
    cycleBacklightLevel();
    Serial.print("[BTN] USR1 cycle backlight pwm="); Serial.print((unsigned)currentBacklightPwm()); Serial.print(" label="); Serial.print(currentBacklightPercent()); Serial.println("%");
    return;
  }

  if (doC) {
    g_headerShowSeeed = !g_headerShowSeeed;
    drawHeaderTitle();
    Serial.print("[BTN] USR3 header=");
    Serial.println(g_headerShowSeeed ? "Seeed Studio" : "Hello,XIAO!");
  }
}

// ========================= PDM mic =========================

void onPdmReceive() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;
  if (bytesAvailable > (int)sizeof(g_pdmBuf)) bytesAvailable = sizeof(g_pdmBuf);

  int bytesRead = PDM.read((void *)g_pdmBuf, bytesAvailable);
  if (bytesRead <= 0) return;

  int samples = bytesRead / 2;
  uint16_t peak = 0;
  uint64_t sumSq = 0;

  for (int i = 0; i < samples; i++) {
    int32_t s = g_pdmBuf[i];
    uint32_t a = abs(s);
    if (a > peak) peak = (uint16_t)((a > 65535) ? 65535 : a);
    sumSq += (uint64_t)(s * s);
  }

  g_micPeak = peak;
  g_micRms = samples ? (uint32_t)sqrt((double)sumSq / (double)samples) : 0;
  g_micBlocks++;
  g_micLastUpdateMs = millis();
}

static bool initMic() {
  // Sketch-layer pin remap: DATA=D1, CLK=D0, no power pin.
  PDM.setPins(PDM_DATA_PIN, PDM_CLK_PIN, -1);
  PDM.onReceive(onPdmReceive);
  PDM.setBufferSize(sizeof(g_pdmBuf));
  PDM.setGain(MIC_GAIN);

  if (!PDM.begin(MIC_CHANNELS, MIC_SAMPLE_RATE_HZ)) {
    Serial.println("[MIC] PDM.begin failed");
    g_micOk = false;
    return false;
  }

  Serial.println("[MIC] OK PDM.setPins(DATA=D1, CLK=D0, PWR=-1)");
  g_micOk = true;
  return true;
}

static uint16_t currentMicPeak() {
  uint32_t now = millis();
  if (now - g_micLastUpdateMs > MIC_DECAY_MS) {
    g_micPeak = (uint16_t)(g_micPeak * 0.75f);
  }
  return g_micPeak;
}

// ========================= Battery =========================

static void enableBatteryDivider() {
  nrf_gpio_cfg_output(READ_BAT_P0_PIN);
  nrf_gpio_pin_clear(READ_BAT_P0_PIN);
}

static void disableBatteryDivider() {
  nrf_gpio_cfg_output(READ_BAT_P0_PIN);
  nrf_gpio_pin_set(READ_BAT_P0_PIN);
}

static bool sampleChargingRawLow() {
  uint8_t lowCount = 0;
  for (uint8_t i = 0; i < CHG_SAMPLE_COUNT; i++) {
    if ((NRF_P0->IN & (1UL << CHG_P0_PIN)) == 0) lowCount++;
    delayMicroseconds(400);
  }

  bool newRawLow = (lowCount >= ((CHG_SAMPLE_COUNT / 2) + 1));
  uint32_t now = millis();
  if (!g_chgRawInitialized) {
    g_chgRawInitialized = true;
    g_lastChgRawChangeMs = now;
  } else if (newRawLow != g_chgRawLow) {
    g_lastChgRawChangeMs = now;
  }
  g_chgRawLow = newRawLow;
  return g_chgRawLow;
}

static bool updateChargingState(bool batValid, float vbat) {
  (void)vbat;
  bool rawLow = sampleChargingRawLow();

  if (!batValid) {
    g_chgState = false;
    g_chgHighStreak = 0;
    return false;
  }

  if (rawLow) {
    g_chgState = true;
    g_lastChgLowMs = millis();
    g_chgHighStreak = 0;
    return true;
  }

  if (g_chgHighStreak < 255) g_chgHighStreak++;
  if (g_chgHighStreak >= CHG_HIGH_CLEAR_COUNT) g_chgState = false;
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
  rawMin = buf[trim];
  rawMax = buf[samples - 1 - trim];

  uint32_t sum = 0;
  uint8_t count = 0;
  for (uint8_t i = trim; i < samples - trim; i++) {
    sum += buf[i];
    count++;
  }
  return count ? (uint16_t)(sum / count) : buf[samples / 2];
}

static void setBatteryAbsentUsb(const char *filterState) {
  g_batState = BAT_USB_ONLY;
  g_batPhysicallyConfirmed = false;
  g_haveLastGoodBat = false;
  g_chgState = false;
  g_chgHighStreak = 0;
  g_batFilterState = filterState;
}

static void confirmBatteryPresent(const BatteryState &measured, const char *filterState) {
  g_batState = BAT_PRESENT;
  g_batPhysicallyConfirmed = true;
  g_insertCandidateStreak = 0;
  g_removeCandidateStreak = 0;
  g_batFilterState = filterState;

  g_bat = measured;
  g_bat.valid = true;
  g_lastGoodBat = g_bat;
  g_haveLastGoodBat = true;
}

static void updateUsbOnlyBaseline(const BatteryState &m, uint16_t spread) {
  if (!g_usbBaselineValid) {
    g_usbBaselineValid = true;
    g_usbBaselineVbat = m.vbat;
    g_usbBaselineRaw = m.raw;
    g_usbBaselineSpread = spread;
    return;
  }

  g_usbBaselineVbat = g_usbBaselineVbat * 0.85f + m.vbat * 0.15f;
  g_usbBaselineRaw = (uint16_t)((float)g_usbBaselineRaw * 0.85f + (float)m.raw * 0.15f);
  g_usbBaselineSpread = (uint16_t)((float)g_usbBaselineSpread * 0.85f + (float)spread * 0.15f);
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
  bool voltagePlausible = (measured.raw > BAT_PRESENT_MIN_RAW && measured.vbat > BAT_VALID_MIN_V && measured.vbat < BAT_VALID_MAX_V);
  bool stableBatch = voltagePlausible && (spread <= BAT_FLOAT_RANGE_RAW);
  bool veryStableBatch = voltagePlausible && (spread <= BAT_STABLE_PRESENT_SPREAD_RAW);

  bool chargingNow = updateChargingState(voltagePlausible, measured.vbat);
  bool chgEdgeLow = (!g_prevChgRawLow && g_chgRawLow);
  g_prevChgRawLow = g_chgRawLow;

  bool closeToLastGood = g_haveLastGoodBat && fabsf(measured.vbat - g_lastGoodBat.vbat) <= BAT_NOISY_CLOSE_DELTA_V;
  bool farFromLastGood = g_haveLastGoodBat && fabsf(measured.vbat - g_lastGoodBat.vbat) >= BAT_REMOVE_DELTA_V;

  disableBatteryDivider();

  if (!voltagePlausible) {
    if (g_batPhysicallyConfirmed) {
      g_removeCandidateStreak++;
      if (g_removeCandidateStreak < BAT_REMOVE_CONFIRM_COUNT) {
        g_bat = g_lastGoodBat;
        g_bat.valid = true;
        g_bat.charging = chargingNow;
        g_batFilterState = "HOLD";
        return;
      }
    }
    setBatteryAbsentUsb("MISS");
    g_bat = measured;
    g_bat.valid = false;
    g_bat.charging = false;
    return;
  }

  if (g_batPhysicallyConfirmed) {
    uint32_t nowMs = millis();
    bool recentChgTransition = (nowMs - g_lastChgRawChangeMs) < BAT_CHG_TRANSIENT_HOLD_MS;
    bool noisyOrJump = (!stableBatch) || farFromLastGood;
    bool chgHighNow = !g_chgRawLow;
    bool likelyRemoved = noisyOrJump && chgHighNow;

    if (noisyOrJump) {
      if (likelyRemoved) {
        if (g_removeCandidateStreak < 255) g_removeCandidateStreak++;
        g_batState = BAT_REMOVE_CANDIDATE;

        // UI switches to USB PWR immediately when removal signature is clear.
        g_bat = measured;
        g_bat.valid = false;
        g_bat.charging = false;

        if (g_removeCandidateStreak >= BAT_REMOVE_CONFIRM_COUNT) {
          setBatteryAbsentUsb("REMOVED");
          g_batFilterState = "REMOVED";
        } else {
          g_batFilterState = "RM_UI";
        }
        return;
      }

      if (closeToLastGood && (recentChgTransition || g_removeCandidateStreak < BAT_PRESENT_NOISY_HOLD_COUNT)) {
        if (g_removeCandidateStreak < 255) g_removeCandidateStreak++;
        g_batState = BAT_PRESENT;
        BatteryState filtered = g_lastGoodBat;
        filtered.valid = true;
        filtered.charging = chargingNow;
        g_bat = filtered;
        g_batFilterState = recentChgTransition ? "USB_TR" : "NOISY_H";
        return;
      }

      if (g_removeCandidateStreak < 255) g_removeCandidateStreak++;
      g_batState = BAT_REMOVE_CANDIDATE;
      g_bat = measured;
      g_bat.valid = false;
      g_bat.charging = false;
      g_batFilterState = "RM_UI";
      return;
    }

    g_removeCandidateStreak = 0;
    if (stableBatch) {
      measured.valid = true;
      measured.charging = chargingNow;
      confirmBatteryPresent(measured, "STABLE");
      return;
    }
  }

  if (stableBatch && !g_chgRawLow) {
    measured.valid = true;
    measured.charging = false;
    confirmBatteryPresent(measured, "BAT_ONLY");
    return;
  }

  // Unconfirmed USB-C state:
  // Do not trust static VBAT, but do trust a real transition:
  //   - CHG HIGH->LOW edge
  //   - noisy USB baseline becoming a very stable battery-like signal
  //   - meaningful VBAT shift from baseline
  //
  // Important: calculate deltas BEFORE updating the baseline. Updating first can
  // swallow the exact hot-plug change we need to detect.
  bool baselineDelta = g_usbBaselineValid && fabsf(measured.vbat - g_usbBaselineVbat) >= BAT_INSERT_DELTA_V;
  bool spreadImproved = g_usbBaselineValid && (g_usbBaselineSpread > BAT_FLOAT_RANGE_RAW) && veryStableBatch;
  bool stableLowAfterNoisyUsb = g_usbBaselineValid &&
                                (g_usbBaselineSpread > BAT_STABLE_PRESENT_SPREAD_RAW) &&
                                stableBatch &&
                                g_chgRawLow;
  bool recentLowEdge = g_chgRawLow && ((millis() - g_lastChgRawChangeMs) < 2500);
  bool insertCandidate = stableBatch && (chgEdgeLow || recentLowEdge || spreadImproved || baselineDelta || stableLowAfterNoisyUsb);

  if (insertCandidate) {
    if (g_insertCandidateStreak < 255) g_insertCandidateStreak++;
    g_batState = BAT_INSERT_CANDIDATE;
    g_batFilterState = stableLowAfterNoisyUsb ? "INS_ST" : "INS?";
    if (g_insertCandidateStreak >= BAT_INSERT_CONFIRM_COUNT) {
      measured.valid = true;
      measured.charging = chargingNow;
      confirmBatteryPresent(measured, chargingNow ? "INSERT_CHG" : "INSERT");
      return;
    }
  } else {
    if (g_insertCandidateStreak > 0) g_insertCandidateStreak--;
    g_batState = BAT_USB_ONLY;
    g_batFilterState = "USBVBAT";
    updateUsbOnlyBaseline(measured, spread);
  }

  g_bat = measured;
  g_bat.valid = false;
  g_bat.charging = false;
}

static String batteryShortText() {
  if (!g_bat.valid) {
    if (g_batState == BAT_USB_ONLY || g_batState == BAT_INSERT_CANDIDATE || g_batState == BAT_REMOVE_CANDIDATE) {
      return "USB PWR";
    }
    return "NO BAT";
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%d%%", g_bat.percent);
  return String(buf);
}

static bool isUsbPowerTextMode() {
  return !g_bat.valid &&
         (g_batState == BAT_USB_ONLY ||
          g_batState == BAT_INSERT_CANDIDATE ||
          g_batState == BAT_REMOVE_CANDIDATE);
}

// ========================= UI update =========================

static uint16_t vuColorForIndex(int idx) {
  if (idx >= 10) return C_RED;
  if (idx >= 7) return C_ORANGE;
  return C_GREEN;
}

static void updateVu() {
  // Directly ported from the 1.47 nRF Dashboard VU behavior:
  //   - faster attack, slower release
  //   - segmented blocks
  //   - green -> orange -> red by segment index
  uint16_t micPeak = currentMicPeak();

  float target = (float)micPeak / 2200.0f;
  if (target < 0.0f) target = 0.0f;
  if (target > 1.0f) target = 1.0f;

  if (target > g_vuSmooth) g_vuSmooth = g_vuSmooth * 0.55f + target * 0.45f;
  else g_vuSmooth = g_vuSmooth * 0.86f + target * 0.14f;

  int activeSegs = (int)roundf(g_vuSmooth * VU_SEG_COUNT);
  if (activeSegs < 0) activeSegs = 0;
  if (activeSegs > VU_SEG_COUNT) activeSegs = VU_SEG_COUNT;

  if (activeSegs == g_cachedVuSegments) return;

  acquireForLcd();
  for (int i = 0; i < VU_SEG_COUNT; ++i) {
    int x = VU_X + 2 + i * (VU_SEG_W + VU_GAP);
    uint16_t color = (i < activeSegs) ? vuColorForIndex(i) : C_BLACK;
    gfx->fillRect(x, VU_Y + 2, VU_SEG_W, VU_H - 4, color);
  }

  g_cachedVuSegments = activeSegs;
}

static void updateUiFast() {
  char buf[48];

  // SYSTEM / battery row
  // v0.9 fixed stale text by clearing the whole BAT row, but that made the row
  // visibly blink whenever battery percent jittered by 1%. v1.0 separates:
  //   - full row redraw: only when display mode changes, e.g. USB PWR <-> battery
  //   - small redraw: only icon/percent/lightning when battery value changes
  bool usbTextMode = isUsbPowerTextMode();
  bool batModeValid = g_bat.valid;
  bool batModeCharging = g_bat.valid && g_bat.charging;

  static bool s_batUiInit = false;
  static bool s_lastUsbTextMode = false;
  static bool s_lastBatValid = false;
  static bool s_lastCharging = false;
  static int s_displayPct = -1;
  static int s_lastRawPct = -1;
  static uint32_t s_lastPctAcceptMs = 0;
  static uint32_t s_chargeStartedMs = 0;
  static uint32_t s_chargeRiseMs = 0;
  static uint32_t s_chargeStoppedMs = 0;

  uint32_t nowBatUi = millis();
  int rawPct = constrain(g_bat.percent, 0, 100);

  bool batteryBecameValid = s_batUiInit && batModeValid && !s_lastBatValid;
  bool chargeStarted = s_batUiInit && batModeValid && batModeCharging && !s_lastCharging;
  bool chargeStopped = s_batUiInit && batModeValid && !batModeCharging && s_lastCharging;

  if (!s_batUiInit || batteryBecameValid) {
    // Cold boot or a newly confirmed battery normally starts from the measured
    // estimate. Special case: if the previous UI state was USB PWR and the newly
    // inserted battery is already charging, rawPct includes charge-voltage lift.
    // Compensate that first estimate, then let the normal slow-charge policy take over.
    bool usbFirstBatteryInsert = s_batUiInit &&
                                 batteryBecameValid &&
                                 s_lastUsbTextMode &&
                                 batModeCharging;

    if (usbFirstBatteryInsert) {
      float compensatedV = g_bat.vbat - BAT_UI_USB_FIRST_COMP_V;
      if (compensatedV < BAT_VALID_MIN_V) compensatedV = BAT_VALID_MIN_V;
      int compensatedPct = lipoPercent(compensatedV);
      // The real battery is still above the 3.00 V critical cutoff. Even if the
      // USB-first voltage compensation pushes the estimated voltage close to
      // 3.00 V and interpolation rounds down, do not present 0% while valid.
      if (g_bat.vbat > 3.00f && compensatedPct < 1) compensatedPct = 1;
      s_displayPct = constrain(compensatedPct, 0, 100);
      Serial.print("[BAT_UI] USB-first insert raw=");
      Serial.print(g_bat.vbat, 3);
      Serial.print("V rawPct=");
      Serial.print(rawPct);
      Serial.print(" compensated=");
      Serial.print(compensatedV, 3);
      Serial.print("V displayPct=");
      Serial.println(s_displayPct);
    } else {
      s_displayPct = rawPct;
    }

    s_lastRawPct = rawPct;
    s_lastPctAcceptMs = nowBatUi;
    s_chargeRiseMs = nowBatUi;

    if (batModeCharging) {
      s_chargeStartedMs = nowBatUi;
    }
  } else if (!batModeValid) {
    // Percentage is hidden in USB-only / no-battery mode. Reset transition timers
    // without allowing an irrelevant ADC value to overwrite the last visible SOC.
    s_lastRawPct = rawPct;
    s_chargeStartedMs = 0;
    s_chargeStoppedMs = 0;
    s_chargeRiseMs = nowBatUi;
  } else {
    if (s_displayPct < 0) s_displayPct = rawPct;

    if (chargeStarted) {
      // Preserve the battery-only percentage. The voltage rise at the USB edge is
      // surface/charge voltage, not an instantaneous SOC increase.
      s_chargeStartedMs = nowBatUi;
      s_chargeRiseMs = nowBatUi;
    }

    if (batModeCharging) {
      // Never reduce the displayed percentage while charging. After the initial
      // lock period, allow at most +1% per interval and never exceed rawPct.
      if ((nowBatUi - s_chargeStartedMs) >= BAT_UI_CHARGE_LOCK_MS &&
          rawPct > s_displayPct &&
          (nowBatUi - s_chargeRiseMs) >= BAT_UI_CHARGE_RISE_MS) {
        s_displayPct++;
        s_chargeRiseMs = nowBatUi;
        s_lastPctAcceptMs = nowBatUi;
      }
    } else {
      if (chargeStopped) {
        s_chargeStoppedMs = nowBatUi;
        s_lastPctAcceptMs = nowBatUi;
      }

      bool unplugRelaxing = s_chargeStoppedMs != 0 &&
                            (nowBatUi - s_chargeStoppedMs) < BAT_UI_UNPLUG_HOLD_MS;

      if (!unplugRelaxing) {
        // Outside charging, gently converge toward the voltage estimate. This keeps
        // normal battery updates responsive but prevents a large post-charge jump.
        bool needCorrection = rawPct != s_displayPct;
        bool correctionDue = (nowBatUi - s_lastPctAcceptMs) >= BAT_UI_CORRECT_STEP_MS;

        if (needCorrection && correctionDue) {
          s_displayPct += (rawPct > s_displayPct) ? 1 : -1;
          s_lastPctAcceptMs = nowBatUi;
        }
      }
    }

    s_lastRawPct = rawPct;
  }

  // Final UI-side safety floor. g_bat.percent and the UI percentage use separate
  // state machines; a previous USB/charging state can leave s_displayPct at 0
  // even after the battery estimator reports 1%. For any confirmed battery above
  // 3.00 V, the final value sent to the LCD must be at least 1%.
  if (batModeValid && g_bat.vbat > 3.00f && s_displayPct < 1) {
    s_displayPct = 1;
    s_lastPctAcceptMs = nowBatUi;
    Serial.println("[BAT_UI] corrected stale display floor to 1%");
  }

  bool modeChanged = (!s_batUiInit ||
                      usbTextMode != s_lastUsbTextMode ||
                      batModeValid != s_lastBatValid ||
                      batModeCharging != s_lastCharging);

  String batKey;
  if (usbTextMode) {
    batKey = "USB";
  } else if (batModeValid) {
    batKey = String("BAT:") + String(s_displayPct) + String(batModeCharging ? ":C" : ":N");
  } else {
    batKey = "NOBAT";
  }

  if (modeChanged) {
    acquireForLcd();

    // Clear only the dynamic battery row area inside the SYSTEM card.
    gfx->fillRect(37, ROW_BAT - 2, 91, 14, C_BLACK);

    gfx->setTextSize(1);
    gfx->setTextColor(C_WHITE, C_BLACK);
    gfx->setCursor(40, ROW_BAT);
    gfx->print("BAT");

    cache_bat = "";          // force content redraw below
    cache_batIconState = -1;
    cache_chgIconState = -1;
  }

  if (batKey != cache_bat) {
    cache_bat = batKey;

    if (usbTextMode) {
      acquireForLcd();
      gfx->fillRect(62, ROW_BAT - 2, 66, 14, C_BLACK);
      gfx->setTextSize(1);
      gfx->setTextColor(C_RED, C_BLACK);
      gfx->setCursor(64, ROW_BAT);
      gfx->print("USB PWR");

      cache_batIconState = -1;
      cache_chgIconState = 0;
    } else {
      uint16_t c = batModeValid ? colorByPercent(s_displayPct) : C_RED;
      if (batModeCharging) c = C_CYAN;

      int iconState = (batModeValid ? 1000 : 0) +
                      (batModeCharging ? 500 : 0) +
                      constrain(s_displayPct, 0, 100);

      if (iconState != cache_batIconState) {
        cache_batIconState = iconState;
        drawBatteryIcon(batModeValid, s_displayPct, batModeCharging);
      }

      acquireForLcd();
      gfx->fillRect(88, ROW_BAT - 2, 25, 14, C_BLACK);
      gfx->setTextSize(1);
      gfx->setTextColor(c, C_BLACK);
      gfx->setCursor(88, ROW_BAT);
      if (batModeValid) {
        gfx->print(s_displayPct);
        gfx->print("%");
      } else {
        gfx->print("--");
      }

      int chgIconState = batModeCharging ? 1 : 0;
      if (chgIconState != cache_chgIconState) {
        cache_chgIconState = chgIconState;
        drawChargeIcon(chgIconState == 1);
      }
    }
  }

  s_batUiInit = true;
  s_lastUsbTextMode = usbTextMode;
  s_lastBatValid = batModeValid;
  s_lastCharging = batModeCharging;

  String groveText;
  if (g_i2cCount == 0) {
    groveText = "none";
  } else {
    char t[24];
    snprintf(t, sizeof(t), "%u 0x%02X", g_i2cCount, g_i2cDevices[0]);
    groveText = String(t);
  }
  bool i2cChangeFlashActive = ((int32_t)(g_i2cChangeFlashUntilMs - millis()) > 0);
  uint16_t groveColor = i2cChangeFlashActive ? C_GREEN : C_CYAN;

  if (groveText != cache_grove || groveColor != cache_groveColor) {
    cache_grove = groveText;
    cache_groveColor = groveColor;
    printTextFixed(50, ROW_GROVE, groveColor, groveText, 11);
  }

  // MOTION
  snprintf(buf, sizeof(buf), "%+.1f %+.1f %+.1f", g_ax, g_ay, g_az);
  String accText = String(buf);
  if (accText != cache_acc) {
    cache_acc = accText;
    printTextFixed(28, ROW_ACC, C_WHITE, accText, 14);
  }

  snprintf(buf, sizeof(buf), "%+.0f %+.0f %+.0f", g_gx, g_gy, g_gz);
  String gyrText = String(buf);
  if (gyrText != cache_gyr) {
    cache_gyr = gyrText;
    printTextFixed(28, ROW_GYR, C_WHITE, gyrText, 14);
  }

  String tapText = String("Tap ") + String(g_doubleTapCount);
  if (tapText != cache_tap) {
    cache_tap = tapText;
    printTextFixed(90, ROW_TAP, C_YELLOW, tapText, 6);
  }

  // MIC
  uint16_t peak = currentMicPeak();
  snprintf(buf, sizeof(buf), "Raw %u", peak);
  String micText = String(buf);
  if (micText != cache_micRaw) {
    cache_micRaw = micText;
    uint16_t rawColor = C_WHITE;
    if (peak > 1900) rawColor = C_RED;
    else if (peak > 1200) rawColor = C_ORANGE;
    printTextFixed(14, ROW_MIC_RAW, rawColor, micText, 18);
  }

  // BACKLIGHT only; button status is intentionally hidden to keep the UI clean.
  String blText = g_blScreenOff ? String("BL OFF") : (String("BL ") + String(currentBacklightPercent()) + "%");
  if (blText != cache_bl) {
    cache_bl = blText;
    printTextFixed(17, ROW_BL, g_blScreenOff ? C_RED : C_CYAN, blText, 12);
  }
}

static void printSerialStatus() {
  Serial.print("[DASH114] mic="); Serial.print((unsigned)currentMicPeak());
  Serial.print(" blocks="); Serial.print((unsigned long)g_micBlocks);
  Serial.print(" acc=("); Serial.print(g_ax, 2); Serial.print(","); Serial.print(g_ay, 2); Serial.print(","); Serial.print(g_az, 2); Serial.print(")");
  Serial.print(" gyr=("); Serial.print(g_gx, 2); Serial.print(","); Serial.print(g_gy, 2); Serial.print(","); Serial.print(g_gz, 2); Serial.print(")");
  Serial.print(" i2c="); Serial.print((unsigned)g_i2cCount);
  Serial.print(" bat="); Serial.print(g_bat.vbat, 3); Serial.print("V pct="); Serial.print(g_bat.percent);
  Serial.print(" valid="); Serial.print(g_bat.valid ? "Y" : "N");
  Serial.print(" chg="); Serial.print(g_bat.charging ? "Y" : "N");
  Serial.print(" chgRaw="); Serial.print(g_chgRawLow ? "LOW" : "HIGH");
  Serial.print(" raw="); Serial.print(g_bat.raw);
  Serial.print(" spread="); Serial.print((unsigned)(g_bat.rawMax - g_bat.rawMin));
  Serial.print(" filt="); Serial.print(g_batFilterState);
  Serial.print(" batState="); Serial.print(batteryPresenceStateName(g_batState));
  Serial.print(" tap="); Serial.print((unsigned long)g_doubleTapCount);
  Serial.print(" bl="); Serial.print((unsigned)currentBacklightPwm()); Serial.print("/"); Serial.print(currentBacklightPercent()); Serial.print("%"); Serial.print(" off="); Serial.print(g_blScreenOff ? "Y" : "N");
  Serial.print(" usr1="); Serial.print(g_btnA ? "P" : "R");
  Serial.print(" usr2="); Serial.print(g_btnB ? "P" : "R");
  Serial.print(" usr3="); Serial.println(g_btnC ? "P" : "R");
}

// ========================= Setup / loop =========================

void setup() {
  Serial.begin(115200);
  delay(900);

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 1.14 Factory Dashboard v1.0.6 timeout-safe I2C + BAT stable ===");
  Serial.println("No touch, no SD. LCD_RST=D17 corrected. BL=D18/LCD_BL_PWM. PDM.setPins(DATA=D1, CLK=D0, PWR=-1).");

  pinMode(BTN_A_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);
  pinMode(BTN_C_PIN, INPUT_PULLUP);

#if defined(ARDUINO_ARCH_NRF52) || defined(NRF52840_XXAA)
  analogWriteResolution(8);
#endif

  g_btnARaw = digitalRead(BTN_A_PIN);
  g_btnBRaw = digitalRead(BTN_B_PIN);
  g_btnCRaw = digitalRead(BTN_C_PIN);
  g_btnALastRawPressed = (g_btnARaw == LOW);
  g_btnBLastRawPressed = (g_btnBRaw == LOW);
  g_btnCLastRawPressed = (g_btnCRaw == LOW);
  g_btnA = g_btnALastRawPressed;
  g_btnB = g_btnBLastRawPressed;
  g_btnC = g_btnCLastRawPressed;
  g_btnALastChangeMs = millis();
  g_btnBLastChangeMs = millis();
  g_btnCLastChangeMs = millis();

  safeTwimBegin();
  nrf_gpio_cfg_input(CHG_P0_PIN, NRF_GPIO_PIN_PULLUP);
  sampleChargingRawLow();

  analogReadResolution(ADC_BITS);
  disableBatteryDivider();

  initLcd();
  drawStaticLayout();

  initMic();
  initImuDoubleTap();
  scanI2cBus();
  updateBattery();
}

void loop() {
  uint32_t now = millis();

  handleImuTapEvent();
  updateButtons();
  handleButtonActions();

  if (now - g_lastVuMs >= UI_VU_MS) {
    g_lastVuMs = now;
    updateVu();
  }

  if (now - g_lastFastUiMs >= UI_FAST_MS) {
    g_lastFastUiMs = now;
    updateUiFast();
  }

  if (now - g_lastSlowUiMs >= UI_SLOW_MS) {
    g_lastSlowUiMs = now;
    updateImu();
    if (!g_imuOk && now - g_lastImuRetryMs >= 1500) {
      g_lastImuRetryMs = now;
      if (i2cLinesReleased() && i2cLinesStableReleased(800)) initImuDoubleTap();
    }
  }

  if (now - g_lastI2cMs >= I2C_SCAN_MS) {
    g_lastI2cMs = now;
    scanI2cBus();
  }

  if (now - g_lastBatMs >= BAT_REFRESH_MS) {
    g_lastBatMs = now;
    updateBattery();
  }

  if (now - g_lastSerialMs >= SERIAL_MS) {
    g_lastSerialMs = now;
    printSerialStatus();
  }

  delay(2);
}
