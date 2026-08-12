/*
  XIAO nRF52840 Plus + 0.96 Inch Display
  Factory Dashboard v1.2.14 - 1.14 verified battery SOC method

  Target:
    0.96 Inch Display Powered by XIAO nRF52840 Plus

  Board difference vs 1.14 / 1.47:
    - LCD: 0.96" ST7789, 80 x 160, very small UI area
    - No touch
    - No SD slot
    - 2 user buttons: USR1 / USR2
    - IMU INT uses D14 on the updated 0.96 pin map
    - KEY1 cycles backlight brightness; KEY2 toggles screen backlight ON/OFF.

  Covered modules:
    - 0.96" IPS ST7789 LCD
    - PDM digital microphone, PDM.setPins(DATA=D1, CLK=D0, PWR=-1)
    - LSM6DS3 IMU, 6-axis read + double-tap count
    - 2 user buttons with IRQ + pending queue
    - Backlight brightness cycle + ON/OFF
    - nRF52840 Plus battery voltage / charging status:
        READ_BAT = P0.14 active-low divider enable
        VBAT ADC = PIN_VBAT
        CHG      = P0.17 active-low charging status

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
static constexpr uint8_t BTN_A_PIN     = D6;   // USR1: backlight ON/OFF
static constexpr uint8_t BTN_B_PIN     = D7;   // USR2: toggle header + reset tap count
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t LCD_RST_PIN   = D17;
static constexpr uint8_t LCD_BL_PIN    = D18;

static constexpr uint8_t READ_BAT_P0_PIN = 14; // P0.14 / ~READ_BAT
static constexpr uint8_t CHG_P0_PIN      = 17; // P0.17 / ~CHG

// Updated 0.96 hardware:
//   IMU_INT = D14 = Arduino pin 33 = P0.09
// Battery monitor:
//   READ_BAT = P0.14 = raw GPIO 14
//
// Important: D14 is NOT raw pin 14. Do not map D14 to 14, or it will fight
// READ_BAT and break battery monitoring. If the board variant does not expose
// D14 as a macro, use Arduino pin 33 from the XIAO nRF52840 Plus pinout.
#if defined(D14)
static constexpr int IMU_INT_PIN = D14;
#else
static constexpr int IMU_INT_PIN = 33;
#endif

static constexpr bool IMU_INT_PIN_SAFE =
  (IMU_INT_PIN != (int)READ_BAT_P0_PIN);  // should be true: D14/P0.09 is Arduino 33

#ifndef PIN_VBAT
#define PIN_VBAT 35
#endif

// ========================= Timing =========================

static constexpr uint32_t UI_FAST_MS       = 100;
static constexpr uint32_t UI_SLOW_MS       = 220;
static constexpr uint32_t UI_VU_MS         = 70;
static constexpr uint32_t BAT_REFRESH_MS   = 500;
static constexpr uint32_t SERIAL_MS        = 700;
static constexpr uint32_t BTN_DEBOUNCE_MS  = 18;
static constexpr uint32_t BTN_ACTION_LOCKOUT_MS = 70;
static constexpr uint32_t TAP_DEBOUNCE_MS  = 220;
static constexpr uint32_t MIC_DECAY_MS     = 120;

// ========================= LCD parameters =========================

static constexpr int LCD_W = 80;
static constexpr int LCD_H = 160;
static constexpr int LCD_ROTATION = 2;
static constexpr bool LCD_IPS = true;
static constexpr bool LCD_INVERT_COLORS = true;

// Verified on previous 0.96 bring-up.
// If the picture is shifted, first try 25,0,25,0 or 26,0,26,0.
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

static constexpr uint16_t C_BLACK   = RGB565_BLACK;
static constexpr uint16_t C_WHITE   = RGB565_WHITE;
static constexpr uint16_t C_GREEN   = RGB565_LIGHTGREEN;
static constexpr uint16_t C_RED     = RGB565_RED;
static constexpr uint16_t C_CYAN    = RGB565_CYAN;
static constexpr uint16_t C_YELLOW  = RGB565_YELLOW;
static constexpr uint16_t C_ORANGE  = 0xFD20;
static constexpr uint16_t C_GRAY    = 0x8410;
static constexpr uint16_t C_LINE    = 0x39E7;
static constexpr uint16_t C_DIM     = 0x2104;
static constexpr uint16_t C_BLUE    = RGB565_BLUE;

// 0.96 panel visual color aliases.
// With current rotation/offset/invert init, RGB565_RED appears blue and
// RGB565_BLUE appears red on this module. These aliases express the intended
// visual color on the actual screen.
static constexpr uint16_t V_RED     = C_BLUE;
static constexpr uint16_t V_BLUE    = C_RED;
static constexpr uint16_t V_YELLOW  = C_CYAN;
static constexpr uint16_t V_CYAN    = C_YELLOW;
static constexpr uint16_t V_GREEN   = C_GREEN;
static constexpr uint16_t V_WHITE   = C_WHITE;

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

// Exact display-side SOC policy used by the verified 1.14 dashboard.
// USB insertion keeps the previous SOC, USB-first battery insertion applies
// a 40 mV charge-voltage compensation, and later changes converge slowly.
static constexpr uint32_t BAT_UI_CHARGE_LOCK_MS = 10000UL;
static constexpr uint32_t BAT_UI_CHARGE_RISE_MS = 120000UL;
static constexpr uint32_t BAT_UI_UNPLUG_HOLD_MS = 8000UL;
static constexpr uint32_t BAT_UI_CORRECT_STEP_MS = 6000UL;
static constexpr float BAT_UI_USB_FIRST_COMP_V = 0.040f;

static constexpr uint8_t CHG_SAMPLE_COUNT = 9;
static constexpr uint8_t CHG_HIGH_CLEAR_COUNT = 1;

// ========================= PDM MIC =========================

static constexpr int MIC_SAMPLE_RATE_HZ = 16000;
static constexpr int MIC_CHANNELS       = 1;
static constexpr int MIC_GAIN           = 30;
static constexpr int MIC_BUF_SAMPLES    = 256;

volatile uint16_t g_micPeak = 0;
volatile uint32_t g_micRms = 0;
volatile uint32_t g_micBlocks = 0;
volatile uint32_t g_micLastUpdateMs = 0;
int16_t g_pdmBuf[MIC_BUF_SAMPLES];
float g_vuSmooth = 0.0f;
float g_vuFast = 0.0f;
int g_vuDisplaySegments = 0;
int g_cachedVuSegments = -1;
int g_cachedVuWidth = -1;
uint16_t g_cachedVuColor = 0xFFFF;
uint32_t g_lastVuStepMs = 0;

// VU anti-flicker state.
// Candidate debounce prevents adjacent blocks from flickering near thresholds.
// Delta drawing avoids full-bar clear/redraw flashes.
int g_vuCandidateSegments = 0;
uint8_t g_vuCandidateCount = 0;
uint32_t g_lastVuRenderMs = 0;

// Anti-rebound guard:
// after the VU starts falling, ignore small upward bounces for a short window.
// This fixes the visible reverse pulse during decay.
uint32_t g_vuFallGuardUntilMs = 0;

// 0.96 MIC calibration.
// The previous fixed thresholds made room noise look like full-scale.
// Track a slow noise floor and draw only signal above that floor.
float g_micNoiseFloor = 65.0f;
bool g_micNoiseReady = false;
uint32_t g_micNoiseStartMs = 0;

// ========================= State =========================

enum ImuType {
  IMU_NONE = 0,
  IMU_LSM6 = 1,
  IMU_QMI8658 = 2,
  IMU_UNKNOWN = 3
};

static ImuType g_imuType = IMU_NONE;
static uint8_t g_imuAddr = 0;

bool g_lcdOk = false;
bool g_micOk = false;
bool g_imuOk = false;
bool g_blOn = true;
static const uint8_t BL_LEVELS[] = {255, 192, 128, 64};  // 100%, 75%, 50%, 25%
static constexpr uint8_t BL_LEVEL_COUNT = sizeof(BL_LEVELS) / sizeof(BL_LEVELS[0]);
uint8_t g_blLevelIndex = 0;  // boot at 100%; KEY1: 100 -> 75 -> 50 -> 25 -> 100
bool g_headerSeeed = false;

float g_ax = 0, g_ay = 0, g_az = 0;
float g_gx = 0, g_gy = 0, g_gz = 0;

bool g_btnA = false;
bool g_btnB = false;
int g_btnARaw = HIGH;
int g_btnBRaw = HIGH;

bool g_btnALastRawPressed = false;
bool g_btnBLastRawPressed = false;
uint32_t g_btnALastChangeMs = 0;
uint32_t g_btnBLastChangeMs = 0;

volatile uint8_t g_btnAIrqCount = 0;
volatile uint8_t g_btnBIrqCount = 0;
bool g_btnAPendingAction = false;
bool g_btnBPendingAction = false;
bool g_btnAArmed = true;
bool g_btnBArmed = true;
uint32_t g_btnALastQueueMs = 0;
uint32_t g_btnBLastQueueMs = 0;
uint32_t g_lastBtnActionMs = 0;

volatile bool g_imuIntFlag = false;
uint32_t g_doubleTapCount = 0;
uint32_t g_lastTapMs = 0;

uint32_t g_lastFastMs = 0;
uint32_t g_lastSlowMs = 0;
uint32_t g_lastVuMs = 0;
uint32_t g_lastBatMs = 0;
uint32_t g_lastSerialMs = 0;
uint32_t g_frameCounter = 0;

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

static constexpr int ROW_TITLE  = 3;
static constexpr int ROW_SUB    = 22;

// Flat compact layout for 80x160: no card borders, only section labels + dividers.
static constexpr int SECTION_X0 = 4;
static constexpr int SECTION_X1 = 75;

static constexpr int Y_SYS_LABEL = 35;
static constexpr int Y_SYS_ROW1  = 47;
static constexpr int Y_SYS_ROW2  = 60;
static constexpr int Y_SYS_DIV   = 70;

static constexpr int Y_MOTION_LABEL = 76;
static constexpr int Y_TAP          = 76;
static constexpr int Y_ACC          = 90;
static constexpr int Y_GYR          = 103;
static constexpr int Y_MOTION_DIV   = 112;

// MIC returns to the 1.47-style compact layout:
// title and volume bar on the same row, Raw below.
static constexpr int Y_MIC_LABEL = 119;
static constexpr int Y_RAW       = 134;

static constexpr int ROW_FOOT = 151;

static constexpr int VU_X = 33;
static constexpr int VU_Y = Y_MIC_LABEL;
static constexpr int VU_W = 41;
static constexpr int VU_H = 9;
static constexpr int VU_SEG_COUNT = 8;
static constexpr int VU_GAP = 1;
static constexpr int VU_SEG_W = (VU_W - 2 - (VU_SEG_COUNT - 1) * VU_GAP) / VU_SEG_COUNT;

String cache_header = "";
String cache_sys = "";
String cache_bl = "";
String cache_acc = "";
String cache_gyr = "";
String cache_tap = "";
String cache_btn = "";
String cache_raw = "";
String cache_ax_txt = "";
String cache_ay_txt = "";
String cache_az_txt = "";
String cache_gx_txt = "";
String cache_gy_txt = "";
String cache_gz_txt = "";
bool g_imuTinyLabelsDrawn = false;

// Battery UI state. This follows the verified 1.14 dashboard policy exactly:
// the raw voltage estimate may jump at USB edges, while the displayed SOC must
// remain physically plausible and converge slowly.
bool g_batUiInit = false;
bool g_batUiUsb = true;
bool g_batUiValid = false;
bool g_batUiCharging = false;
int g_batUiPct = -1;
int g_batUiLastRawPct = -1;
uint32_t g_lastBatUiCommitMs = 0;
uint32_t g_batUiChargeStartedMs = 0;
uint32_t g_batUiChargeRiseMs = 0;
uint32_t g_batUiChargeStoppedMs = 0;


// ========================= Helpers =========================

static String padRight(const String &s, int width) {
  String out = s;
  while ((int)out.length() < width) out += ' ';
  if ((int)out.length() > width) out = out.substring(0, width);
  return out;
}

static uint16_t colorByPercent(int pct) {
  if (pct <= 15) return V_RED;
  if (pct <= 35) return V_YELLOW;
  return V_GREEN;
}

static int lipoPercent(float v) {
  struct Point { float v; int p; };

  // Same low-voltage tail as the verified 1.14 dashboard.
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
      if (v > 3.00f && pct < 1) pct = 1;
      return pct;
    }
  }

  return 0;
}

static const char *batStateName(BatteryPresenceState s) {
  switch (s) {
    case BAT_BOOT: return "BOOT";
    case BAT_USB_ONLY: return "USB";
    case BAT_INSERT_CANDIDATE: return "INS?";
    case BAT_PRESENT: return "BAT";
    case BAT_REMOVE_CANDIDATE: return "RM?";
    default: return "?";
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

static void cleanScreenEdges() {
  if (!g_lcdOk) return;
  acquireForLcd();
  gfx->drawFastVLine(0, 0, LCD_H, C_BLACK);
  gfx->drawFastVLine(1, 0, LCD_H, C_BLACK);
  gfx->drawFastHLine(0, 0, LCD_W, C_BLACK);
}

// ========================= Backlight =========================

static uint8_t currentBacklightLevel() {
  if (g_blLevelIndex >= BL_LEVEL_COUNT) g_blLevelIndex = BL_LEVEL_COUNT - 1;
  return BL_LEVELS[g_blLevelIndex];
}

static int currentBacklightPercent() {
  return (int)roundf((float)currentBacklightLevel() * 100.0f / 255.0f);
}

static String backlightStatusText() {
  if (!g_blOn) return String("OFF");
  return String(currentBacklightPercent()) + String("%");
}

static void applyBacklight() {
  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, g_blOn ? currentBacklightLevel() : 0);
}

static void toggleBacklight() {
  g_blOn = !g_blOn;
  applyBacklight();
  cache_bl = "";
}

static void cycleBacklightBrightness() {
  // KEY1 cycles brightness in descending order.
  // Boot level is 100%; each press goes 100 -> 75 -> 50 -> 25 -> 100.
  // If the screen is currently off, turn it back on and then move to the next level.
  if (!g_blOn) g_blOn = true;
  g_blLevelIndex = (g_blLevelIndex + 1) % BL_LEVEL_COUNT;
  applyBacklight();
  cache_bl = "";
}

// ========================= Button IRQ / queue =========================

static void queueButtonA(uint32_t now) {
  if (!g_btnAArmed) return;
  if (now - g_btnALastQueueMs < BTN_DEBOUNCE_MS) return;
  g_btnALastQueueMs = now;
  g_btnAPendingAction = true;
  g_btnAArmed = false;
}

static void queueButtonB(uint32_t now) {
  if (!g_btnBArmed) return;
  if (now - g_btnBLastQueueMs < BTN_DEBOUNCE_MS) return;
  g_btnBLastQueueMs = now;
  g_btnBPendingAction = true;
  g_btnBArmed = false;
}

void btnAIrqIsr() {
  if (g_btnAIrqCount < 250) g_btnAIrqCount++;
}

void btnBIrqIsr() {
  if (g_btnBIrqCount < 250) g_btnBIrqCount++;
}

static void updateButtons() {
  uint32_t now = millis();
  uint8_t irqA = 0;
  uint8_t irqB = 0;
  noInterrupts();
  irqA = g_btnAIrqCount;
  irqB = g_btnBIrqCount;
  g_btnAIrqCount = 0;
  g_btnBIrqCount = 0;
  interrupts();

  g_btnARaw = digitalRead(BTN_A_PIN);
  g_btnBRaw = digitalRead(BTN_B_PIN);
  bool rawA = (g_btnARaw == LOW);
  bool rawB = (g_btnBRaw == LOW);

  if (irqA) {
    queueButtonA(now);
    g_btnALastRawPressed = rawA;
    g_btnALastChangeMs = now;
  }
  if (irqB) {
    queueButtonB(now);
    g_btnBLastRawPressed = rawB;
    g_btnBLastChangeMs = now;
  }

  if (rawA != g_btnALastRawPressed) {
    g_btnALastRawPressed = rawA;
    g_btnALastChangeMs = now;
  }
  if (rawB != g_btnBLastRawPressed) {
    g_btnBLastRawPressed = rawB;
    g_btnBLastChangeMs = now;
  }

  if ((now - g_btnALastChangeMs) >= BTN_DEBOUNCE_MS && rawA != g_btnA) {
    bool old = g_btnA;
    g_btnA = rawA;
    if (g_btnA && !old) queueButtonA(now);
  }
  if ((now - g_btnBLastChangeMs) >= BTN_DEBOUNCE_MS && rawB != g_btnB) {
    bool old = g_btnB;
    g_btnB = rawB;
    if (g_btnB && !old) queueButtonB(now);
  }

  if (!rawA && g_btnA && (now - g_btnALastChangeMs) >= BTN_DEBOUNCE_MS) {
    g_btnA = false;
    g_btnAArmed = true;
  }
  if (!rawB && g_btnB && (now - g_btnBLastChangeMs) >= BTN_DEBOUNCE_MS) {
    g_btnB = false;
    g_btnBArmed = true;
  }

  // If an IRQ captured a very short tap that has already been released before
  // polling sees it, arm the next physical press again.
  if (!rawA && !g_btnA) g_btnAArmed = true;
  if (!rawB && !g_btnB) g_btnBArmed = true;
}

static void handleButtonActions() {
  if (!g_btnAPendingAction && !g_btnBPendingAction) return;

  uint32_t now = millis();
  if (now - g_lastBtnActionMs < BTN_ACTION_LOCKOUT_MS) return;

  if (g_btnAPendingAction) {
    g_btnAPendingAction = false;

    // KEY1 / USR1: cycle backlight brightness.
    cycleBacklightBrightness();
    g_lastBtnActionMs = now;

    Serial.print("[BTN] KEY1 brightness=");
    Serial.print(currentBacklightPercent());
    Serial.println("%");
    return;
  }

  if (g_btnBPendingAction) {
    g_btnBPendingAction = false;

    // KEY2 / USR2: screen backlight ON/OFF.
    toggleBacklight();
    g_lastBtnActionMs = now;

    Serial.print("[BTN] KEY2 backlight=");
    Serial.print(g_blOn ? "ON " : "OFF ");
    Serial.println(backlightStatusText());
    return;
  }
}

// ========================= Battery =========================// ========================= Battery =========================

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
  for (uint8_t i = 0; i < 6; i++) { (void)analogRead(PIN_VBAT); delay(2); }
  for (uint8_t i = 0; i < samples; i++) { buf[i] = analogRead(PIN_VBAT); delay(2); }
  for (uint8_t i = 0; i < samples; i++) {
    for (uint8_t j = i + 1; j < samples; j++) {
      if (buf[j] < buf[i]) { uint16_t t = buf[i]; buf[i] = buf[j]; buf[j] = t; }
    }
  }
  uint8_t trim = samples >= 16 ? 4 : 1;
  rawMin = buf[trim];
  rawMax = buf[samples - 1 - trim];
  uint32_t sum = 0;
  uint8_t count = 0;
  for (uint8_t i = trim; i < samples - trim; i++) { sum += buf[i]; count++; }
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

        // Same 1.14 behavior: switch the UI to USB as soon as a clear
        // battery-removal signature appears, while confirmation continues.
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

  // Exact 1.14 USB-first insertion decision. Calculate deltas before updating
  // the USB-only baseline so the hot-plug change cannot be swallowed.
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

static String batteryTinyText() {
  if (!g_bat.valid) {
    if (g_batState == BAT_USB_ONLY || g_batState == BAT_INSERT_CANDIDATE || g_batState == BAT_REMOVE_CANDIDATE) return "USB";
    return "NO";
  }
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", g_bat.percent);
  return String(buf);
}


// ========================= Tiny text + battery icons =========================

static uint8_t tinyGlyphRow(char ch, uint8_t row) {
  // 3x5 tiny font. Bit2 is left pixel, bit0 is right pixel.
  // Deliberately supports lowercase n/l/u/s so the footer can show "nRF".
  switch (ch) {
    case '0': { static const uint8_t g[5] = {0b111,0b101,0b101,0b101,0b111}; return g[row]; }
    case '1': { static const uint8_t g[5] = {0b010,0b110,0b010,0b010,0b111}; return g[row]; }
    case '2': { static const uint8_t g[5] = {0b111,0b001,0b111,0b100,0b111}; return g[row]; }
    case '3': { static const uint8_t g[5] = {0b111,0b001,0b111,0b001,0b111}; return g[row]; }
    case '4': { static const uint8_t g[5] = {0b101,0b101,0b111,0b001,0b001}; return g[row]; }
    case '5': { static const uint8_t g[5] = {0b111,0b100,0b111,0b001,0b111}; return g[row]; }
    case '6': { static const uint8_t g[5] = {0b111,0b100,0b111,0b101,0b111}; return g[row]; }
    case '7': { static const uint8_t g[5] = {0b111,0b001,0b010,0b010,0b010}; return g[row]; }
    case '8': { static const uint8_t g[5] = {0b111,0b101,0b111,0b101,0b111}; return g[row]; }
    case '9': { static const uint8_t g[5] = {0b111,0b101,0b111,0b001,0b111}; return g[row]; }
    case '.': { static const uint8_t g[5] = {0b000,0b000,0b000,0b000,0b010}; return g[row]; }

    case 'A': { static const uint8_t g[5] = {0b111,0b101,0b111,0b101,0b101}; return g[row]; }
    case 'B': { static const uint8_t g[5] = {0b110,0b101,0b110,0b101,0b110}; return g[row]; }
    case 'C': { static const uint8_t g[5] = {0b111,0b100,0b100,0b100,0b111}; return g[row]; }
    case 'D': { static const uint8_t g[5] = {0b110,0b101,0b101,0b101,0b110}; return g[row]; }
    case 'E': { static const uint8_t g[5] = {0b111,0b100,0b110,0b100,0b111}; return g[row]; }
    case 'F': { static const uint8_t g[5] = {0b111,0b100,0b111,0b100,0b100}; return g[row]; }
    case 'G': { static const uint8_t g[5] = {0b111,0b100,0b101,0b101,0b111}; return g[row]; }
    case 'H': { static const uint8_t g[5] = {0b101,0b101,0b111,0b101,0b101}; return g[row]; }
    case 'I': { static const uint8_t g[5] = {0b111,0b010,0b010,0b010,0b111}; return g[row]; }
    case 'L': { static const uint8_t g[5] = {0b100,0b100,0b100,0b100,0b111}; return g[row]; }
    case 'N': { static const uint8_t g[5] = {0b101,0b111,0b111,0b111,0b101}; return g[row]; }
    case 'O': { static const uint8_t g[5] = {0b111,0b101,0b101,0b101,0b111}; return g[row]; }
    case 'P': { static const uint8_t g[5] = {0b111,0b101,0b111,0b100,0b100}; return g[row]; }
    case 'R': { static const uint8_t g[5] = {0b110,0b101,0b110,0b101,0b101}; return g[row]; }
    case 'S': { static const uint8_t g[5] = {0b111,0b100,0b111,0b001,0b111}; return g[row]; }
    case 'T': { static const uint8_t g[5] = {0b111,0b010,0b010,0b010,0b010}; return g[row]; }
    case 'U': { static const uint8_t g[5] = {0b101,0b101,0b101,0b101,0b111}; return g[row]; }
    case 'X': { static const uint8_t g[5] = {0b101,0b101,0b010,0b101,0b101}; return g[row]; }
    case 'Y': { static const uint8_t g[5] = {0b101,0b101,0b010,0b010,0b010}; return g[row]; }

    case 'a': { static const uint8_t g[5] = {0b000,0b111,0b001,0b111,0b111}; return g[row]; }
    case 'c': { static const uint8_t g[5] = {0b000,0b111,0b100,0b100,0b111}; return g[row]; }
    case 'd': { static const uint8_t g[5] = {0b001,0b001,0b111,0b101,0b111}; return g[row]; }
    case 'e': { static const uint8_t g[5] = {0b000,0b111,0b111,0b100,0b111}; return g[row]; }
    case 'h': { static const uint8_t g[5] = {0b100,0b100,0b111,0b101,0b101}; return g[row]; }
    case 'i': { static const uint8_t g[5] = {0b010,0b000,0b010,0b010,0b010}; return g[row]; }
    case 'l': { static const uint8_t g[5] = {0b110,0b010,0b010,0b010,0b111}; return g[row]; }
    case 'n': { static const uint8_t g[5] = {0b000,0b110,0b101,0b101,0b101}; return g[row]; }
    case 'p': { static const uint8_t g[5] = {0b000,0b110,0b101,0b110,0b100}; return g[row]; }
    case 's': { static const uint8_t g[5] = {0b011,0b100,0b110,0b001,0b110}; return g[row]; }
    case 'u': { static const uint8_t g[5] = {0b000,0b101,0b101,0b101,0b111}; return g[row]; }
    case 'y': { static const uint8_t g[5] = {0b000,0b101,0b111,0b001,0b110}; return g[row]; }

    case '+': { static const uint8_t g[5] = {0b000,0b010,0b111,0b010,0b000}; return g[row]; }
    case '-': { static const uint8_t g[5] = {0b000,0b000,0b111,0b000,0b000}; return g[row]; }
    case ' ': default: return 0;
  }
}

static void drawTinyText(int x, int y, const char *text, uint16_t color) {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->fillRect(0, y - 1, LCD_W, 8, C_BLACK);

  int cx = x;
  for (const char *p = text; *p; ++p) {
    for (uint8_t row = 0; row < 5; row++) {
      uint8_t bits = tinyGlyphRow(*p, row);
      for (uint8_t col = 0; col < 3; col++) {
        if (bits & (1 << (2 - col))) {
          gfx->drawPixel(cx + col, y + row, color);
        }
      }
    }
    cx += 4;
    if (cx > LCD_W - 3) break;
  }
}

static void drawTinyTextNoClear(int x, int y, const char *text, uint16_t color) {
  if (!g_lcdOk) return;

  acquireForLcd();

  int cx = x;
  for (const char *p = text; *p; ++p) {
    for (uint8_t row = 0; row < 5; row++) {
      uint8_t bits = tinyGlyphRow(*p, row);
      for (uint8_t col = 0; col < 3; col++) {
        if (bits & (1 << (2 - col))) {
          gfx->drawPixel(cx + col, y + row, color);
        }
      }
    }
    cx += 4;
    if (cx > LCD_W - 3) break;
  }
}

static void drawTiny3x5Field(int x, int y, const String &s, uint16_t color, uint8_t fieldChars) {
  if (!g_lcdOk) return;

  acquireForLcd();

  int w = fieldChars * 4 - 1;
  gfx->fillRect(x, y, w, 5, C_BLACK);
  drawTinyTextNoClear(x, y, s.c_str(), color);
}

static void drawBatteryIconTiny(int x, int y, bool valid, int pct, bool charging) {
  if (!g_lcdOk) return;

  acquireForLcd();

  uint16_t c = valid ? (charging ? V_CYAN : colorByPercent(pct)) : V_RED;

  gfx->fillRect(x - 1, y - 1, 15, 10, C_BLACK);
  gfx->drawRect(x, y, 11, 7, valid ? V_WHITE : V_RED);
  gfx->fillRect(x + 11, y + 2, 2, 3, valid ? V_WHITE : V_RED);

  if (!valid) {
    gfx->drawLine(x + 2, y + 1, x + 8, y + 6, V_RED);
    gfx->drawLine(x + 8, y + 1, x + 2, y + 6, V_RED);
    return;
  }

  int fillW = map(constrain(pct, 0, 100), 0, 100, 0, 9);
  if (fillW > 0) {
    gfx->fillRect(x + 1, y + 1, fillW, 5, c);
  }
}

static void drawChargeIconTiny(int x, int y, bool show) {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->fillRect(x - 1, y - 1, 9, 11, C_BLACK);
  if (!show) return;

  gfx->fillTriangle(x + 4, y + 0, x + 0, y + 5, x + 4, y + 5, V_YELLOW);
  gfx->fillTriangle(x + 3, y + 4, x + 7, y + 4, x + 2, y + 10, V_YELLOW);
  gfx->drawLine(x + 4, y + 0, x + 0, y + 5, V_YELLOW);
  gfx->drawLine(x + 7, y + 4, x + 2, y + 10, V_YELLOW);
}

static bool isUsbPowerTextMode() {
  return !g_bat.valid &&
         (g_batState == BAT_USB_ONLY ||
          g_batState == BAT_INSERT_CANDIDATE ||
          g_batState == BAT_REMOVE_CANDIDATE);
}

static void updateBatteryUiSnapshot() {
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
      s_displayPct = constrain(lipoPercent(compensatedV), 0, 100);
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

  // 0.96 rendering adapter only. These values mirror the exact 1.14 UI state;
  // they do not participate in SOC calculation or transition decisions.
  g_batUiUsb = usbTextMode;
  g_batUiValid = batModeValid;
  g_batUiCharging = batModeCharging;
  g_batUiPct = s_displayPct;
  g_batUiLastRawPct = s_lastRawPct;
  g_lastBatUiCommitMs = s_lastPctAcceptMs;
  g_batUiChargeStartedMs = s_chargeStartedMs;
  g_batUiChargeRiseMs = s_chargeRiseMs;
  g_batUiChargeStoppedMs = s_chargeStoppedMs;

  s_batUiInit = true;
  s_lastUsbTextMode = usbTextMode;
  s_lastBatValid = batModeValid;
  s_lastCharging = batModeCharging;
}

static void drawBatteryRowTiny() {
  if (!g_lcdOk) return;

  bool usbMode = g_batUiUsb;
  bool batValid = g_batUiValid;
  bool charging = g_batUiCharging;
  int pct = g_batUiPct;

  acquireForLcd();

  // Clear SYS dynamic area.
  gfx->fillRect(SECTION_X0, Y_SYS_ROW1 - 2, 72, 18, C_BLACK);

  gfx->setTextSize(1);
  if (usbMode) {
    gfx->setTextColor(V_RED, C_BLACK);
    gfx->setCursor(SECTION_X0 + 2, Y_SYS_ROW1);
    gfx->print("USB PWR");
  } else {
    gfx->setTextColor(V_WHITE, C_BLACK);
    gfx->setCursor(SECTION_X0 + 2, Y_SYS_ROW1);
    gfx->print("BAT");

    drawBatteryIconTiny(SECTION_X0 + 24, Y_SYS_ROW1 - 1, batValid, pct, charging);

    uint16_t pc = batValid ? (charging ? V_CYAN : colorByPercent(pct)) : V_RED;
    gfx->setTextColor(pc, C_BLACK);
    gfx->setCursor(SECTION_X0 + 41, Y_SYS_ROW1);
    if (batValid) {
      gfx->print(pct);
      gfx->print("%");
    } else {
      gfx->print("NO");
    }

    drawChargeIconTiny(SECTION_X1 - 8, Y_SYS_ROW1 - 2, batValid && charging);
  }

  // BL row is independent and always shown.
  gfx->fillRect(SECTION_X0, Y_SYS_ROW2 - 1, 54, 9, C_BLACK);
  gfx->setTextColor(g_blOn ? V_CYAN : V_RED, C_BLACK);
  gfx->setCursor(SECTION_X0 + 2, Y_SYS_ROW2);
  gfx->print("BL ");
  if (g_blOn) {
    gfx->print(currentBacklightPercent());
    gfx->print("%");
  } else {
    gfx->print("OFF");
  }
}

// ========================= LCD/UI =========================

static bool initLcd() {
  applyBacklight();

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
  Serial.println("[LCD] OK 0.96 ST7789, flat dashboard UI");
  return true;
}

static void drawSectionLabel(int x, int y, uint16_t color, const char *label) {
  if (!g_lcdOk) return;
  acquireForLcd();
  gfx->setTextSize(1);
  gfx->setTextColor(color, C_BLACK);
  gfx->setCursor(x, y);
  gfx->print(label);
}

static void drawSectionDivider(int y, uint16_t color) {
  if (!g_lcdOk) return;
  acquireForLcd();
  gfx->drawFastHLine(SECTION_X0 + 1, y, 70, color);
}

static void drawVuFrame() {
  if (!g_lcdOk) return;
  acquireForLcd();
  gfx->drawRoundRect(VU_X, VU_Y, VU_W, VU_H, 2, V_GREEN);
}

static void drawHeader() {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->fillRect(0, 0, LCD_W, 32, C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(V_GREEN, C_BLACK);
  gfx->setCursor(6, ROW_TITLE);
  gfx->print(g_headerSeeed ? "XIAO" : "Hello");

  gfx->setTextSize(1);
  gfx->setTextColor(V_CYAN, C_BLACK);
  gfx->setCursor(4, ROW_SUB);
  gfx->print("0.96 Display");

  gfx->drawFastHLine(5, 31, 70, V_WHITE);
}

static void drawStaticLayout() {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->fillScreen(C_BLACK);
  cleanScreenEdges();
  drawHeader();

  drawSectionLabel(SECTION_X0 + 1, Y_SYS_LABEL, V_CYAN, "SYSTEM");
  drawSectionDivider(Y_SYS_DIV, V_WHITE);

  drawSectionLabel(SECTION_X0 + 1, Y_MOTION_LABEL, V_YELLOW, "MOTION");
  drawSectionDivider(Y_MOTION_DIV, V_WHITE);

  drawSectionLabel(SECTION_X0 + 1, Y_MIC_LABEL, V_GREEN, "MIC");
  drawVuFrame();

    gfx->drawFastHLine(5, 147, 70, V_WHITE);
  drawTinyText(6, ROW_FOOT, "XIAO nRF52840 PLUS", V_YELLOW);

  cache_header = "";
  cache_sys = "";
  cache_bl = "";
  cache_acc = "";
  cache_gyr = "";
  cache_tap = "";
  cache_btn = "";
  cache_raw = "";
  cache_ax_txt = "";
  cache_ay_txt = "";
  cache_az_txt = "";
  cache_gx_txt = "";
  cache_gy_txt = "";
  cache_gz_txt = "";
  g_imuTinyLabelsDrawn = false;
  g_cachedVuSegments = -1;
  g_cachedVuWidth = -1;
  g_cachedVuColor = 0xFFFF;
}

// ========================= IMU =========================
//
// v1.2.9:
// The standalone IMU test that works on this hardware uses raw I2C detection/read,
// not the old SparkFun-only dashboard path. Port that working path into Dashboard.
// Also update the MCU-side INT pin from old D14 to D14.

static constexpr uint8_t LSM6DS3_ADDR_A      = 0x6A;
static constexpr uint8_t LSM6DS3_ADDR_B      = 0x6B;

static constexpr uint8_t LSM_REG_WHO_AM_I    = 0x0F;
static constexpr uint8_t LSM_REG_TAP_SRC     = 0x1C;
static constexpr uint8_t LSM_REG_CTRL1_XL    = 0x10;
static constexpr uint8_t LSM_REG_CTRL2_G     = 0x11;
static constexpr uint8_t LSM_REG_CTRL3_C     = 0x12;
static constexpr uint8_t LSM_REG_TAP_CFG     = 0x58;
static constexpr uint8_t LSM_REG_TAP_THS_6D  = 0x59;
static constexpr uint8_t LSM_REG_INT_DUR2    = 0x5A;
static constexpr uint8_t LSM_REG_WAKE_UP_THS = 0x5B;
static constexpr uint8_t LSM_REG_MD1_CFG     = 0x5E;

static int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;

  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t *val) {
  return i2cRead(addr, reg, val, 1);
}

static bool imuWriteReg(uint8_t reg, uint8_t val) {
  if (!g_imuAddr) return false;
  return i2cWrite8(g_imuAddr, reg, val);
}

static bool imuReadReg(uint8_t reg, uint8_t &val) {
  if (!g_imuAddr) return false;
  return i2cRead8(g_imuAddr, reg, &val);
}

void imuIntIsr() {
  g_imuIntFlag = true;
}

static bool initLsm6(uint8_t addr) {
  uint8_t who = 0;
  if (!i2cRead8(addr, LSM_REG_WHO_AM_I, &who)) return false;

  // Common LSM6 family WHO_AM_I values.
  if (who != 0x69 && who != 0x6A && who != 0x6C) return false;

  // Basic data path setup.
  // BDU=1, IF_INC=1.
  i2cWrite8(addr, LSM_REG_CTRL3_C, 0x44);
  // Accel: ODR 416 Hz, +/-2g.
  i2cWrite8(addr, LSM_REG_CTRL1_XL, 0x60);
  // Gyro: ODR 416 Hz, 2000 dps.
  i2cWrite8(addr, LSM_REG_CTRL2_G, 0x6C);

  // Double-tap config routed to INT1. MCU side listens on updated D14.
  bool tapOk = true;
  tapOk &= i2cWrite8(addr, LSM_REG_TAP_CFG, 0x8E);
  tapOk &= i2cWrite8(addr, LSM_REG_TAP_THS_6D, 0x0C);
  tapOk &= i2cWrite8(addr, LSM_REG_INT_DUR2, 0x7F);
  tapOk &= i2cWrite8(addr, LSM_REG_WAKE_UP_THS, 0x80);
  tapOk &= i2cWrite8(addr, LSM_REG_MD1_CFG, 0x08);

  uint8_t dummy = 0;
  i2cRead8(addr, LSM_REG_TAP_SRC, &dummy);

  g_imuType = IMU_LSM6;
  g_imuAddr = addr;
  g_imuOk = true;

  Serial.print("[IMU] LSM6 raw-I2C OK addr=0x");
  Serial.print(addr, HEX);
  Serial.print(" WHO=0x");
  Serial.print(who, HEX);
  Serial.print(" tap=");
  Serial.println(tapOk ? "OK" : "WARN");

  return true;
}

static bool initQmi8658(uint8_t addr) {
  uint8_t who = 0;
  if (!i2cRead8(addr, 0x00, &who)) return false;
  if (who == 0x00 || who == 0xFF) return false;

  // Minimal QMI8658 setup, same direction as the working standalone test.
  i2cWrite8(addr, 0x02, 0x60);  // CTRL1
  i2cWrite8(addr, 0x03, 0x03);  // CTRL2 accel config
  i2cWrite8(addr, 0x04, 0x53);  // CTRL3 gyro config
  i2cWrite8(addr, 0x08, 0x03);  // CTRL7 enable accel + gyro
  delay(30);

  uint8_t data[12] = {};
  if (!i2cRead(addr, 0x35, data, sizeof(data))) return false;

  g_imuType = IMU_QMI8658;
  g_imuAddr = addr;
  g_imuOk = true;

  Serial.print("[IMU] QMI8658 raw-I2C OK addr=0x");
  Serial.print(addr, HEX);
  Serial.print(" REG0x00=0x");
  Serial.print(who, HEX);
  Serial.println(" INT level-only");

  return true;
}

static bool initImuDoubleTap() {
  if (IMU_INT_PIN_SAFE) {
    pinMode(IMU_INT_PIN, INPUT_PULLDOWN);
    detachInterrupt(digitalPinToInterrupt(IMU_INT_PIN));
  } else {
    // Keep battery monitor safe. Do not touch pin 14 because it is READ_BAT.
    Serial.println("[IMU] INT pin conflict detected; tap interrupt disabled to protect READ_BAT");
  }

  g_imuType = IMU_NONE;
  g_imuAddr = 0;
  g_imuOk = false;

  const uint8_t candidates[] = {LSM6DS3_ADDR_A, LSM6DS3_ADDR_B};

  for (uint8_t i = 0; i < sizeof(candidates); i++) {
    uint8_t addr = candidates[i];
    if (!i2cPing(addr)) continue;

    if (initLsm6(addr) || initQmi8658(addr)) {
      if (IMU_INT_PIN_SAFE) {
        attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIntIsr, RISING);
      }
      cache_acc = "";
      cache_gyr = "";
      cache_tap = "";
      g_imuTinyLabelsDrawn = false;
      return true;
    }

    g_imuType = IMU_UNKNOWN;
    g_imuAddr = addr;
    Serial.print("[IMU] ACK but unknown type at 0x");
    Serial.println(addr, HEX);
  }

  Serial.println("[IMU] not found at 0x6A/0x6B");
  return false;
}

static void updateImu() {
  if (!g_imuOk) return;

  uint8_t data[12] = {};

  if (g_imuType == IMU_LSM6) {
    // LSM6 OUTX_L_G starts at 0x22:
    // gyro X/Y/Z, then accel X/Y/Z.
    if (!i2cRead(g_imuAddr, 0x22, data, sizeof(data))) {
      g_imuOk = false;
      Serial.println("[IMU] LSM6 read failed");
      return;
    }

    int16_t gxRaw = le16(&data[0]);
    int16_t gyRaw = le16(&data[2]);
    int16_t gzRaw = le16(&data[4]);
    int16_t axRaw = le16(&data[6]);
    int16_t ayRaw = le16(&data[8]);
    int16_t azRaw = le16(&data[10]);

    g_ax = axRaw * 0.000061f;  // +/-2g
    g_ay = ayRaw * 0.000061f;
    g_az = azRaw * 0.000061f;
    g_gx = gxRaw * 0.070f;     // 2000 dps
    g_gy = gyRaw * 0.070f;
    g_gz = gzRaw * 0.070f;
    return;
  }

  if (g_imuType == IMU_QMI8658) {
    if (!i2cRead(g_imuAddr, 0x35, data, sizeof(data))) {
      g_imuOk = false;
      Serial.println("[IMU] QMI8658 read failed");
      return;
    }

    int16_t axRaw = le16(&data[0]);
    int16_t ayRaw = le16(&data[2]);
    int16_t azRaw = le16(&data[4]);
    int16_t gxRaw = le16(&data[6]);
    int16_t gyRaw = le16(&data[8]);
    int16_t gzRaw = le16(&data[10]);

    g_ax = axRaw / 16384.0f;
    g_ay = ayRaw / 16384.0f;
    g_az = azRaw / 16384.0f;
    g_gx = gxRaw / 16.4f;
    g_gy = gyRaw / 16.4f;
    g_gz = gzRaw / 16.4f;
  }
}

static void handleImuTapEvent() {
  if (!IMU_INT_PIN_SAFE) return;

  bool shouldCheck = false;
  noInterrupts();
  if (g_imuIntFlag) {
    g_imuIntFlag = false;
    shouldCheck = true;
  }
  interrupts();

  if (digitalRead(IMU_INT_PIN) == HIGH) shouldCheck = true;
  if (!shouldCheck || !g_imuOk) return;

  // Double-tap source is only configured for LSM6.
  if (g_imuType != IMU_LSM6) return;

  uint8_t src = 0;
  if (!imuReadReg(LSM_REG_TAP_SRC, src)) return;

  if (src & 0x10) {
    uint32_t now = millis();
    if (now - g_lastTapMs > TAP_DEBOUNCE_MS) {
      g_lastTapMs = now;
      g_doubleTapCount++;
      cache_tap = "";
      Serial.print("[TAP] double tap count=");
      Serial.print(g_doubleTapCount);
      Serial.print(" src=0x");
      Serial.println(src, HEX);
    }
  }
}

// ========================= MIC =========================

static uint16_t currentMicPeak() {
  uint32_t now = millis();
  if (now - g_micLastUpdateMs > MIC_DECAY_MS) g_micPeak = (uint16_t)(g_micPeak * 0.75f);
  return g_micPeak;
}

void onPdmReceive() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;
  if (bytesAvailable > (int)sizeof(g_pdmBuf)) bytesAvailable = sizeof(g_pdmBuf);
  int bytesRead = PDM.read((void *)g_pdmBuf, bytesAvailable);
  if (bytesRead <= 0) return;
  uint32_t peak = 0;
  uint64_t sumSq = 0;
  int samples = bytesRead / 2;
  for (int i = 0; i < samples; i++) {
    int32_t v = g_pdmBuf[i];
    int32_t a = abs(v);
    if ((uint32_t)a > peak) peak = (uint32_t)a;
    sumSq += (uint64_t)((int64_t)v * (int64_t)v);
  }
  uint32_t rms = 0;
  if (samples > 0) rms = (uint32_t)sqrt((double)sumSq / (double)samples);
  g_micPeak = (uint16_t)min<uint32_t>(peak, 65535);
  g_micRms = rms;
  g_micBlocks++;
  g_micLastUpdateMs = millis();
}

static bool initMic() {
  PDM.setPins(PDM_DATA_PIN, PDM_CLK_PIN, -1);
  PDM.onReceive(onPdmReceive);
  PDM.setBufferSize(sizeof(g_pdmBuf));
  PDM.setGain(MIC_GAIN);
  if (!PDM.begin(MIC_CHANNELS, MIC_SAMPLE_RATE_HZ)) {
    Serial.println("[MIC] PDM.begin failed");
    g_micOk = false;
    return false;
  }
  g_micOk = true;
  Serial.println("[MIC] OK PDM DATA=D1 CLK=D0");
  return true;
}

static uint16_t vuColorForSegment(int idx) {
  // 1.14-style block colors: green -> yellow -> red.
  if (idx >= 6) return V_RED;
  if (idx >= 4) return V_YELLOW;
  return V_GREEN;
}

static int vuLevelToSegments(float level) {
  // v1.2.6: thresholds are intentionally wider than v1.2.
  // This prevents 1-block jitter when the signal sits around a boundary.
  if (level < 0.13f) return 0;
  if (level < 0.25f) return 1;
  if (level < 0.38f) return 2;
  if (level < 0.51f) return 3;
  if (level < 0.64f) return 4;
  if (level < 0.77f) return 5;
  if (level < 0.88f) return 6;
  if (level < 0.96f) return 7;
  return 8;
}

static void drawVuBlocks(int active) {
  if (!g_lcdOk) return;

  active = constrain(active, 0, VU_SEG_COUNT);

  acquireForLcd();

  // First draw: clear and draw the frame once.
  if (g_cachedVuSegments < 0) {
    gfx->fillRect(VU_X, VU_Y, VU_W + 1, VU_H + 1, C_BLACK);
    gfx->drawRoundRect(VU_X, VU_Y, VU_W, VU_H, 2, V_GREEN);

    for (int i = 0; i < VU_SEG_COUNT; i++) {
      int x = VU_X + 1 + i * (VU_SEG_W + VU_GAP);
      uint16_t c = (i < active) ? vuColorForSegment(i) : C_BLACK;
      gfx->fillRect(x, VU_Y + 2, VU_SEG_W, VU_H - 4, c);
    }

    g_cachedVuSegments = active;
    g_cachedVuWidth = active;
    g_cachedVuColor = (active > 0) ? vuColorForSegment(active - 1) : V_GREEN;
    g_lastVuRenderMs = millis();
    return;
  }

  // Delta update only. Do not clear/redraw the whole VU each time.
  // This is the key fix for visible flicker.
  if (active > g_cachedVuSegments) {
    for (int i = g_cachedVuSegments; i < active; i++) {
      int x = VU_X + 1 + i * (VU_SEG_W + VU_GAP);
      gfx->fillRect(x, VU_Y + 2, VU_SEG_W, VU_H - 4, vuColorForSegment(i));
    }
  } else if (active < g_cachedVuSegments) {
    for (int i = active; i < g_cachedVuSegments; i++) {
      int x = VU_X + 1 + i * (VU_SEG_W + VU_GAP);
      gfx->fillRect(x, VU_Y + 2, VU_SEG_W, VU_H - 4, C_BLACK);
    }
  }

  g_cachedVuSegments = active;
  g_cachedVuWidth = active;
  g_cachedVuColor = (active > 0) ? vuColorForSegment(active - 1) : V_GREEN;
  g_lastVuRenderMs = millis();
}

static int debounceVuDesired(int desired, uint32_t now) {
  desired = constrain(desired, 0, VU_SEG_COUNT);

  int current = g_vuDisplaySegments;
  int delta = desired - current;

  if (desired == current) {
    g_vuCandidateSegments = desired;
    g_vuCandidateCount = 0;
    return current;
  }

  // During decay, raw/RMS may bounce upward by 1~2 blocks.
  // Visually this is the reverse pulse: falling -> suddenly grows -> falls.
  // Suppress only small rebounds. A real loud input can still break through.
  if (delta > 0 && now < g_vuFallGuardUntilMs) {
    // v1.2.6: suppress only tiny +1 rebounds during decay.
    // +2 or larger is treated as a new intentional sound and can rise.
    if (delta <= 1) {
      return current;
    }
  }

  if (desired != g_vuCandidateSegments) {
    g_vuCandidateSegments = desired;
    g_vuCandidateCount = 1;
  } else if (g_vuCandidateCount < 255) {
    g_vuCandidateCount++;
  }

  bool bigJump = abs(delta) >= 3;
  bool strongRise = delta >= 2;
  uint32_t minInterval = (delta > 0) ? 45UL : 80UL;

  if ((now - g_lastVuStepMs) < minInterval && !bigJump && !strongRise) {
    return current;
  }

  uint8_t neededCount = (bigJump || strongRise) ? 1 : 2;
  if (g_vuCandidateCount < neededCount) {
    return current;
  }

  int next = current;

  if (delta > 0) {
    // v1.2.6: rise up to 3 blocks per commit for a snappier response.
    next += min(3, delta);
    g_vuFallGuardUntilMs = 0;
  } else if (delta < 0) {
    next -= 1;
    g_vuFallGuardUntilMs = now + 260UL;
  }

  next = constrain(next, 0, VU_SEG_COUNT);
  g_lastVuStepMs = now;
  return next;
}

static void updateVu() {
  if (!g_lcdOk) return;

  uint16_t peak = currentMicPeak();

  // v1.2.6 block VU:
  // RMS remains the main source. Peak only helps attack slightly.
  float metric = (float)g_micRms * 0.82f + (float)peak * 0.012f;
  if (metric < 0.0f) metric = 0.0f;
  if (metric > 240.0f) metric = 240.0f;

  uint32_t now = millis();

  if (!g_micNoiseReady) {
    if (g_micNoiseStartMs == 0) g_micNoiseStartMs = now;
    g_micNoiseFloor = g_micNoiseFloor * 0.84f + metric * 0.16f;
    if (now - g_micNoiseStartMs > 1200UL) g_micNoiseReady = true;
  } else {
    // Track ambient floor. Do not chase short upward speech spikes.
    if (metric < g_micNoiseFloor) {
      g_micNoiseFloor = g_micNoiseFloor * 0.84f + metric * 0.16f;
    } else if (metric < g_micNoiseFloor + 5.0f) {
      g_micNoiseFloor = g_micNoiseFloor * 0.992f + metric * 0.008f;
    }
  }

  float signal = metric - g_micNoiseFloor - 26.0f;
  if (signal < 0.0f) signal = 0.0f;

  float target = signal / 108.0f;
  if (target > 1.0f) target = 1.0f;

  // v1.2.6: faster attack while keeping fast release.
  if (target > g_vuFast) {
    g_vuFast = g_vuFast * 0.35f + target * 0.65f;
  } else {
    g_vuFast = g_vuFast * 0.22f + target * 0.78f;
  }

  if (g_vuFast > g_vuSmooth) {
    // More responsive visible rise than v1.2.6.
    g_vuSmooth = g_vuSmooth * 0.58f + g_vuFast * 0.42f;
  } else {
    // Keep falling quick.
    g_vuSmooth = g_vuSmooth * 0.34f + g_vuFast * 0.66f;
  }

  if (g_vuSmooth < 0.055f) g_vuSmooth = 0.0f;

  int desired = vuLevelToSegments(g_vuSmooth);
  int next = debounceVuDesired(desired, now);

  if (next == g_cachedVuSegments) return;

  g_vuDisplaySegments = next;
  drawVuBlocks(next);
}

// ========================= UI updates =========================

static String fmtAxisInt(float v) {
  int iv = (int)roundf(v);
  if (iv > 99) iv = 99;
  if (iv < -99) iv = -99;
  char buf[6];
  snprintf(buf, sizeof(buf), "%+d", iv);
  return String(buf);
}

static String fmtAxis1Dec(float v) {
  // 0.96 panel width is tight. Clamp to +/-9.9 and use one decimal.
  if (v > 9.9f) v = 9.9f;
  if (v < -9.9f) v = -9.9f;
  if (fabsf(v) < 0.05f) v = 0.0f;

  char buf[8];
  snprintf(buf, sizeof(buf), "%.1f", v);
  return String(buf);
}

static String formatTapText(uint32_t count) {
  if (count <= 99) return String("TAP ") + String(count);
  return String("T99+");
}

static void updateUiFast() {
  if (!g_lcdOk) return;

  String header = g_headerSeeed ? "XIAO" : "Hello";
  if (header != cache_header) {
    cache_header = header;
    drawHeader();
  }

  updateBatteryUiSnapshot();

  String sys = g_batUiUsb ? String("USB") :
               (String("BAT:") +
                String(g_batUiValid ? g_batUiPct : -1) +
                String(g_batUiValid ? ":V" : ":X") +
                String(g_batUiCharging ? ":C" : ":N"));
  String bl = backlightStatusText();

  if (sys != cache_sys || bl != cache_bl) {
    cache_sys = sys;
    cache_bl = bl;
    drawBatteryRowTiny();
  }

  String tap = formatTapText(g_doubleTapCount);
  if (tap != cache_tap) {
    cache_tap = tap;
    // Match the ESP32-S3 0.96 small-font UI: uppercase TAP, clear gap after MOTION.
    acquireForLcd();
    gfx->fillRect(45, Y_TAP, 32, 8, C_BLACK);
    drawTinyTextNoClear(48, Y_TAP + 2, tap.c_str(), V_YELLOW);
  }

  uint16_t peak = currentMicPeak();
  String raw = String("Raw ") + String((unsigned)peak);
  if (raw != cache_raw) {
    cache_raw = raw;
    uint16_t c = V_WHITE;
    if (g_vuDisplaySegments >= 6) c = V_RED;
    else if (g_vuDisplaySegments >= 4) c = V_YELLOW;
    printTextFixed(SECTION_X0 + 1, Y_RAW, c, raw, 10);
  }
}

static void updateUiSlow() {
  if (!g_lcdOk) return;
  cleanScreenEdges();

  if (g_imuOk) {
    if (!g_imuTinyLabelsDrawn) {
      acquireForLcd();
      gfx->fillRect(SECTION_X0 + 1, Y_ACC, 76, 8, C_BLACK);
      gfx->fillRect(SECTION_X0 + 1, Y_GYR, 76, 8, C_BLACK);
      drawTiny3x5Field(SECTION_X0 + 1, Y_ACC + 1, String("A"), V_YELLOW, 1);
      drawTiny3x5Field(SECTION_X0 + 1, Y_GYR + 1, String("G"), V_YELLOW, 1);
      g_imuTinyLabelsDrawn = true;
      cache_ax_txt = cache_ay_txt = cache_az_txt = "";
      cache_gx_txt = cache_gy_txt = cache_gz_txt = "";
    }

    // Same small-font format as the ESP32-S3 0.96 UI:
    // A 0.0 0.9 -0.2 / G 0.0 0.1 -0.4
    // Redraw by field, not by full line, to reduce flicker.
    String axTxt = fmtAxis1Dec(g_ax);
    String ayTxt = fmtAxis1Dec(g_ay);
    String azTxt = fmtAxis1Dec(g_az);
    if (axTxt != cache_ax_txt) {
      cache_ax_txt = axTxt;
      drawTiny3x5Field(14, Y_ACC + 1, axTxt, V_WHITE, 4);
    }
    if (ayTxt != cache_ay_txt) {
      cache_ay_txt = ayTxt;
      drawTiny3x5Field(35, Y_ACC + 1, ayTxt, V_WHITE, 4);
    }
    if (azTxt != cache_az_txt) {
      cache_az_txt = azTxt;
      drawTiny3x5Field(56, Y_ACC + 1, azTxt, V_WHITE, 4);
    }

    String gxTxt = fmtAxis1Dec(g_gx / 10.0f);
    String gyTxt = fmtAxis1Dec(g_gy / 10.0f);
    String gzTxt = fmtAxis1Dec(g_gz / 10.0f);
    if (gxTxt != cache_gx_txt) {
      cache_gx_txt = gxTxt;
      drawTiny3x5Field(14, Y_GYR + 1, gxTxt, V_WHITE, 4);
    }
    if (gyTxt != cache_gy_txt) {
      cache_gy_txt = gyTxt;
      drawTiny3x5Field(35, Y_GYR + 1, gyTxt, V_WHITE, 4);
    }
    if (gzTxt != cache_gz_txt) {
      cache_gz_txt = gzTxt;
      drawTiny3x5Field(56, Y_GYR + 1, gzTxt, V_WHITE, 4);
    }
  } else {
    if (cache_acc != "NOIMU") {
      cache_acc = "NOIMU";
      cache_gyr = "";
      acquireForLcd();
      gfx->fillRect(SECTION_X0 + 1, Y_ACC, 76, 20, C_BLACK);
      drawTinyTextNoClear(SECTION_X0 + 1, Y_ACC + 1, "IMU NO", V_RED);
      g_imuTinyLabelsDrawn = false;
      cache_ax_txt = cache_ay_txt = cache_az_txt = "";
      cache_gx_txt = cache_gy_txt = cache_gz_txt = "";
    }
  }
}

static void printSerialStatus() {
  Serial.print("[DASH096] bat="); Serial.print(g_bat.vbat, 3);
  Serial.print("V pct="); Serial.print(g_bat.percent);
  Serial.print(" uiPct="); Serial.print(g_batUiValid ? g_batUiPct : -1);
  Serial.print(" valid="); Serial.print(g_bat.valid ? "Y" : "N");
  Serial.print(" chg="); Serial.print(g_bat.charging ? "Y" : "N");
  Serial.print(" batState="); Serial.print(batStateName(g_batState));
  Serial.print(" filt="); Serial.print(g_batFilterState);
  Serial.print(" raw="); Serial.print(g_bat.raw);
  Serial.print(" spread="); Serial.print((int)(g_bat.rawMax - g_bat.rawMin));
  Serial.print(" chgRaw="); Serial.print(g_chgRawLow ? "L" : "H");
  Serial.print(" ins="); Serial.print(g_insertCandidateStreak);
  Serial.print(" imu="); Serial.print(g_imuOk ? "OK" : "NO");
  Serial.print(" imuType=");
  if (g_imuType == IMU_LSM6) Serial.print("LSM6");
  else if (g_imuType == IMU_QMI8658) Serial.print("QMI8658");
  else if (g_imuType == IMU_UNKNOWN) Serial.print("UNKNOWN");
  else Serial.print("NONE");
  Serial.print(" imuAddr=0x"); Serial.print(g_imuAddr, HEX);
  Serial.print(" intD14=");
  if (IMU_INT_PIN_SAFE) Serial.print(digitalRead(IMU_INT_PIN) == HIGH ? "H" : "L");
  else Serial.print("NA");
  Serial.print(" intPin="); Serial.print(IMU_INT_PIN);
  Serial.print(" batPinSafe="); Serial.print(IMU_INT_PIN_SAFE ? "Y" : "N_CONFLICT");
  Serial.print(" acc=("); Serial.print(g_ax, 2); Serial.print(","); Serial.print(g_ay, 2); Serial.print(","); Serial.print(g_az, 2); Serial.print(")");
  Serial.print(" gyr=("); Serial.print(g_gx, 2); Serial.print(","); Serial.print(g_gy, 2); Serial.print(","); Serial.print(g_gz, 2); Serial.print(")");
  Serial.print(" micPeak="); Serial.print((unsigned)currentMicPeak());
  Serial.print(" micRms="); Serial.print((unsigned long)g_micRms);
  Serial.print(" micFloor="); Serial.print(g_micNoiseFloor, 1);
  Serial.print(" vu="); Serial.print(g_vuSmooth, 2);
  Serial.print(" fast="); Serial.print(g_vuFast, 2);
  Serial.print(" vuSeg="); Serial.print(g_vuDisplaySegments);
  Serial.print(" vuCand="); Serial.print(g_vuCandidateSegments);
  Serial.print("/"); Serial.print(g_vuCandidateCount);
  Serial.print(" fallGuard="); Serial.print((long)(g_vuFallGuardUntilMs - millis()));
  Serial.print(" usr1="); Serial.print(g_btnA ? "P" : "R");
  Serial.print(" usr2="); Serial.print(g_btnB ? "P" : "R");
  Serial.print(" tap="); Serial.print((unsigned long)g_doubleTapCount);
  Serial.print(" bl="); Serial.print(g_blOn ? "ON" : "OFF");
  Serial.print(" blLevel="); Serial.print(backlightStatusText());
  Serial.print(" frame="); Serial.println((unsigned long)g_frameCounter++);
}

// ========================= Arduino =========================

void setup() {
  Serial.begin(115200);
  delay(800);
  pinMode(BTN_A_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);
  pinMode(LCD_BL_PIN, OUTPUT);
  g_btnARaw = digitalRead(BTN_A_PIN);
  g_btnBRaw = digitalRead(BTN_B_PIN);
  g_btnALastRawPressed = (g_btnARaw == LOW);
  g_btnBLastRawPressed = (g_btnBRaw == LOW);
  g_btnA = g_btnALastRawPressed;
  g_btnB = g_btnBLastRawPressed;
  g_btnAArmed = !g_btnA;
  g_btnBArmed = !g_btnB;
  g_btnALastChangeMs = millis();
  g_btnBLastChangeMs = millis();
  attachInterrupt(digitalPinToInterrupt(BTN_A_PIN), btnAIrqIsr, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_B_PIN), btnBIrqIsr, FALLING);
  Wire.begin();
  nrf_gpio_cfg_input(CHG_P0_PIN, NRF_GPIO_PIN_PULLUP);
  sampleChargingRawLow();
  analogReadResolution(ADC_BITS);
  disableBatteryDivider();

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 0.96 Factory Dashboard v1.2.14 exact 1.14 battery logic ===");
  Serial.print("[PIN] IMU_INT D14 Arduino pin="); Serial.println(IMU_INT_PIN);
  Serial.println("[LCD] ST7789 80x160 rotation=2 offset=24,0,24,0 invert=Y");
  Serial.println("[BTN] KEY1/USR1=Brightness cycle, KEY2/USR2=Backlight ON/OFF");
  Serial.print("[PIN] PIN_VBAT="); Serial.println(PIN_VBAT);

  g_micNoiseStartMs = millis();
  g_micNoiseReady = false;
  g_micNoiseFloor = 65.0f;
  g_vuSmooth = 0.0f;
  g_vuFast = 0.0f;
  g_vuDisplaySegments = 0;
  g_vuCandidateSegments = 0;
  g_vuCandidateCount = 0;
  g_vuFallGuardUntilMs = 0;
  g_cachedVuSegments = -1;
  g_cachedVuWidth = -1;
  g_cachedVuColor = 0xFFFF;
  g_lastVuStepMs = millis();
  g_lastVuRenderMs = millis();

  initLcd();
  drawStaticLayout();
  initMic();
  initImuDoubleTap();
  updateBattery();
  updateImu();
  updateUiFast();
  updateUiSlow();
  updateVu();
  printSerialStatus();
}

void loop() {
  uint32_t now = millis();
  handleImuTapEvent();
  updateButtons();
  handleButtonActions();
  if (now - g_lastBatMs >= BAT_REFRESH_MS) { g_lastBatMs = now; updateBattery(); }
  if (now - g_lastVuMs >= UI_VU_MS) { g_lastVuMs = now; updateVu(); }
  if (now - g_lastFastMs >= UI_FAST_MS) { g_lastFastMs = now; updateUiFast(); }
  if (now - g_lastSlowMs >= UI_SLOW_MS) {
    g_lastSlowMs = now;
    if (!g_imuOk && (now % 2000UL) < UI_SLOW_MS) initImuDoubleTap();
    updateImu();
    updateUiSlow();
  }
  if (now - g_lastSerialMs >= SERIAL_MS) { g_lastSerialMs = now; printSerialStatus(); }
  delay(2);
}
