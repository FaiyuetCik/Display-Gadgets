/*
  XIAO ESP32-S3 Plus + 1.47 Touch Display
  Factory Dashboard v7.14 - D16 live sense + SD scheduler

  Based on:
    - 0519_DashBoard_147_ESP32.ino
    - 0522_DashBoard_147_nRF52840.ino

  Goals:
    - Keep the nRF52840 Plus 1.47 Dashboard visual style and card layout.
    - Keep ESP32-S3 peripherals from the previous 0519 bring-up:
        LCD, Touch, SD probe, IMU, PDM mic, USR1/USR2.
    - Add ESP32-S3 battery measurement:
        VBAT -- 316K -- ADC node -- 160K -- GND
        ratio = (316K + 160K) / 160K = 2.975
    - Product title differs only by board name:
        XIAO ESP32-S3 Plus

  Notes:
    - This is the non-camera dashboard build.
    - 1.47 LCD keeps the proven ST7789-compatible JD9853A path:
        IPS=false, 172x320, offset 34/0/34/0, MADCTL 0x48.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR >= 5
  #include <driver/i2s_pdm.h>
#else
  #include <driver/i2s.h>
#endif

#include <driver/gpio.h>
#include <math.h>
#include <stdarg.h>
#include <esp_log.h>
#include "esp_lcd_touch_axs5106l.h"

// ========================= Build switches =========================

#define USE_PRD_PINMAP 1
#define LCD_USE_ESP32SPI 0
#define REDUCE_PDM_CLK_DRIVE 1

// Critical for the live dynamic dashboard:
// SD and LCD share D8/D10. SD.begin()/SD.open() can reconfigure or hold those pins.
// In v4.5, SD is probed once, then SD.end() is called immediately.
// No automatic SD file write is done at boot.
#define SD_PROBE_ONLY_MODE 1
#define SD_BUTTON_WRITE_TEST 0
#define g_sdBusy g_sdProbeBusy

// ========================= Timing =========================

static constexpr uint32_t UI_TEXT_FAST_MS   = 130;
static constexpr uint32_t UI_TEXT_SLOW_MS   = 220;
static constexpr uint32_t UI_VU_MS          = 75;
static constexpr uint32_t UI_MIC_TEXT_MS    = 360;
static constexpr uint32_t SERIAL_MS         = 1000;
static constexpr uint32_t SD_REFRESH_MS     = 1200;   // automatic SD quick-probe interval
static constexpr uint32_t SD_FULL_RETRY_COOLDOWN_MS = 6000;  // avoid repeated full SD.begin() on false positives
static constexpr uint32_t SD_LCD_IDLE_GAP_MS = 10;   // start SD probe only after LCD has been idle briefly
static constexpr uint32_t BAT_REFRESH_MS    = 250;
static constexpr uint32_t VSENSE_UI_MS      = 500;    // D16 live display check interval
static constexpr float    VSENSE_D16_DELTA_V  = 0.005f; // live D16 redraw threshold
static constexpr float    VSENSE_CALC_DELTA_V = 0.015f; // live calculated voltage redraw threshold
static constexpr uint32_t TAP_DEBOUNCE_MS   = 220;
static constexpr uint32_t MIC_DECAY_MS      = 60;
static constexpr uint32_t FOOTER_MS         = 500;

static constexpr uint32_t BTN_DEBOUNCE_MS = 18;
static constexpr uint32_t BTN_ACTION_LOCKOUT_MS = 70;

static const uint8_t BL_LEVELS[] = {255, 191, 128, 64, 0};
static constexpr uint8_t BL_LEVEL_COUNT = sizeof(BL_LEVELS) / sizeof(BL_LEVELS[0]);

// ========================= MIC config =========================

static constexpr int MIC_SAMPLE_RATE_HZ = 16000;
static constexpr bool MIC_CLK_INVERT = false;

// This is only display scaling, not recording gain.
static constexpr float MIC_DISPLAY_SCALE = 2200.0f;
static constexpr size_t MIC_SAMPLES_PER_READ = 256;

// ========================= Battery config =========================
//
// New ESP32-S3 hardware:
//   VBAT -- 316K -- ADC node -- 160K -- GND
// ratio = 476K / 160K = 2.975
//
// BAT_ADC_PIN is D16 from the 0519 ESP32-S3 Dashboard pin map.
// v7.3 policy:
//   BAT monitor only answers two questions:
//     1) Is a battery connected?
//     2) If connected, what is its approximate percentage?
//   No charging state is inferred or displayed.
// Because there is no CHG/STAT/VBUS/BAT_PRESENT pin, USB-only floating voltage
// cannot be perfectly distinguished from a real battery in every boot condition.
// This state machine is designed to be conservative for hotplug cases:
//   - reject impossible high jumps after a known real battery
//   - show USB PWR after battery removal under USB
//   - prevent USB insertion from making percentage jump instantly

static constexpr float BAT_R_TOP_KOHM        = 316.0f;
static constexpr float BAT_R_BOTTOM_KOHM     = 160.0f;
static constexpr float BAT_DIVIDER_RATIO     = (BAT_R_TOP_KOHM + BAT_R_BOTTOM_KOHM) / BAT_R_BOTTOM_KOHM;
static constexpr float BAT_CAL_FACTOR        = 1.000f;
static constexpr float BAT_VALID_MIN_V       = 2.80f;
static constexpr float BAT_VALID_MAX_V       = 4.60f;
static constexpr uint16_t BAT_PRESENT_MIN_MV = 850;  // ADC node mV, about 2.53V BAT after divider.
static constexpr uint16_t BAT_FLOAT_RANGE_MV = 90;   // large spread usually means floating/no battery.
static constexpr uint8_t BAT_VALID_CONFIRM_COUNT = 3;
static constexpr uint8_t BAT_INVALID_CONFIRM_COUNT = 3;

// Reject impossible high jump:
// Example: real battery 17% -> remove cell while USB is present -> ADC reports ~70%.
// That is not a real battery SoC jump, so classify it as USB/FLOAT.
static constexpr float BAT_IMPOSSIBLE_JUMP_V     = 0.28f;
static constexpr int   BAT_IMPOSSIBLE_JUMP_PCT   = 22;
static constexpr uint8_t BAT_FLOAT_REJECT_STREAK = 2;

// Reinsert unlock:
// After high-float rejection, accept battery again when measured voltage drops
// close to the last real battery voltage.
static constexpr float BAT_REINSERT_DROP_V       = 0.16f;

// Display percentage limiter.
// USB insertion may raise surface voltage. UI percentage must not jump upward.
// Falling percentage is immediate; rising percentage is slow.
static constexpr uint32_t BAT_PCT_RISE_INTERVAL_MS = 90000; // max +1% per 90s

// USB-only cold boot guard:
// With only BAT_ADC and no VBUS/STAT/BAT_PRESENT pin, USB-only floating voltage
// can look like a valid ~70% battery at boot. There is no perfect software-only
// truth source here. This guard is intentionally conservative for your observed
// case: if no real battery has ever been confirmed in this boot, a stable high
// reading above this percentage is treated as USB_PWR rather than BAT.
//
// Trade-off: a truly high-SOC battery-only cold boot may be classified as USB_PWR.
// If that use case matters more, set BAT_COLD_BOOT_HIGH_AS_USB to false or raise
// BAT_COLD_BOOT_SUSPECT_PCT.
static constexpr bool BAT_COLD_BOOT_HIGH_AS_USB = true;
static constexpr int  BAT_COLD_BOOT_SUSPECT_PCT = 60;

// USB-first then battery-insert compensation:
// Observed case:
//   1) USB-only boot -> USB PWR
//   2) insert battery while USB is still present -> displayed SoC jumps 27% -> 39%
//   3) unplug USB -> display drops back to 27%
// That means BAT ADC is seeing charger/surface voltage, not open-circuit battery voltage.
// Without VBUS/CHG/BAT_PRESENT pin, the best practical fix is to detect this path and
// subtract a conservative surface-voltage offset while USB compensation is active.
static constexpr float BAT_USB_INSERT_SURFACE_COMP_V = 0.085f; // about 8~12 percentage points around 30%.
static constexpr float BAT_USB_INSERT_RELEASE_DROP_V = 0.045f; // raw drop means USB removed or surface settled.
static constexpr float BAT_EFFECTIVE_MIN_V           = 3.25f;
// ========================= Pins =========================


static constexpr uint8_t MIC_CLK_PIN   = D0;
static constexpr uint8_t MIC_DATA_PIN  = D1;
static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t I2C_SDA_PIN   = D4;
static constexpr uint8_t I2C_SCL_PIN   = D5;
static constexpr uint8_t SD_CS_PIN     = D6;
static constexpr uint8_t TOUCH_INT_PIN = D7;
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t SD_MISO_PIN   = D9;
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t IMU_INT_PIN   = D14;
static constexpr uint8_t BTN_B_PIN     = D15;
static constexpr uint8_t BAT_ADC_PIN   = D16;

#if USE_PRD_PINMAP
static constexpr uint8_t LCD_RST_PIN   = D17;
static constexpr uint8_t BTN_A_PIN     = D19;
#else
static constexpr uint8_t LCD_RST_PIN   = D19;
static constexpr uint8_t BTN_A_PIN     = D17;
#endif

static constexpr uint8_t LCD_BL_PIN    = D18;

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
static constexpr uint16_t C_ACCENT  = 0x04FF;
static constexpr uint16_t C_PANEL_2 = 0x1082;
static constexpr uint16_t C_BLUE    = RGB565_BLUE;

// ========================= LCD object =========================

#if LCD_USE_ESP32SPI
Arduino_DataBus *lcdBus = new Arduino_ESP32SPI(
  LCD_DC_PIN, LCD_CS_PIN, LCD_SCK_PIN, LCD_MOSI_PIN
);
#else
Arduino_DataBus *lcdBus = new Arduino_SWSPI(
  LCD_DC_PIN, LCD_CS_PIN, LCD_SCK_PIN, LCD_MOSI_PIN, GFX_NOT_DEFINED
);
#endif

Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus, LCD_RST_PIN, 0, false, 172, 320, 34, 0, 34, 0
);

// ========================= I2S object =========================

#if ESP_IDF_VERSION_MAJOR >= 5
static i2s_chan_handle_t g_i2sRxChan = nullptr;
#else
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
#endif

// ========================= State / UI layout =========================

bool g_lcdOk = false;

bool g_sdMounted = false;
bool g_sdSessionOpen = false;
uint32_t g_sdOkFreq = 0;
uint64_t g_sdCardSizeMB = 0;
bool g_sdProbeDone = false;

volatile bool g_sdProbeBusy = false;
volatile bool g_sdProbeRequest = false;
volatile bool g_sdProbeResultReady = false;
volatile bool g_sdProbePresentResult = false;
volatile uint32_t g_sdProbeFreqResult = 0;
volatile uint32_t g_sdProbeSizeMbResult = 0;
volatile uint32_t g_lastLcdOpMs = 0;
uint32_t g_sdFullRetryHoldUntilMs = 0;
uint8_t g_sdInsertStable = 0;
uint8_t g_sdRemoveStable = 0;

enum ImuType { IMU_NONE = 0, IMU_QMI8658, IMU_LSM6DS3 };
ImuType g_imuType = IMU_NONE;
uint8_t g_imuAddr = 0;
bool g_imuOk = false;
float g_ax = 0, g_ay = 0, g_az = 0;
float g_gx = 0, g_gy = 0, g_gz = 0;

bool g_touchFound = false;
uint8_t g_touchAddr = 0;
bool g_touchValid = false;
int g_touchX = -1;
int g_touchY = -1;
int g_touchRawX = -1;
int g_touchRawY = -1;
int g_touchIntRaw = HIGH;
uint8_t g_touchRawBytes[8] = {0};
uint32_t g_touchLastSeenMs = 0;
int g_touchLastX = -1;
int g_touchLastY = -1;

bool g_btnA = false;
bool g_btnB = false;
int g_btnARaw = HIGH;
int g_btnBRaw = HIGH;

bool g_btnALastRawPressed = false;
bool g_btnBLastRawPressed = false;
uint32_t g_btnALastChangeMs = 0;
uint32_t g_btnBLastChangeMs = 0;
bool g_btnAPendingAction = false;
bool g_btnBPendingAction = false;
uint32_t g_btnALastQueueMs = 0;
uint32_t g_btnBLastQueueMs = 0;
uint32_t g_lastBtnActionMs = 0;

uint8_t g_blIndex = 0;
uint8_t g_blRestoreIndex = 0;

bool g_micOk = false;
uint16_t g_micPeak = 0;
uint32_t g_micRms = 0;
uint32_t g_micLastUpdateMs = 0;
int16_t g_pdmBuf[MIC_SAMPLES_PER_READ];

struct BatteryState {
  uint16_t raw = 0;
  uint16_t rawMin = 0;
  uint16_t rawMax = 0;
  uint16_t mv = 0;
  uint16_t mvMin = 0;
  uint16_t mvMax = 0;
  float vadc = 0.0f;
  float vbat = 0.0f;
  int percent = 0;
  bool charging = false;
  bool valid = false;
};

BatteryState g_bat;
BatteryState g_vsense;       // raw live D16 ADC measurement
BatteryState g_vsensePrev;   // previous live D16 measurement for diagnostics
BatteryState g_lastGoodBat;
bool g_haveLastGoodBat = false;
uint8_t g_batValidStreak = 0;
uint8_t g_batInvalidStreak = 0;
const char *g_batFilterState = "BOOT";

uint32_t g_lastFastUiMs = 0;
uint32_t g_lastSlowUiMs = 0;
uint32_t g_lastVuMs = 0;
uint32_t g_lastMicTextMs = 0;
uint32_t g_lastSerialMs = 0;
uint32_t g_lastSdMs = 0;
uint32_t g_lastBatMs = 0;
uint32_t g_lastVsenseUiMs = 0;
uint32_t g_lastFooterMs = 0;

String cache_sd = "";
String cache_touch = "";
String cache_bat = "";
String cache_d16_adc = "";
String cache_d16_calc = "";
float g_d16ShownVadc = -1.0f;
float g_d16ShownCalc = -1.0f;
int cache_chargeIcon = -1;
int cache_batIconState = -1;
String cache_acc = "";
String cache_gyr = "";
String cache_btn1 = "";
String cache_btn2 = "";
String cache_tap = "";
String cache_bl = "";
String cache_mic = "";
String cache_footer = "";
String cache_tick = "";

float g_vuSmooth = 0.0f;
int cache_vuSegments = -1;
uint32_t g_frameCounter = 0;

volatile bool g_imuIntFlag = false;
bool g_imuTapConfigured = false;
uint32_t g_doubleTapCount = 0;
uint32_t g_lastTapMs = 0;

int g_batDisplayPercent = -1;
uint32_t g_batLastPctRiseMs = 0;

bool g_batFloatReject = false;
uint8_t g_batFloatRejectStreak = 0;
uint8_t g_batReinsertStreak = 0;
float g_batLastRealV = 0.0f;
int g_batLastRealPercent = -1;
uint8_t g_batColdUsbSuspectStreak = 0;

bool g_batUsbOnlySeen = false;
bool g_batUsbInsertCompActive = false;
float g_batUsbInsertRawStartV = 0.0f;

// nRF52 visual layout
static constexpr int CARD_X = 7;
static constexpr int CARD_W = 158;

static constexpr int Y_TITLE = 8;
static constexpr int Y_SUB1  = 32;

static constexpr int Y_SYS  = 54;
static constexpr int H_SYS  = 66;

static constexpr int Y_MOTION = 125;
static constexpr int H_MOTION = 58;

static constexpr int Y_MIC  = 189;
static constexpr int H_MIC  = 62;

static constexpr int Y_BTN  = 258;
static constexpr int H_BTN  = 48;

static constexpr int Y_PRODUCT = 311;

static constexpr int ROW_SD     = Y_SYS + 23;
static constexpr int ROW_TOUCH  = Y_SYS + 38;
static constexpr int ROW_BAT    = Y_SYS + 53;

static constexpr int BAT_ICON_X = 62;
static constexpr int BAT_ICON_Y = ROW_BAT - 2;
static constexpr int BAT_ICON_W = 22;
static constexpr int BAT_ICON_H = 12;

static constexpr int CHG_ICON_X = 148;
static constexpr int CHG_ICON_Y = ROW_BAT - 2;
static constexpr int CHG_ICON_W = 12;
static constexpr int CHG_ICON_H = 14;

static constexpr int ROW_ACC    = Y_MOTION + 23;
static constexpr int ROW_GYR    = Y_MOTION + 39;
static constexpr int ROW_TAP_MOTION = Y_MOTION + 8;

static constexpr int ROW_MIC_RAW = Y_MIC + 42;

static constexpr int ROW_BTN1   = Y_BTN + 20;
static constexpr int ROW_BTN2   = Y_BTN + 20;
static constexpr int ROW_BL     = Y_BTN + 34;

static constexpr int VU_X = CARD_X + 16;
static constexpr int VU_Y = Y_MIC + 27;
static constexpr int VU_W = 124;
static constexpr int VU_H = 14;
static constexpr int VU_SEG_COUNT = 12;
static constexpr int VU_GAP = 2;
static constexpr int VU_SEG_W = (VU_W - 4 - (VU_SEG_COUNT - 1) * VU_GAP) / VU_SEG_COUNT;

// LCD/SD share the same physical SPI pins. The SD probe runs in a background task.
// These flags prevent the background task from touching the bus while a LCD draw is
// in progress, and prevent UI cache updates when a draw is skipped.
volatile bool g_lcdBusy = false;

// ========================= Logging =========================

static void logf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
}

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

static bool beginLcdOp() {
  if (g_sdProbeBusy) return false;

  g_lcdBusy = true;

  // Race guard: SD task may have started after the first check.
  if (g_sdProbeBusy) {
    g_lcdBusy = false;
    return false;
  }

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  restoreLcdSwSpiPins();
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  delayMicroseconds(2);
  return true;
}

static void endLcdOp() {
  digitalWrite(LCD_CS_PIN, HIGH);
  g_lastLcdOpMs = millis();
  g_lcdBusy = false;
}

static bool lcdBusAvailable() {
  return !g_sdProbeBusy;
}

static void acquireForLcd() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  restoreLcdSwSpiPins();
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void acquireForSd() {
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  // MISO can float when no card is inserted. Pull it high so no-card reads as 0xFF
  // instead of random low values that look like a false CMD0 response.
  pinMode(SD_MISO_PIN, INPUT_PULLUP);
  delayMicroseconds(2);
}

static void restoreLcdSwSpiPins() {
#if !LCD_USE_ESP32SPI
  // SD.begin() attaches D8/D10 to the ESP32 SPI peripheral.
  // Before software-SPI LCD writes, force these pins back to GPIO mode.
  pinMode(LCD_SCK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
#endif
}

static int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
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

static bool i2cRawRead(uint8_t addr, uint8_t *buf, size_t len) {
  size_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;

  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

static void scanI2c() {
  Serial.print("[I2C] scan:");
  int count = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      count++;
    }
  }
  if (!count) Serial.print(" none");
  Serial.println();
}

// ========================= LCD =========================

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
  lcdBus->writeCommand(0x36);
  lcdBus->write(0x48);
  lcdBus->endWrite();
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
    g_blIndex = BL_LEVEL_COUNT - 1;
  }
  applyBacklight();
}

static void queueButtonA(uint32_t now) {
  if (now - g_btnALastQueueMs < BTN_DEBOUNCE_MS) return;
  g_btnALastQueueMs = now;
  g_btnAPendingAction = true;
}

static void queueButtonB(uint32_t now) {
  if (now - g_btnBLastQueueMs < BTN_DEBOUNCE_MS) return;
  g_btnBLastQueueMs = now;
  g_btnBPendingAction = true;
}

static void handleBacklightButtons() {
  if (!g_btnAPendingAction && !g_btnBPendingAction) return;

  uint32_t now = millis();
  if (now - g_lastBtnActionMs < BTN_ACTION_LOCKOUT_MS) return;

  if (g_btnBPendingAction) {
    g_btnBPendingAction = false;
    g_btnAPendingAction = false;

    toggleBacklightOffRestore();
    g_lastBtnActionMs = now;
    cache_bl = "";
    Serial.print("[BL] USR2 toggle, pwm=");
    Serial.print(currentBacklightPwm());
    Serial.print(" pct=");
    Serial.println(currentBacklightPercent());
    return;
  }

  if (g_btnAPendingAction) {
    g_btnAPendingAction = false;

    cycleBacklightLevel();
    g_lastBtnActionMs = now;
    cache_bl = "";
    Serial.print("[BL] USR1 cycle, pwm=");
    Serial.print(currentBacklightPwm());
    Serial.print(" pct=");
    Serial.println(currentBacklightPercent());
  }
}

static bool initLcd() {
  applyBacklight();

  restoreLcdSwSpiPins();
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(10);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(30);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(150);

#if LCD_USE_ESP32SPI
  if (!gfx->begin(20000000)) {
#else
  if (!gfx->begin()) {
#endif
    Serial.println("[LCD] gfx->begin() failed");
    g_lcdOk = false;
    return false;
  }

  lcdWriteMadctlFix();

  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  g_lcdOk = true;
  Serial.println("[LCD] OK");
  return true;
}

static bool printTextFixed(int x, int y, uint16_t color, const String &text, int widthChars) {
  if (!beginLcdOp()) return false;
  gfx->setCursor(x, y);
  gfx->setTextSize(1);
  gfx->setTextColor(color, C_BLACK);
  gfx->print(padRight(text, widthChars));
  endLcdOp();
  return true;
}

static void drawVuFrame() {
  if (!beginLcdOp()) return;
  gfx->drawRoundRect(VU_X, VU_Y, VU_W, VU_H, 3, C_GREEN);
  for (int i = 0; i < VU_SEG_COUNT; ++i) {
    int x = VU_X + 2 + i * (VU_SEG_W + VU_GAP);
    gfx->fillRect(x, VU_Y + 2, VU_SEG_W, VU_H - 4, C_BLACK);
  }
  endLcdOp();
}

static void drawCard(int x, int y, int w, int h, uint16_t accent, const char *title) {
  if (!beginLcdOp()) return;
  gfx->drawRoundRect(x, y, w, h, 6, accent);
  gfx->drawRoundRect(x + 1, y + 1, w - 2, h - 2, 6, C_LINE);
  gfx->fillRect(x + 5, y + 12, 4, h - 24, accent);
  gfx->setTextSize(1);
  gfx->setTextColor(accent, C_BLACK);
  gfx->setCursor(x + 14, y + 8);
  gfx->print(title);
  endLcdOp();
}

static bool drawChargeIcon(bool charging) {
  if (!beginLcdOp()) return false;

  gfx->fillRect(CHG_ICON_X, CHG_ICON_Y, CHG_ICON_W, CHG_ICON_H, C_BLACK);

  if (charging) {
    const int x = CHG_ICON_X;
    const int y = CHG_ICON_Y;

    gfx->fillTriangle(x + 6, y + 0,  x + 1, y + 7,  x + 6, y + 7,  C_YELLOW);
    gfx->fillTriangle(x + 5, y + 6,  x + 11, y + 6, x + 4, y + 13, C_YELLOW);

    gfx->drawLine(x + 6, y + 0, x + 1, y + 7, C_ORANGE);
    gfx->drawLine(x + 1, y + 7, x + 6, y + 7, C_ORANGE);
    gfx->drawLine(x + 11, y + 6, x + 4, y + 13, C_ORANGE);
  }

  endLcdOp();
  return true;
}

static bool drawBatteryIcon(bool valid, int percent, bool charging) {
  if (!beginLcdOp()) return false;

  const int x = BAT_ICON_X;
  const int y = BAT_ICON_Y;

  gfx->fillRect(x - 1, y - 1, BAT_ICON_W + 5, BAT_ICON_H + 2, C_BLACK);

  uint16_t outline = valid ? C_WHITE : C_GRAY;
  uint16_t fillColor = valid ? colorByPercent(percent) : C_RED;
  gfx->drawRoundRect(x, y, BAT_ICON_W, BAT_ICON_H, 2, outline);
  gfx->fillRect(x + BAT_ICON_W, y + 4, 3, 4, outline);

  if (!valid) {
    gfx->drawLine(x + 4, y + 3, x + BAT_ICON_W - 4, y + BAT_ICON_H - 4, C_RED);
    gfx->drawLine(x + BAT_ICON_W - 4, y + 3, x + 4, y + BAT_ICON_H - 4, C_RED);
    endLcdOp();
    return true;
  }

  int fillW = map(percent, 0, 100, 0, BAT_ICON_W - 4);
  if (fillW < 0) fillW = 0;
  if (fillW > BAT_ICON_W - 4) fillW = BAT_ICON_W - 4;

  if (fillW > 0) {
    gfx->fillRect(x + 2, y + 2, fillW, BAT_ICON_H - 4, fillColor);
  }

  endLcdOp();
  return true;
}

static void drawStaticLayout() {
  if (!beginLcdOp()) return;
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
  endLcdOp();

  drawCard(CARD_X, Y_SYS, CARD_W, H_SYS, C_CYAN, "SYSTEM");
  if (beginLcdOp()) {
    gfx->setTextColor(C_WHITE, C_BLACK);
    gfx->setCursor(20, ROW_SD);    gfx->print("SD");
    gfx->setCursor(20, ROW_TOUCH); gfx->print("Touch");
    gfx->setCursor(20, ROW_BAT);   gfx->print("D16");
    gfx->setCursor(82, ROW_BAT);   gfx->print("Calc");
    endLcdOp();
  }

  drawCard(CARD_X, Y_MOTION, CARD_W, H_MOTION, C_YELLOW, "MOTION");
  if (beginLcdOp()) {
    gfx->setTextColor(C_YELLOW, C_BLACK);
    gfx->setCursor(110, ROW_TAP_MOTION); gfx->print("Tap 0");
    gfx->setTextColor(C_WHITE, C_BLACK);
    gfx->setCursor(20, ROW_ACC); gfx->print("Acc");
    gfx->setCursor(20, ROW_GYR); gfx->print("Gyr");
    endLcdOp();
  }

  drawCard(CARD_X, Y_MIC, CARD_W, H_MIC, C_GREEN, "MIC LEVEL");
  drawVuFrame();

  drawCard(CARD_X, Y_BTN, CARD_W, H_BTN, C_BLUE, "BACKLIGHT / BUTTON");
  if (beginLcdOp()) {
    gfx->setTextColor(C_WHITE, C_BLACK);
    gfx->setCursor(20, ROW_BTN1); gfx->print("Brightness");
    gfx->setTextColor(C_CYAN, C_BLACK);
    gfx->setCursor(20, ROW_BL); gfx->print("USR1 level  USR2 off");

    gfx->setTextSize(1);
    gfx->setTextColor(C_YELLOW, C_BLACK);
    gfx->setCursor(30, Y_PRODUCT);
    gfx->print("XIAO ESP32-S3 Plus");
    endLcdOp();
  }

  cache_sd = "";
  cache_touch = "";
  cache_bat = "";
  cache_d16_adc = "";
  cache_d16_calc = "";
  g_d16ShownVadc = -1.0f;
  g_d16ShownCalc = -1.0f;
  cache_chargeIcon = -1;
  cache_batIconState = -1;
  cache_acc = "";
  cache_gyr = "";
  cache_btn1 = "";
  cache_btn2 = "";
  cache_tap = "";
  cache_bl = "";
  cache_mic = "";
  cache_footer = "";
  cache_tick = "";
  cache_vuSegments = -1;
}

// ========================= SD =========================

static void closeSdSessionAndRestoreLcd() {
  SD.end();
  SPI.end();

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(SD_MISO_PIN, INPUT_PULLUP);
  restoreLcdSwSpiPins();
}

static uint8_t sdReadR1(uint8_t maxTries) {
  for (uint8_t i = 0; i < maxTries; i++) {
    uint8_t r = SPI.transfer(0xFF);
    if ((r & 0x80) == 0) return r;
  }
  return 0xFF;
}

static uint8_t sdSendCmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
  SPI.transfer(0x40 | cmd);
  SPI.transfer((arg >> 24) & 0xFF);
  SPI.transfer((arg >> 16) & 0xFF);
  SPI.transfer((arg >> 8) & 0xFF);
  SPI.transfer(arg & 0xFF);
  SPI.transfer(crc);
  return sdReadR1(8);
}

static void waitForLcdIdleBeforeSd(uint32_t idleMs, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (g_lcdBusy || (millis() - g_lastLcdOpMs) < idleMs) {
    if (millis() - start >= timeoutMs) break;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static bool sdQuickPresenceProbe() {
  // Raw SPI probe: only checks whether a card answers CMD0 with a valid idle R1.
  // No card should read as 0xFF because SD_MISO is pulled up. Treat anything other
  // than 0x01 as no-card to avoid falling into expensive SD.begin() timeouts.
  waitForLcdIdleBeforeSd(SD_LCD_IDLE_GAP_MS, 30);

  acquireForSd();
  SPI.begin(LCD_SCK_PIN, SD_MISO_PIN, LCD_MOSI_PIN, SD_CS_PIN);
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));

  digitalWrite(SD_CS_PIN, HIGH);
  for (int i = 0; i < 10; i++) SPI.transfer(0xFF);  // 80 clocks

  digitalWrite(SD_CS_PIN, LOW);
  uint8_t r = sdSendCmd(0, 0, 0x95);  // CMD0: GO_IDLE_STATE
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.transfer(0xFF);

  SPI.endTransaction();
  closeSdSessionAndRestoreLcd();

  return (r == 0x01);
}

static bool sdFullMountProbe(uint32_t &okFreq, uint32_t &cardSizeMb) {
  okFreq = 0;
  cardSizeMb = 0;

  bool quickPresent = sdQuickPresenceProbe();
  if (!quickPresent) {
    return false;
  }

  // If the card is already shown as inserted, do not full-mount on every poll.
  // This keeps automatic removal detection responsive without repeating SD.begin().
  if (g_sdMounted) {
    okFreq = g_sdOkFreq ? g_sdOkFreq : 400000;
    cardSizeMb = g_sdCardSizeMB ? (uint32_t)g_sdCardSizeMB : 1;
    return true;
  }

  // If a previous full mount failed, do not retry immediately. False positives
  // from floating/noisy MISO would otherwise cause repeated long SD.begin() stalls.
  uint32_t now = millis();
  if (g_sdFullRetryHoldUntilMs != 0 && (int32_t)(now - g_sdFullRetryHoldUntilMs) < 0) {
    return false;
  }

  // Candidate card detected and not shown inserted yet: full mount only now.
  waitForLcdIdleBeforeSd(SD_LCD_IDLE_GAP_MS, 40);

  acquireForSd();
  SPI.begin(LCD_SCK_PIN, SD_MISO_PIN, LCD_MOSI_PIN, SD_CS_PIN);
  delay(1);

  const uint32_t freqs[] = {400000};
  for (uint8_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
    if (SD.begin(SD_CS_PIN, SPI, freqs[i])) {
      okFreq = freqs[i];
      uint64_t mb = SD.cardSize() / (1024ULL * 1024ULL);
      if (mb == 0) mb = 1;
      if (mb > 0xFFFFFFFFUL) mb = 0xFFFFFFFFUL;
      cardSizeMb = (uint32_t)mb;

      g_sdFullRetryHoldUntilMs = 0;
      closeSdSessionAndRestoreLcd();
      return true;
    }

    SD.end();
    SPI.end();
    pinMode(SD_MISO_PIN, INPUT_PULLUP);
    restoreLcdSwSpiPins();
    delay(1);
  }

  g_sdFullRetryHoldUntilMs = millis() + SD_FULL_RETRY_COOLDOWN_MS;
  closeSdSessionAndRestoreLcd();
  return false;
}

static void sdProbeTask(void *param) {
  (void)param;

  for (;;) {
    if (g_sdProbeRequest && !g_sdProbeBusy) {
      g_sdProbeRequest = false;
      g_sdProbeBusy = true;

      uint32_t freq = 0;
      uint32_t sizeMb = 0;
      bool present = sdFullMountProbe(freq, sizeMb);

      g_sdProbePresentResult = present;
      g_sdProbeFreqResult = freq;
      g_sdProbeSizeMbResult = sizeMb;
      g_sdProbeResultReady = true;

      g_sdProbeBusy = false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void startSdProbeTask() {
  static bool started = false;
  if (started) return;
  started = true;

  xTaskCreatePinnedToCore(
    sdProbeTask,
    "sd_probe",
    4096,
    nullptr,
    1,
    nullptr,
    0
  );
}

static void requestSdProbe() {
  if (g_sdProbeBusy || g_sdProbeRequest) return;
  g_sdProbeRequest = true;
}

static void consumeSdProbeResult() {
  if (!g_sdProbeResultReady) return;

  bool present = g_sdProbePresentResult;
  uint32_t freq = g_sdProbeFreqResult;
  uint32_t sizeMb = g_sdProbeSizeMbResult;

  g_sdProbeResultReady = false;

  // Debounce:
  // - insert requires two positive probes.
  // - removal requires one negative probe for quick unplug response.
  if (present) {
    if (g_sdInsertStable < 3) g_sdInsertStable++;
    g_sdRemoveStable = 0;
  } else {
    if (g_sdRemoveStable < 3) g_sdRemoveStable++;
    g_sdInsertStable = 0;
  }

  bool oldState = g_sdMounted;

  if (present && g_sdInsertStable >= 2) {
    g_sdMounted = true;
    g_sdOkFreq = freq;
    g_sdCardSizeMB = sizeMb ? sizeMb : 1;
  }

  if (!present && g_sdRemoveStable >= 1) {
    g_sdMounted = false;
    g_sdOkFreq = 0;
    g_sdCardSizeMB = 0;
  }

  if (oldState != g_sdMounted) {
    cache_sd = "";  // force LCD row update immediately
  }

  // Do not print every unchanged SD poll. USB Serial output also costs time, and
  // repeated no-card logs make the dashboard look less responsive.
  if (oldState != g_sdMounted) {
    Serial.printf("[SD] state changed present=%u shown=%u freq=%lu size=%lluMB busy=%u\n",
                  present ? 1 : 0,
                  g_sdMounted ? 1 : 0,
                  (unsigned long)g_sdOkFreq,
                  g_sdCardSizeMB,
                  g_sdProbeBusy ? 1 : 0);
  }
}

static void updateSdStatus() {
  consumeSdProbeResult();
  requestSdProbe();
}

static void writeSdTestFileOnce() {
  // Dashboard mode: USR2 only requests an immediate async probe.
  requestSdProbe();
  Serial.println("[SD] async status refresh requested");
}

// ========================= Touch =========================

static bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void initTouch() {
  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);

  if (!i2cDevicePresent(AXS5106L_ADDR)) {
    g_touchFound = false;
    g_touchAddr = 0;
    Serial.println("[TOUCH] AXS5106L not found at 0x63");
    return;
  }

  // The Waveshare BSP library resets the touch controller with tp_rst and attaches
  // the falling-edge interrupt on tp_int.
  //
  // Important for our board:
  // D17 is currently shared/used as LCD_RST in our pin map. Calling this before
  // initLcd() is safe because LCD is initialized after the touch reset pulse.
  bsp_touch_init(&Wire, LCD_RST_PIN, TOUCH_INT_PIN, 0, 172, 320);

  g_touchFound = true;
  g_touchAddr = AXS5106L_ADDR;
  g_touchValid = false;
  g_touchX = -1;
  g_touchY = -1;

  Serial.println("[TOUCH] AXS5106L BSP driver OK at 0x63");
}

static void normalizeTouchToScreen(int rawX, int rawY) {
  g_touchRawX = rawX;
  g_touchRawY = rawY;

  int x = rawX;
  int y = rawY;

  // This product is portrait 172x320.
  // Some controllers report landscape 320x172, so swap in that case.
  if (rawX >= 0 && rawY >= 0 && rawX <= 320 && rawY <= 172 && rawX > 171) {
    x = rawY;
    y = rawX;
  }

  // Some controllers may report 12-bit ADC coordinates.
  if (x > 360 || y > 360) {
    x = map(rawX, 0, 4095, 0, 171);
    y = map(rawY, 0, 4095, 0, 319);
  }

  if (x < 0) x = 0;
  if (x > 171) x = 171;
  if (y < 0) y = 0;
  if (y > 319) y = 319;

  g_touchX = x;
  g_touchY = y;
}

static bool validTouchCandidate(int x, int y) {
  if (x < 0 || y < 0) return false;
  if (x > 4095 || y > 4095) return false;
  // Reject the most common empty patterns.
  if ((x == 0 && y == 0) || (x == 4095 && y == 4095)) return false;
  return true;
}

static bool decodeCandidateCstStyle(const uint8_t *b, int base, int &x, int &y) {
  uint8_t fingers = b[base + 1] & 0x0F;
  if (fingers == 0 || fingers > 5) return false;

  x = ((uint16_t)(b[base + 2] & 0x0F) << 8) | b[base + 3];
  y = ((uint16_t)(b[base + 4] & 0x0F) << 8) | b[base + 5];

  return validTouchCandidate(x, y);
}

static bool decodeCandidateFtStyle(const uint8_t *b, int base, int &x, int &y) {
  uint8_t fingers = b[base] & 0x0F;
  if (fingers == 0 || fingers > 5) return false;

  x = ((uint16_t)(b[base + 1] & 0x0F) << 8) | b[base + 2];
  y = ((uint16_t)(b[base + 3] & 0x0F) << 8) | b[base + 4];

  return validTouchCandidate(x, y);
}

static bool decodeRawTouchPacket(const uint8_t *b, int &x, int &y) {
  // Raw 0x63 read is the only transaction we use for AXS5106L-like touch.
  // It avoids the repeated I2C NACK spam caused by invalid register reads.
  //
  // Try several compact 5/6-byte layouts. The correct one will be selected
  // by range validation and then normalized to 172x320.
  int candidates[6][2];

  candidates[0][0] = ((int)(b[1] & 0x0F) << 8) | b[2];
  candidates[0][1] = ((int)(b[3] & 0x0F) << 8) | b[4];

  candidates[1][0] = ((int)(b[2] & 0x0F) << 8) | b[3];
  candidates[1][1] = ((int)(b[4] & 0x0F) << 8) | b[5];

  candidates[2][0] = ((int)b[1] << 8) | b[2];
  candidates[2][1] = ((int)b[3] << 8) | b[4];

  candidates[3][0] = ((int)b[2] << 8) | b[3];
  candidates[3][1] = ((int)b[4] << 8) | b[5];

  candidates[4][0] = b[2];
  candidates[4][1] = b[4];

  candidates[5][0] = b[1];
  candidates[5][1] = b[3];

  for (int i = 0; i < 6; i++) {
    int cx = candidates[i][0];
    int cy = candidates[i][1];

    // Native portrait or landscape coordinate windows.
    bool directPortrait = (cx >= 0 && cx <= 171 && cy >= 0 && cy <= 319);
    bool landscape = (cx >= 0 && cx <= 319 && cy >= 0 && cy <= 171);
    bool adc12 = (cx >= 0 && cx <= 4095 && cy >= 0 && cy <= 4095 && (cx > 360 || cy > 360));

    if (directPortrait || landscape || adc12) {
      // Reject empty/release-looking packets.
      if ((cx == 0 && cy == 0) || (cx == 4095 && cy == 4095)) continue;
      x = cx;
      y = cy;
      return true;
    }
  }

  return false;
}

static bool readTouchAxs5106lLike(int &x, int &y) {
  uint8_t b[6] = {};

  // One short raw read only. No register write, no long reads.
  if (!i2cRawRead(0x63, b, sizeof(b))) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    g_touchRawBytes[i] = b[i];
  }

  // If INT exists and is high, many touch controllers are in release state.
  // Still decode raw bytes below, because some boards wire INT differently.
  if (!decodeRawTouchPacket(b, x, y)) {
    return false;
  }

  return true;
}

static void updateTouch() {
  g_touchValid = false;
  g_touchX = -1;
  g_touchY = -1;
  g_touchRawX = -1;
  g_touchRawY = -1;
  g_touchIntRaw = digitalRead(TOUCH_INT_PIN);

  for (int i = 0; i < 8; i++) {
    g_touchRawBytes[i] = 0;
  }

  if (!g_touchFound) {
    return;
  }

  // BSP path from esp_lcd_touch_axs5106l:
  // 1) bsp_touch_read() updates the library's internal g_touch_data only when INT fires.
  // 2) bsp_touch_get_coordinates() returns transformed coordinates.
  bsp_touch_read();

  touch_data_t data = {};
  if (bsp_touch_get_coordinates(&data)) {
    int rawX = data.coords[0].x;
    int rawY = data.coords[0].y;

    g_touchRawX = rawX;
    g_touchRawY = rawY;

    normalizeTouchToScreen(rawX, rawY);

    g_touchLastX = g_touchX;
    g_touchLastY = g_touchY;
    g_touchLastSeenMs = millis();

    g_touchValid = true;
    return;
  }

  // The BSP library is interrupt-driven. Some touch panels only fire INT on down/move,
  // so keep the latest coordinate visible briefly instead of flickering immediately
  // back to release.
  if (g_touchLastSeenMs != 0 && (millis() - g_touchLastSeenMs) < 350) {
    g_touchX = g_touchLastX;
    g_touchY = g_touchLastY;
    g_touchRawX = g_touchLastX;
    g_touchRawY = g_touchLastY;
    g_touchValid = true;
    return;
  }
}

// ========================= IMU =========================

static bool initQmi(uint8_t addr) {
  uint8_t who = 0;
  if (!i2cRead8(addr, 0x00, &who)) return false;
  if (who == 0x00 || who == 0xFF) return false;

  i2cWrite8(addr, 0x02, 0x60);
  i2cWrite8(addr, 0x03, 0x03);
  i2cWrite8(addr, 0x04, 0x53);
  i2cWrite8(addr, 0x08, 0x03);
  delay(20);

  uint8_t data[12] = {};
  if (!i2cRead(addr, 0x35, data, sizeof(data))) return false;

  g_imuType = IMU_QMI8658;
  g_imuAddr = addr;
  g_imuOk = true;
  Serial.printf("[IMU] QMI8658-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

static bool initLsm(uint8_t addr) {
  uint8_t who = 0;
  if (!i2cRead8(addr, 0x0F, &who)) return false;
  if (who != 0x69 && who != 0x6A && who != 0x6C) return false;

  i2cWrite8(addr, 0x10, 0x60);
  i2cWrite8(addr, 0x11, 0x60);
  delay(20);

  g_imuType = IMU_LSM6DS3;
  g_imuAddr = addr;
  g_imuOk = true;
  Serial.printf("[IMU] LSM6-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

// ========================= IMU double-tap interrupt =========================
//
// LSM6DS3/LSM6DSO-family double tap registers:
//   TAP_SRC       0x1C: read to clear tap event; bit4 is DOUBLE_TAP.
//   TAP_CFG       0x58: enable tap recognition on X/Y/Z and embedded interrupts.
//   TAP_THS_6D    0x59: tap threshold.
//   INT_DUR2      0x5A: shock/quiet/duration timing.
//   WAKE_UP_THS   0x5B: bit7 enables single/double-tap recognition.
//   MD1_CFG       0x5E: route double-tap event to INT1.
//
// If taps are hard to trigger, lower TAP_THS_6D from 0x0C to 0x08.
// If false triggers occur, raise it to 0x10.

static constexpr uint8_t LSM_REG_TAP_SRC     = 0x1C;
static constexpr uint8_t LSM_REG_CTRL1_XL    = 0x10;
static constexpr uint8_t LSM_REG_TAP_CFG     = 0x58;
static constexpr uint8_t LSM_REG_TAP_THS_6D  = 0x59;
static constexpr uint8_t LSM_REG_INT_DUR2    = 0x5A;
static constexpr uint8_t LSM_REG_WAKE_UP_THS = 0x5B;
static constexpr uint8_t LSM_REG_MD1_CFG     = 0x5E;

void IRAM_ATTR imuIntIsr() {
  g_imuIntFlag = true;
}

static bool initImuDoubleTap() {
  g_imuTapConfigured = false;

  if (g_imuType != IMU_LSM6DS3 || g_imuAddr == 0) {
    Serial.println("[IMU] double-tap INT skipped: non-LSM IMU");
    return false;
  }

  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);

  bool ok = true;

  // Accelerometer on, 416Hz, +/-2g.
  // Tap detection needs a known accelerometer ODR.
  ok &= i2cWrite8(g_imuAddr, LSM_REG_CTRL1_XL, 0x60);

  // 0x8E: embedded interrupts enabled, tap recognition on X/Y/Z.
  ok &= i2cWrite8(g_imuAddr, LSM_REG_TAP_CFG, 0x8E);

  // Medium threshold/timing for small handheld board.
  ok &= i2cWrite8(g_imuAddr, LSM_REG_TAP_THS_6D, 0x0C);
  ok &= i2cWrite8(g_imuAddr, LSM_REG_INT_DUR2, 0x7F);

  // Enable single/double-tap recognition mode.
  ok &= i2cWrite8(g_imuAddr, LSM_REG_WAKE_UP_THS, 0x80);

  // Route double-tap event to INT1.
  ok &= i2cWrite8(g_imuAddr, LSM_REG_MD1_CFG, 0x08);

  uint8_t dummy = 0;
  (void)i2cRead8(g_imuAddr, LSM_REG_TAP_SRC, &dummy); // clear stale event.

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIntIsr, RISING);

  g_imuTapConfigured = ok;

  Serial.print("[IMU] double-tap INT1 on D14 init ");
  Serial.println(ok ? "OK" : "FAILED");

  return ok;
}

static void handleImuTapEvent() {
  if (!g_imuTapConfigured || g_imuType != IMU_LSM6DS3) return;

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
  if (!i2cRead8(g_imuAddr, LSM_REG_TAP_SRC, &src)) return;

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

static void initImu() {
  pinMode(IMU_INT_PIN, INPUT_PULLUP);

  g_imuType = IMU_NONE;
  g_imuOk = false;

  if (initQmi(0x6B)) {
    initImuDoubleTap();
    return;
  }
  if (initQmi(0x6A)) {
    initImuDoubleTap();
    return;
  }
  if (initLsm(0x6A)) {
    initImuDoubleTap();
    return;
  }
  if (initLsm(0x6B)) {
    initImuDoubleTap();
    return;
  }

  Serial.println("[IMU] not found");
}

static void updateImu() {
  if (g_imuType == IMU_QMI8658) {
    uint8_t d[12] = {};
    if (!i2cRead(g_imuAddr, 0x35, d, sizeof(d))) {
      g_imuOk = false;
      return;
    }

    g_ax = le16(&d[0]) / 16384.0f;
    g_ay = le16(&d[2]) / 16384.0f;
    g_az = le16(&d[4]) / 16384.0f;

    g_gx = le16(&d[6]) / 64.0f;
    g_gy = le16(&d[8]) / 64.0f;
    g_gz = le16(&d[10]) / 64.0f;

    g_imuOk = true;
  } else if (g_imuType == IMU_LSM6DS3) {
    uint8_t g[6] = {};
    uint8_t a[6] = {};

    if (!i2cRead(g_imuAddr, 0x22, g, sizeof(g)) ||
        !i2cRead(g_imuAddr, 0x28, a, sizeof(a))) {
      g_imuOk = false;
      return;
    }

    g_gx = le16(&g[0]) * 0.00875f;
    g_gy = le16(&g[2]) * 0.00875f;
    g_gz = le16(&g[4]) * 0.00875f;

    g_ax = le16(&a[0]) * 0.000061f;
    g_ay = le16(&a[2]) * 0.000061f;
    g_az = le16(&a[4]) * 0.000061f;

    g_imuOk = true;
  } else {
    g_imuOk = false;
  }
}

// ========================= Buttons =========================

static void updateButtons() {
  uint32_t now = millis();

  g_btnARaw = digitalRead(BTN_A_PIN);
  g_btnBRaw = digitalRead(BTN_B_PIN);
  g_btnA = (g_btnARaw == LOW);
  g_btnB = (g_btnBRaw == LOW);

  bool rawAPressed = g_btnA;
  bool rawBPressed = g_btnB;

  if (rawAPressed != g_btnALastRawPressed) {
    g_btnALastRawPressed = rawAPressed;
    g_btnALastChangeMs = now;
    if (rawAPressed) queueButtonA(now);
  }

  if (rawBPressed != g_btnBLastRawPressed) {
    g_btnBLastRawPressed = rawBPressed;
    g_btnBLastChangeMs = now;
    if (rawBPressed) queueButtonB(now);
  }

  handleBacklightButtons();
}

// ========================= MIC =========================

static void deinitMic() {
#if ESP_IDF_VERSION_MAJOR >= 5
  if (g_i2sRxChan) {
    i2s_channel_disable(g_i2sRxChan);
    i2s_del_channel(g_i2sRxChan);
    g_i2sRxChan = nullptr;
  }
#else
  i2s_driver_uninstall(I2S_PORT);
#endif
  g_micOk = false;
}

static bool initMic() {
  deinitMic();

#if ESP_IDF_VERSION_MAJOR >= 5
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 4;
  chanCfg.dma_frame_num = MIC_SAMPLES_PER_READ;

  esp_err_t err = i2s_new_channel(&chanCfg, nullptr, &g_i2sRxChan);
  if (err != ESP_OK) {
    Serial.printf("[MIC] i2s_new_channel failed: %d\n", (int)err);
    return false;
  }

  i2s_pdm_rx_config_t pdmCfg = {};
  pdmCfg.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE_HZ);
  pdmCfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  pdmCfg.gpio_cfg.clk = (gpio_num_t)MIC_CLK_PIN;
  pdmCfg.gpio_cfg.din = (gpio_num_t)MIC_DATA_PIN;
  pdmCfg.gpio_cfg.invert_flags.clk_inv = MIC_CLK_INVERT;

  err = i2s_channel_init_pdm_rx_mode(g_i2sRxChan, &pdmCfg);
  if (err != ESP_OK) {
    Serial.printf("[MIC] init_pdm failed: %d\n", (int)err);
    deinitMic();
    return false;
  }

#if REDUCE_PDM_CLK_DRIVE
  gpio_set_drive_capability((gpio_num_t)MIC_CLK_PIN, GPIO_DRIVE_CAP_0);
#endif

  err = i2s_channel_enable(g_i2sRxChan);
  if (err != ESP_OK) {
    Serial.printf("[MIC] enable failed: %d\n", (int)err);
    deinitMic();
    return false;
  }
#else
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  cfg.sample_rate = MIC_SAMPLE_RATE_HZ;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = MIC_SAMPLES_PER_READ;

  esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  if (err != ESP_OK) return false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_PIN_NO_CHANGE;
  pins.ws_io_num = MIC_CLK_PIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = MIC_DATA_PIN;
  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) return false;
#endif

  g_micOk = true;
  Serial.println("[MIC] OK");
  return true;
}

static bool readMic(size_t *bytesRead, uint32_t timeoutMs) {
  *bytesRead = 0;

#if ESP_IDF_VERSION_MAJOR >= 5
  if (!g_i2sRxChan) return false;
  esp_err_t err = i2s_channel_read(g_i2sRxChan, g_pdmBuf, sizeof(g_pdmBuf), bytesRead, pdMS_TO_TICKS(timeoutMs));
#else
  esp_err_t err = i2s_read(I2S_PORT, g_pdmBuf, sizeof(g_pdmBuf), bytesRead, pdMS_TO_TICKS(timeoutMs));
#endif

  return (err == ESP_OK && *bytesRead > 0);
}

static void updateMicPeak() {
  if (!g_micOk) return;

  size_t bytesRead = 0;
  if (!readMic(&bytesRead, 2)) {
    return;
  }

  int samples = bytesRead / sizeof(int16_t);
  if (samples <= 0) return;

  int64_t sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += g_pdmBuf[i];
  }
  int32_t mean = (int32_t)(sum / samples);

  uint32_t peak = 0;
  uint64_t sq = 0;
  for (int i = 0; i < samples; ++i) {
    int32_t v = (int32_t)g_pdmBuf[i] - mean;
    uint32_t a = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
    if (a > peak) peak = a;
    sq += (uint64_t)a * (uint64_t)a;
  }

  g_micPeak = (uint16_t)((peak > 65535) ? 65535 : peak);
  g_micRms = (uint32_t)sqrt((double)sq / samples);
  g_micLastUpdateMs = millis();
}

static uint16_t currentMicPeak() {
  uint32_t now = millis();
  if (now - g_micLastUpdateMs > MIC_DECAY_MS) {
    g_micPeak = (uint16_t)(g_micPeak * 0.75f);
    g_micRms = (uint32_t)(g_micRms * 0.75f);
  }
  return g_micPeak;
}

static uint16_t vuColorForIndex(int idx) {
  if (idx >= 10) return C_RED;
  if (idx >= 7) return C_ORANGE;
  return C_GREEN;
}

// ========================= UI updates =========================

static void updateVuMeter() {
  uint16_t micPeak = currentMicPeak();

  float target = (float)micPeak / MIC_DISPLAY_SCALE;
  if (target < 0.0f) target = 0.0f;
  if (target > 1.0f) target = 1.0f;

  if (target > g_vuSmooth) g_vuSmooth = g_vuSmooth * 0.55f + target * 0.45f;
  else g_vuSmooth = g_vuSmooth * 0.86f + target * 0.14f;

  int activeSegs = (int)roundf(g_vuSmooth * VU_SEG_COUNT);
  if (activeSegs < 0) activeSegs = 0;
  if (activeSegs > VU_SEG_COUNT) activeSegs = VU_SEG_COUNT;

  if (activeSegs == cache_vuSegments) return;

  if (!beginLcdOp()) return;
  for (int i = 0; i < VU_SEG_COUNT; ++i) {
    int x = VU_X + 2 + i * (VU_SEG_W + VU_GAP);
    uint16_t color = (i < activeSegs) ? vuColorForIndex(i) : C_BLACK;
    gfx->fillRect(x, VU_Y + 2, VU_SEG_W, VU_H - 4, color);
  }
  endLcdOp();

  cache_vuSegments = activeSegs;
}

static void updateMicText() {
  uint16_t micPeak = currentMicPeak();
  String micText = String("Raw ") + String((unsigned)micPeak);
  if (micText != cache_mic) {
    if (printTextFixed(18, ROW_MIC_RAW, C_WHITE, micText, 18)) {
      cache_mic = micText;
    }
  }
}


// ========================= Battery =========================

static void initBatteryAdc() {
  analogReadResolution(12);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
}

static BatteryState readBatteryMeasured() {
  BatteryState b;
  uint32_t rawSum = 0;
  uint32_t mvSum = 0;
  uint16_t rawMin = 65535, rawMax = 0;
  uint16_t mvMin = 65535, mvMax = 0;

  // v7.14: D16 live sense mode.
  // Use a light sampler so plug/unplug/switch changes are visible quickly.
  // Keep analogReadMilliVolts() for calibrated voltage, but avoid the old 32-sample
  // battery-percent averaging path.
  for (int i = 0; i < 2; i++) {
    (void)analogRead(BAT_ADC_PIN);
    delayMicroseconds(80);
  }

  static constexpr int N = 8;
  for (int i = 0; i < N; i++) {
    uint16_t raw = analogRead(BAT_ADC_PIN);
    uint16_t mv = analogReadMilliVolts(BAT_ADC_PIN);

    rawSum += raw;
    mvSum += mv;

    if (raw < rawMin) rawMin = raw;
    if (raw > rawMax) rawMax = raw;
    if (mv < mvMin) mvMin = mv;
    if (mv > mvMax) mvMax = mv;

    delayMicroseconds(80);
  }

  b.raw = rawSum / N;
  b.rawMin = rawMin;
  b.rawMax = rawMax;
  b.mv = mvSum / N;
  b.mvMin = mvMin;
  b.mvMax = mvMax;
  b.vadc = (float)b.mv / 1000.0f;
  b.vbat = b.vadc * BAT_DIVIDER_RATIO * BAT_CAL_FACTOR;
  b.percent = lipoPercent(b.vbat);
  b.charging = false;
  b.valid = true;
  return b;
}

static int updatePresencePercent(int measuredPercent, bool freshBattery) {
  measuredPercent = constrain(measuredPercent, 0, 100);
  uint32_t now = millis();

  if (freshBattery || g_batDisplayPercent < 0) {
    g_batDisplayPercent = measuredPercent;
    g_batLastPctRiseMs = now;
    return g_batDisplayPercent;
  }

  // Falling percentage is allowed immediately.
  if (measuredPercent < g_batDisplayPercent) {
    g_batDisplayPercent = measuredPercent;
    return g_batDisplayPercent;
  }

  // Rising percentage is rate-limited. This prevents USB insertion / charger
  // surface voltage from making 17% become 25% or 70% instantly.
  if (measuredPercent > g_batDisplayPercent &&
      (now - g_batLastPctRiseMs >= BAT_PCT_RISE_INTERVAL_MS)) {
    g_batDisplayPercent++;
    g_batLastPctRiseMs = now;
  }

  return g_batDisplayPercent;
}

static bool isImpossibleHighJump(const BatteryState &m) {
  if (!g_haveLastGoodBat) return false;

  float dv = m.vbat - g_lastGoodBat.vbat;
  int dp = m.percent - g_lastGoodBat.percent;

  return (dv >= BAT_IMPOSSIBLE_JUMP_V) || (dp >= BAT_IMPOSSIBLE_JUMP_PCT);
}

static void markBatteryAbsentLikeUsb(const BatteryState &measured, const char *state) {
  g_batUsbOnlySeen = true;
  g_batUsbInsertCompActive = false;
  g_batUsbInsertRawStartV = 0.0f;

  g_bat = measured;
  g_bat.valid = false;
  g_bat.charging = false;
  g_bat.percent = 0;

  // Do not clear g_lastGoodBat here. We need last real battery memory to
  // identify re-insertion after USB/floating node.
  g_batFilterState = state;

  cache_bat = "";
  cache_chargeIcon = -1;
  cache_batIconState = -1;
}

static float batteryEffectiveVoltageForPercent(float rawV, bool freshBattery) {
  // Activate compensation only in the specific path:
  // this boot has already seen USB_PWR/USB_BOOT_HIGH, and then a battery becomes
  // plausible for the first time. Battery-only boot is untouched.
  if (freshBattery && g_batUsbOnlySeen && !g_batUsbInsertCompActive) {
    g_batUsbInsertCompActive = true;
    g_batUsbInsertRawStartV = rawV;
    Serial.print("[BAT] USB-first battery insert compensation ON rawStart=");
    Serial.println(g_batUsbInsertRawStartV, 3);
  }

  if (g_batUsbInsertCompActive) {
    // If the raw voltage falls by enough, USB was probably unplugged or the
    // charger surface voltage settled. Stop compensating; otherwise it would
    // under-report once the battery returns to its real voltage.
    if (g_batUsbInsertRawStartV > 0.0f &&
        rawV <= (g_batUsbInsertRawStartV - BAT_USB_INSERT_RELEASE_DROP_V)) {
      g_batUsbInsertCompActive = false;
      Serial.print("[BAT] USB-insert compensation OFF raw=");
      Serial.println(rawV, 3);
      return rawV;
    }

    float compensated = rawV - BAT_USB_INSERT_SURFACE_COMP_V;
    if (compensated < BAT_EFFECTIVE_MIN_V) compensated = BAT_EFFECTIVE_MIN_V;
    return compensated;
  }

  return rawV;
}

static bool isColdBootUsbSuspect(const BatteryState &m, bool stable) {
  if (!BAT_COLD_BOOT_HIGH_AS_USB) return false;
  if (g_haveLastGoodBat) return false;
  if (!stable) return false;
  return (m.percent >= BAT_COLD_BOOT_SUSPECT_PCT);
}

static void updateBattery() {
  BatteryState measured = readBatteryMeasured();

  // v7.14: D16 voltage sense only.
  // Do not run the old BAT_PRESENT / USB_BOOT_HIGH / percentage state machine.
  // That state machine was for battery SoC display and can make testing confusing.
  g_vsensePrev = g_vsense;
  g_vsense = measured;
  g_bat = measured;

  uint16_t mvSpread = measured.mvMax - measured.mvMin;

  if (measured.mv < 50) {
    g_batFilterState = "LOW";
  } else if (mvSpread > BAT_FLOAT_RANGE_MV) {
    g_batFilterState = "NOISY";
  } else if (measured.vbat < BAT_VALID_MIN_V || measured.vbat > BAT_VALID_MAX_V) {
    g_batFilterState = "OUT_RANGE";
  } else {
    g_batFilterState = "LIVE";
  }
}

static String voltageFixedText(float v) {
  char buf[12];
  snprintf(buf, sizeof(buf), "%.2fV", v);
  return String(buf);
}

static void updateUiFast() {
  String sdText = g_sdMounted ? (String("OK ") + String(g_sdOkFreq / 1000) + "k") : "Unplugged";
  if (sdText != cache_sd) {
    if (printTextFixed(62, ROW_SD, g_sdMounted ? C_GREEN : C_RED, sdText, 15)) {
      cache_sd = sdText;
    }
  }

  String touchText = g_touchValid ? (String("(") + g_touchX + "," + g_touchY + ")") : String("release");
  if (!g_touchFound) touchText = "not found";
  if (touchText != cache_touch) {
    if (printTextFixed(62, ROW_TOUCH, g_touchFound ? C_CYAN : C_RED, touchText, 15)) {
      cache_touch = touchText;
    }
  }

  // D16 voltage row:
  // Labels are static. Numeric fields redraw only on meaningful voltage changes.
  // This avoids 0.01 V ADC jitter causing repeated software-SPI text redraws.
  if (millis() - g_lastVsenseUiMs >= VSENSE_UI_MS ||
      cache_d16_adc.length() == 0 || cache_d16_calc.length() == 0) {
    g_lastVsenseUiMs = millis();

    float d16Now = roundf(g_vsense.vadc * 100.0f) / 100.0f;
    float calcNow = roundf(g_vsense.vbat * 100.0f) / 100.0f;

    bool needD16 = (g_d16ShownVadc < 0.0f) || (fabsf(d16Now - g_d16ShownVadc) >= VSENSE_D16_DELTA_V);
    if (needD16) {
      String d16Text = voltageFixedText(d16Now);
      if (printTextFixed(46, ROW_BAT, C_YELLOW, d16Text, 6)) {
        cache_d16_adc = d16Text;
        g_d16ShownVadc = d16Now;
      }
    }

    bool needCalc = (g_d16ShownCalc < 0.0f) || (fabsf(calcNow - g_d16ShownCalc) >= VSENSE_CALC_DELTA_V);
    if (needCalc) {
      String calcText = voltageFixedText(calcNow);
      if (printTextFixed(116, ROW_BAT, C_YELLOW, calcText, 6)) {
        cache_d16_calc = calcText;
        g_d16ShownCalc = calcNow;
      }
    }
  }

  // No battery icon or charge icon: this row is voltage sense, not battery status.
  cache_batIconState = -1;
  cache_chargeIcon = 0;

  String blText = String(currentBacklightPercent()) + "%";
  if (blText != cache_bl) {
    uint16_t c = currentBacklightPwm() == 0 ? C_RED : C_CYAN;
    if (printTextFixed(118, ROW_BTN1, c, blText, 4)) {
      cache_bl = blText;
    }
  }

  String tapText = String("Tap ") + String(g_doubleTapCount);
  if (tapText != cache_tap) {
    if (printTextFixed(110, ROW_TAP_MOTION, C_YELLOW, tapText, 7)) {
      cache_tap = tapText;
    }
  }
}

static void updateUiSlow() {
  char buf[64];

  String accText;
  if (g_imuOk) {
    snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", g_ax, g_ay, g_az);
    accText = String(buf);
  } else {
    accText = "not found";
  }

  if (accText != cache_acc) {
    if (printTextFixed(50, ROW_ACC, g_imuOk ? C_WHITE : C_RED, accText, 16)) {
      cache_acc = accText;
    }
  }

  String gyrText;
  if (g_imuOk) {
    snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", g_gx, g_gy, g_gz);
    gyrText = String(buf);
  } else {
    gyrText = "not found";
  }

  if (gyrText != cache_gyr) {
    if (printTextFixed(50, ROW_GYR, g_imuOk ? C_WHITE : C_RED, gyrText, 16)) {
      cache_gyr = gyrText;
    }
  }
}

static void updateFooter() {
  // Footer is static in this nRF52-parity UI.
}

static void printSerialStatus() {
  Serial.print("[DASH_S3] sd=");
  Serial.print(g_sdMounted ? "Inserted" : "Unplugged");
  Serial.print(" freq=");
  Serial.print((unsigned long)g_sdOkFreq);

  Serial.print(" touch=");
  Serial.print(g_touchValid ? "Y" : "N");
  Serial.print(" x=");
  Serial.print(g_touchX);
  Serial.print(" y=");
  Serial.print(g_touchY);

  Serial.print(" d16_adc=");
  Serial.print(g_vsense.vadc, 3);
  Serial.print("V calc=");
  Serial.print(g_vsense.vbat, 3);
  Serial.print("V raw=");
  Serial.print(g_vsense.raw);
  Serial.print(" rawMin=");
  Serial.print(g_vsense.rawMin);
  Serial.print(" rawMax=");
  Serial.print(g_vsense.rawMax);
  Serial.print(" mv=");
  Serial.print(g_vsense.mv);
  Serial.print(" spread=");
  Serial.print((unsigned)(g_vsense.mvMax - g_vsense.mvMin));
  Serial.print(" dCalc=");
  Serial.print(g_vsense.vbat - g_vsensePrev.vbat, 3);
  Serial.print("V vstate=");
  Serial.print(g_batFilterState);

  Serial.print(" imu=");
  Serial.print(g_imuOk ? "OK" : "NO");
  Serial.print(" tapCfg=");
  Serial.print(g_imuTapConfigured ? "Y" : "N");
  Serial.print(" tap=");
  Serial.print((unsigned long)g_doubleTapCount);
  Serial.print(" type=");
  Serial.print((int)g_imuType);
  Serial.print(" acc=(");
  Serial.print(g_ax, 2);
  Serial.print(",");
  Serial.print(g_ay, 2);
  Serial.print(",");
  Serial.print(g_az, 2);
  Serial.print(") gyr=(");
  Serial.print(g_gx, 2);
  Serial.print(",");
  Serial.print(g_gy, 2);
  Serial.print(",");
  Serial.print(g_gz, 2);
  Serial.print(")");

  Serial.print(" micPeak=");
  Serial.print((unsigned)currentMicPeak());
  Serial.print(" micRms=");
  Serial.print((unsigned long)g_micRms);

  Serial.print(" usr1=");
  Serial.print(g_btnA ? "Pressed" : "Released");
  Serial.print(" raw1=");
  Serial.print(g_btnARaw);
  Serial.print(" usr2=");
  Serial.print(g_btnB ? "Pressed" : "Released");
  Serial.print(" raw2=");
  Serial.print(g_btnBRaw);

  Serial.print(" bl=");
  Serial.print((unsigned)currentBacklightPwm());
  Serial.print("/");
  Serial.print(currentBacklightPercent());
  Serial.print("%");

  Serial.print(" tick=");
  Serial.println((unsigned long)g_frameCounter);
}

// ========================= Setup / loop =========================

static void printHeader() {
  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus 1.47 Factory Dashboard v7.14 D16-live-sense ===");
  Serial.println("nRF52 visual parity + ESP32-S3 D16 live voltage sense display");
  Serial.printf("LCD: %s, CS=D2(%d), DC=D3(%d), SCK=D8(%d), MOSI=D10(%d), RST=%s(%d), BL=D18(%d)\n",
                LCD_USE_ESP32SPI ? "ESP32SPI" : "SWSPI",
                LCD_CS_PIN, LCD_DC_PIN, LCD_SCK_PIN, LCD_MOSI_PIN,
                USE_PRD_PINMAP ? "D17" : "D19", LCD_RST_PIN, LCD_BL_PIN);
  Serial.printf("MIC: CLK=D0(%d), DATA=D1(%d), sample=%d\n", MIC_CLK_PIN, MIC_DATA_PIN, MIC_SAMPLE_RATE_HZ);
  Serial.printf("SD : CS=D6(%d), SCK=D8(%d), MISO=D9(%d), MOSI=D10(%d)\n", SD_CS_PIN, LCD_SCK_PIN, SD_MISO_PIN, LCD_MOSI_PIN);
  Serial.printf("I2C: SDA=D4(%d), SCL=D5(%d), TOUCH_INT=D7(%d), IMU_INT=D14(%d)\n", I2C_SDA_PIN, I2C_SCL_PIN, TOUCH_INT_PIN, IMU_INT_PIN);
  Serial.printf("BTN: USR1=%d, USR2=%d\n", BTN_A_PIN, BTN_B_PIN);
  Serial.printf("D16: ADC=D16(%d), divider=316K/160K, calc ratio=%.3f, live-sense mode\n", BAT_ADC_PIN, BAT_DIVIDER_RATIO);
  Serial.println("TOUCH: Waveshare esp_lcd_touch_axs5106l BSP API");
  Serial.println("SD: async full-mount probe task with LCD bus guard");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Avoid flooding serial with ESP-IDF I2C NACK diagnostics while probing touch.
  // We still report touch status in our own [DASH] line.
  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  printHeader();

  pinMode(BTN_A_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);
  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(SD_MISO_PIN, INPUT_PULLUP);
  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(LCD_CS_PIN, HIGH);

  initBatteryAdc();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  scanI2c();

  initTouch();
  initImu();

  if (!initLcd()) {
    Serial.println("[FAIL] LCD init failed");
    return;
  }

  drawStaticLayout();

  if (!initMic()) {
    Serial.println("[WARN] MIC init failed, VU may stay zero");
  }

  startSdProbeTask();
  Serial.println("[SD] async probe task started");
  requestSdProbe();

  updateTouch();
  updateImu();
  updateButtons();
  handleImuTapEvent();
  updateMicPeak();
  updateBattery();

  // Keep the base UI already drawn above; do not redraw it while the first SD
  // background probe might be using the bus.
  updateUiFast();
  updateUiSlow();
  updateVuMeter();
  updateMicText();
  updateFooter();
  printSerialStatus();

  Serial.println("[BOOT] ESP32-S3 1.47 Dashboard v7.14 D16-live-sense ready");
}

void loop() {
  uint32_t now = millis();

  // Update raw states as often as possible.
  updateMicPeak();
  updateButtons();
  handleImuTapEvent();

  // Full SD.begin() probe runs in a background task.
  // Main loop only consumes results and requests the next probe, so the UI stays smooth.
  consumeSdProbeResult();
  if (now - g_lastSdMs >= SD_REFRESH_MS) {
    g_lastSdMs = now;
    requestSdProbe();
  }

  if (now - g_lastBatMs >= BAT_REFRESH_MS) {
    g_lastBatMs = now;
    updateBattery();
  }

  if (now - g_lastSlowUiMs >= UI_TEXT_SLOW_MS) {
    g_lastSlowUiMs = now;
    updateImu();
    updateTouch();
    updateButtons();
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

  if (now - g_lastFooterMs >= FOOTER_MS) {
    g_lastFooterMs = now;
    updateFooter();
  }

  if (now - g_lastSerialMs >= SERIAL_MS) {
    g_lastSerialMs = now;
    printSerialStatus();
  }

  delay(1);
}
