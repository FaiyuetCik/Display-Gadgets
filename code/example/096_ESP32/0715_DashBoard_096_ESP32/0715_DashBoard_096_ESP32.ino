/*
  XIAO ESP32-S3 PLUS + 0.96 Inch Display
  Factory Dashboard v1.13.3 - KEY1 descending brightness + KEY2 backlight toggle

  References:
    - 0526_DashBoard_096_nRF52840.ino
    - xiao_esp32s3plus_096_factory_dashboard_v0_1.ino
    - completed XIAO ESP32-S3 PLUS BAT presence/percentage algorithm

  Target:
    0.96 Inch Display Powered by XIAO ESP32-S3 PLUS

  Board difference:
    - LCD: 0.96" ST7789, 80 x 160
    - No touch
    - No SD
    - 2 user buttons: USR1 / USR2
    - IMU INT uses D9 on the 0.96 pin map
    - KEY1 cycles backlight brightness; KEY2 toggles screen backlight ON/OFF
    - BAT divider:
        VBAT -- 316K -- ADC(D16) -- 160K -- GND
      This firmware only shows:
        1) whether a real battery is connected
        2) battery percentage when connected
      No charging state is inferred because there is no CHG/STAT/VBUS pin.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include "esp_idf_version.h"
#include <driver/gpio.h>
#include <math.h>

#if ESP_IDF_VERSION_MAJOR >= 5
  #include <driver/i2s_pdm.h>
#else
  #include <driver/i2s.h>
#endif

// ========================= Build switches =========================

#define LCD_USE_ESP32SPI 1
#define REDUCE_PDM_CLK_DRIVE 1

// ========================= Pins =========================

static constexpr uint8_t MIC_CLK_PIN   = D0;
static constexpr uint8_t MIC_DATA_PIN  = D1;
static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t I2C_SDA_PIN   = D4;
static constexpr uint8_t I2C_SCL_PIN   = D5;
static constexpr uint8_t BTN_A_PIN     = D6;   // USR1: backlight ON/OFF
static constexpr uint8_t BTN_B_PIN     = D7;   // USR2: toggle header Hello/Seeed
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t IMU_INT_PIN   = D14;   // 0.96: IMU_INT
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t BAT_ADC_PIN   = D16;
static constexpr uint8_t LCD_RST_PIN   = D17;
static constexpr uint8_t LCD_BL_PIN    = D18;

// ========================= Timing =========================

static constexpr uint32_t UI_FAST_MS       = 100;
static constexpr uint32_t UI_SLOW_MS       = 220;
static constexpr uint32_t UI_VU_MS         = 70;
static constexpr uint32_t BAT_REFRESH_MS   = 600;
static constexpr float VSENSE_D16_DELTA_V   = 0.02f;   // redraw only above ADC jitter
static constexpr float VSENSE_CALC_DELTA_V  = 0.05f;   // redraw only above calculated voltage jitter
static constexpr uint32_t SERIAL_MS        = 700;
static constexpr uint32_t BTN_DEBOUNCE_MS  = 18;
static constexpr uint32_t BTN_ACTION_LOCKOUT_MS = 70;
static constexpr uint32_t TAP_DEBOUNCE_MS  = 220;

// ========================= LCD parameters =========================

static constexpr int LCD_W = 80;
static constexpr int LCD_H = 160;
static constexpr int LCD_ROTATION = 2;
static constexpr bool LCD_IPS = true;
static constexpr bool LCD_INVERT_COLORS = true;

// Verified by previous 0.96 bring-up.
static constexpr int LCD_COL_OFFSET_1 = 24;
static constexpr int LCD_ROW_OFFSET_1 = 0;
static constexpr int LCD_COL_OFFSET_2 = 24;
static constexpr int LCD_ROW_OFFSET_2 = 0;

#if LCD_USE_ESP32SPI
Arduino_DataBus *lcdBus = new Arduino_ESP32SPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  LCD_SCK_PIN,
  LCD_MOSI_PIN
);
#else
Arduino_DataBus *lcdBus = new Arduino_SWSPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  LCD_SCK_PIN,
  LCD_MOSI_PIN,
  GFX_NOT_DEFINED
);
#endif

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

// 0.96 panel visual aliases.
// Same convention as nRF52840 0.96 reference: color order appears remapped.
static constexpr uint16_t V_RED     = C_BLUE;
static constexpr uint16_t V_BLUE    = C_RED;
static constexpr uint16_t V_YELLOW  = C_CYAN;
static constexpr uint16_t V_CYAN    = C_YELLOW;
static constexpr uint16_t V_GREEN   = C_GREEN;
static constexpr uint16_t V_WHITE   = C_WHITE;

// ========================= Battery =========================
//
// S3 battery policy:
//   - battery presence + percentage only
//   - no charging inference
//
// Divider:
//   VBAT -- 316K -- ADC -- 160K -- GND
//   ratio = 2.975

static constexpr float BAT_R_TOP_KOHM        = 316.0f;
static constexpr float BAT_R_BOTTOM_KOHM     = 160.0f;
static constexpr float BAT_DIVIDER_RATIO     = (BAT_R_TOP_KOHM + BAT_R_BOTTOM_KOHM) / BAT_R_BOTTOM_KOHM;
static constexpr float BAT_CAL_FACTOR        = 1.000f;

static constexpr float BAT_VALID_MIN_V       = 2.80f;
static constexpr float BAT_VALID_MAX_V       = 4.60f;
static constexpr uint16_t BAT_PRESENT_MIN_MV = 850;
static constexpr uint16_t BAT_FLOAT_RANGE_MV = 90;

static constexpr uint8_t BAT_VALID_CONFIRM_COUNT = 3;
static constexpr uint8_t BAT_INVALID_CONFIRM_COUNT = 3;

// USB-only cold boot guard.
// Observed on 1.47 S3: USB-only can show stable fake ~70%.
static constexpr bool BAT_COLD_BOOT_HIGH_AS_USB = true;
static constexpr int  BAT_COLD_BOOT_SUSPECT_PCT = 60;

// Reject impossible high jump after a known real battery.
static constexpr float BAT_IMPOSSIBLE_JUMP_V     = 0.28f;
static constexpr int   BAT_IMPOSSIBLE_JUMP_PCT   = 22;
static constexpr uint8_t BAT_FLOAT_REJECT_STREAK = 2;

// Reinsert unlock after FLOAT_LOCK.
static constexpr float BAT_REINSERT_DROP_V       = 0.16f;

// USB-first then battery-insert compensation.
static constexpr float BAT_USB_INSERT_SURFACE_COMP_V = 0.085f;
static constexpr float BAT_USB_INSERT_RELEASE_DROP_V = 0.045f;
static constexpr float BAT_EFFECTIVE_MIN_V           = 3.25f;

// UI percentage limiter.
static constexpr uint32_t BAT_PCT_RISE_INTERVAL_MS = 90000;

// ADC sampling.
static constexpr uint8_t BAT_SAMPLE_COUNT = 12;
static constexpr uint16_t BAT_SAMPLE_DELAY_US = 700;

// ========================= MIC =========================

static constexpr int MIC_SAMPLE_RATE_HZ = 16000;
static constexpr bool MIC_CLK_INVERT = false;
static constexpr size_t MIC_SAMPLES_PER_READ = 256;

#if ESP_IDF_VERSION_MAJOR >= 5
static i2s_chan_handle_t g_i2sRxChan = nullptr;
#else
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
#endif

int16_t g_pdmBuf[MIC_SAMPLES_PER_READ];

uint32_t g_micPeak = 0;
uint32_t g_micRms = 0;
uint32_t g_micLastUpdateMs = 0;

float g_vuSmooth = 0.0f;
float g_vuFast = 0.0f;
int g_vuDisplaySegments = 0;
int g_cachedVuSegments = -1;
int g_cachedVuWidth = -1;
uint16_t g_cachedVuColor = 0xFFFF;
uint32_t g_lastVuStepMs = 0;

// VU anti-flicker state.
int g_vuCandidateSegments = 0;
uint8_t g_vuCandidateCount = 0;
uint32_t g_lastVuRenderMs = 0;
uint32_t g_vuFallGuardUntilMs = 0;

float g_micNoiseFloor = 65.0f;
bool g_micNoiseReady = false;
uint32_t g_micNoiseStartMs = 0;

// ========================= IMU =========================

enum ImuType {
  IMU_NONE = 0,
  IMU_QMI8658,
  IMU_LSM6DS3
};

ImuType g_imuType = IMU_NONE;
uint8_t g_imuAddr = 0;
bool g_imuOk = false;

float g_ax = 0, g_ay = 0, g_az = 0;
float g_gx = 0, g_gy = 0, g_gz = 0;

volatile bool g_imuIntFlag = false;
bool g_imuTapConfigured = false;
uint32_t g_doubleTapCount = 0;
uint32_t g_lastTapMs = 0;

// ========================= General state =========================

bool g_lcdOk = false;
bool g_micOk = false;
bool g_blOn = true;
static const uint8_t BL_LEVELS[] = {255, 192, 128, 64};  // 100%, 75%, 50%, 25%
static constexpr uint8_t BL_LEVEL_COUNT = sizeof(BL_LEVELS) / sizeof(BL_LEVELS[0]);
uint8_t g_blLevelIndex = 0;  // boot at 100%; KEY1: 100 -> 75 -> 50 -> 25 -> 100
bool g_headerSeeed = false;

// Buttons
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

// Battery
struct BatteryState {
  uint16_t raw = 0;
  uint16_t rawMin = 0;
  uint16_t rawMax = 0;

  uint16_t mv = 0;
  uint16_t mvMin = 0;
  uint16_t mvMax = 0;
  uint16_t spreadMv = 0;

  float vadc = 0.0f;
  float rawVbat = 0.0f;
  float vbat = 0.0f;
  int percent = 0;

  bool valid = false;
  const char *state = "BOOT";
};

BatteryState g_bat;
BatteryState g_vsense;       // raw D16 ADC measurement before BAT/USB filtering
BatteryState g_lastGoodBat;
bool g_haveLastGoodBat = false;
const char *g_batFilterState = "BOOT";

uint8_t g_batValidStreak = 0;
uint8_t g_batInvalidStreak = 0;

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

// UI timing/cache
uint32_t g_lastFastMs = 0;
uint32_t g_lastSlowMs = 0;
uint32_t g_lastVuMs = 0;
uint32_t g_lastBatMs = 0;
uint32_t g_lastSerialMs = 0;
uint32_t g_frameCounter = 0;

String cache_header = "";
String cache_sys = "";
String cache_bl = "";
float g_d16ShownVadc = -1.0f;
float g_d16ShownCalc = -1.0f;
String cache_acc = "";
String cache_gyr = "";
String cache_tap = "";
String cache_raw = "";
String cache_ax_txt = "";
String cache_ay_txt = "";
String cache_az_txt = "";
String cache_gx_txt = "";
String cache_gy_txt = "";
String cache_gz_txt = "";
bool g_imuTinyLabelsDrawn = false;

// Battery UI snapshot, tiny UI anti-flicker.
bool g_batUiInit = false;
bool g_batUiUsb = true;
bool g_batUiValid = false;
int g_batUiPct = -1;
uint32_t g_lastBatUiCommitMs = 0;

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

// MIC: title and volume bar on same row, Raw below.
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

// ========================= Helpers =========================

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

  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t *val) {
  return i2cRead(addr, reg, val, 1);
}

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
  static const Point table[] = {
    {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.92f, 70}, {3.85f, 60},
    {3.79f, 50}, {3.72f, 40}, {3.66f, 30}, {3.58f, 20}, {3.50f, 10}, {3.30f, 0}
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

static uint16_t tinyGlyph3x5(char ch) {
  // 3x5 bitmap font, bits are row-major.
  // This is used only for the 0.96 footer, where the built-in 5x7 font cannot
  // fit "XIAO ESP32-S3 PLUS" inside 80 px, including lowercase letters in Plus.
  switch (ch) {
    case 'A': return 0b010101111101101;
    case 'C': return 0b111100100100111;
    case 'D': return 0b110101101101110;
    case 'E': return 0b111100110100111;
    case 'I': return 0b111010010010111;
    case 'L': return 0b100100100100111;
    case 'O': return 0b111101101101111;
    case 'P': return 0b110101110100100;
    case 'S': return 0b111100111001111;
    case 'T': return 0b111010010010010;
    case 'U': return 0b101101101101111;
    case 'V': return 0b101101101101010;
    case 'X': return 0b101101010101101;
    case 'a': return 0b000011101101011;
    case 'p': return 0b000110101110100;
    case 'G': return 0b111100101101111;
    case '0': return 0b111101101101111;
    case '1': return 0b010110010010111;
    case '2': return 0b111001111100111;
    case '3': return 0b111001111001111;
    case '4': return 0b101101111001001;
    case '5': return 0b111100111001111;
    case '6': return 0b111100111101111;
    case '7': return 0b111001010010010;
    case '8': return 0b111101111101111;
    case '9': return 0b111101111001111;
    case '.': return 0b000000000000010;
    case '+': return 0b000010111010000;
    case '-': return 0b000000111000000;
    case '|': return 0b010010010010010;
    case 'l': return 0b010010010010011;
    case 's': return 0b111100111001111; // match uppercase S glyph for better readability on 3x5 footer
    case 'u': return 0b000000101101111;
    case ' ': return 0;
    default:  return 0;
  }
}

static void drawTiny3x5Text(int x, int y, const char *s, uint16_t c, uint16_t bg) {
  if (!g_lcdOk) return;

  acquireForLcd();

  int cx = x;
  while (*s) {
    char ch = *s++;
    uint16_t bits = tinyGlyph3x5(ch);

    if (ch == ' ') {
      gfx->fillRect(cx, y, 3, 5, bg);
      cx += 4;
      continue;
    }

    for (int row = 0; row < 5; row++) {
      for (int col = 0; col < 3; col++) {
        int bitIndex = 14 - (row * 3 + col);
        bool on = (bits >> bitIndex) & 0x01;
        gfx->drawPixel(cx + col, y + row, on ? c : bg);
      }
    }
    cx += 4;
  }
}

static void drawTiny3x5Field(int x, int y, const String &s, uint16_t c, uint8_t fieldChars) {
  if (!g_lcdOk) return;

  acquireForLcd();

  int w = fieldChars * 4 - 1;
  gfx->fillRect(x, y, w, 5, C_BLACK);
  drawTiny3x5Text(x, y, s.c_str(), c, C_BLACK);
}

static void drawTinyText(int x, int y, const char *s, uint16_t c) {
  if (!g_lcdOk) return;
  acquireForLcd();
  gfx->setTextSize(1);
  gfx->setTextColor(c, C_BLACK);
  gfx->setCursor(x, y);
  gfx->print(s);
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

void IRAM_ATTR btnAIrqIsr() {
  if (g_btnAIrqCount < 250) g_btnAIrqCount++;
}

void IRAM_ATTR btnBIrqIsr() {
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

// ========================= D16 voltage sense measurement =========================

static uint16_t readBatteryMilliVolts() {
  return (uint16_t)analogReadMilliVolts(BAT_ADC_PIN);
}

static BatteryState readBatteryMeasured() {
  BatteryState r;

  uint32_t rawSum = 0;
  uint32_t mvSum = 0;
  uint16_t rawMin = 65535;
  uint16_t rawMax = 0;
  uint16_t mvMin = 65535;
  uint16_t mvMax = 0;

  for (uint8_t i = 0; i < BAT_SAMPLE_COUNT; i++) {
    uint16_t raw = (uint16_t)analogRead(BAT_ADC_PIN);
    uint16_t mv = readBatteryMilliVolts();

    rawSum += raw;
    mvSum += mv;

    if (raw < rawMin) rawMin = raw;
    if (raw > rawMax) rawMax = raw;
    if (mv < mvMin) mvMin = mv;
    if (mv > mvMax) mvMax = mv;

    delayMicroseconds(BAT_SAMPLE_DELAY_US);
  }

  r.raw = rawSum / BAT_SAMPLE_COUNT;
  r.rawMin = rawMin;
  r.rawMax = rawMax;

  r.mv = mvSum / BAT_SAMPLE_COUNT;
  r.mvMin = mvMin;
  r.mvMax = mvMax;
  r.spreadMv = mvMax - mvMin;

  r.vadc = r.mv / 1000.0f;
  r.rawVbat = r.vadc * BAT_DIVIDER_RATIO * BAT_CAL_FACTOR;
  r.vbat = r.rawVbat;
  r.percent = lipoPercent(r.vbat);
  r.valid = false;
  r.state = "MEASURED";

  return r;
}

static int updatePresencePercent(int measuredPercent, bool freshBattery) {
  measuredPercent = constrain(measuredPercent, 0, 100);
  uint32_t now = millis();

  if (freshBattery || g_batDisplayPercent < 0) {
    g_batDisplayPercent = measuredPercent;
    g_batLastPctRiseMs = now;
    return g_batDisplayPercent;
  }

  if (measuredPercent < g_batDisplayPercent) {
    g_batDisplayPercent = measuredPercent;
    return g_batDisplayPercent;
  }

  if (measuredPercent > g_batDisplayPercent &&
      (now - g_batLastPctRiseMs >= BAT_PCT_RISE_INTERVAL_MS)) {
    g_batDisplayPercent++;
    g_batLastPctRiseMs = now;
  }

  return g_batDisplayPercent;
}

static bool isImpossibleHighJump(const BatteryState &m) {
  if (!g_haveLastGoodBat) return false;

  float dv = m.rawVbat - g_lastGoodBat.rawVbat;
  int dp = m.percent - g_lastGoodBat.percent;

  return (dv >= BAT_IMPOSSIBLE_JUMP_V) || (dp >= BAT_IMPOSSIBLE_JUMP_PCT);
}

static bool isColdBootUsbSuspect(const BatteryState &m, bool stable) {
  if (!BAT_COLD_BOOT_HIGH_AS_USB) return false;
  if (g_haveLastGoodBat) return false;
  if (!stable) return false;
  return (m.percent >= BAT_COLD_BOOT_SUSPECT_PCT);
}

static void markBatteryAbsentLikeUsb(const BatteryState &measured, const char *stateName) {
  g_batUsbOnlySeen = true;
  g_batUsbInsertCompActive = false;
  g_batUsbInsertRawStartV = 0.0f;

  g_bat = measured;
  g_bat.valid = false;
  g_bat.percent = 0;
  g_bat.vbat = measured.rawVbat;
  g_bat.state = stateName;
  g_batFilterState = stateName;

  cache_sys = "";
}

static float batteryEffectiveVoltageForPercent(float rawV, bool freshBattery) {
  if (freshBattery && g_batUsbOnlySeen && !g_batUsbInsertCompActive) {
    g_batUsbInsertCompActive = true;
    g_batUsbInsertRawStartV = rawV;

    Serial.print("[BAT] USB-first battery insert compensation ON rawStart=");
    Serial.println(g_batUsbInsertRawStartV, 3);
  }

  if (g_batUsbInsertCompActive) {
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

static void updateBattery() {
  BatteryState measured = readBatteryMeasured();

  // UI reports D16 voltage sense data, not battery presence.
  // Keep raw measured values before BAT/USB filtering or percentage smoothing.
  g_vsense = measured;

  bool plausible = (measured.mv >= BAT_PRESENT_MIN_MV &&
                    measured.rawVbat >= BAT_VALID_MIN_V &&
                    measured.rawVbat <= BAT_VALID_MAX_V);

  bool stable = plausible && (measured.spreadMv <= BAT_FLOAT_RANGE_MV);

  if (isColdBootUsbSuspect(measured, stable)) {
    if (g_batColdUsbSuspectStreak < 255) g_batColdUsbSuspectStreak++;

    if (g_batColdUsbSuspectStreak >= BAT_VALID_CONFIRM_COUNT) {
      g_batValidStreak = 0;
      if (g_batInvalidStreak < 255) g_batInvalidStreak++;
      markBatteryAbsentLikeUsb(measured, "USB_BOOT_HIGH");
      return;
    }
  } else {
    if (g_batColdUsbSuspectStreak > 0) g_batColdUsbSuspectStreak--;
  }

  if (g_batFloatReject && stable) {
    bool looksLikeReinsert = (g_batLastRealV > 0.0f) &&
                             (measured.rawVbat <= g_batLastRealV + BAT_REINSERT_DROP_V);

    if (!looksLikeReinsert) {
      g_batValidStreak = 0;
      if (g_batInvalidStreak < 255) g_batInvalidStreak++;
      markBatteryAbsentLikeUsb(measured, "FLOAT_LOCK");
      return;
    }

    if (g_batReinsertStreak < 255) g_batReinsertStreak++;

    if (g_batReinsertStreak < BAT_VALID_CONFIRM_COUNT) {
      markBatteryAbsentLikeUsb(measured, "REINSERT_CAND");
      return;
    }

    g_batFloatReject = false;
    g_batFloatRejectStreak = 0;
    g_batReinsertStreak = 0;
    g_batColdUsbSuspectStreak = 0;
    g_haveLastGoodBat = false;
    g_batDisplayPercent = -1;
  }

  if (stable && isImpossibleHighJump(measured)) {
    if (g_batFloatRejectStreak < 255) g_batFloatRejectStreak++;

    if (g_batFloatRejectStreak >= BAT_FLOAT_REJECT_STREAK) {
      g_batFloatReject = true;
      g_batReinsertStreak = 0;
      g_batValidStreak = 0;
      if (g_batInvalidStreak < 255) g_batInvalidStreak++;
      markBatteryAbsentLikeUsb(measured, "JUMP_FLOAT");
      return;
    }
  } else {
    if (g_batFloatRejectStreak > 0) g_batFloatRejectStreak--;
  }

  if (stable) {
    bool freshBattery = !g_haveLastGoodBat;

    if (g_batValidStreak < 255) g_batValidStreak++;
    g_batInvalidStreak = 0;

    measured.valid = true;

    float effectiveV = batteryEffectiveVoltageForPercent(measured.rawVbat, freshBattery);

    if (g_haveLastGoodBat) {
      effectiveV = g_lastGoodBat.vbat * 0.88f + effectiveV * 0.12f;
    }

    measured.vbat = effectiveV;
    measured.percent = updatePresencePercent(lipoPercent(measured.vbat), freshBattery);

    if (g_batValidStreak >= BAT_VALID_CONFIRM_COUNT) {
      g_bat = measured;
      g_bat.state = g_batUsbInsertCompActive ? "BAT_USB_COMP" : "BAT_PRESENT";
      g_batFilterState = g_bat.state;

      g_lastGoodBat = g_bat;
      g_lastGoodBat.valid = true;
      g_haveLastGoodBat = true;

      g_batLastRealV = g_bat.vbat;
      g_batLastRealPercent = g_bat.percent;

      if (!g_batUsbOnlySeen) {
        g_batUsbInsertCompActive = false;
        g_batUsbInsertRawStartV = 0.0f;
      }
    } else if (g_haveLastGoodBat) {
      g_bat = g_lastGoodBat;
      g_bat.state = "BAT_HOLD";
      g_batFilterState = g_bat.state;
    } else {
      g_bat = measured;
      g_bat.state = "BAT_CAND";
      g_batFilterState = g_bat.state;
    }

    return;
  }

  g_batValidStreak = 0;
  if (g_batInvalidStreak < 255) g_batInvalidStreak++;

  if (!g_batFloatReject && g_haveLastGoodBat && g_batInvalidStreak < BAT_INVALID_CONFIRM_COUNT) {
    g_bat = g_lastGoodBat;
    g_bat.valid = true;
    g_bat.state = "BAT_HOLD";
    g_batFilterState = g_bat.state;
    return;
  }

  markBatteryAbsentLikeUsb(measured, plausible ? "FLOAT" : "USB_PWR");
}

static bool batteryUsbTextMode() {
  return !g_bat.valid;
}

static void updateBatteryUiSnapshot() {
  uint32_t now = millis();

  bool newUsb = batteryUsbTextMode();
  bool newValid = g_bat.valid && !newUsb;
  int newPct = newValid ? constrain(g_bat.percent, 0, 100) : -1;

  if (!g_batUiInit) {
    g_batUiInit = true;
    g_batUiUsb = newUsb;
    g_batUiValid = newValid;
    g_batUiPct = newPct;
    g_lastBatUiCommitMs = now;
    return;
  }

  // USB <-> BAT is a real state transition; update immediately.
  if (newUsb != g_batUiUsb || newValid != g_batUiValid) {
    g_batUiUsb = newUsb;
    g_batUiValid = newValid;
    g_batUiPct = newPct;
    g_lastBatUiCommitMs = now;
    return;
  }

  if (newUsb) {
    return;
  }

  // Percent hysteresis:
  // - ignore +/-1% jitter
  // - commit >=2% every 1.5s
  // - commit >=4% immediately
  int pctDiff = abs(newPct - g_batUiPct);
  if (pctDiff >= 4 || (pctDiff >= 2 && (now - g_lastBatUiCommitMs) > 1500UL)) {
    g_batUiPct = newPct;
    g_lastBatUiCommitMs = now;
  }
}

// ========================= LCD/UI =========================

static bool initLcd() {
  applyBacklight();

#if LCD_USE_ESP32SPI
  bool ok = gfx->begin(40000000);
#else
  bool ok = gfx->begin();
#endif

  if (!ok) {
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

static String voltageD16LineText(float d16) {
  char buf[16];
  snprintf(buf, sizeof(buf), "D16 %.2fV", d16);
  return String(buf);
}

static String voltageCalcLineText(float calc) {
  char buf[18];
  snprintf(buf, sizeof(buf), "Calc %.2fV", calc);
  return String(buf);
}

static void drawBatteryIconTiny(int x, int y, bool valid, int pct) {
  if (!g_lcdOk) return;

  acquireForLcd();

  uint16_t c = valid ? colorByPercent(pct) : V_RED;

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

static void drawBatteryRowTiny() {
  if (!g_lcdOk) return;

  float d16Now = (g_d16ShownVadc >= 0.0f) ? g_d16ShownVadc : roundf(g_vsense.vadc * 100.0f) / 100.0f;
  float calcNow = (g_d16ShownCalc >= 0.0f) ? g_d16ShownCalc : roundf(g_vsense.rawVbat * 100.0f) / 100.0f;

  String d16Line = voltageD16LineText(d16Now);
  String calcLine = voltageCalcLineText(calcNow);

  acquireForLcd();

  // Two-line voltage display. Clear the full SYS dynamic area to avoid residue
  // from the previous one-line BAT/USB/D16 layout.
  gfx->fillRect(SECTION_X0, Y_SYS_ROW1 - 2, 72, 21, C_BLACK);

  gfx->setTextSize(1);
  gfx->setTextColor(V_YELLOW, C_BLACK);
  gfx->setCursor(SECTION_X0 + 2, Y_SYS_ROW1);
  gfx->print(d16Line);

  gfx->setTextColor(V_YELLOW, C_BLACK);
  gfx->setCursor(SECTION_X0 + 2, Y_SYS_ROW2);
  gfx->print(calcLine);
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
  // Full product name rendered with a custom 3x5 footer font.
  // Width: 18 chars * 4 px - 1 = 71 px, fits the 80 px panel.
  acquireForLcd();
  gfx->fillRect(0, ROW_FOOT - 1, LCD_W, 8, C_BLACK);
  drawTiny3x5Text(4, ROW_FOOT + 1, "XIAO ESP32-S3 PLUS", V_YELLOW, C_BLACK);

  cache_header = "";
  cache_sys = "";
  cache_bl = "";
  g_d16ShownVadc = -1.0f;
  g_d16ShownCalc = -1.0f;
  cache_acc = "";
  cache_gyr = "";
  cache_tap = "";
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

// LSM6 double-tap registers.
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
  ok &= i2cWrite8(g_imuAddr, LSM_REG_CTRL1_XL, 0x60);
  ok &= i2cWrite8(g_imuAddr, LSM_REG_TAP_CFG, 0x8E);
  ok &= i2cWrite8(g_imuAddr, LSM_REG_TAP_THS_6D, 0x0C);
  ok &= i2cWrite8(g_imuAddr, LSM_REG_INT_DUR2, 0x7F);
  ok &= i2cWrite8(g_imuAddr, LSM_REG_WAKE_UP_THS, 0x80);
  ok &= i2cWrite8(g_imuAddr, LSM_REG_MD1_CFG, 0x08);

  uint8_t dummy = 0;
  (void)i2cRead8(g_imuAddr, LSM_REG_TAP_SRC, &dummy);

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIntIsr, RISING);

  g_imuTapConfigured = ok;
  Serial.print("[IMU] LSM6 double tap D9 ");
  Serial.println(ok ? "OK" : "FAILED");

  return ok;
}

static void initImu() {
  pinMode(IMU_INT_PIN, INPUT_PULLUP);

  g_imuType = IMU_NONE;
  g_imuAddr = 0;
  g_imuOk = false;
  g_imuTapConfigured = false;

  if (initLsm(0x6A)) {
    initImuDoubleTap();
    return;
  }
  if (initLsm(0x6B)) {
    initImuDoubleTap();
    return;
  }
  if (initQmi(0x6B)) {
    initImuDoubleTap();
    return;
  }
  if (initQmi(0x6A)) {
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

static void handleImuTapEvent() {
  if (!g_imuTapConfigured || g_imuType != IMU_LSM6DS3) return;

  bool shouldCheck = false;

  noInterrupts();
  if (g_imuIntFlag) {
    g_imuIntFlag = false;
    shouldCheck = true;
  }
  interrupts();

  if (digitalRead(IMU_INT_PIN) == HIGH) {
    shouldCheck = true;
  }

  if (!shouldCheck) return;

  uint8_t src = 0;
  if (!i2cRead8(g_imuAddr, LSM_REG_TAP_SRC, &src)) return;

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

static void updateMic() {
  if (!g_micOk) return;

  size_t bytesRead = 0;
  if (!readMic(&bytesRead, 2)) return;

  uint32_t count = bytesRead / sizeof(int16_t);
  if (!count) return;

  int64_t sum = 0;
  for (uint32_t i = 0; i < count; i++) {
    sum += g_pdmBuf[i];
  }

  int32_t mean = (int32_t)(sum / count);
  uint32_t peak = 0;
  uint64_t sq = 0;

  for (uint32_t i = 0; i < count; i++) {
    int32_t v = (int32_t)g_pdmBuf[i] - mean;
    uint32_t a = v < 0 ? (uint32_t)(-v) : (uint32_t)v;
    if (a > peak) peak = a;
    sq += (uint64_t)a * a;
  }

  g_micPeak = peak;
  g_micRms = (uint32_t)sqrt((double)sq / count);
  g_micLastUpdateMs = millis();
}

static uint32_t currentMicPeak() {
  uint32_t now = millis();

  if (now - g_micLastUpdateMs > 120) {
    g_micPeak = (uint32_t)(g_micPeak * 0.80f);
    g_micRms = (uint32_t)(g_micRms * 0.80f);
  }

  return g_micPeak;
}

// ========================= VU =========================

static int vuLevelToSegments(float level) {
  if (level <= 0.0f) return 0;

  if (level < 0.12f) return 1;
  if (level < 0.22f) return 2;
  if (level < 0.34f) return 3;
  if (level < 0.48f) return 4;
  if (level < 0.64f) return 5;
  if (level < 0.80f) return 6;
  if (level < 0.93f) return 7;
  return 8;
}

static int debounceVuDesired(int desired, uint32_t now) {
  if (desired == g_vuDisplaySegments) {
    g_vuCandidateSegments = desired;
    g_vuCandidateCount = 0;
    return g_vuDisplaySegments;
  }

  if (desired > g_vuDisplaySegments) {
    if (now < g_vuFallGuardUntilMs && desired <= g_vuDisplaySegments + 1) {
      return g_vuDisplaySegments;
    }
  }

  if (desired < g_vuDisplaySegments) {
    g_vuFallGuardUntilMs = now + 150;
    return desired;
  }

  if (desired != g_vuCandidateSegments) {
    g_vuCandidateSegments = desired;
    g_vuCandidateCount = 1;
  } else if (g_vuCandidateCount < 255) {
    g_vuCandidateCount++;
  }

  if (g_vuCandidateCount >= 2 || desired >= g_vuDisplaySegments + 2) {
    return desired;
  }

  return g_vuDisplaySegments;
}

static uint16_t vuColorForSegment(int i) {
  if (i >= 6) return V_RED;
  if (i >= 4) return V_YELLOW;
  return V_GREEN;
}

static void drawVuBlocks(int segs) {
  if (!g_lcdOk) return;

  acquireForLcd();
  for (int i = 0; i < VU_SEG_COUNT; i++) {
    int x = VU_X + 1 + i * (VU_SEG_W + VU_GAP);
    uint16_t c = (i < segs) ? vuColorForSegment(i) : C_BLACK;
    gfx->fillRect(x, VU_Y + 2, VU_SEG_W, VU_H - 4, c);
  }

  g_cachedVuSegments = segs;
}

static void updateVu() {
  if (!g_lcdOk) return;

  uint16_t peak = currentMicPeak();

  // Same 0.96 compact VU behavior: RMS main source, peak helps attack.
  float metric = (float)g_micRms * 0.82f + (float)peak * 0.012f;
  if (metric < 0.0f) metric = 0.0f;
  if (metric > 240.0f) metric = 240.0f;

  uint32_t now = millis();

  if (!g_micNoiseReady) {
    if (g_micNoiseStartMs == 0) g_micNoiseStartMs = now;
    g_micNoiseFloor = g_micNoiseFloor * 0.84f + metric * 0.16f;
    if (now - g_micNoiseStartMs > 1200UL) g_micNoiseReady = true;
  } else {
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

  if (target > g_vuFast) {
    g_vuFast = g_vuFast * 0.35f + target * 0.65f;
  } else {
    g_vuFast = g_vuFast * 0.22f + target * 0.78f;
  }

  if (g_vuFast > g_vuSmooth) {
    g_vuSmooth = g_vuSmooth * 0.58f + g_vuFast * 0.42f;
  } else {
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

static String formatTapText(uint32_t count) {
  // 0.96 screen is only 80 px wide.
  // "Tap 10" is 6 chars = 36 px at text size 1.
  // Keep 2-digit values readable and cap 3+ digits to avoid overflow.
  if (count <= 99) return String("TAP ") + String(count);
  return String("T99+");
}

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
  // This keeps rows like "A 0.0 -0.1 1.0" inside 80 px with the 3x5 font.
  if (v > 9.9f) v = 9.9f;
  if (v < -9.9f) v = -9.9f;
  if (fabsf(v) < 0.05f) v = 0.0f;

  char buf[8];
  snprintf(buf, sizeof(buf), "%.1f", v);
  return String(buf);
}

static String fmtAxis1DecFixed(float v) {
  // Fixed 4-char output: +0.0 / -0.5 / +9.9
  // Fixed length lets the 3x5 renderer overwrite old pixels without clearing
  // the whole line, which removes the visible IMU flicker.
  if (v > 9.9f) v = 9.9f;
  if (v < -9.9f) v = -9.9f;
  if (fabsf(v) < 0.05f) v = 0.0f;

  char buf[8];
  snprintf(buf, sizeof(buf), "%+.1f", v);
  return String(buf);
}

static void updateUiFast() {
  if (!g_lcdOk) return;

  String header = g_headerSeeed ? "XIAO" : "Hello";
  if (header != cache_header) {
    cache_header = header;
    drawHeader();
  }

  float d16Now = roundf(g_vsense.vadc * 100.0f) / 100.0f;
  float calcNow = roundf(g_vsense.rawVbat * 100.0f) / 100.0f;

  bool voltageChanged =
    (g_d16ShownVadc < 0.0f) ||
    (g_d16ShownCalc < 0.0f) ||
    (fabsf(d16Now - g_d16ShownVadc) >= VSENSE_D16_DELTA_V) ||
    (fabsf(calcNow - g_d16ShownCalc) >= VSENSE_CALC_DELTA_V);

  if (voltageChanged || cache_sys.length() == 0) {
    g_d16ShownVadc = d16Now;
    g_d16ShownCalc = calcNow;
    cache_sys = voltageD16LineText(d16Now) + String("|") + voltageCalcLineText(calcNow);
    drawBatteryRowTiny();
  }

  String tap = formatTapText(g_doubleTapCount);
  if (tap != cache_tap) {
    cache_tap = tap;
    // Use the 3x5 font so "MOTION" and "TAP" can have a clear space
    // while Tap 10 / Tap 99 still fit inside 80 px.
    acquireForLcd();
    gfx->fillRect(45, Y_TAP, 32, 8, C_BLACK);
    drawTiny3x5Text(48, Y_TAP + 2, tap.c_str(), V_YELLOW, C_BLACK);
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
      // Clear once after the static layout. Do not draw default 5x7 A/G labels,
      // otherwise they overlap the 3x5 IMU values.
      acquireForLcd();
      gfx->fillRect(SECTION_X0 + 1, Y_ACC, 76, 8, C_BLACK);
      gfx->fillRect(SECTION_X0 + 1, Y_GYR, 76, 8, C_BLACK);
      drawTiny3x5Field(SECTION_X0 + 1, Y_ACC + 1, String("A"), V_YELLOW, 1);
      drawTiny3x5Field(SECTION_X0 + 1, Y_GYR + 1, String("G"), V_YELLOW, 1);
      g_imuTinyLabelsDrawn = true;
      cache_ax_txt = cache_ay_txt = cache_az_txt = "";
      cache_gx_txt = cache_gy_txt = cache_gz_txt = "";
    }

    // Keep the readable spaced small-font format:
    // A 0.0 0.9 -0.2 / G 0.0 0.1 -0.4
    // Each value is redrawn only in its own small field to avoid full-line flicker.
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
      drawTiny3x5Text(SECTION_X0 + 1, Y_ACC + 1, "IMU NO", V_RED, C_BLACK);
      g_imuTinyLabelsDrawn = false;
      cache_ax_txt = cache_ay_txt = cache_az_txt = "";
      cache_gx_txt = cache_gy_txt = cache_gz_txt = "";
    }
  }
}

static void printSerialStatus() {
  Serial.print("[DASH096_S3] d16_adc=");
  Serial.print(g_vsense.vadc, 3);
  Serial.print("V calc=");
  Serial.print(g_vsense.rawVbat, 3);
  Serial.print("V raw=");
  Serial.print(g_vsense.raw);
  Serial.print(" mv=");
  Serial.print(g_vsense.mv);
  Serial.print(" spread=");
  Serial.print((unsigned)g_vsense.spreadMv);
  Serial.print(" vstate=");
  Serial.print(g_batFilterState);
  Serial.print(" imu=");
  Serial.print(g_imuOk ? "OK" : "NO");
  Serial.print(" tapCfg=");
  Serial.print(g_imuTapConfigured ? "Y" : "N");
  Serial.print(" tap=");
  Serial.print((unsigned long)g_doubleTapCount);
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
  Serial.print(" micFloor=");
  Serial.print(g_micNoiseFloor, 1);
  Serial.print(" vu=");
  Serial.print(g_vuSmooth, 2);
  Serial.print(" fast=");
  Serial.print(g_vuFast, 2);
  Serial.print(" vuSeg=");
  Serial.print(g_vuDisplaySegments);
  Serial.print(" usr1=");
  Serial.print(g_btnA ? "P" : "R");
  Serial.print(" usr2=");
  Serial.print(g_btnB ? "P" : "R");
  Serial.print(" bl=");
  Serial.print(g_blOn ? "ON" : "OFF");
  Serial.print(" blLevel=");
  Serial.print(backlightStatusText());
  Serial.print(" frame=");
  Serial.println((unsigned long)g_frameCounter++);
}

// ========================= Arduino =========================

static void printHeader() {
  Serial.println();
  Serial.println("=== XIAO ESP32-S3 PLUS 0.96 Factory Dashboard v1.13.3 KEY1-desc-brightness KEY2-backlight ===");
  Serial.println("[UI] nRF52840 0.96 visual parity");
  Serial.println("[D16] voltage sense display: two-line D16 ADC + calculated external voltage");
  Serial.printf("[LCD] ST7789 %dx%d rotation=%d offset=%d,%d,%d,%d invert=%s bus=%s\n",
                LCD_W, LCD_H, LCD_ROTATION,
                LCD_COL_OFFSET_1, LCD_ROW_OFFSET_1, LCD_COL_OFFSET_2, LCD_ROW_OFFSET_2,
                LCD_INVERT_COLORS ? "Y" : "N",
                LCD_USE_ESP32SPI ? "ESP32SPI" : "SWSPI");
  Serial.printf("[PIN] MIC CLK=D0(%d), DATA=D1(%d)\n", MIC_CLK_PIN, MIC_DATA_PIN);
  Serial.printf("[PIN] LCD CS=D2(%d), DC=D3(%d), SCK=D8(%d), MOSI=D10(%d), RST=D17(%d), BL=D18(%d)\n",
                LCD_CS_PIN, LCD_DC_PIN, LCD_SCK_PIN, LCD_MOSI_PIN, LCD_RST_PIN, LCD_BL_PIN);
  Serial.printf("[PIN] I2C SDA=D4(%d), SCL=D5(%d), IMU_INT=D9(%d), BAT=D16(%d)\n",
                I2C_SDA_PIN, I2C_SCL_PIN, IMU_INT_PIN, BAT_ADC_PIN);
  Serial.printf("[BTN] KEY1/USR1=D6(%d) Brightness cycle, KEY2/USR2=D7(%d) Backlight ON/OFF\n", BTN_A_PIN, BTN_B_PIN);
}

void setup() {
  Serial.begin(115200);
  delay(800);

  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  printHeader();

  pinMode(BTN_A_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);
  pinMode(IMU_INT_PIN, INPUT_PULLUP);

  g_btnARaw = digitalRead(BTN_A_PIN);
  g_btnBRaw = digitalRead(BTN_B_PIN);
  g_btnA = (g_btnARaw == LOW);
  g_btnB = (g_btnBRaw == LOW);
  g_btnALastRawPressed = g_btnA;
  g_btnBLastRawPressed = g_btnB;
  g_btnAArmed = !g_btnA;
  g_btnBArmed = !g_btnB;
  g_btnALastChangeMs = millis();
  g_btnBLastChangeMs = millis();

  attachInterrupt(digitalPinToInterrupt(BTN_A_PIN), btnAIrqIsr, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_B_PIN), btnBIrqIsr, FALLING);

  analogReadResolution(12);
#if defined(ADC_11db)
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

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
  initImu();

  updateBattery();
  cache_sys = "";
  drawBatteryRowTiny();

  updateImu();
  updateMic();
  updateUiFast();
  updateUiSlow();
  updateVu();
  printSerialStatus();

  Serial.println("[BOOT] 0.96 ESP32-S3 dashboard v1.13.1 D16-two-line-yellow ready");
}

void loop() {
  uint32_t now = millis();

  handleImuTapEvent();
  updateButtons();
  handleButtonActions();
  updateMic();

  if (now - g_lastBatMs >= BAT_REFRESH_MS) {
    g_lastBatMs = now;
    updateBattery();
  }

  if (now - g_lastVuMs >= UI_VU_MS) {
    g_lastVuMs = now;
    updateVu();
  }

  if (now - g_lastFastMs >= UI_FAST_MS) {
    g_lastFastMs = now;
    updateUiFast();
  }

  if (now - g_lastSlowMs >= UI_SLOW_MS) {
    g_lastSlowMs = now;
    updateImu();
    updateUiSlow();
  }

  if (now - g_lastSerialMs >= SERIAL_MS) {
    g_lastSerialMs = now;
    printSerialStatus();
  }

  delay(2);
}
