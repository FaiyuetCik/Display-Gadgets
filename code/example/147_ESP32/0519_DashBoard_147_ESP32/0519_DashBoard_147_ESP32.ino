/*
  XIAO ESP32-S3 Plus + 1.47 Touch Display
  Factory Dynamic Dashboard v6.6 - footer product title and brighter cards

  Goal:
    - Same visual/behavior target as the proven nRF52840 factory dashboard.
    - LCD page is dynamic: SD / Touch / IMU / MIC / USR1 / USR2 / serial counter all refresh.
    - No recording and no WAV saving in this version.
    - Focus on reliable dynamic display first.

  Important changes from v3:
    - Ported the nRF dashboard update model more closely.
    - Added acquireForLcd() before every LCD write to force SD_CS high and LCD_CS idle.
    - Added independent refresh periods for SD / IMU+Touch+Buttons / VU / MIC text / Serial.
    - Added screen frame counter so the display visibly changes even if a sensor is missing.
    - Added I2C address scan line in serial for debugging.
    - Touch supports CST816-compatible basic read at 0x15.
    - IMU supports QMI8658-compatible and LSM6DS3-compatible basic reads.

  Required Arduino library:
    - Arduino_GFX_Library

  Pin map:
    D0 MIC_CLK, D1 MIC_DATA, D2 LCD_CS, D3 LCD_DC,
    D4 SDA, D5 SCL, D6 SD_CS, D7 TOUCH_INT,
    D8 SCK, D9 MISO, D10 MOSI,
    D14 IMU_INT, D15 USR2, D16 BAT_ADC,
    D17 LCD_RST, D18 LCD_BL, D19 USR1
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
static constexpr uint32_t UI_TEXT_SLOW_MS   = 280;
static constexpr uint32_t UI_VU_MS          = 90;
static constexpr uint32_t UI_MIC_TEXT_MS    = 420;
static constexpr uint32_t SERIAL_MS         = 450;
static constexpr uint32_t SD_REFRESH_MS     = 900;    // async SD probe request interval   // raw card-present probe, bounded and fast
static constexpr uint32_t MIC_DECAY_MS      = 80;
static constexpr uint32_t FOOTER_MS         = 500;

// ========================= MIC config =========================

static constexpr int MIC_SAMPLE_RATE_HZ = 16000;
static constexpr bool MIC_CLK_INVERT = false;

// This is only display scaling, not recording gain.
static constexpr float MIC_DISPLAY_SCALE = 2200.0f;
static constexpr size_t MIC_SAMPLES_PER_READ = 256;

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

// ========================= State =========================

struct Row { int y; };
static constexpr Row ROW_SD    {64};
static constexpr Row ROW_TOUCH {82};
static constexpr Row ROW_ACC   {122};
static constexpr Row ROW_GYR   {140};
static constexpr Row ROW_MIC_L {170};
static constexpr Row ROW_MIC_V {214};
static constexpr Row ROW_BTN1  {250};
static constexpr Row ROW_BTN2  {266};
static constexpr Row ROW_HINT  {282};
static constexpr Row ROW_TICK  {294};

static constexpr int VU_X = 14;
static constexpr int VU_Y = 190;
static constexpr int VU_W = 144;
static constexpr int VU_H = 18;
static constexpr int VU_SEG_COUNT = 12;
static constexpr int VU_GAP = 2;
static constexpr int VU_SEG_W = (VU_W - (VU_SEG_COUNT - 1) * VU_GAP) / VU_SEG_COUNT;

bool g_lcdOk = false;

bool g_sdMounted = false;
bool g_sdSessionOpen = false;
uint32_t g_sdOkFreq = 0;
uint64_t g_sdCardSizeMB = 0;
bool g_sdProbeDone = false;

// Async SD probe state. Full SD.begin() is done in a background task so the
// dashboard loop does not block and the LCD stays smooth.
volatile bool g_sdProbeBusy = false;
volatile bool g_sdProbeRequest = false;
volatile bool g_sdProbeResultReady = false;
volatile bool g_sdProbePresentResult = false;
volatile uint32_t g_sdProbeFreqResult = 0;
volatile uint32_t g_sdProbeSizeMbResult = 0;
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

bool g_micOk = false;
uint16_t g_micPeak = 0;
uint32_t g_micRms = 0;
uint32_t g_micLastUpdateMs = 0;
int16_t g_pdmBuf[MIC_SAMPLES_PER_READ];

uint32_t g_lastFastUiMs = 0;
uint32_t g_lastSlowUiMs = 0;
uint32_t g_lastVuMs = 0;
uint32_t g_lastMicTextMs = 0;
uint32_t g_lastSerialMs = 0;
uint32_t g_lastSdMs = 0;
uint32_t g_lastFooterMs = 0;

String cache_sd = "";
String cache_touch = "";
String cache_acc = "";
String cache_gyr = "";
String cache_btn1 = "";
String cache_btn2 = "";
String cache_mic = "";
String cache_footer = "";
String cache_tick = "";

float g_vuSmooth = 0.0f;
int cache_vuSegments = -1;
uint32_t g_frameCounter = 0;

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

static bool initLcd() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);

  restoreLcdSwSpiPins();

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
  gfx->drawRoundRect(VU_X, VU_Y, VU_W, VU_H, 4, C_LINE);
  for (int i = 0; i < VU_SEG_COUNT; ++i) {
    int x = VU_X + 1 + i * (VU_SEG_W + VU_GAP);
    gfx->fillRect(x, VU_Y + 1, VU_SEG_W, VU_H - 2, C_BLACK);
  }
  endLcdOp();
}

static void drawStaticLayout() {
  if (!beginLcdOp()) return;
  gfx->fillScreen(C_BLACK);

  // Premium dashboard UI:
  // keep "Hello,XIAO!" as the hero title, then group data into compact cards.
  gfx->setTextSize(2);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(8, 5);
  gfx->print("Hello,XIAO!");

  // Top subtitle kept simple and bright.
  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(8, 28);
  gfx->print("1.47 Inch Touch Display");

  gfx->drawFastHLine(8, 44, 156, C_LINE);

  // SYSTEM card
  gfx->drawRoundRect(6, 50, 160, 48, 6, C_CYAN);
  gfx->fillRect(10, 57, 3, 34, C_CYAN);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(18, 54);
  gfx->print("SYSTEM");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(18, ROW_SD.y);    gfx->print("SD");
  gfx->setCursor(18, ROW_TOUCH.y); gfx->print("Touch");

  // MOTION card
  gfx->drawRoundRect(6, 102, 160, 50, 6, C_YELLOW);
  gfx->fillRect(10, 112, 3, 32, C_YELLOW);
  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(18, 108);
  gfx->print("MOTION");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(18, ROW_ACC.y); gfx->print("Acc");
  gfx->setCursor(18, ROW_GYR.y); gfx->print("Gyr");

  // AUDIO card
  gfx->drawRoundRect(6, 158, 160, 70, 6, C_GREEN);
  gfx->fillRect(10, 168, 3, 52, C_GREEN);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(18, ROW_MIC_L.y);
  gfx->print("MIC LEVEL");

  // BUTTON card
  gfx->drawRoundRect(6, 236, 160, 42, 6, C_ACCENT);
  gfx->fillRect(10, 244, 3, 26, C_ACCENT);
  gfx->setTextColor(C_ACCENT, C_BLACK);
  gfx->setCursor(18, 240);
  gfx->print("BUTTON");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(18, ROW_BTN1.y); gfx->print("USR1");
  gfx->setCursor(18, ROW_BTN2.y); gfx->print("USR2");

  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(8, ROW_HINT.y); gfx->print("Serial synced output");

  // Lower footer product line: split into two compact lines so it is readable.
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(8, ROW_TICK.y); gfx->print("Powered by XIAO");

  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(8, ROW_TICK.y + 12); gfx->print("ESP32-S3 Plus");

  endLcdOp();

  drawVuFrame();

  cache_sd = "";
  cache_touch = "";
  cache_acc = "";
  cache_gyr = "";
  cache_btn1 = "";
  cache_btn2 = "";
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
  restoreLcdSwSpiPins();
}

static bool sdFullMountProbe(uint32_t &okFreq, uint32_t &cardSizeMb) {
  // Full mount probe with no file access.
  // This is reliable for insert/remove, but runs in a background task.
  okFreq = 0;
  cardSizeMb = 0;

  // Do not steal the shared bus in the middle of a LCD write.
  while (g_lcdBusy) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  acquireForSd();
  SPI.begin(LCD_SCK_PIN, SD_MISO_PIN, LCD_MOSI_PIN, SD_CS_PIN);
  delay(3);

  const uint32_t freqs[] = {400000};
  for (uint8_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
    if (SD.begin(SD_CS_PIN, SPI, freqs[i])) {
      okFreq = freqs[i];
      uint64_t mb = SD.cardSize() / (1024ULL * 1024ULL);
      if (mb == 0) mb = 1;
      if (mb > 0xFFFFFFFFUL) mb = 0xFFFFFFFFUL;
      cardSizeMb = (uint32_t)mb;

      closeSdSessionAndRestoreLcd();
      return true;
    }

    SD.end();
    SPI.end();
    restoreLcdSwSpiPins();
    delay(2);
  }

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

  Serial.printf("[SD] async result present=%u shown=%u freq=%lu size=%lluMB busy=%u\n",
                present ? 1 : 0,
                g_sdMounted ? 1 : 0,
                (unsigned long)g_sdOkFreq,
                g_sdCardSizeMB,
                g_sdProbeBusy ? 1 : 0);
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

static void initImu() {
  pinMode(IMU_INT_PIN, INPUT_PULLUP);

  g_imuType = IMU_NONE;
  g_imuOk = false;

  if (initQmi(0x6B)) return;
  if (initQmi(0x6A)) return;
  if (initLsm(0x6A)) return;
  if (initLsm(0x6B)) return;

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
  g_btnARaw = digitalRead(BTN_A_PIN);
  g_btnBRaw = digitalRead(BTN_B_PIN);
  g_btnA = (g_btnARaw == LOW);
  g_btnB = (g_btnBRaw == LOW);
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

  // attack faster, release slower
  if (target > g_vuSmooth) g_vuSmooth = g_vuSmooth * 0.55f + target * 0.45f;
  else g_vuSmooth = g_vuSmooth * 0.86f + target * 0.14f;

  int activeSegs = (int)roundf(g_vuSmooth * VU_SEG_COUNT);
  if (activeSegs < 0) activeSegs = 0;
  if (activeSegs > VU_SEG_COUNT) activeSegs = VU_SEG_COUNT;

  if (activeSegs == cache_vuSegments) return;

  if (!beginLcdOp()) return;
  for (int i = 0; i < VU_SEG_COUNT; ++i) {
    int x = VU_X + 1 + i * (VU_SEG_W + VU_GAP);
    uint16_t color = (i < activeSegs) ? vuColorForIndex(i) : C_BLACK;
    gfx->fillRect(x, VU_Y + 1, VU_SEG_W, VU_H - 2, color);
  }
  endLcdOp();

  cache_vuSegments = activeSegs;
}

static void updateMicText() {
  uint16_t micPeak = currentMicPeak();
  String micText = String("Raw ") + String((unsigned)micPeak);
  if (micText != cache_mic) {
    if (printTextFixed(18, ROW_MIC_V.y, C_WHITE, micText, 18)) {
      cache_mic = micText;
    }
  }
}

static void updateUiFast() {
  String sdText;
  if (g_sdMounted) {
    sdText = String("OK  ") + String(g_sdOkFreq / 1000) + "k";
  } else {
    sdText = "NO CARD";
  }

  if (sdText != cache_sd) {
    if (printTextFixed(52, ROW_SD.y, g_sdMounted ? C_GREEN : C_RED, sdText, 14)) {
      cache_sd = sdText;
    }
  }

  String touchText;
  if (!g_touchFound) {
    touchText = "not found";
  } else if (g_touchValid) {
    touchText = String(g_touchX) + "," + g_touchY;
  } else {
    touchText = "release";
  }

  if (touchText != cache_touch) {
    if (printTextFixed(66, ROW_TOUCH.y, g_touchFound ? C_CYAN : C_RED, touchText, 12)) {
      cache_touch = touchText;
    }
  }

  String btn1Text = g_btnA ? "Pressed " : "Released";
  if (btn1Text != cache_btn1) {
    if (printTextFixed(62, ROW_BTN1.y, g_btnA ? C_GREEN : C_WHITE, btn1Text, 11)) {
      cache_btn1 = btn1Text;
    }
  }

  String btn2Text = g_btnB ? "Pressed " : "Released";
  if (btn2Text != cache_btn2) {
    if (printTextFixed(62, ROW_BTN2.y, g_btnB ? C_GREEN : C_WHITE, btn2Text, 11)) {
      cache_btn2 = btn2Text;
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
    if (printTextFixed(52, ROW_ACC.y, g_imuOk ? C_WHITE : C_RED, accText, 14)) {
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
    if (printTextFixed(52, ROW_GYR.y, g_imuOk ? C_WHITE : C_RED, gyrText, 14)) {
      cache_gyr = gyrText;
    }
  }
}

static void updateFooter() {
  // Footer is static in v6.6:
  // - "Serial synced output"
  // - "Powered by XIAO"
  // - "ESP32-S3 Plus"
}

static void printSerialStatus() {
  logf("[DASH] sd=%s freq=%lu size=%lluMB touch=%s x=%d y=%d imu=%s type=%d "
       "acc=(%.2f,%.2f,%.2f) gyr=(%.2f,%.2f,%.2f) micPeak=%u micRms=%lu "
       "usr1=%s raw1=%d usr2=%s raw2=%d tick=%lu touchAddr=0x%02X int=%d rawTouch=(%d,%d)\n",
       g_sdMounted ? "Inserted" : "Unplugged",
       (unsigned long)g_sdOkFreq,
       g_sdCardSizeMB,
       g_touchValid ? "Y" : "N",
       g_touchX, g_touchY,
       g_imuOk ? "OK" : "NO",
       (int)g_imuType,
       g_ax, g_ay, g_az,
       g_gx, g_gy, g_gz,
       (unsigned)currentMicPeak(),
       (unsigned long)g_micRms,
       g_btnA ? "Pressed" : "Released", g_btnARaw,
       g_btnB ? "Pressed" : "Released", g_btnBRaw,
       (unsigned long)g_frameCounter,
       g_touchAddr,
       g_touchIntRaw,
       g_touchRawX, g_touchRawY);
}

// ========================= Setup / loop =========================

static void printHeader() {
  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus Factory Dynamic Dashboard v6.6 ===");
  Serial.println("nRF-style dynamic UI port, dynamic raw-SD detect + AXS5106L touch coords");
  Serial.printf("LCD: %s, CS=D2(%d), DC=D3(%d), SCK=D8(%d), MOSI=D10(%d), RST=%s(%d), BL=D18(%d)\n",
                LCD_USE_ESP32SPI ? "ESP32SPI" : "SWSPI",
                LCD_CS_PIN, LCD_DC_PIN, LCD_SCK_PIN, LCD_MOSI_PIN,
                USE_PRD_PINMAP ? "D17" : "D19", LCD_RST_PIN, LCD_BL_PIN);
  Serial.printf("MIC: CLK=D0(%d), DATA=D1(%d), sample=%d\n", MIC_CLK_PIN, MIC_DATA_PIN, MIC_SAMPLE_RATE_HZ);
  Serial.printf("SD : CS=D6(%d), SCK=D8(%d), MISO=D9(%d), MOSI=D10(%d)\n", SD_CS_PIN, LCD_SCK_PIN, SD_MISO_PIN, LCD_MOSI_PIN);
  Serial.printf("I2C: SDA=D4(%d), SCL=D5(%d), TOUCH_INT=D7(%d), IMU_INT=D14(%d)\n", I2C_SDA_PIN, I2C_SCL_PIN, TOUCH_INT_PIN, IMU_INT_PIN);
  Serial.printf("BTN: USR1=%d, USR2=%d\n", BTN_A_PIN, BTN_B_PIN);
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
  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(LCD_CS_PIN, HIGH);

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
  updateMicPeak();

  // Keep the base UI already drawn above; do not redraw it while the first SD
  // background probe might be using the bus.
  updateUiFast();
  updateUiSlow();
  updateVuMeter();
  updateMicText();
  updateFooter();
  printSerialStatus();

  Serial.println("[BOOT] dynamic dashboard v6.6 ready");
}

void loop() {
  uint32_t now = millis();

  // Update raw states as often as possible.
  updateMicPeak();

  // Full SD.begin() probe runs in a background task.
  // Main loop only consumes results and requests the next probe, so the UI stays smooth.
  consumeSdProbeResult();
  if (now - g_lastSdMs >= SD_REFRESH_MS) {
    g_lastSdMs = now;
    requestSdProbe();
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
