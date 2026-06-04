/*
  XIAO ESP32-S3 Plus + 1.14 Inch Display
  Factory Dynamic Test Firmware v0.4

  Target board:
    1.14 Inch Display Powered by XIAO ESP32-S3 Plus

  Covered modules:
    - 1.14" IPS LCD, ST7789, 135x240
    - PDM digital microphone
    - 6-axis IMU, QMI8658 / LSM6DS3 compatible probing
    - 3 user buttons: USR1 / USR2 / USR3
    - Grove I2C bus scan

  Required Arduino library:
    - Arduino_GFX_Library

  PRD pin map:
    D0  = MIC_CLK
    D1  = MIC_DATA
    D2  = LCD_CS
    D3  = LCD_DC
    D4  = SDA
    D5  = SCL
    D6  = USR1
    D7  = USR2
    D8  = SCK
    D9  = NC
    D10 = MOSI
    D11 = I2S_SD test pad
    D12 = I2S_SCK test pad
    D13 = I2S_WS test pad
    D14 = IMU_INT
    D15 = NC / reserved test pad
    D16 = BAT_ADC
    D17 = LCD_RST
    D18 = LCD_BL
    D19 = USR3

  Notes:
    - This 1.14" board has no Touch and no SD slot.
    - LCD uses hardware SPI by default because there is no SD sharing the SPI bus.
    - If the LCD is shifted, adjust LCD_COL_OFFSET_* / LCD_ROW_OFFSET_* below.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
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

// ========================= Build switches =========================

#define LCD_USE_ESP32SPI 1
#define REDUCE_PDM_CLK_DRIVE 1

// ========================= Timing =========================

static constexpr uint32_t UI_FAST_MS = 120;
static constexpr uint32_t UI_SLOW_MS = 260;
static constexpr uint32_t UI_VU_MS = 70;
static constexpr uint32_t SERIAL_MS = 600;
static constexpr uint32_t I2C_SCAN_MS = 2500;

// ========================= LCD parameters =========================
//
// Typical ST7789 1.14" 135x240 offset is 52,40.
// If the display is shifted or clipped, tune these four values first.

static constexpr int LCD_W = 135;
static constexpr int LCD_H = 240;

static constexpr int LCD_ROTATION = 0;
static constexpr bool LCD_IPS = true;

static constexpr int LCD_COL_OFFSET_1 = 52;
static constexpr int LCD_ROW_OFFSET_1 = 40;
static constexpr int LCD_COL_OFFSET_2 = 53;
static constexpr int LCD_ROW_OFFSET_2 = 40;

// ========================= MIC parameters =========================

static constexpr int MIC_SAMPLE_RATE_HZ = 16000;
static constexpr bool MIC_CLK_INVERT = false;
static constexpr size_t MIC_SAMPLES_PER_READ = 256;

// Display-only scaling, not recording gain.
static constexpr float MIC_DISPLAY_SCALE = 2400.0f;

// ========================= Battery =========================
//
// Placeholder divider. Adjust after measuring the board's battery divider.

static constexpr float BATTERY_DIVIDER_RATIO = 2.0f;

// ========================= Pins =========================

static constexpr uint8_t MIC_CLK_PIN   = D0;
static constexpr uint8_t MIC_DATA_PIN  = D1;
static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t I2C_SDA_PIN   = D4;
static constexpr uint8_t I2C_SCL_PIN   = D5;
static constexpr uint8_t USR1_PIN     = D6;
static constexpr uint8_t USR2_PIN     = D7;
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t LCD_MISO_PIN  = D9;   // NC, unused by LCD
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t I2S_SD_PAD    = D11;
static constexpr uint8_t I2S_SCK_PAD   = D12;
static constexpr uint8_t I2S_WS_PAD    = D13;
static constexpr uint8_t IMU_INT_PIN   = D14;
static constexpr uint8_t BAT_ADC_PIN   = D16;
static constexpr uint8_t LCD_RST_PIN   = D17;
static constexpr uint8_t LCD_BL_PIN    = D18;
static constexpr uint8_t USR3_PIN     = D19;

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
static constexpr uint16_t C_PANEL   = 0x0841;
static constexpr uint16_t C_LINE    = 0x39E7;

// ========================= LCD object =========================

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

// ========================= I2S object =========================

#if ESP_IDF_VERSION_MAJOR >= 5
static i2s_chan_handle_t g_i2sRxChan = nullptr;
#else
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
#endif

// ========================= State =========================

bool g_lcdOk = false;
bool g_micOk = false;

struct AudioStats {
  int32_t mean;
  uint32_t peak;
  uint32_t rms;
};

AudioStats g_micStats = {};
int16_t g_pdmBuf[MIC_SAMPLES_PER_READ];
uint32_t g_micLastUpdateMs = 0;
float g_vuSmooth = 0.0f;
int g_cachedVuPixels = -1;

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

bool g_btnA = false;
bool g_btnB = false;
bool g_btnC = false;
int g_btnARaw = HIGH;
int g_btnBRaw = HIGH;
int g_btnCRaw = HIGH;

float g_batV = 0.0f;

uint8_t g_i2cDevices[12] = {};
uint8_t g_i2cCount = 0;

uint32_t g_lastFastMs = 0;
uint32_t g_lastSlowMs = 0;
uint32_t g_lastVuMs = 0;
uint32_t g_lastSerialMs = 0;
uint32_t g_lastI2cScanMs = 0;
uint32_t g_frameCounter = 0;

// UI row positions, tuned for 135x240.
static constexpr int ROW_TITLE = 4;
static constexpr int ROW_SUB_1 = 26;
static constexpr int ROW_SUB_2 = 37;
static constexpr int ROW_MIC_TITLE = 56;
static constexpr int ROW_MIC_RAW = 98;
static constexpr int ROW_IMU_TITLE = 118;
static constexpr int ROW_ACC = 134;
static constexpr int ROW_GYR = 150;
static constexpr int ROW_SYS_TITLE = 168;
static constexpr int ROW_USR1 = 183;
static constexpr int ROW_USR2 = 196;
static constexpr int ROW_USR3 = 209;
static constexpr int ROW_FOOTER = 225;

static constexpr int VU_X = 8;
static constexpr int VU_Y = 76;
static constexpr int VU_W = 119;
static constexpr int VU_H = 14;

String cacheMicRaw = "";
String cacheAcc = "";
String cacheGyr = "";
String cacheBtn = "";
String cacheFooter = "";

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

  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t *val) {
  return i2cRead(addr, reg, val, 1);
}

static String padRight(String s, int width) {
  while ((int)s.length() < width) s += ' ';
  if ((int)s.length() > width) s = s.substring(0, width);
  return s;
}

// ========================= LCD =========================

static bool initLcd() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);

  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);

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

  gfx->fillScreen(C_BLACK);
  gfx->setTextWrap(false);

  g_lcdOk = true;
  Serial.println("[LCD] OK");
  return true;
}

static bool printTextFixed(int x, int y, uint16_t color, const String &s, int widthChars) {
  if (!g_lcdOk) return false;

  gfx->setTextSize(1);
  gfx->setTextColor(color, C_BLACK);
  gfx->setCursor(x, y);
  gfx->print(padRight(s, widthChars));
  return true;
}

static void drawVuFrame() {
  if (!g_lcdOk) return;

  gfx->drawRoundRect(VU_X, VU_Y, VU_W, VU_H, 3, C_LINE);
  gfx->drawFastHLine(VU_X + 2, VU_Y + VU_H + 4, VU_W - 4, C_DIM);
}

static void drawStaticLayout() {
  if (!g_lcdOk) return;

  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(2, ROW_TITLE);
  gfx->print("Hello,XIAO!");

  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(4, ROW_SUB_1);
  gfx->print("1.14 Inch Display");

  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(4, ROW_SUB_2);
  gfx->print("XIAO ESP32-S3 Plus");

  gfx->drawFastHLine(4, 50, 127, C_LINE);

  // MIC card
  gfx->drawRoundRect(3, 53, 129, 58, 5, C_GREEN);
  gfx->fillRect(7, 62, 3, 40, C_GREEN);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(14, ROW_MIC_TITLE);
  gfx->print("MIC LEVEL");
  drawVuFrame();

  // IMU card
  gfx->drawRoundRect(3, 114, 129, 50, 5, C_YELLOW);
  gfx->fillRect(7, 123, 3, 32, C_YELLOW);
  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(14, ROW_IMU_TITLE);
  gfx->print("IMU");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(14, ROW_ACC);
  gfx->print("Acc");
  gfx->setCursor(14, ROW_GYR);
  gfx->print("Gyr");

  // System card
  gfx->drawRoundRect(3, 165, 129, 56, 5, C_CYAN);
  gfx->fillRect(7, 174, 3, 38, C_CYAN);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(14, ROW_SYS_TITLE);
  gfx->print("SYSTEM");
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(14, ROW_USR1);
  gfx->print("USR1");
  gfx->setCursor(14, ROW_USR2);
  gfx->print("USR2");
  gfx->setCursor(14, ROW_USR3);
  gfx->print("USR3");

  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(4, ROW_FOOTER);
  gfx->print("Grove I2C:");

  cacheMicRaw = "";
  cacheAcc = "";
  cacheGyr = "";
  cacheBtn = "";
  cacheFooter = "";
  g_cachedVuPixels = -1;
}

// ========================= I2C scan =========================

static void scanI2cBus() {
  g_i2cCount = 0;

  for (uint8_t addr = 1; addr < 127 && g_i2cCount < sizeof(g_i2cDevices); addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      g_i2cDevices[g_i2cCount++] = addr;
    }
  }

  Serial.print("[I2C] scan:");
  if (!g_i2cCount) {
    Serial.print(" none");
  } else {
    for (uint8_t i = 0; i < g_i2cCount; i++) {
      Serial.printf(" 0x%02X", g_i2cDevices[i]);
    }
  }
  Serial.println();
}

// ========================= IMU =========================

static bool initQmi(uint8_t addr) {
  uint8_t who = 0;
  if (!i2cRead8(addr, 0x00, &who)) return false;
  if (who == 0x00 || who == 0xFF) return false;

  // Basic QMI8658 config: enable accel + gyro.
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

  // Basic LSM6DS3/LSM6DSO compatible config.
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
  g_imuAddr = 0;
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

// ========================= Buttons / Battery =========================

static void updateButtons() {
  g_btnARaw = digitalRead(USR1_PIN);
  g_btnBRaw = digitalRead(USR2_PIN);
  g_btnCRaw = digitalRead(USR3_PIN);

  g_btnA = (g_btnARaw == LOW);
  g_btnB = (g_btnBRaw == LOW);
  g_btnC = (g_btnCRaw == LOW);
}

static void updateBattery() {
#if defined(ARDUINO_ARCH_ESP32)
  uint32_t mv = analogReadMilliVolts(BAT_ADC_PIN);
  g_batV = (mv / 1000.0f) * BATTERY_DIVIDER_RATIO;
#else
  int raw = analogRead(BAT_ADC_PIN);
  g_batV = (raw / 4095.0f) * 3.3f * BATTERY_DIVIDER_RATIO;
#endif
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

static AudioStats statsOf(const int16_t *samples, uint32_t count) {
  AudioStats s = {};
  if (!count) return s;

  int64_t sum = 0;
  for (uint32_t i = 0; i < count; i++) {
    sum += samples[i];
  }

  s.mean = (int32_t)(sum / count);

  uint32_t peak = 0;
  uint64_t sq = 0;
  for (uint32_t i = 0; i < count; i++) {
    int32_t v = (int32_t)samples[i] - s.mean;
    uint32_t a = v < 0 ? (uint32_t)(-v) : (uint32_t)v;
    if (a > peak) peak = a;
    sq += (uint64_t)a * a;
  }

  s.peak = peak;
  s.rms = (uint32_t)sqrt((double)sq / count);
  return s;
}

static void updateMic() {
  if (!g_micOk) return;

  size_t bytesRead = 0;
  if (!readMic(&bytesRead, 2)) return;

  uint32_t samples = bytesRead / sizeof(int16_t);
  if (samples > 0) {
    g_micStats = statsOf(g_pdmBuf, samples);
    g_micLastUpdateMs = millis();
  }
}

static uint32_t currentMicPeak() {
  uint32_t now = millis();

  if (now - g_micLastUpdateMs > 120) {
    g_micStats.peak = (uint32_t)(g_micStats.peak * 0.80f);
    g_micStats.rms = (uint32_t)(g_micStats.rms * 0.80f);
  }

  return g_micStats.peak;
}

// ========================= UI update =========================

static void updateVuMeter() {
  if (!g_lcdOk) return;

  float target = (float)currentMicPeak() / MIC_DISPLAY_SCALE;
  if (target < 0.0f) target = 0.0f;
  if (target > 1.0f) target = 1.0f;

  if (target > g_vuSmooth) {
    g_vuSmooth = g_vuSmooth * 0.55f + target * 0.45f;
  } else {
    g_vuSmooth = g_vuSmooth * 0.86f + target * 0.14f;
  }

  int fill = (int)(g_vuSmooth * (VU_W - 4));
  if (fill < 2 && currentMicPeak() > 0) fill = 2;
  if (fill > VU_W - 4) fill = VU_W - 4;

  if (fill == g_cachedVuPixels) return;

  gfx->fillRect(VU_X + 2, VU_Y + 2, VU_W - 4, VU_H - 4, C_BLACK);

  uint16_t color = C_GREEN;
  if (fill > (VU_W * 8) / 10) color = C_RED;
  else if (fill > (VU_W * 6) / 10) color = C_YELLOW;

  if (fill > 0) {
    gfx->fillRect(VU_X + 2, VU_Y + 2, fill, VU_H - 4, color);
  }

  g_cachedVuPixels = fill;
}

static void updateUiFast() {
  String micText = String("Raw ") + String((unsigned)g_micStats.peak);
  if (micText != cacheMicRaw) {
    if (printTextFixed(14, ROW_MIC_RAW, C_WHITE, micText, 18)) {
      cacheMicRaw = micText;
    }
  }

  String btnText = String(g_btnA ? "P" : "R") + "|" +
                   String(g_btnB ? "P" : "R") + "|" +
                   String(g_btnC ? "P" : "R");

  if (btnText != cacheBtn) {
    bool ok1 = printTextFixed(56, ROW_USR1, g_btnA ? C_GREEN : C_WHITE, g_btnA ? "Pressed" : "Released", 8);
    bool ok2 = printTextFixed(56, ROW_USR2, g_btnB ? C_GREEN : C_WHITE, g_btnB ? "Pressed" : "Released", 8);
    bool ok3 = printTextFixed(56, ROW_USR3, g_btnC ? C_GREEN : C_WHITE, g_btnC ? "Pressed" : "Released", 8);

    if (ok1 && ok2 && ok3) {
      cacheBtn = btnText;
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

  if (accText != cacheAcc) {
    if (printTextFixed(38, ROW_ACC, g_imuOk ? C_WHITE : C_RED, accText, 15)) {
      cacheAcc = accText;
    }
  }

  String gyrText;
  if (g_imuOk) {
    snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", g_gx, g_gy, g_gz);
    gyrText = String(buf);
  } else {
    gyrText = "not found";
  }

  if (gyrText != cacheGyr) {
    if (printTextFixed(38, ROW_GYR, g_imuOk ? C_WHITE : C_RED, gyrText, 15)) {
      cacheGyr = gyrText;
    }
  }

  String groveText;
  if (g_i2cCount == 0) {
    groveText = "Grove I2C: none";
  } else if (g_i2cCount == 1) {
    snprintf(buf, sizeof(buf), "Grove I2C: 1 0x%02X", g_i2cDevices[0]);
    groveText = String(buf);
  } else {
    snprintf(buf, sizeof(buf), "Grove I2C: %u 0x%02X+", g_i2cCount, g_i2cDevices[0]);
    groveText = String(buf);
  }

  if (groveText != cacheFooter) {
    if (printTextFixed(4, ROW_FOOTER, C_CYAN, groveText, 21)) {
      cacheFooter = groveText;
    }
  }
}

static void printSerialStatus() {
  Serial.printf("[DASH] micPeak=%lu micRms=%lu imu=%s type=%d "
                "acc=(%.2f,%.2f,%.2f) gyr=(%.2f,%.2f,%.2f) "
                "usr1=%u raw1=%d usr2=%u raw2=%d usr3=%u raw3=%d "
                "i2cCount=%u frame=%lu\n",
                (unsigned long)g_micStats.peak,
                (unsigned long)g_micStats.rms,
                g_imuOk ? "OK" : "NO",
                (int)g_imuType,
                g_ax, g_ay, g_az,
                g_gx, g_gy, g_gz,
                g_btnA ? 1 : 0, g_btnARaw,
                g_btnB ? 1 : 0, g_btnBRaw,
                g_btnC ? 1 : 0, g_btnCRaw,
                g_i2cCount,
                (unsigned long)g_frameCounter);
}

// ========================= Setup / loop =========================

static void printHeader() {
  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus 1.14 Factory Dashboard v0.4 ===");
  Serial.printf("LCD: %s, ST7789 %dx%d, CS=D2(%d), DC=D3(%d), SCK=D8(%d), MOSI=D10(%d), RST=D17(%d), BL=D18(%d)\n",
                LCD_USE_ESP32SPI ? "ESP32SPI" : "SWSPI",
                LCD_W, LCD_H,
                LCD_CS_PIN, LCD_DC_PIN, LCD_SCK_PIN, LCD_MOSI_PIN, LCD_RST_PIN, LCD_BL_PIN);
  Serial.printf("MIC: CLK=D0(%d), DATA=D1(%d), sample=%d\n",
                MIC_CLK_PIN, MIC_DATA_PIN, MIC_SAMPLE_RATE_HZ);
  Serial.printf("I2C: SDA=D4(%d), SCL=D5(%d), IMU_INT=D14(%d)\n",
                I2C_SDA_PIN, I2C_SCL_PIN, IMU_INT_PIN);
  Serial.printf("USR: USR1=D6(%d), USR2=D7(%d), USR3=D19(%d)\n",
                USR1_PIN, USR2_PIN, USR3_PIN);
  Serial.printf("TEST PAD: D11=%d, D12=%d, D13=%d, D15=reserved\n",
                I2S_SD_PAD, I2S_SCK_PAD, I2S_WS_PAD);
}

void setup() {
  Serial.begin(115200);
  delay(900);

  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  printHeader();

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
  pinMode(USR3_PIN, INPUT_PULLUP);

  pinMode(IMU_INT_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  initLcd();
  drawStaticLayout();

  scanI2cBus();
  initImu();
  initMic();

  updateButtons();
  updateImu();
  updateMic();
  updateUiFast();
  updateUiSlow();
  updateVuMeter();

  Serial.println("[BOOT] 1.14 dashboard v0.4 ready");
}

void loop() {
  uint32_t now = millis();

  updateMic();

  if (now - g_lastVuMs >= UI_VU_MS) {
    g_lastVuMs = now;
    updateVuMeter();
  }

  if (now - g_lastFastMs >= UI_FAST_MS) {
    g_lastFastMs = now;
    updateButtons();
    updateUiFast();
  }

  if (now - g_lastSlowMs >= UI_SLOW_MS) {
    g_lastSlowMs = now;
    updateImu();
      updateUiSlow();
  }

  if (now - g_lastI2cScanMs >= I2C_SCAN_MS) {
    g_lastI2cScanMs = now;
    scanI2cBus();
  }

  if (now - g_lastSerialMs >= SERIAL_MS) {
    g_lastSerialMs = now;
    printSerialStatus();
  }

  delay(2);
}
