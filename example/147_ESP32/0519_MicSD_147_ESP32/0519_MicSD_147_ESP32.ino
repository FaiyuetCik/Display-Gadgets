/*
  XIAO ESP32-S3 Plus + 1.47 Touch Display
  LCD + MIC + SD coexist recorder v6

  Goal:
    Keep the LCD ON during recording, while reducing/removing the audio noise
    seen when LCD is active.

  What this sketch does:
    1. Uses LCD SWSPI to avoid LCD/SD hardware SPI contention and white stripe artifacts.
    2. Draws a static "Recording..." UI and keeps the screen ON.
    3. Stops SD before capture, then captures PDM MIC to RAM only.
    4. Does NOT refresh LCD during capture.
    5. Saves three files after capture:
         /COEX_RAW.WAV   raw capture
         /COEX_HP.WAV    DC removed + high-pass + gain
         /COEX_DSP.WAV   HP + auto notch filtering
       and a debug CSV:
         /COEXSTAT.CSV
    6. LCD remains visible during capture; it is not reset or turned off.

  Test instruction:
    - During the first 1 second after recording starts, keep quiet.
      The firmware profiles the LCD-related tonal noise and finds a notch frequency.
    - Then speak normally.
    - Listen to COEX_HP.WAV and COEX_DSP.WAV first.

  Audio format:
    16000 Hz, 16-bit signed PCM, mono, WAV, 5 seconds.

  Required Arduino library:
    Arduino_GFX_Library

  Target pin map:
    D0  = MIC_CLK
    D1  = MIC_DATA
    D2  = LCD_CS
    D3  = LCD_DC
    D6  = SD_CS
    D8  = SPI_SCK
    D9  = SPI_MISO
    D10 = SPI_MOSI
    D15 = USR2 / record again
    D17 = LCD_RST
    D18 = LCD_BL
*/

#include <Arduino.h>
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
#include <stdlib.h>

// ========================= User config =========================

#define USE_PRD_PINMAP 1

// Keep LCD on, but use software SPI to avoid LCD/SD hardware SPI collisions.
#define LCD_USE_ESP32SPI 0

// Sends a small set of ST7789-style low-noise commands after drawing the UI.
// If color looks strange or the panel behaves oddly, set this to 0.
#define LCD_LOW_NOISE_MODE 1

// Keep backlight ON during recording.
// If audio is still noisy, test with 0 to isolate backlight contribution,
// but the product target is 1.
#define LCD_BACKLIGHT_ON_DURING_RECORDING 1

static constexpr uint32_t RECORD_SECONDS = 5;
static constexpr uint32_t MIC_SAMPLE_RATE_HZ = 16000;

// C0/C1 were similar in the MIC-only test; keep normal clock first.
static constexpr bool MIC_CLK_INVERT = false;

// Cleaned file gain. Increase to 2.5f or 3.0f if too quiet.
// Reduce to 1.5f if clipping.
static constexpr float CLEAN_GAIN = 2.0f;

// Noise gate is OFF for honest evaluation. Try 30~80 later only for demo polish.
static constexpr int32_t CLEAN_NOISE_GATE = 0;

// Auto notch parameters for /COEX_DSP.WAV
static constexpr float NOTCH_Q = 18.0f;
static constexpr uint32_t PROFILE_MS = 900;
static constexpr uint32_t NOTCH_SEARCH_START_HZ = 300;
static constexpr uint32_t NOTCH_SEARCH_END_HZ = 7600;
static constexpr uint32_t NOTCH_SEARCH_STEP_HZ = 100;

// Lower PDM CLK drive strength to reduce EMI/coupling.
// Set 0 if PDM becomes unstable.
#define REDUCE_PDM_CLK_DRIVE 1

// ========================= Pin map =========================

static constexpr uint8_t MIC_CLK_PIN    = D0;
static constexpr uint8_t MIC_DATA_PIN   = D1;

static constexpr uint8_t LCD_CS_PIN     = D2;
static constexpr uint8_t LCD_DC_PIN     = D3;

static constexpr uint8_t SD_CS_PIN      = D6;
static constexpr uint8_t SPI_SCK_PIN    = D8;
static constexpr uint8_t SD_MISO_PIN    = D9;
static constexpr uint8_t SPI_MOSI_PIN   = D10;

static constexpr uint8_t BTN_RECORD_PIN = D15;

#if USE_PRD_PINMAP
static constexpr uint8_t LCD_RST_PIN    = D17;
#else
static constexpr uint8_t LCD_RST_PIN    = D19;
#endif

static constexpr uint8_t LCD_BL_PIN     = D18;

// ========================= Display =========================

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;

static constexpr uint16_t C_BLACK  = 0x0000;
static constexpr uint16_t C_WHITE  = 0xFFFF;
static constexpr uint16_t C_GREEN  = 0x07E0;
static constexpr uint16_t C_RED    = 0xF800;
static constexpr uint16_t C_CYAN   = 0x07FF;
static constexpr uint16_t C_YELLOW = 0xFFE0;
static constexpr uint16_t C_ORANGE = 0xFD20;
static constexpr uint16_t C_GRAY   = 0x8410;
static constexpr uint16_t C_DIM    = 0x2104;

#if LCD_USE_ESP32SPI
Arduino_DataBus *lcdBus = new Arduino_ESP32SPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  SPI_SCK_PIN,
  SPI_MOSI_PIN
);
#else
Arduino_DataBus *lcdBus = new Arduino_SWSPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  SPI_SCK_PIN,
  SPI_MOSI_PIN,
  GFX_NOT_DEFINED
);
#endif

Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus,
  LCD_RST_PIN,
  0,
  false,
  LCD_W,
  LCD_H,
  34, 0,
  34, 0
);

// ========================= Audio =========================

static constexpr size_t MIC_SAMPLES_PER_READ = 256;
static constexpr uint16_t AUDIO_BITS_PER_SAMPLE = 16;
static constexpr uint16_t AUDIO_CHANNELS = 1;
static constexpr uint32_t AUDIO_BYTE_RATE = MIC_SAMPLE_RATE_HZ * AUDIO_CHANNELS * AUDIO_BITS_PER_SAMPLE / 8;
static constexpr uint16_t AUDIO_BLOCK_ALIGN = AUDIO_CHANNELS * AUDIO_BITS_PER_SAMPLE / 8;
static constexpr uint32_t RECORD_TARGET_BYTES = MIC_SAMPLE_RATE_HZ * AUDIO_BLOCK_ALIGN * RECORD_SECONDS;
static constexpr uint32_t RECORD_TARGET_SAMPLES = RECORD_TARGET_BYTES / sizeof(int16_t);
static constexpr uint32_t PROFILE_SAMPLES = (MIC_SAMPLE_RATE_HZ * PROFILE_MS) / 1000;

int16_t micBuf[MIC_SAMPLES_PER_READ];
uint8_t *recBuffer = nullptr;

#if ESP_IDF_VERSION_MAJOR >= 5
static i2s_chan_handle_t g_i2sRxChan = nullptr;
#else
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
#endif

struct AudioStats {
  int32_t mean;
  uint32_t peak;
  uint32_t rms;
  uint32_t bytes;
  uint32_t elapsedMs;
};

bool lcdOk = false;
bool sdMounted = false;
bool busy = false;
uint32_t lastButtonMs = 0;

AudioStats rawStats = {};
AudioStats hpStats = {};
AudioStats dspStats = {};
float detectedNoiseHz = 0.0f;

// ========================= GPIO and system =========================

static void disableRadios() {
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);
}

static void spiDevicesIdle() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(SD_CS_PIN, HIGH);
}

static void printHeader() {
  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus LCD+MIC+SD coexist recorder v6 ===");
  Serial.printf("LCD: %s, CS=D2(%d), DC=D3(%d), SCK=D8(%d), MOSI=D10(%d), RST=%s(%d), BL=D18(%d), low_noise=%u\n",
                LCD_USE_ESP32SPI ? "ESP32SPI" : "SWSPI",
                LCD_CS_PIN, LCD_DC_PIN, SPI_SCK_PIN, SPI_MOSI_PIN,
                USE_PRD_PINMAP ? "D17" : "D19", LCD_RST_PIN, LCD_BL_PIN,
                LCD_LOW_NOISE_MODE ? 1 : 0);
  Serial.printf("MIC: CLK=D0(%d), DATA=D1(%d), sample=%lu Hz, clk_inv=%u\n",
                MIC_CLK_PIN, MIC_DATA_PIN, (unsigned long)MIC_SAMPLE_RATE_HZ, MIC_CLK_INVERT ? 1 : 0);
  Serial.printf("SD : CS=D6(%d), SCK=D8(%d), MISO=D9(%d), MOSI=D10(%d)\n",
                SD_CS_PIN, SPI_SCK_PIN, SD_MISO_PIN, SPI_MOSI_PIN);
  Serial.printf("Record buffer: %lu bytes, free heap=%lu\n",
                (unsigned long)RECORD_TARGET_BYTES, (unsigned long)ESP.getFreeHeap());
}

// ========================= LCD helpers =========================

static void lcdCmd(uint8_t cmd) {
  lcdBus->beginWrite();
  lcdBus->writeCommand(cmd);
  lcdBus->endWrite();
}

static void lcdCmdData(uint8_t cmd, const uint8_t *data, size_t len) {
  lcdBus->beginWrite();
  lcdBus->writeCommand(cmd);
  for (size_t i = 0; i < len; i++) {
    lcdBus->write(data[i]);
  }
  lcdBus->endWrite();
}

static void lcdWriteMadctlFix() {
  const uint8_t madctl = 0x48;
  lcdCmdData(0x36, &madctl, 1);
}

static void lcdApplyLowNoiseMode() {
#if LCD_LOW_NOISE_MODE
  // ST7789-style commands. On some JD9853A-compatible modules, these are accepted.
  // 0x20: display inversion off, can reduce panel drive noise on some screens.
  // 0x39: idle mode on, keeps display visible with reduced color depth / activity.
  // 0xC6: frame-rate control in normal mode; value is panel-dependent.
  // These are intentionally optional.
  lcdCmd(0x20);              // INVOFF
  delay(5);
  lcdCmd(0x39);              // IDMON
  delay(5);
  uint8_t fr = 0x1F;          // lower frame activity attempt
  lcdCmdData(0xC6, &fr, 1);
  delay(5);
#endif
}

static bool initLcd() {
  spiDevicesIdle();

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);

#if LCD_USE_ESP32SPI
  bool ok = gfx->begin(20000000);
#else
  bool ok = gfx->begin();
#endif

  if (!ok) {
    Serial.println("[LCD] gfx->begin failed");
    lcdOk = false;
    return false;
  }

  lcdWriteMadctlFix();
  gfx->setTextWrap(false);
  lcdOk = true;

  Serial.println("[LCD] OK");
  return true;
}

static void lcdText(int x, int y, const char *s, uint16_t color = C_WHITE, uint16_t bg = C_BLACK, uint8_t size = 1) {
  if (!lcdOk) return;
  gfx->setTextSize(size);
  gfx->setTextColor(color, bg);
  gfx->setCursor(x, y);
  gfx->print(s);
}

static void drawFrame(const char *title) {
  if (!lcdOk) return;
  gfx->fillScreen(C_BLACK);
  gfx->drawRect(0, 0, LCD_W, LCD_H, C_DIM);
  lcdText(8, 8, title, C_CYAN);
  gfx->drawFastHLine(0, 34, LCD_W, C_DIM);
}

static void showMessage(const char *l1, uint16_t c1, const char *l2 = nullptr, uint16_t c2 = C_WHITE) {
  if (!lcdOk) return;
  gfx->fillRect(4, 44, LCD_W - 8, 120, C_BLACK);
  lcdText(8, 48, l1, c1);
  if (l2) lcdText(8, 68, l2, c2);
}

static int vuSegments(uint32_t rms) {
  const uint32_t fullScale = 3000;
  int seg = (int)((rms * 12UL) / fullScale);
  if (seg < 0) seg = 0;
  if (seg > 12) seg = 12;
  return seg;
}

static void drawVu(uint32_t rms) {
  if (!lcdOk) return;

  const int count = 12;
  const int gap = 2;
  const int x = 12;
  const int y = 154;
  const int w = 148;
  const int h = 14;
  const int segW = (w - (count - 1) * gap) / count;
  int segs = vuSegments(rms);

  gfx->drawRect(8, 148, 156, 26, C_GRAY);

  for (int i = 0; i < count; i++) {
    uint16_t color = C_DIM;
    if (i < segs) {
      if (i < 7) color = C_GREEN;
      else if (i < 10) color = C_YELLOW;
      else color = C_RED;
    }
    gfx->fillRect(x + i * (segW + gap), y, segW, h, color);
  }
}

static void showReady() {
  if (!lcdOk) initLcd();

  drawFrame("Coexist Rec v6");
  showMessage("Ready", C_GREEN, "USR2: record", C_YELLOW);
  lcdText(8, 98, "Screen stays ON", C_WHITE);
  lcdText(8, 116, "No LCD refresh in rec", C_GRAY);
  lcdText(8, 134, "No SD write in rec", C_GRAY);
}

static void showRecordingStatic() {
  if (!lcdOk) initLcd();

  drawFrame("Coexist Rec v6");
  showMessage("Recording 5s...", C_ORANGE, "Keep quiet first 1s", C_YELLOW);
  lcdText(8, 98, "LCD is ON", C_WHITE);
  lcdText(8, 116, "Static UI only", C_GRAY);
  lcdText(8, 134, "RAM capture only", C_GRAY);

  lcdApplyLowNoiseMode();

#if LCD_BACKLIGHT_ON_DURING_RECORDING
  digitalWrite(LCD_BL_PIN, HIGH);
#else
  digitalWrite(LCD_BL_PIN, LOW);
#endif

  // End every LCD transaction and keep CS high before PDM starts.
  digitalWrite(LCD_CS_PIN, HIGH);
}

static void showDone(bool ok) {
  if (!lcdOk) initLcd();

  drawFrame("Coexist Rec v6");

  if (ok) {
    showMessage("Saved OK", C_GREEN, "/COEX_DSP.WAV", C_WHITE);

    char buf[48];
    snprintf(buf, sizeof(buf), "HP  P:%lu R:%lu", (unsigned long)hpStats.peak, (unsigned long)hpStats.rms);
    lcdText(8, 98, buf, C_CYAN);

    snprintf(buf, sizeof(buf), "DSP P:%lu R:%lu", (unsigned long)dspStats.peak, (unsigned long)dspStats.rms);
    lcdText(8, 116, buf, C_GREEN);

    snprintf(buf, sizeof(buf), "Notch:%4.0fHz", detectedNoiseHz);
    lcdText(8, 134, buf, C_YELLOW);

    drawVu(dspStats.rms);

    lcdText(8, 198, "/COEX_RAW.WAV", C_GRAY);
    lcdText(8, 216, "/COEX_HP.WAV", C_WHITE);
    lcdText(8, 234, "/COEX_DSP.WAV", C_GREEN);
    lcdText(8, 264, "USR2: again", C_YELLOW);
  } else {
    showMessage("Failed", C_RED, "Check serial log", C_WHITE);
    lcdText(8, 100, "USR2: retry", C_YELLOW);
  }
}

// ========================= WAV helpers =========================

static void writeLE16(File &f, uint16_t v) {
  f.write((uint8_t)(v & 0xFF));
  f.write((uint8_t)((v >> 8) & 0xFF));
}

static void writeLE32(File &f, uint32_t v) {
  f.write((uint8_t)(v & 0xFF));
  f.write((uint8_t)((v >> 8) & 0xFF));
  f.write((uint8_t)((v >> 16) & 0xFF));
  f.write((uint8_t)((v >> 24) & 0xFF));
}

static void writeWavHeader(File &f, uint32_t dataBytes) {
  f.write((const uint8_t *)"RIFF", 4);
  writeLE32(f, 36 + dataBytes);
  f.write((const uint8_t *)"WAVE", 4);

  f.write((const uint8_t *)"fmt ", 4);
  writeLE32(f, 16);
  writeLE16(f, 1);
  writeLE16(f, AUDIO_CHANNELS);
  writeLE32(f, MIC_SAMPLE_RATE_HZ);
  writeLE32(f, AUDIO_BYTE_RATE);
  writeLE16(f, AUDIO_BLOCK_ALIGN);
  writeLE16(f, AUDIO_BITS_PER_SAMPLE);

  f.write((const uint8_t *)"data", 4);
  writeLE32(f, dataBytes);
}

// ========================= SD =========================

static bool mountSd() {
  if (sdMounted) return true;

  spiDevicesIdle();
  SPI.begin(SPI_SCK_PIN, SD_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
  delay(30);

  const uint32_t freqs[] = { 4000000, 1000000, 400000 };
  for (uint8_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
    digitalWrite(SD_CS_PIN, HIGH);
    Serial.printf("[SD] Trying %lu Hz...\n", (unsigned long)freqs[i]);

    if (SD.begin(SD_CS_PIN, SPI, freqs[i])) {
      sdMounted = true;
      uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);
      Serial.printf("[SD] OK, card size=%llu MB, freq=%lu Hz\n",
                    cardSizeMB, (unsigned long)freqs[i]);
      return true;
    }
  }

  Serial.println("[SD] Mount failed");
  return false;
}

static void sdEndForRecording() {
  if (sdMounted) {
    Serial.println("[SD] end before capture");
    SD.end();
    sdMounted = false;
    delay(80);
  }

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
}

static bool writeBufferToWav(const char *path, const uint8_t *data, uint32_t bytes) {
  if (!mountSd()) return false;

  SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    Serial.printf("[SAVE] failed to open %s\n", path);
    return false;
  }

  writeWavHeader(f, bytes);

  uint32_t offset = 0;
  while (offset < bytes) {
    uint32_t chunk = bytes - offset;
    if (chunk > 4096) chunk = 4096;

    size_t written = f.write(data + offset, chunk);
    if (written == 0) {
      Serial.printf("[SAVE] write returned 0: %s\n", path);
      f.close();
      return false;
    }

    offset += written;
    yield();
  }

  f.flush();
  f.close();

  Serial.printf("[SAVE] OK %s, data=%lu, total=%lu\n",
                path, (unsigned long)bytes, (unsigned long)(bytes + 44));
  return true;
}

static void writeStatsCsv() {
  if (!mountSd()) return;

  SD.remove("/COEXSTAT.CSV");
  File f = SD.open("/COEXSTAT.CSV", FILE_WRITE);
  if (!f) return;

  f.println("type,mean,peak,rms,bytes,elapsed_ms,notch_hz");
  f.printf("raw,%ld,%lu,%lu,%lu,%lu,%.1f\n",
           (long)rawStats.mean,
           (unsigned long)rawStats.peak,
           (unsigned long)rawStats.rms,
           (unsigned long)rawStats.bytes,
           (unsigned long)rawStats.elapsedMs,
           detectedNoiseHz);
  f.printf("hp,%ld,%lu,%lu,%lu,%lu,%.1f\n",
           (long)hpStats.mean,
           (unsigned long)hpStats.peak,
           (unsigned long)hpStats.rms,
           (unsigned long)hpStats.bytes,
           (unsigned long)hpStats.elapsedMs,
           detectedNoiseHz);
  f.printf("dsp,%ld,%lu,%lu,%lu,%lu,%.1f\n",
           (long)dspStats.mean,
           (unsigned long)dspStats.peak,
           (unsigned long)dspStats.rms,
           (unsigned long)dspStats.bytes,
           (unsigned long)dspStats.elapsedMs,
           detectedNoiseHz);
  f.close();
}

// ========================= I2S PDM =========================

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
}

static bool initMicPdm() {
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
    Serial.printf("[MIC] i2s_channel_init_pdm_rx_mode failed: %d\n", (int)err);
    deinitMic();
    return false;
  }

#if REDUCE_PDM_CLK_DRIVE
  gpio_set_drive_capability((gpio_num_t)MIC_CLK_PIN, GPIO_DRIVE_CAP_0);
#endif

  err = i2s_channel_enable(g_i2sRxChan);
  if (err != ESP_OK) {
    Serial.printf("[MIC] i2s_channel_enable failed: %d\n", (int)err);
    deinitMic();
    return false;
  }

  Serial.println("[MIC] PDM RX OK");
  return true;

#else
  i2s_driver_uninstall(I2S_PORT);

  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  cfg.sample_rate = MIC_SAMPLE_RATE_HZ;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = MIC_SAMPLES_PER_READ;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;

  esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[MIC] i2s_driver_install failed: %d\n", (int)err);
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_PIN_NO_CHANGE;
  pins.ws_io_num = MIC_CLK_PIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = MIC_DATA_PIN;

  err = i2s_set_pin(I2S_PORT, &pins);
  if (err != ESP_OK) {
    Serial.printf("[MIC] i2s_set_pin failed: %d\n", (int)err);
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  Serial.println("[MIC] PDM RX OK legacy");
  return true;
#endif
}

static bool readMicBlock(size_t *bytesRead) {
  *bytesRead = 0;

#if ESP_IDF_VERSION_MAJOR >= 5
  if (!g_i2sRxChan) return false;
  esp_err_t err = i2s_channel_read(
    g_i2sRxChan,
    (void *)micBuf,
    sizeof(micBuf),
    bytesRead,
    pdMS_TO_TICKS(50)
  );
#else
  esp_err_t err = i2s_read(
    I2S_PORT,
    (void *)micBuf,
    sizeof(micBuf),
    bytesRead,
    pdMS_TO_TICKS(50)
  );
#endif

  return (err == ESP_OK && *bytesRead > 0);
}

static void discardWarmup(uint32_t ms) {
  uint32_t start = millis();
  size_t bytesRead = 0;

  while (millis() - start < ms) {
    readMicBlock(&bytesRead);
    yield();
  }
}

// ========================= Audio processing =========================

static AudioStats computeStats(const int16_t *samples, uint32_t sampleCount, uint32_t bytes, uint32_t elapsedMs) {
  AudioStats s = {};
  s.bytes = bytes;
  s.elapsedMs = elapsedMs;

  if (sampleCount == 0) return s;

  int64_t sum = 0;
  for (uint32_t i = 0; i < sampleCount; i++) {
    sum += samples[i];
  }

  s.mean = (int32_t)(sum / (int64_t)sampleCount);

  uint32_t peak = 0;
  uint64_t sq = 0;

  for (uint32_t i = 0; i < sampleCount; i++) {
    int32_t v = (int32_t)samples[i] - s.mean;
    uint32_t a = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
    if (a > peak) peak = a;
    sq += (uint64_t)a * (uint64_t)a;
  }

  s.peak = peak;
  s.rms = (uint32_t)sqrt((double)sq / (double)sampleCount);
  return s;
}

static float goertzelPower(const int16_t *samples, uint32_t sampleCount, float freqHz) {
  float normalizedFreq = freqHz / (float)MIC_SAMPLE_RATE_HZ;
  float coeff = 2.0f * cosf(2.0f * PI * normalizedFreq);

  float q0 = 0.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;

  for (uint32_t i = 0; i < sampleCount; i++) {
    q0 = coeff * q1 - q2 + (float)samples[i];
    q2 = q1;
    q1 = q0;
  }

  return q1 * q1 + q2 * q2 - coeff * q1 * q2;
}

static float detectDominantNoiseHz(const int16_t *samples, uint32_t sampleCount) {
  if (!samples || sampleCount < 1024) return 0.0f;

  // Remove mean from a short profile in place would damage raw;
  // instead estimate mean and subtract during Goertzel by using a temporary centered value is expensive.
  // For tonal LCD noise search above 300 Hz, DC has little effect, so raw search is OK.

  float bestHz = 0.0f;
  float bestPower = 0.0f;

  for (uint32_t f = NOTCH_SEARCH_START_HZ; f <= NOTCH_SEARCH_END_HZ; f += NOTCH_SEARCH_STEP_HZ) {
    float p = goertzelPower(samples, sampleCount, (float)f);
    if (p > bestPower) {
      bestPower = p;
      bestHz = (float)f;
    }
    yield();
  }

  return bestHz;
}

static void makeHpInPlace(int16_t *samples, uint32_t sampleCount, AudioStats *statsOut, uint32_t elapsedMs) {
  if (!samples || sampleCount == 0) return;

  int64_t sum = 0;
  for (uint32_t i = 0; i < sampleCount; i++) {
    sum += samples[i];
  }

  const float dc = (float)((int32_t)(sum / (int64_t)sampleCount));
  const float alpha = 0.97f;

  float prevX = 0.0f;
  float prevY = 0.0f;

  uint32_t peak = 0;
  uint64_t sq = 0;

  for (uint32_t i = 0; i < sampleCount; i++) {
    float x = (float)samples[i] - dc;
    float y = alpha * (prevY + x - prevX);
    prevX = x;
    prevY = y;

    int32_t v = (int32_t)(y * CLEAN_GAIN);

    if (CLEAN_NOISE_GATE > 0 && abs(v) < CLEAN_NOISE_GATE) {
      v = 0;
    }

    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;

    samples[i] = (int16_t)v;

    uint32_t a = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
    if (a > peak) peak = a;
    sq += (uint64_t)a * (uint64_t)a;
  }

  statsOut->mean = 0;
  statsOut->peak = peak;
  statsOut->rms = (uint32_t)sqrt((double)sq / (double)sampleCount);
  statsOut->bytes = sampleCount * sizeof(int16_t);
  statsOut->elapsedMs = elapsedMs;
}

static void applyNotchInPlace(int16_t *samples, uint32_t sampleCount, float freqHz, float q) {
  if (!samples || sampleCount == 0) return;
  if (freqHz < 50.0f || freqHz > ((float)MIC_SAMPLE_RATE_HZ * 0.48f)) return;

  float w0 = 2.0f * PI * freqHz / (float)MIC_SAMPLE_RATE_HZ;
  float cw = cosf(w0);
  float sw = sinf(w0);
  float alpha = sw / (2.0f * q);

  float b0 = 1.0f;
  float b1 = -2.0f * cw;
  float b2 = 1.0f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cw;
  float a2 = 1.0f - alpha;

  b0 /= a0;
  b1 /= a0;
  b2 /= a0;
  a1 /= a0;
  a2 /= a0;

  float x1 = 0.0f;
  float x2 = 0.0f;
  float y1 = 0.0f;
  float y2 = 0.0f;

  for (uint32_t i = 0; i < sampleCount; i++) {
    float x0 = (float)samples[i];
    float y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

    x2 = x1;
    x1 = x0;
    y2 = y1;
    y1 = y0;

    int32_t v = (int32_t)y0;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    samples[i] = (int16_t)v;
  }
}

static void applyAutoDspInPlace(int16_t *samples, uint32_t sampleCount, float baseHz, AudioStats *statsOut, uint32_t elapsedMs) {
  if (!samples || sampleCount == 0) return;

  if (baseHz >= 100.0f) {
    Serial.printf("[DSP] apply notch %.1f Hz\n", baseHz);
    applyNotchInPlace(samples, sampleCount, baseHz, NOTCH_Q);

    float h2 = baseHz * 2.0f;
    float h3 = baseHz * 3.0f;
    if (h2 < MIC_SAMPLE_RATE_HZ * 0.48f) {
      Serial.printf("[DSP] apply notch %.1f Hz\n", h2);
      applyNotchInPlace(samples, sampleCount, h2, NOTCH_Q);
    }
    if (h3 < MIC_SAMPLE_RATE_HZ * 0.48f) {
      Serial.printf("[DSP] apply notch %.1f Hz\n", h3);
      applyNotchInPlace(samples, sampleCount, h3, NOTCH_Q);
    }
  }

  *statsOut = computeStats(samples, sampleCount, sampleCount * sizeof(int16_t), elapsedMs);
}

// ========================= Recording flow =========================

static bool captureWithLcdOn() {
  if (!recBuffer) {
    Serial.println("[REC] no RAM buffer");
    return false;
  }

  sdEndForRecording();
  showRecordingStatic();

  Serial.println("[REC] start capture: LCD ON/static, SD ended, RAM only");

  if (!initMicPdm()) {
    Serial.println("[REC] mic init failed");
    return false;
  }

  discardWarmup(300);

  uint32_t bytesCaptured = 0;
  uint32_t startMs = millis();

  while (bytesCaptured < RECORD_TARGET_BYTES) {
    size_t bytesRead = 0;
    if (!readMicBlock(&bytesRead)) {
      yield();
      continue;
    }

    uint32_t remain = RECORD_TARGET_BYTES - bytesCaptured;
    uint32_t copyBytes = (bytesRead > remain) ? remain : bytesRead;

    memcpy(recBuffer + bytesCaptured, (const uint8_t *)micBuf, copyBytes);
    bytesCaptured += copyBytes;

    yield();
  }

  uint32_t elapsedMs = millis() - startMs;
  deinitMic();

  rawStats = computeStats((const int16_t *)recBuffer, RECORD_TARGET_SAMPLES, bytesCaptured, elapsedMs);

  Serial.printf("[REC] done: bytes=%lu elapsed=%lu ms mean=%ld peak=%lu rms=%lu\n",
                (unsigned long)rawStats.bytes,
                (unsigned long)rawStats.elapsedMs,
                (long)rawStats.mean,
                (unsigned long)rawStats.peak,
                (unsigned long)rawStats.rms);

  return (bytesCaptured == RECORD_TARGET_BYTES);
}

static bool runRecordSaveFlow() {
  if (busy) return false;
  busy = true;

  bool ok = captureWithLcdOn();

  if (ok) {
    uint32_t profileSamples = PROFILE_SAMPLES;
    if (profileSamples > RECORD_TARGET_SAMPLES) profileSamples = RECORD_TARGET_SAMPLES;

    detectedNoiseHz = detectDominantNoiseHz((const int16_t *)recBuffer, profileSamples);
    Serial.printf("[DSP] detected dominant profile frequency: %.1f Hz\n", detectedNoiseHz);

    Serial.println("[SAVE] save raw WAV");
    ok = writeBufferToWav("/COEX_RAW.WAV", recBuffer, RECORD_TARGET_BYTES);
  }

  if (ok) {
    Serial.println("[PROC] make HP in place");
    makeHpInPlace((int16_t *)recBuffer, RECORD_TARGET_SAMPLES, &hpStats, rawStats.elapsedMs);

    Serial.printf("[PROC] HP stats: peak=%lu rms=%lu gain=%.1f\n",
                  (unsigned long)hpStats.peak,
                  (unsigned long)hpStats.rms,
                  CLEAN_GAIN);

    Serial.println("[SAVE] save HP WAV");
    ok = writeBufferToWav("/COEX_HP.WAV", recBuffer, RECORD_TARGET_BYTES);
  }

  if (ok) {
    Serial.println("[PROC] apply auto notch DSP in place");
    applyAutoDspInPlace((int16_t *)recBuffer, RECORD_TARGET_SAMPLES, detectedNoiseHz, &dspStats, rawStats.elapsedMs);

    Serial.printf("[PROC] DSP stats: peak=%lu rms=%lu notch=%.1f\n",
                  (unsigned long)dspStats.peak,
                  (unsigned long)dspStats.rms,
                  detectedNoiseHz);

    Serial.println("[SAVE] save DSP WAV");
    ok = writeBufferToWav("/COEX_DSP.WAV", recBuffer, RECORD_TARGET_BYTES);
  }

  if (ok) {
    writeStatsCsv();
  }

  showDone(ok);

  busy = false;
  return ok;
}

// ========================= Button =========================

static void checkButton() {
  if (busy) return;

  uint32_t now = millis();
  if (now - lastButtonMs < 80) return;

  static int lastStable = HIGH;
  int current = digitalRead(BTN_RECORD_PIN);

  if (current != lastStable) {
    lastButtonMs = now;
    lastStable = current;

    if (current == LOW) {
      runRecordSaveFlow();
    }
  }
}

// ========================= Arduino entry =========================

void setup() {
  Serial.begin(115200);
  delay(900);

  disableRadios();
  printHeader();

  spiDevicesIdle();
  pinMode(BTN_RECORD_PIN, INPUT_PULLUP);

  recBuffer = (uint8_t *)malloc(RECORD_TARGET_BYTES);
  if (!recBuffer) {
    Serial.printf("[RAM] malloc failed: %lu bytes, free=%lu\n",
                  (unsigned long)RECORD_TARGET_BYTES,
                  (unsigned long)ESP.getFreeHeap());
    return;
  }

  Serial.printf("[RAM] OK: %lu bytes allocated, free=%lu\n",
                (unsigned long)RECORD_TARGET_BYTES,
                (unsigned long)ESP.getFreeHeap());

  initLcd();
  showReady();

  delay(900);
  runRecordSaveFlow();
}

void loop() {
  checkButton();
  delay(5);
}
