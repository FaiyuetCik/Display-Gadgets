/*
  XIAO ESP32-S3 Plus + 1.47 Inch Display SD recorder.

  Operation:
    USR1 records 5 seconds from the onboard PDM microphone.
    The latest recording is saved to the microSD card as /REC_RAW.WAV.
    USR2 plays the saved WAV through a MAX98357A on D11/D12/D13.

  Hardware:
    LCD and SD card share one SPI bus:
      SD CS = D6
      SCK     = D8   MISO  = D9   MOSI = D10
      LCD RST = 13   LCD BL = 12
    Buttons: USR1 = D19 (left), USR2 = D15 (right)

  Required libraries:
    - Seeed_GFX2
    - ESP32 Arduino core 3.x (SD and ESP-IDF v5 I2S APIs are built in)

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  Arduino_SWSPI + Arduino_ST7789 bus/panel construction, the manual RST pulse /
  backlight-on / gfx->begin() / setRotation(0) / MADCTL 0x36/0x48 write /
  invertDisplay(false). Config_XIAO_1inch47_Touch_JD9853A bakes 172x320 BGR
  invert=false rot0. PDM record (I2S PDM RX) and MAX98357A playback (I2S STD),
  the WAV write/read, and the button state machine are unchanged. On ESP32-S3
  the SD uses the default SPI/FSPI host while the LCD uses HSPI, so the two
  devices share the physical pins without competing for one SPI host.
  Arduino_GFX's RGB565_* color macros are replaced with their exact RGB565 hex
  values (note RGB565_GREEN is 0x0400, not TFT_GREEN 0x07E0).
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_JD9853A.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR < 5
  #error "This demo uses the ESP32 Arduino core 3.x / ESP-IDF 5 I2S API."
#endif

#include <driver/gpio.h>
#include <driver/i2s_pdm.h>
#include <driver/i2s_std.h>

static constexpr uint8_t MIC_CLK_PIN   = D0;
static constexpr uint8_t MIC_DATA_PIN  = D1;
static constexpr uint8_t SD_CS_PIN     = D6;
static constexpr uint8_t SPI_SCK_PIN   = D8;
static constexpr uint8_t SD_MISO_PIN   = D9;
static constexpr uint8_t SPI_MOSI_PIN  = D10;
static constexpr uint8_t I2S_DOUT_PIN  = 38;  // D11 -> GPIO38
static constexpr uint8_t I2S_BCLK_PIN  = 39;  // D12 -> GPIO39
static constexpr uint8_t I2S_LRCLK_PIN = 40;  // D13 -> GPIO40
static constexpr uint8_t USR1_PIN      = 11;  // D19 -> GPIO11
static constexpr uint8_t USR2_PIN      = 42;  // D15 -> GPIO42

static constexpr int8_t LCD_RST_PIN = 13;  // XIAO ESP32-S3 Plus
static constexpr int8_t LCD_BL_PIN  = 12;

static constexpr uint32_t SAMPLE_RATE_HZ = 16000;
static constexpr uint32_t RECORD_SECONDS = 5;
static constexpr uint32_t MIC_WARMUP_MS  = 300;
static constexpr uint32_t CAPTURE_TIMEOUT_MS = (RECORD_SECONDS + 2) * 1000UL;
static constexpr uint32_t RECORD_SAMPLES = SAMPLE_RATE_HZ * RECORD_SECONDS;
static constexpr uint32_t RECORD_BYTES   = RECORD_SAMPLES * sizeof(int16_t);
static constexpr size_t   AUDIO_FRAMES   = 256;
static constexpr float    PLAYBACK_GAIN  = 0.75f;
static const char WAV_PATH[] = "/REC_RAW.WAV";

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;

Seeed_GFX display;

static constexpr uint16_t C_BLACK     = 0x0000;  // RGB565_BLACK
static constexpr uint16_t C_WHITE     = 0xFFFF;  // RGB565_WHITE
static constexpr uint16_t C_RED       = 0xF800;  // RGB565_RED
static constexpr uint16_t C_GREEN     = 0x0400;  // RGB565_GREEN
static constexpr uint16_t C_CYAN      = 0x07FF;  // RGB565_CYAN
static constexpr uint16_t C_YELLOW    = 0xFFE0;  // RGB565_YELLOW
static constexpr uint16_t C_LIGHTGREY = 0xC618;
static constexpr uint16_t C_DARKGREY  = 0x7BEF;

i2s_chan_handle_t micChan = nullptr;
i2s_chan_handle_t txChan  = nullptr;

static int16_t recordBuffer[RECORD_SAMPLES];
static int16_t micBuffer[AUDIO_FRAMES];
static int16_t stereoBuffer[AUDIO_FRAMES * 2];
static bool hasRecording = false;
static bool sdMounted = false;
static bool lastUsr1 = HIGH;
static bool lastUsr2 = HIGH;

struct WavHeader {
  char riff[4];
  uint32_t chunkSize;
  char wave[4];
  char fmt[4];
  uint32_t subchunk1Size;
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;
  char data[4];
  uint32_t subchunk2Size;
};

static WavHeader makeWavHeader(uint32_t dataBytes) {
  WavHeader h = {
    {'R', 'I', 'F', 'F'}, 36 + dataBytes, {'W', 'A', 'V', 'E'},
    {'f', 'm', 't', ' '}, 16, 1, 1, SAMPLE_RATE_HZ,
    SAMPLE_RATE_HZ * 2, 2, 16, {'d', 'a', 't', 'a'}, dataBytes
  };
  return h;
}

// --- Shared SPI bus helpers (LCD and SD share SCK/MOSI) -------------
// Seeed_GFX's Bus_SPI owns LCD_CS (D2) and parks it HIGH between transactions;
// only SD_CS (D6) is idled here so the SD card stays off the shared bus.

static void spiDevicesIdle() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
}

static void acquireForLcd() {
  spiDevicesIdle();
  delayMicroseconds(2);
}

static void acquireForSd() {
  spiDevicesIdle();
  delayMicroseconds(2);
}

// --- Display ---------------------------------------------------------

static void initDisplay() {
  spiDevicesIdle();

  if (!display.begin<Board_XIAO_1inch47_Touch_Display<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_XIAO_1inch47_Touch_JD9853A>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(C_BLACK);
}

static void screen(const char *title, const char *line1, const char *line2, uint16_t color) {
  acquireForLcd();
  display.fillScreen(C_BLACK);
  display.setTextSize(2);
  display.setTextColor(color, C_BLACK);
  display.setCursor(8, 10);
  display.print(title);
  display.drawFastHLine(0, 34, LCD_W, color);
  display.setTextSize(1);
  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(8, 58);
  display.print(line1);
  display.setTextColor(C_LIGHTGREY, C_BLACK);
  display.setCursor(8, 78);
  display.print(line2);
}

static void readyScreen() {
  screen("SD Recorder", "USR1: record", hasRecording ? "USR2: play" : "No recording", C_CYAN);
}

static void progressScreen(uint32_t samples) {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 250 && samples < RECORD_SAMPLES) return;
  lastMs = millis();

  uint32_t percent = min((uint32_t)100, samples * 100 / RECORD_SAMPLES);

  acquireForLcd();
  char text[28];
  snprintf(text, sizeof(text), "%lu%%  %lu/%lus",
           (unsigned long)percent,
           (unsigned long)(samples / SAMPLE_RATE_HZ),
           (unsigned long)RECORD_SECONDS);
  display.fillRect(8, 86, LCD_W - 16, 14, C_BLACK);
  display.setTextSize(1);
  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(8, 88);
  display.print(text);

  int x = 8, y = 110, w = LCD_W - 16, h = 14;
  display.drawRect(x, y, w, h, C_DARKGREY);
  display.fillRect(x + 1, y + 1, w - 2, h - 2, C_BLACK);
  display.fillRect(x + 1, y + 1, (w - 2) * (int)percent / 100, h - 2, C_RED);
}

// --- SD card ---------------------------------------------------------

static bool beginSd() {
  if (sdMounted) return true;

  acquireForSd();
  // LCD uses HSPI through Seeed_GFX2; use the default FSPI instance for SD.
  SPI.begin(SPI_SCK_PIN, SD_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
  delay(30);

  const uint32_t freqs[] = {8000000, 4000000, 1000000, 400000};
  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
    spiDevicesIdle();
    Serial.printf("[SD] trying %lu Hz\n", (unsigned long)freqs[i]);
    if (SD.begin(SD_CS_PIN, SPI, freqs[i])) {
      sdMounted = true;
      Serial.printf("[SD] mounted at %lu Hz\n", (unsigned long)freqs[i]);
      return true;
    }
    SD.end();
    delay(50);
  }
  Serial.println("[SD] mount failed");
  return false;
}

static void endSdSession() {
  if (!sdMounted) return;
  Serial.println("[SD] ending session");
  SD.end();
  sdMounted = false;
  SPI.end();
  delay(10);
}

static bool saveRecording() {
  if (!beginSd()) return false;
  acquireForSd();
  Serial.println("[SD] opening recording");
  File file = SD.open(WAV_PATH, FILE_WRITE);
  if (!file) return false;

  WavHeader h = makeWavHeader(RECORD_BYTES);
  Serial.println("[SD] writing WAV data");
  bool ok = file.write((const uint8_t *)&h, sizeof(h)) == sizeof(h);

  const uint8_t *data = (const uint8_t *)recordBuffer;
  uint32_t offset = 0;
  while (ok && offset < RECORD_BYTES) {
    uint32_t chunk = min((uint32_t)4096, RECORD_BYTES - offset);
    size_t written = file.write(data + offset, chunk);
    if (written == 0) ok = false;
    else offset += written;
  }

  file.flush();
  file.close();
  if (!ok) SD.remove(WAV_PATH);
  Serial.println(ok ? "[SD] recording saved" : "[SD] recording write failed");
  return ok;
}

static bool loadRecording() {
  if (!beginSd()) return false;
  acquireForSd();
  Serial.println("[SD] opening recording for playback");
  File file = SD.open(WAV_PATH, FILE_READ);
  if (!file) return false;
  if (file.size() < sizeof(WavHeader) + RECORD_BYTES || !file.seek(sizeof(WavHeader))) {
    file.close();
    return false;
  }

  uint8_t *dst = (uint8_t *)recordBuffer;
  uint32_t remaining = RECORD_BYTES;
  while (remaining > 0) {
    uint32_t chunk = min((uint32_t)4096, remaining);
    int n = file.read(dst, chunk);
    if (n <= 0) {
      file.close();
      return false;
    }
    dst += n;
    remaining -= (uint32_t)n;
  }

  file.close();
  return true;
}

// --- PDM mic (I2S PDM RX) -------------------------------------------

static void deinitMic() {
  if (!micChan) return;
  i2s_channel_disable(micChan);
  i2s_del_channel(micChan);
  micChan = nullptr;
}

static bool initMic() {
  deinitMic();
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 4;
  chanCfg.dma_frame_num = AUDIO_FRAMES;
  if (i2s_new_channel(&chanCfg, nullptr, &micChan) != ESP_OK) return false;

  i2s_pdm_rx_config_t pdmCfg = {};
  pdmCfg.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ);
  pdmCfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  pdmCfg.gpio_cfg.clk = (gpio_num_t)MIC_CLK_PIN;
  pdmCfg.gpio_cfg.din = (gpio_num_t)MIC_DATA_PIN;
  pdmCfg.gpio_cfg.invert_flags.clk_inv = false;

  if (i2s_channel_init_pdm_rx_mode(micChan, &pdmCfg) != ESP_OK) {
    deinitMic();
    return false;
  }
  gpio_set_drive_capability((gpio_num_t)MIC_CLK_PIN, GPIO_DRIVE_CAP_0);
  if (i2s_channel_enable(micChan) == ESP_OK) return true;
  deinitMic();
  return false;
}

// --- I2S playback (MAX98357A) ---------------------------------------

static void deinitI2S() {
  if (!txChan) return;
  i2s_channel_disable(txChan);
  i2s_del_channel(txChan);
  txChan = nullptr;
}

static bool initI2S() {
  deinitI2S();
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 4;
  chanCfg.dma_frame_num = AUDIO_FRAMES;
  if (i2s_new_channel(&chanCfg, &txChan, nullptr) != ESP_OK) return false;

  i2s_std_config_t cfg = {};
  cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ);
  cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  cfg.gpio_cfg.bclk = (gpio_num_t)I2S_BCLK_PIN;
  cfg.gpio_cfg.ws = (gpio_num_t)I2S_LRCLK_PIN;
  cfg.gpio_cfg.dout = (gpio_num_t)I2S_DOUT_PIN;
  cfg.gpio_cfg.din = I2S_GPIO_UNUSED;

  if (i2s_channel_init_std_mode(txChan, &cfg) != ESP_OK) {
    deinitI2S();
    return false;
  }
  if (i2s_channel_enable(txChan) == ESP_OK) return true;
  deinitI2S();
  return false;
}

// --- Record / Playback ----------------------------------------------

static void recordToSd() {
  screen("SD Recorder", "Starting mic...", "", C_YELLOW);
  if (!initMic()) {
    screen("Error", "Mic init failed", "Check mic", C_RED);
    return;
  }

  screen("Recording", "Warming up mic...", "Please wait", C_YELLOW);
  uint32_t warmupStart = millis();
  while (millis() - warmupStart < MIC_WARMUP_MS) {
    size_t discarded = 0;
    i2s_channel_read(micChan, micBuffer, sizeof(micBuffer), &discarded, pdMS_TO_TICKS(50));
  }

  screen("Recording", "Capturing 5 seconds", "", C_RED);

  uint32_t captured = 0;
  uint32_t captureStart = millis();
  while (captured < RECORD_SAMPLES) {
    size_t bytesRead = 0;
    esp_err_t readResult = i2s_channel_read(
      micChan, micBuffer, sizeof(micBuffer), &bytesRead, pdMS_TO_TICKS(300)
    );
    if (readResult == ESP_OK && bytesRead > 0) {
      uint32_t samples = min((uint32_t)(bytesRead / sizeof(int16_t)), RECORD_SAMPLES - captured);
      memcpy(&recordBuffer[captured], micBuffer, samples * sizeof(int16_t));
      captured += samples;
      progressScreen(captured);
    }
    if (millis() - captureStart > CAPTURE_TIMEOUT_MS) {
      deinitMic();
      screen("Error", "Mic capture timeout", "Try recording again", C_RED);
      return;
    }
  }
  deinitMic();

  screen("SD Recorder", "Saving to SD...", WAV_PATH, C_YELLOW);
  Serial.println("[LCD] stopping LCD bus for SD write");
  display.end();
  hasRecording = saveRecording();
  endSdSession();
  Serial.println("[LCD] restarting LCD after SD write");
  initDisplay();
  screen(hasRecording ? "Done" : "Error",
         hasRecording ? "Saved SD WAV" : "SD write failed",
         hasRecording ? "USR2: play" : "Check SD card", hasRecording ? C_GREEN : C_RED);
}

static int16_t scaleSample(int16_t sample) {
  int32_t v = (int32_t)(sample * PLAYBACK_GAIN);
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static void playRecording() {
  if (!hasRecording) {
    screen("Playback", "No recording", "Press USR1 first", C_YELLOW);
    delay(900);
    readyScreen();
    return;
  }

  screen("Playback", "Loading SD WAV...", WAV_PATH, C_GREEN);
  Serial.println("[LCD] stopping LCD bus for SD read");
  display.end();
  bool loaded = loadRecording();
  endSdSession();
  Serial.println("[LCD] restarting LCD after SD read");
  initDisplay();
  if (!loaded || !initI2S()) {
    screen("Error", "Playback failed", "Record again", C_RED);
    return;
  }

  screen("Playback", "Playing...", "Please wait", C_GREEN);
  for (uint32_t pos = 0; pos < RECORD_SAMPLES; pos += AUDIO_FRAMES) {
    uint32_t frames = min((uint32_t)AUDIO_FRAMES, RECORD_SAMPLES - pos);
    for (uint32_t i = 0; i < frames; ++i) {
      int16_t s = scaleSample(recordBuffer[pos + i]);
      stereoBuffer[i * 2 + 0] = s;
      stereoBuffer[i * 2 + 1] = s;
    }
    size_t bytesWritten = 0;
    i2s_channel_write(txChan, stereoBuffer, frames * 2 * sizeof(int16_t), &bytesWritten, portMAX_DELAY);
  }
  deinitI2S();
  screen("Playback", "Finished", "USR1: rec  USR2: play", C_GREEN);
}

// --- Setup / Loop ----------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);

  // Mount SD before starting the LCD's separate HSPI host.
  Serial.println("[BOOT] mounting SD");
  bool sdOk = beginSd();
  if (sdOk) {
    // Remove the previous file while the LCD bus is not initialized. SD.remove
    // can block when called after the LCD has taken ownership of its SPI host.
    Serial.println("[BOOT] clearing old recording");
    bool removed = SD.remove(WAV_PATH);
    Serial.println(removed ? "[BOOT] old recording cleared"
                           : "[BOOT] no old recording to clear");
    endSdSession();
  }
  Serial.println("[BOOT] SD stage done");
  Serial.println("[BOOT] starting LCD");
  initDisplay();
  Serial.println("[BOOT] LCD stage done");
  readyScreen();
  Serial.println("[BOOT] ready screen done");

  if (!sdOk) {
    screen("Error", "SD mount failed", "Check SD card", C_RED);
    Serial.println("[BOOT] SD error screen done");
  } else {
    // Do not probe the filesystem here. On this shared SPI wiring, an
    // immediate SD.exists() after LCD startup can block the ESP32-S3 host.
    // A recording is marked available only after a successful save.
    hasRecording = false;
    Serial.println("[BOOT] setup done");
  }
}

void loop() {
  bool usr1 = digitalRead(USR1_PIN);
  bool usr2 = digitalRead(USR2_PIN);

  static uint32_t lastButtonDebug = 0;
  if (millis() - lastButtonDebug >= 1000) {
    lastButtonDebug = millis();
    Serial.printf("[BTN] USR1=%d USR2=%d\n", usr1, usr2);
  }

  if (lastUsr1 == HIGH && usr1 == LOW) {
    delay(25);
    recordToSd();
    readyScreen();
  }
  if (lastUsr2 == HIGH && usr2 == LOW) {
    delay(25);
    playRecording();
    delay(600);
    readyScreen();
  }

  lastUsr1 = usr1;
  lastUsr2 = usr2;
  delay(10);
}
