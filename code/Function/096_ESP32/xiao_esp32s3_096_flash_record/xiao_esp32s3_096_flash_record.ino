/*
  XIAO ESP32-S3 Plus + 0.96 Inch Display Flash recorder.

  Operation:
    USR1 records 5 seconds from the onboard PDM microphone.
    The latest recording is saved to onboard Flash as /REC_RAW.WAV.
    USR2 plays the saved WAV through a MAX98357A on D11/D12/D13.

  Required libraries:
    - GFX Library for Arduino (Arduino_GFX)
    - ESP32 Arduino core 3.x (uses the ESP-IDF v5 I2S PDM API)
*/

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <LittleFS.h>
#include <math.h>
#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR < 5
  #error "This demo uses the ESP32 Arduino core 3.x / ESP-IDF 5 I2S API."
#endif

#include <driver/gpio.h>
#include <driver/i2s_pdm.h>
#include <driver/i2s_std.h>

static constexpr uint8_t MIC_CLK_PIN   = D0;
static constexpr uint8_t MIC_DATA_PIN  = D1;
static constexpr uint8_t USR1_PIN      = D6;
static constexpr uint8_t USR2_PIN      = D7;
static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t LCD_RST_PIN   = D17;
static constexpr uint8_t LCD_BL_PIN    = D18;
static constexpr uint8_t I2S_DOUT_PIN  = D11;
static constexpr uint8_t I2S_BCLK_PIN  = D12;
static constexpr uint8_t I2S_LRCLK_PIN = D13;

static constexpr uint32_t SAMPLE_RATE_HZ  = 16000;
static constexpr float    RECORD_SECONDS  = 5.0f;
static constexpr uint32_t MIC_WARMUP_MS   = 300;
static constexpr uint32_t CAPTURE_TIMEOUT_MS = (uint32_t)((RECORD_SECONDS + 2.0f) * 1000.0f);
static constexpr uint32_t RECORD_SAMPLES  = (uint32_t)(SAMPLE_RATE_HZ * RECORD_SECONDS);
static constexpr uint32_t RECORD_BYTES    = RECORD_SAMPLES * sizeof(int16_t);
static constexpr size_t   AUDIO_FRAMES    = 256;
static constexpr float    PLAYBACK_GAIN   = 0.75f;
static const char WAV_PATH[] = "/REC_RAW.WAV";

static constexpr int LCD_W = 80;
static constexpr int LCD_H = 160;
static constexpr int LCD_ROTATION = 2;
static constexpr bool LCD_IPS = true;
static constexpr int LCD_COL_OFFSET_1 = 24;
static constexpr int LCD_ROW_OFFSET_1 = 0;
static constexpr int LCD_COL_OFFSET_2 = 24;
static constexpr int LCD_ROW_OFFSET_2 = 0;

Arduino_DataBus *lcdBus = new Arduino_ESP32SPI(
  LCD_DC_PIN, LCD_CS_PIN, LCD_SCK_PIN, LCD_MOSI_PIN
);
Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus, LCD_RST_PIN, LCD_ROTATION, LCD_IPS, LCD_W, LCD_H,
  LCD_COL_OFFSET_1, LCD_ROW_OFFSET_1, LCD_COL_OFFSET_2, LCD_ROW_OFFSET_2
);
Arduino_GFX &tft = *gfx;

static constexpr uint16_t TFT_BLACK     = RGB565_BLACK;
static constexpr uint16_t TFT_WHITE     = RGB565_WHITE;
static constexpr uint16_t TFT_LIGHTGREY = RGB565_LIGHTGREY;
static constexpr uint16_t TFT_DARKGREY  = RGB565_DARKGREY;
static constexpr uint16_t TFT_RED       = RGB565_RED;
static constexpr uint16_t TFT_GREEN     = RGB565_LIGHTGREEN;
static constexpr uint16_t TFT_CYAN      = RGB565_CYAN;
static constexpr uint16_t TFT_YELLOW    = RGB565_YELLOW;

i2s_chan_handle_t micChan = nullptr;
i2s_chan_handle_t txChan  = nullptr;

static int16_t recordBuffer[RECORD_SAMPLES];
static int16_t micBuffer[AUDIO_FRAMES];
// One 32-bit packed word per stereo frame, matching the nRF52840 demo.
static uint32_t stereoBuffer[AUDIO_FRAMES];
static bool hasRecording = false;
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

static void initDisplay() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);

  if (!tft.begin(40000000)) {
    Serial.println("[LCD] Arduino_GFX begin failed");
  }
  tft.setRotation(LCD_ROTATION);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
}

static void drawTopCenteredText(const char *text, int16_t centerX, int16_t topY, uint8_t size) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.setTextSize(size);
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(centerX - (int16_t)w / 2, topY);
  tft.print(text);
}

// Compact 80x160 UI. All text uses font size 1 unless noted.
static void screen(const char *title, const char *line1, const char *line2, uint16_t color) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  drawTopCenteredText(title, LCD_W / 2, 6, 1);
  tft.drawFastHLine(4, 20, LCD_W - 8, color);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(4, 36);
  tft.print(line1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(4, 54);
  tft.print(line2);
}

static void readyScreen() {
  screen("Recorder", "USR1: record", hasRecording ? "USR2: play" : "No recording", TFT_CYAN);
}

static void progressScreen(uint32_t samples) {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 150 && samples < RECORD_SAMPLES) return;
  lastMs = millis();

  uint32_t percent = min((uint32_t)100, samples * 100 / RECORD_SAMPLES);

  tft.fillRect(0, 22, LCD_W, LCD_H - 22, TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  drawTopCenteredText("REC", LCD_W / 2, 40, 2);

  char text[24];
  snprintf(text, sizeof(text), "%lu%%", (unsigned long)percent);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  drawTopCenteredText(text, LCD_W / 2, 76, 2);

  snprintf(text, sizeof(text), "%.1f/%.1fs",
           (double)samples / (double)SAMPLE_RATE_HZ,
           (double)RECORD_SECONDS);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  drawTopCenteredText(text, LCD_W / 2, 104, 1);

  int x = 8, y = 130, w = LCD_W - 16, h = 10;
  tft.drawRect(x, y, w, h, TFT_DARKGREY);
  tft.fillRect(x + 1, y + 1, (w - 2) * (int)percent / 100, h - 2, TFT_RED);
}

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
  // R8 (0 ohm) pulls MIC1 SEL to GND; R6, the alternative 3V3 strap, is DNP.
  // Therefore the onboard microphone drives the left PDM slot.
  pdmCfg.slot_cfg.slot_mask = I2S_PDM_SLOT_LEFT;
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

static void deinitI2S() {
  if (!txChan) return;
  i2s_channel_disable(txChan);
  i2s_del_channel(txChan);
  txChan = nullptr;
}

static bool initI2S() {
  deinitI2S();
  // Match the verified ESP32-S3 speaker test: use I2S0 explicitly and keep
  // enough DMA descriptors to avoid underruns while the UI task is running.
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 6;
  chanCfg.dma_frame_num = AUDIO_FRAMES;
  if (i2s_new_channel(&chanCfg, &txChan, nullptr) != ESP_OK) return false;

  i2s_std_config_t cfg = {};
  cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ);
  // Use the ESP32-S3 240 MHz PLL as the I2S clock source.
  cfg.clk_cfg.clk_src = I2S_CLK_SRC_PLL_240M;
  cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  cfg.gpio_cfg.bclk = (gpio_num_t)I2S_BCLK_PIN;
  cfg.gpio_cfg.ws = (gpio_num_t)I2S_LRCLK_PIN;
  cfg.gpio_cfg.dout = (gpio_num_t)I2S_DOUT_PIN;
  cfg.gpio_cfg.din = I2S_GPIO_UNUSED;
  cfg.gpio_cfg.invert_flags.mclk_inv = false;
  cfg.gpio_cfg.invert_flags.bclk_inv = false;
  cfg.gpio_cfg.invert_flags.ws_inv = false;

  if (i2s_channel_init_std_mode(txChan, &cfg) != ESP_OK) {
    deinitI2S();
    return false;
  }
  if (i2s_channel_enable(txChan) == ESP_OK) return true;
  deinitI2S();
  return false;
}

static bool saveRecording() {
  fs::File f = LittleFS.open(WAV_PATH, "w");
  if (!f) return false;
  WavHeader h = makeWavHeader(RECORD_BYTES);
  bool ok = f.write((const uint8_t *)&h, sizeof(h)) == sizeof(h);
  ok = ok && f.write((const uint8_t *)recordBuffer, RECORD_BYTES) == RECORD_BYTES;
  f.close();
  return ok;
}

static bool loadRecording() {
  fs::File f = LittleFS.open(WAV_PATH, "r");
  if (!f || f.size() < (int)(sizeof(WavHeader) + RECORD_BYTES)) return false;
  f.seek(sizeof(WavHeader));
  bool ok = f.readBytes((char *)recordBuffer, RECORD_BYTES) == RECORD_BYTES;
  f.close();
  return ok;
}

static void recordToFlash() {
  screen("Recorder", "Starting mic...", "", TFT_YELLOW);
  if (!initMic()) {
    screen("Error", "Mic init failed", "Check mic", TFT_RED);
    Serial.println("[MIC] initialization failed");
    delay(1200);
    return;
  }

  screen("Recorder", "Mic warm-up...", "Please wait", TFT_YELLOW);
  uint32_t warmupStart = millis();
  while (millis() - warmupStart < MIC_WARMUP_MS) {
    size_t discarded = 0;
    i2s_channel_read(micChan, micBuffer, sizeof(micBuffer), &discarded, pdMS_TO_TICKS(50));
  }

  uint32_t captured = 0;
  uint32_t captureStart = millis();
  int16_t sampleMin = INT16_MAX;
  int16_t sampleMax = INT16_MIN;
  int64_t sampleSum = 0;
  while (captured < RECORD_SAMPLES) {
    size_t bytesRead = 0;
    esp_err_t readResult = i2s_channel_read(
      micChan, micBuffer, sizeof(micBuffer), &bytesRead, pdMS_TO_TICKS(300)
    );
    if (readResult == ESP_OK && bytesRead > 0) {
      uint32_t samples = min((uint32_t)(bytesRead / sizeof(int16_t)), RECORD_SAMPLES - captured);
      for (uint32_t i = 0; i < samples; ++i) {
        int16_t sample = micBuffer[i];
        if (sample < sampleMin) sampleMin = sample;
        if (sample > sampleMax) sampleMax = sample;
        sampleSum += sample;
      }
      memcpy(&recordBuffer[captured], micBuffer, samples * sizeof(int16_t));
      captured += samples;
      progressScreen(captured);
    }
    if (millis() - captureStart > CAPTURE_TIMEOUT_MS) {
      deinitMic();
      screen("Error", "Mic timeout", "Try again", TFT_RED);
      Serial.printf("[MIC] capture timeout: %lu/%lu samples\n",
                    (unsigned long)captured, (unsigned long)RECORD_SAMPLES);
      delay(1200);
      return;
    }
  }
  deinitMic();

  Serial.printf("[MIC] captured=%lu min=%d max=%d mean=%ld slot=LEFT\n",
                (unsigned long)captured, sampleMin, sampleMax,
                (long)(sampleSum / (int64_t)captured));

  screen("Recorder", "Saving...", WAV_PATH, TFT_YELLOW);
  hasRecording = saveRecording();
  screen(hasRecording ? "Done" : "Error",
         hasRecording ? "Saved WAV" : "Write failed",
         hasRecording ? "USR2: play" : "Check flash", hasRecording ? TFT_GREEN : TFT_RED);
  Serial.println(hasRecording ? "[FLASH] recording saved" : "[FLASH] recording save failed");
  delay(1000);
}

static int16_t scaleSample(int16_t sample) {
  int32_t v = (int32_t)(sample * PLAYBACK_GAIN);
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static uint32_t packStereoSample(int16_t sample) {
  uint16_t s = (uint16_t)scaleSample(sample);
  return ((uint32_t)s << 16) | s;
}

static void playRecording() {
  if (!hasRecording) {
    screen("Playback", "No recording", "Press USR1 first", TFT_YELLOW);
    delay(900);
    readyScreen();
    return;
  }

  screen("Playback", "Loading WAV...", WAV_PATH, TFT_GREEN);
  if (!loadRecording() || !initI2S()) {
    screen("Error", "Playback failed", "Record again", TFT_RED);
    return;
  }

  screen("Playback", "Playing...", "Please wait", TFT_GREEN);
  for (uint32_t pos = 0; pos < RECORD_SAMPLES; pos += AUDIO_FRAMES) {
    uint32_t frames = min((uint32_t)AUDIO_FRAMES, RECORD_SAMPLES - pos);
    for (uint32_t i = 0; i < frames; ++i) {
      stereoBuffer[i] = packStereoSample(recordBuffer[pos + i]);
    }
    size_t bytesWritten = 0;
    const size_t bytesToWrite = frames * sizeof(uint32_t);
    esp_err_t writeResult = i2s_channel_write(txChan, stereoBuffer, bytesToWrite, &bytesWritten, portMAX_DELAY);
    if (writeResult != ESP_OK || bytesWritten != bytesToWrite) {
      Serial.printf("[I2S] recording write err=%d bytes=%u/%u\n",
                    (int)writeResult, (unsigned)bytesWritten, (unsigned)bytesToWrite);
    }
  }
  deinitI2S();
  screen("Playback", "Finished", "USR1: rec  USR2: play", TFT_GREEN);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
  initDisplay();

  hasRecording = LittleFS.begin(true) && LittleFS.exists(WAV_PATH);
  readyScreen();
}

void loop() {
  bool usr1 = digitalRead(USR1_PIN);
  bool usr2 = digitalRead(USR2_PIN);

  if (lastUsr1 == HIGH && usr1 == LOW) {
    delay(25);
    recordToFlash();
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
