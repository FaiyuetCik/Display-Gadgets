/**
 * Product: XIAO 1.14 inch LCD Board (ST7789 135x240 IPS, no touch)
 * Display: ST7789 135x240, RGB, no touch
 * Target:  XIAO ESP32-S3 Plus (RST=13, BL=12). For nRF52840 Plus use the sibling
 *          nRF52840 Plus folder (RST=38, BL=37).
 * Wiring:  CS=D2, DC=D3, MOSI=D10, SCLK=D8; RST=13, BL=12.
 * Demo:    USR1 records 5s of PDM mic audio to on-board Flash (LittleFS, /REC_RAW.WAV); USR2 plays it back over I2S to a MAX98357A. LCD shows status/progress.
 *
 * Ported from XIAO-Display-Board-main (xiao_esp32s3_114_flash_record, TFT_eSPI). The Seeed_GFX
 * Board/Config templates replace the original bus+panel setup, manual MADCTL/invert/
 * swapbytes, and (for TFT_eSPI) the driver.h User_Setup. TIER-2: requires an external
 * MAX98357A I2S amp (DIN=D11/BCLK=D12/LRCLK=D13) and LittleFS.
 */

#include <Arduino.h>
#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include <LittleFS.h>
#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR < 5
  #error "This demo uses the ESP32 Arduino core 3.x / ESP-IDF 5 I2S API."
#endif

#include <driver/gpio.h>
#include <driver/i2s_pdm.h>
#include <driver/i2s_std.h>

Seeed_GFX display;

static constexpr int8_t LCD_RST_PIN = 13;  // XIAO ESP32-S3 Plus
static constexpr int8_t LCD_BL_PIN  = 12;
static constexpr int LCD_W = 135;
static constexpr int LCD_H = 240;

static constexpr uint8_t MIC_CLK_PIN = D0;
static constexpr uint8_t MIC_DATA_PIN = D1;
static constexpr uint8_t USR1_PIN = D6;
static constexpr uint8_t USR2_PIN = D7;
static constexpr uint8_t I2S_DOUT_PIN = 38;  // D11 -> GPIO38
static constexpr uint8_t I2S_BCLK_PIN = 39;  // D12 -> GPIO39
static constexpr uint8_t I2S_LRCLK_PIN = 40;  // D13 -> GPIO40

static constexpr uint32_t SAMPLE_RATE_HZ = 16000;
static constexpr uint32_t RECORD_SECONDS = 5;
static constexpr uint32_t MIC_WARMUP_MS = 300;
static constexpr uint32_t RECORD_SAMPLES = SAMPLE_RATE_HZ * RECORD_SECONDS;
static constexpr uint32_t RECORD_BYTES = RECORD_SAMPLES * sizeof(int16_t);
static constexpr uint32_t CAPTURE_TIMEOUT_MS = (RECORD_SECONDS + 2) * 1000UL;
static constexpr size_t AUDIO_FRAMES = 256;
static constexpr float PLAYBACK_GAIN = 0.75f;
static const char WAV_PATH[] = "/REC_RAW.WAV";

i2s_chan_handle_t micChan = nullptr;
i2s_chan_handle_t txChan = nullptr;

static int16_t recordBuffer[RECORD_SAMPLES];
static int16_t micBuffer[AUDIO_FRAMES];
static int16_t stereoBuffer[AUDIO_FRAMES * 2];
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
  if (!display.begin<Board_XIAO_1inch14_LCD<LCD_RST_PIN, LCD_BL_PIN>, Config_Seeed_1inch14_LCD_ST7789>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(TFT_BLACK);
}

static void screen(const char *title, const char *line1, const char *line2, uint16_t color) {
  display.fillScreen(TFT_BLACK);
  // The 135 px-wide 1.14-inch panel cannot fit long titles at size 2.
  display.setTextSize(1);
  display.setTextColor(color, TFT_BLACK);
  display.setCursor(8, 10);
  display.print(title);
  display.drawFastHLine(0, 34, LCD_W, color);
  display.setTextSize(1);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(8, 58);
  display.print(line1);
  display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  display.setCursor(8, 78);
  display.print(line2);
}

static void readyScreen() {
  screen("Flash Recorder", "USR1: record", hasRecording ? "USR2: play Flash WAV" : "No saved recording", TFT_CYAN);
}

static void progressScreen(uint32_t samples) {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 250 && samples < RECORD_SAMPLES) return;
  lastMs = millis();

  uint32_t percent = min((uint32_t)100, samples * 100 / RECORD_SAMPLES);
  display.fillRect(0, 52, LCD_W, 100, TFT_BLACK);
  display.setTextColor(TFT_RED, TFT_BLACK);
  display.setTextSize(2);
  display.setCursor(8, 58);
  display.print("Recording");
  display.setTextSize(1);
  display.setTextColor(TFT_WHITE, TFT_BLACK);

  char text[24];
  snprintf(text, sizeof(text), "%lu%%", (unsigned long)percent);
  display.setCursor(8, 90);
  display.print(text);

  snprintf(text, sizeof(text), "%lu / %lu s",
           (unsigned long)(samples / SAMPLE_RATE_HZ),
           (unsigned long)RECORD_SECONDS);
  display.setCursor(55, 90);
  display.print(text);

  int x = 8, y = 112, w = LCD_W - 16, h = 14;
  display.drawRect(x, y, w, h, TFT_DARKGREY);
  display.fillRect(x + 1, y + 1, (w - 2) * percent / 100, h - 2, TFT_RED);
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

static bool saveRecording() {
  File f = LittleFS.open(WAV_PATH, "w");
  if (!f) return false;
  WavHeader h = makeWavHeader(RECORD_BYTES);
  bool ok = f.write((const uint8_t *)&h, sizeof(h)) == sizeof(h);
  ok = ok && f.write((const uint8_t *)recordBuffer, RECORD_BYTES) == RECORD_BYTES;
  f.close();
  return ok;
}

static bool loadRecording() {
  File f = LittleFS.open(WAV_PATH, "r");
  if (!f || f.size() < (int)(sizeof(WavHeader) + RECORD_BYTES)) return false;
  f.seek(sizeof(WavHeader));
  bool ok = f.readBytes((char *)recordBuffer, RECORD_BYTES) == RECORD_BYTES;
  f.close();
  return ok;
}

static void recordToFlash() {
  hasRecording = false;
  screen("Flash Recorder", "Starting PDM...", "", TFT_YELLOW);
  if (!initMic()) {
    screen("Error", "PDM init failed", "Check microphone", TFT_RED);
    return;
  }

  // Match the known-good original recorder: continuously drain the PDM DMA
  // buffers during a short warm-up period before capturing real audio.
  screen("Recording", "Warming up mic...", "Please wait", TFT_YELLOW);
  uint32_t warmupStart = millis();
  while (millis() - warmupStart < MIC_WARMUP_MS) {
    size_t discarded = 0;
    i2s_channel_read(micChan, micBuffer, sizeof(micBuffer), &discarded,
                     pdMS_TO_TICKS(50));
  }

  screen("Recording", "Capturing 5 seconds", "", TFT_RED);

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
      // Do not refresh the LCD from inside the PDM capture loop. The
      // Seeed_GFX2 SPI transfer can block long enough to starve the PDM DMA.
      // The screen remains on "Capturing 5 seconds" until capture completes.
    }
    if (millis() - captureStart > CAPTURE_TIMEOUT_MS) {
      deinitMic();
      screen("Error", "Mic capture timeout", "Try recording again", TFT_RED);
      return;
    }
  }
  deinitMic();

  screen("Flash Recorder", "Saving to Flash...", WAV_PATH, TFT_YELLOW);
  hasRecording = saveRecording();
  screen(hasRecording ? "Done" : "Error",
         hasRecording ? "Saved Flash WAV" : "Flash write failed",
         hasRecording ? "USR2: play" : "Check partition", hasRecording ? TFT_GREEN : TFT_RED);
}

static int16_t scaleSample(int16_t sample) {
  int32_t v = (int32_t)(sample * PLAYBACK_GAIN);
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static void playRecording() {
  if (!hasRecording) {
    screen("Playback", "No recording", "Press USR1 first", TFT_YELLOW);
    delay(900);
    readyScreen();
    return;
  }

  screen("Playback", "Loading Flash WAV...", WAV_PATH, TFT_GREEN);
  if (!loadRecording() || !initI2S()) {
    screen("Error", "Playback init failed", "Record again", TFT_RED);
    return;
  }

  screen("Playback", "Playing raw audio", "Please wait...", TFT_GREEN);
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
  screen("Playback", "Finished", "USR1: record  USR2: play", TFT_GREEN);
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
