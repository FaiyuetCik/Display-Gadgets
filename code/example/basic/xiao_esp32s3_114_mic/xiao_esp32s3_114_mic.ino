/*
  XIAO ESP32-S3 Plus + 1.14 Inch Display PDM microphone basic.

  MIC_CLK=D0, MIC_DATA=D1.
*/

#include <Arduino.h>
#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR >= 5
  #include <driver/i2s_pdm.h>
#else
  #include <driver/i2s.h>
#endif

#include <driver/gpio.h>
#include <math.h>

static constexpr uint8_t MIC_CLK_PIN = D0;
static constexpr uint8_t MIC_DATA_PIN = D1;
static constexpr int MIC_SAMPLE_RATE_HZ = 16000;
static constexpr size_t MIC_SAMPLES_PER_READ = 256;
static constexpr bool MIC_CLK_INVERT = false;

#if ESP_IDF_VERSION_MAJOR >= 5
static i2s_chan_handle_t i2sRxChan = nullptr;
#else
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
#endif

static int16_t sampleBuf[MIC_SAMPLES_PER_READ];

struct AudioStats {
  int32_t mean;
  uint32_t peak;
  uint32_t rms;
};

static void deinitMic() {
#if ESP_IDF_VERSION_MAJOR >= 5
  if (i2sRxChan) {
    i2s_channel_disable(i2sRxChan);
    i2s_del_channel(i2sRxChan);
    i2sRxChan = nullptr;
  }
#else
  i2s_driver_uninstall(I2S_PORT);
#endif
}

static bool initMic() {
  deinitMic();

#if ESP_IDF_VERSION_MAJOR >= 5
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 4;
  chanCfg.dma_frame_num = MIC_SAMPLES_PER_READ;

  esp_err_t err = i2s_new_channel(&chanCfg, nullptr, &i2sRxChan);
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

  err = i2s_channel_init_pdm_rx_mode(i2sRxChan, &pdmCfg);
  if (err != ESP_OK) {
    Serial.printf("[MIC] init_pdm failed: %d\n", (int)err);
    deinitMic();
    return false;
  }

  gpio_set_drive_capability((gpio_num_t)MIC_CLK_PIN, GPIO_DRIVE_CAP_0);

  err = i2s_channel_enable(i2sRxChan);
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
    deinitMic();
    return false;
  }
#endif

  Serial.println("[MIC] OK");
  return true;
}

static bool readMic(size_t *bytesRead, uint32_t timeoutMs) {
  *bytesRead = 0;

#if ESP_IDF_VERSION_MAJOR >= 5
  if (!i2sRxChan) return false;
  esp_err_t err = i2s_channel_read(i2sRxChan, sampleBuf, sizeof(sampleBuf), bytesRead, pdMS_TO_TICKS(timeoutMs));
#else
  esp_err_t err = i2s_read(I2S_PORT, sampleBuf, sizeof(sampleBuf), bytesRead, pdMS_TO_TICKS(timeoutMs));
#endif

  return err == ESP_OK && *bytesRead > 0;
}

static AudioStats calcStats(const int16_t *samples, uint32_t count) {
  AudioStats s = {};
  if (!count) return s;

  int64_t sum = 0;
  for (uint32_t i = 0; i < count; ++i) {
    sum += samples[i];
  }
  s.mean = (int32_t)(sum / count);

  uint32_t peak = 0;
  uint64_t sq = 0;
  for (uint32_t i = 0; i < count; ++i) {
    int32_t v = (int32_t)samples[i] - s.mean;
    uint32_t a = v < 0 ? (uint32_t)(-v) : (uint32_t)v;
    if (a > peak) peak = a;
    sq += (uint64_t)a * a;
  }

  s.peak = peak;
  s.rms = (uint32_t)sqrt((double)sq / count);
  return s;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== 1.14 PDM mic basic ===");
  Serial.println("MIC_CLK=D0, MIC_DATA=D1");

  if (!initMic()) {
    Serial.println("[MIC] init failed");
  }
}

void loop() {
  size_t bytesRead = 0;
  if (readMic(&bytesRead, 100)) {
    uint32_t samples = bytesRead / sizeof(int16_t);
    AudioStats s = calcStats(sampleBuf, samples);
    Serial.printf("[MIC] samples=%lu mean=%ld peak=%lu rms=%lu\n",
                  (unsigned long)samples,
                  (long)s.mean,
                  (unsigned long)s.peak,
                  (unsigned long)s.rms);
  } else {
    Serial.println("[MIC] read timeout");
  }

  delay(120);
}
