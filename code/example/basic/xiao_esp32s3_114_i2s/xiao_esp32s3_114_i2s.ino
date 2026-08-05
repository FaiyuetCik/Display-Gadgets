/*
  XIAO ESP32-S3 Plus + 1.14 Inch Display I2S output basic.

  Wiring for MAX98357A:
    D11 -> DIN
    D12 -> BCLK
    D13 -> LRC / WS
    3V3 -> VIN
    GND -> GND

  Outputs a low-volume continuous test tone.
*/

#include <Arduino.h>
#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR >= 5
  #include <driver/i2s_std.h>
#else
  #include <driver/i2s.h>
#endif

static constexpr uint8_t I2S_DOUT_PIN = D11;
static constexpr uint8_t I2S_BCLK_PIN = D12;
static constexpr uint8_t I2S_LRCLK_PIN = D13;

static constexpr uint32_t SAMPLE_RATE_HZ = 16000;
static constexpr size_t SAMPLE_FRAMES = 256;
static constexpr int16_t TONE_AMPLITUDE = 900;

int16_t txBuffer[SAMPLE_FRAMES * 2];
bool i2sReady = false;

#if ESP_IDF_VERSION_MAJOR >= 5
static i2s_chan_handle_t txChan = nullptr;
#else
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
#endif

static void fillToneBuffer() {
  for (size_t frame = 0; frame < SAMPLE_FRAMES; ++frame) {
    int16_t sample = ((frame / 18) & 1) ? TONE_AMPLITUDE : -TONE_AMPLITUDE;
    txBuffer[frame * 2 + 0] = sample;
    txBuffer[frame * 2 + 1] = sample;
  }
}

static bool initI2S() {
#if ESP_IDF_VERSION_MAJOR >= 5
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 4;
  chanCfg.dma_frame_num = SAMPLE_FRAMES;

  esp_err_t err = i2s_new_channel(&chanCfg, &txChan, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[I2S] i2s_new_channel failed: %d\n", (int)err);
    return false;
  }

  i2s_std_config_t stdCfg = {};
  stdCfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ);
  stdCfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  stdCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  stdCfg.gpio_cfg.bclk = (gpio_num_t)I2S_BCLK_PIN;
  stdCfg.gpio_cfg.ws = (gpio_num_t)I2S_LRCLK_PIN;
  stdCfg.gpio_cfg.dout = (gpio_num_t)I2S_DOUT_PIN;
  stdCfg.gpio_cfg.din = I2S_GPIO_UNUSED;

  err = i2s_channel_init_std_mode(txChan, &stdCfg);
  if (err != ESP_OK) {
    Serial.printf("[I2S] init_std failed: %d\n", (int)err);
    return false;
  }

  err = i2s_channel_enable(txChan);
  if (err != ESP_OK) {
    Serial.printf("[I2S] enable failed: %d\n", (int)err);
    return false;
  }
#else
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE_HZ;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = SAMPLE_FRAMES;

  esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[I2S] i2s_driver_install failed: %d\n", (int)err);
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_BCLK_PIN;
  pins.ws_io_num = I2S_LRCLK_PIN;
  pins.data_out_num = I2S_DOUT_PIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  err = i2s_set_pin(I2S_PORT, &pins);
  if (err != ESP_OK) {
    Serial.printf("[I2S] i2s_set_pin failed: %d\n", (int)err);
    return false;
  }
#endif

  Serial.println("[I2S] OK");
  return true;
}

static void writeTone() {
  size_t bytesWritten = 0;
#if ESP_IDF_VERSION_MAJOR >= 5
  i2s_channel_write(txChan, txBuffer, sizeof(txBuffer), &bytesWritten, portMAX_DELAY);
#else
  i2s_write(I2S_PORT, txBuffer, sizeof(txBuffer), &bytesWritten, portMAX_DELAY);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== 1.14 I2S output basic ===");
  Serial.println("D11=DIN, D12=BCLK, D13=LRC/WS");

  fillToneBuffer();
  i2sReady = initI2S();
}

void loop() {
  if (!i2sReady) {
    delay(1000);
    return;
  }

  writeTone();
}
