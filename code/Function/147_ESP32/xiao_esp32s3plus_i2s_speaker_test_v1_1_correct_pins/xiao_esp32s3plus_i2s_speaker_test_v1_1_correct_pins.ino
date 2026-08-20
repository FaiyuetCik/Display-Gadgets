/*
  XIAO ESP32-S3 Plus - I2S Speaker Test Firmware v1.1

  Target:
    XIAO ESP32-S3 Plus + I2S amplifier / speaker module
    Example amplifier: MAX98357A / NS4168 / other I2S DAC amp modules

  Default wiring based on XIAO ESP32-S3 Plus pinout:
    XIAO ESP32-S3 Plus  ->  I2S Amp
    D12 / GPIO39        ->  BCLK / SCK
    D13 / GPIO40        ->  LRCLK / WS
    D11 / GPIO38        ->  DIN / SDIN
    3V3 or 5V           ->  VIN, according to your amplifier module
    GND                 ->  GND

  If your board uses different pins, change only:
    I2S_BCLK_PIN
    I2S_LRCLK_PIN
    I2S_DOUT_PIN

  Serial commands:
    h : help
    t : 1 kHz tone
    s : frequency sweep
    l : left channel only
    r : right channel only
    b : both channels
    m : mute
*/

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
#error "This firmware is for XIAO ESP32-S3 Plus / ESP32-S3 Arduino core."
#endif

#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR >= 5
  #include <driver/i2s_std.h>
#else
  #include <driver/i2s.h>
#endif

#include <math.h>

#ifndef I2S_BCLK_PIN
#define I2S_BCLK_PIN  D12  // GPIO39 / I2S_SCK
#endif

#ifndef I2S_LRCLK_PIN
#define I2S_LRCLK_PIN D13  // GPIO40 / I2S_WS
#endif

#ifndef I2S_DOUT_PIN
#define I2S_DOUT_PIN  D11  // GPIO38 / I2S_SD
#endif

#ifndef AMP_EN_PIN
#define AMP_EN_PIN -1
#endif

static constexpr uint32_t SAMPLE_RATE_HZ = 16000;
static constexpr uint16_t FRAMES_PER_BLOCK = 256;
static constexpr float DEFAULT_GAIN = 0.28f;

static int16_t g_audio[FRAMES_PER_BLOCK * 2];

enum ChannelMode {
  CH_BOTH = 0,
  CH_LEFT_ONLY,
  CH_RIGHT_ONLY,
  CH_MUTE
};

static ChannelMode g_channelMode = CH_BOTH;
static float g_phase = 0.0f;
static float g_freqHz = 1000.0f;

#if ESP_IDF_VERSION_MAJOR >= 5
static i2s_chan_handle_t g_txChan = nullptr;
#endif

static void ampEnable(bool en) {
  if (AMP_EN_PIN < 0) return;
  pinMode(AMP_EN_PIN, OUTPUT);
  digitalWrite(AMP_EN_PIN, en ? HIGH : LOW);
}

static bool initI2s() {
  ampEnable(true);

#if ESP_IDF_VERSION_MAJOR >= 5
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 6;
  chan_cfg.dma_frame_num = FRAMES_PER_BLOCK;

  esp_err_t err = i2s_new_channel(&chan_cfg, &g_txChan, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[I2S] i2s_new_channel failed: %d\n", (int)err);
    return false;
  }

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK_PIN,
      .ws = (gpio_num_t)I2S_LRCLK_PIN,
      .dout = (gpio_num_t)I2S_DOUT_PIN,
      .din = I2S_GPIO_UNUSED,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  err = i2s_channel_init_std_mode(g_txChan, &std_cfg);
  if (err != ESP_OK) {
    Serial.printf("[I2S] i2s_channel_init_std_mode failed: %d\n", (int)err);
    return false;
  }

  err = i2s_channel_enable(g_txChan);
  if (err != ESP_OK) {
    Serial.printf("[I2S] i2s_channel_enable failed: %d\n", (int)err);
    return false;
  }
#else
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = (int)SAMPLE_RATE_HZ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 6,
    .dma_buf_len = FRAMES_PER_BLOCK,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRCLK_PIN,
    .data_out_num = I2S_DOUT_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[I2S] i2s_driver_install failed: %d\n", (int)err);
    return false;
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[I2S] i2s_set_pin failed: %d\n", (int)err);
    return false;
  }

  i2s_zero_dma_buffer(I2S_NUM_0);
#endif

  Serial.println("[I2S] init OK");
  Serial.printf("[PIN] BCLK=%d LRCLK=%d DOUT=%d AMP_EN=%d\n", (int)I2S_BCLK_PIN, (int)I2S_LRCLK_PIN, (int)I2S_DOUT_PIN, (int)AMP_EN_PIN);
  Serial.printf("[AUDIO] sampleRate=%luHz block=%u frames\n", (unsigned long)SAMPLE_RATE_HZ, FRAMES_PER_BLOCK);
  return true;
}

static void fillToneBlock(float freqHz, float gain) {
  const float twoPi = 6.28318530718f;
  const float step = twoPi * freqHz / (float)SAMPLE_RATE_HZ;

  for (uint16_t i = 0; i < FRAMES_PER_BLOCK; i++) {
    float s = sinf(g_phase);
    int16_t v = (int16_t)(s * 32767.0f * gain);

    g_phase += step;
    if (g_phase >= twoPi) g_phase -= twoPi;

    int16_t left = 0;
    int16_t right = 0;

    switch (g_channelMode) {
      case CH_BOTH:       left = v; right = v; break;
      case CH_LEFT_ONLY:  left = v; right = 0; break;
      case CH_RIGHT_ONLY: left = 0; right = v; break;
      case CH_MUTE:
      default:            left = 0; right = 0; break;
    }

    g_audio[i * 2 + 0] = left;
    g_audio[i * 2 + 1] = right;
  }
}

static void writeAudioBlock() {
  size_t bytesWritten = 0;
  const size_t bytesToWrite = sizeof(g_audio);

#if ESP_IDF_VERSION_MAJOR >= 5
  esp_err_t err = i2s_channel_write(g_txChan, g_audio, bytesToWrite, &bytesWritten, portMAX_DELAY);
#else
  esp_err_t err = i2s_write(I2S_NUM_0, g_audio, bytesToWrite, &bytesWritten, portMAX_DELAY);
#endif

  if (err != ESP_OK || bytesWritten != bytesToWrite) {
    Serial.printf("[I2S] write err=%d bytes=%u/%u\n", (int)err, (unsigned)bytesWritten, (unsigned)bytesToWrite);
  }
}

static void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h : help");
  Serial.println("  t : 1 kHz tone");
  Serial.println("  s : sweep 200 Hz -> 4 kHz");
  Serial.println("  l : left channel only");
  Serial.println("  r : right channel only");
  Serial.println("  b : both channels");
  Serial.println("  m : mute");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus I2S Speaker Test v1.1 ===");

  if (!initI2s()) {
    Serial.println("[BOOT] I2S init failed");
    while (1) delay(1000);
  }

  printHelp();
  Serial.println("[BOOT] playing 1 kHz tone");
}

void loop() {
  static bool sweepMode = false;
  static uint32_t lastSweepMs = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 'h' || c == 'H') {
      printHelp();
    } else if (c == 't' || c == 'T') {
      sweepMode = false;
      g_freqHz = 1000.0f;
      g_channelMode = CH_BOTH;
      Serial.println("[CMD] 1 kHz tone, both channels");
    } else if (c == 's' || c == 'S') {
      sweepMode = true;
      g_freqHz = 200.0f;
      g_channelMode = CH_BOTH;
      lastSweepMs = millis();
      Serial.println("[CMD] sweep mode");
    } else if (c == 'l' || c == 'L') {
      g_channelMode = CH_LEFT_ONLY;
      Serial.println("[CMD] left only");
    } else if (c == 'r' || c == 'R') {
      g_channelMode = CH_RIGHT_ONLY;
      Serial.println("[CMD] right only");
    } else if (c == 'b' || c == 'B') {
      g_channelMode = CH_BOTH;
      Serial.println("[CMD] both channels");
    } else if (c == 'm' || c == 'M') {
      g_channelMode = CH_MUTE;
      Serial.println("[CMD] mute");
    }
  }

  if (sweepMode && millis() - lastSweepMs >= 120) {
    lastSweepMs = millis();
    g_freqHz *= 1.08f;
    if (g_freqHz > 4000.0f) g_freqHz = 200.0f;
    Serial.print("[SWEEP] freq=");
    Serial.println(g_freqHz, 1);
  }

  fillToneBlock(g_freqHz, DEFAULT_GAIN);
  writeAudioBlock();
}
