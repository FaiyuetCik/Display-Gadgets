/*
  XIAO ESP32-S3 Plus 1.14 Inch Display
  Voice Bar - real-time PDM microphone visualization

  Hardware:
    - XIAO ESP32-S3 Plus + 1.14" Display (135x240 ST7789)
    - Onboard PDM microphone (CLK=D0, DATA=D1)

  Features:
    - Real-time equalizer-style waveform
    - 10-segment volume bar (green/yellow/red)
    - Volume percentage display

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  driver.h + manual pin setup + tft.init()/setRotation(0)/invertDisplay(true).
  DROP driver.h + initDisplay(). Config_Seeed_1inch14_LCD_ST7789 bakes
  135x240 RGB invert=true rot0. I2S PDM microphone (ESP-IDF v5) unchanged.

  Required libraries:
    - Seeed_GFX2
    - ESP32 Arduino core 3.x (uses the ESP-IDF v5 I2S PDM API)
*/

#include <Arduino.h>
#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR < 5
  #error "This demo uses the ESP32 Arduino core 3.x / ESP-IDF 5 I2S PDM API."
#endif

#include <driver/gpio.h>
#include <driver/i2s_pdm.h>

// ========================= Pins =========================

static constexpr int8_t LCD_RST_PIN = 13;  // XIAO ESP32-S3 Plus
static constexpr int8_t LCD_BL_PIN = 12;

static constexpr uint8_t MIC_CLK_PIN  = D0;
static constexpr uint8_t MIC_DATA_PIN = D1;

// ========================= Display =========================

Seeed_GFX display;

// This panel is physically wired for BGR color order. Keep this override
// local to the sketch; do not modify the Seeed_GFX2 library configuration.
struct Config_XIAO_1inch14_LCD_ST7789_BGR {
  using Driver = Driver_ST7789;
  using Panel = Panel_TFT;
  static constexpr uint16_t width = 135;
  static constexpr uint16_t height = 240;
  static constexpr uint8_t colorDepth = 16;
  static constexpr uint8_t rgbOrder = 0x08;  // BGR
  static constexpr bool invert = true;
};

static constexpr int LCD_W = 135;
static constexpr int LCD_H = 240;
static constexpr int CX    = LCD_W / 2;

// ========================= PDM Mic (I2S PDM RX, ESP-IDF v5) =========================

static constexpr int PDM_SAMPLE_RATE = 16000;
static constexpr int PDM_BUF_SAMPLES = 256;

int16_t g_pdmBuf[PDM_BUF_SAMPLES];
i2s_chan_handle_t g_i2sRxChan = nullptr;

// Waveform bar buffer - filled and consumed entirely inside loop().
static constexpr int WF_BARS = 27;            // 135 px / 5 px per bar
int32_t g_wfSamples[WF_BARS];

static bool initMic() {
  if (g_i2sRxChan) {
    i2s_channel_disable(g_i2sRxChan);
    i2s_del_channel(g_i2sRxChan);
    g_i2sRxChan = nullptr;
  }

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = 4;
  chanCfg.dma_frame_num = PDM_BUF_SAMPLES;

  esp_err_t err = i2s_new_channel(&chanCfg, nullptr, &g_i2sRxChan);
  if (err != ESP_OK) {
    Serial.printf("[MIC] i2s_new_channel failed: %d\n", (int)err);
    return false;
  }

  i2s_pdm_rx_config_t pdmCfg = {};
  pdmCfg.clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(PDM_SAMPLE_RATE);
  pdmCfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  pdmCfg.gpio_cfg.clk = (gpio_num_t)MIC_CLK_PIN;
  pdmCfg.gpio_cfg.din = (gpio_num_t)MIC_DATA_PIN;
  pdmCfg.gpio_cfg.invert_flags.clk_inv = false;

  err = i2s_channel_init_pdm_rx_mode(g_i2sRxChan, &pdmCfg);
  if (err != ESP_OK) {
    Serial.printf("[MIC] i2s_channel_init_pdm_rx_mode failed: %d\n", (int)err);
    i2s_del_channel(g_i2sRxChan);
    g_i2sRxChan = nullptr;
    return false;
  }

  // Reduce PDM CLK drive strength to cut EMI / coupling noise.
  gpio_set_drive_capability((gpio_num_t)MIC_CLK_PIN, GPIO_DRIVE_CAP_0);

  err = i2s_channel_enable(g_i2sRxChan);
  if (err != ESP_OK) {
    Serial.printf("[MIC] i2s_channel_enable failed: %d\n", (int)err);
    i2s_channel_disable(g_i2sRxChan);
    i2s_del_channel(g_i2sRxChan);
    g_i2sRxChan = nullptr;
    return false;
  }

  Serial.println("[MIC] PDM RX ready (ESP-IDF v5)");
  return true;
}

// ========================= Layout =========================

// Waveform area (equalizer bars)
static constexpr int WF_Y        = 30;
static constexpr int WF_H        = 65;
static constexpr int WF_BASELINE = WF_Y + WF_H / 2;
static constexpr int WF_AMP      = WF_H / 2 - 3;   // keep 3 px margin top/bottom

// Segmented volume bar
static constexpr int BAR_X     = 32;
static constexpr int BAR_W     = LCD_W - BAR_X * 2;   // 71
static constexpr int BAR_H     = 95;
static constexpr int BAR_Y_BOT = LCD_H - 14;
static constexpr int BAR_Y_TOP = BAR_Y_BOT - BAR_H;

static constexpr int SEGS    = 10;
static constexpr int SEG_GAP = 2;
static constexpr int SEG_H   = (BAR_H - (SEGS - 1) * SEG_GAP) / SEGS;  // 7 px

// ========================= Volume processing =========================

// Tuned for the ESP32-S3 16-bit PDM I2S samples: a comfortable speaking level
// maps to roughly 40..2400 raw peak (after removing the DC offset).
static constexpr int   VOL_FLOOR = 20;
static constexpr int   VOL_CEIL  = 2400;
static constexpr float SMOOTH    = 0.20f;

float g_vol     = 0.0f;
int   g_lastLit = -1;
int   g_lastPct = -1;

static float normalize(int raw) {
  if (raw <= VOL_FLOOR) return 0.0f;
  if (raw >= VOL_CEIL)  return 1.0f;
  return (float)(raw - VOL_FLOOR) / (float)(VOL_CEIL - VOL_FLOOR);
}

// ========================= Drawing helpers =========================

static uint16_t segColor(int idx) {
  if (idx < 5) return TFT_GREEN;       // 0-4
  if (idx < 9) return TFT_YELLOW;      // 5-8
  return TFT_RED;                       // 9
}

static int segTop(int idx) {
  return BAR_Y_BOT - (idx + 1) * SEG_H - idx * SEG_GAP;
}

// ========================= Draw waveform =========================

static void drawWaveform(const int32_t* samples, int count, float vol) {
  // Clear the waveform band
  display.fillRect(0, WF_Y, LCD_W, WF_H, TFT_BLACK);

  // Subtle center baseline
  display.drawFastHLine(0, WF_BASELINE, LCD_W, display.color565(45, 45, 45));

  // Color from smoothed volume (aligned with volume bar & label)
  uint16_t color;
  if (vol > 0.90f)       color = TFT_RED;
  else if (vol > 0.50f)  color = TFT_YELLOW;
  else if (vol > 0.10f)  color = TFT_GREEN;
  else                   color = display.color565(0, 100, 0);

  int barW  = LCD_W / count;
  int gap   = max(1, barW / 5);
  int drawW = barW - gap;

  for (int i = 0; i < count; i++) {
    int val = abs((int)samples[i]);
    int h   = (int)((int64_t)val * WF_AMP / VOL_CEIL);
    if (h < 1)  h = 1;
    if (h > WF_AMP) h = WF_AMP;

    int x = i * barW;
    display.fillRect(x, WF_BASELINE - h, drawW, h * 2, color);
  }
}

// ========================= Draw volume bar =========================

static void drawBarFrame() {
  display.drawRect(BAR_X - 2, BAR_Y_TOP - 2, BAR_W + 4, BAR_H + 4, TFT_WHITE);
}

static void drawBar(float vol) {
  int lit = (int)(vol * SEGS + 0.5f);
  if (lit > SEGS) lit = SEGS;
  if (lit == g_lastLit) return;

  for (int i = 0; i < SEGS; i++) {
    int y0 = segTop(i);
    if (i < lit)
      display.fillRect(BAR_X, y0, BAR_W, SEG_H, segColor(i));
    else
      display.fillRect(BAR_X, y0, BAR_W, SEG_H, TFT_BLACK);
  }
  g_lastLit = lit;
}

// ========================= Draw percentage label =========================

static void drawLabel(float vol) {
  int pct = (int)(vol * 100.0f + 0.5f);
  if (pct == g_lastPct) return;
  g_lastPct = pct;

  // Clear the band between waveform and volume bar
  int y0 = WF_Y + WF_H + 2;
  int h  = BAR_Y_TOP - y0 - 2;
  display.fillRect(0, y0, LCD_W, h, TFT_BLACK);

  char buf[16];
  snprintf(buf, sizeof(buf), "%d%%", pct);

  display.setTextDatum(MC_DATUM);
  if (pct < 50)
    display.setTextColor(TFT_GREEN, TFT_BLACK);
  else if (pct < 90)
    display.setTextColor(TFT_YELLOW, TFT_BLACK);
  else
    display.setTextColor(TFT_RED, TFT_BLACK);

  display.drawString(buf, CX, y0 + h / 2, 4);
}

// ========================= Draw title =========================

static void drawTitle() {
  display.setTextDatum(TL_DATUM);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString("VOICE BAR", 6, 4, 2);
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.drawString("Speak or blow into mic", 6, 22, 1);
}

// ========================= Setup =========================

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("=== Voice Bar | XIAO ESP32-S3 Plus 1.14 ===");

  if (!display.begin<Board_XIAO_1inch14_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_XIAO_1inch14_LCD_ST7789_BGR>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(TFT_BLACK);

  drawTitle();
  drawBarFrame();

  if (!initMic()) {
    display.fillScreen(TFT_BLACK);
    display.setTextDatum(MC_DATUM);
    display.setTextColor(TFT_RED, TFT_BLACK);
    display.drawString("MIC ERROR", CX, 120, 2);
    while (1) delay(1000);
  }

  Serial.println("[MIC] ready");
}

// ========================= Loop =========================

void loop() {
  size_t bytesRead = 0;
  esp_err_t err = i2s_channel_read(
    g_i2sRxChan,
    (void *)g_pdmBuf,
    sizeof(g_pdmBuf),
    &bytesRead,
    pdMS_TO_TICKS(20)
  );

  int  raw   = 0;
  bool hasWf = false;

  if (err == ESP_OK && bytesRead > 0) {
    int samples = bytesRead / sizeof(int16_t);

    // Remove DC offset so the peak reflects actual loudness.
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) sum += g_pdmBuf[i];
    int mean = (int)(sum / samples);

    int peak = 0;
    for (int i = 0; i < samples; i++) {
      int v = g_pdmBuf[i] - mean;
      int a = v < 0 ? -v : v;
      if (a > peak) peak = a;
    }
    raw = peak;

    // Down-sample for the waveform visualizer.
    int step = max(1, samples / WF_BARS);
    int wi = 0;
    for (int i = 0; i < samples && wi < WF_BARS; i += step) {
      g_wfSamples[wi++] = g_pdmBuf[i] - mean;
    }
    hasWf = true;
  }

  // Smooth volume
  if (raw > 0) {
    float instant = normalize(raw);
    g_vol += SMOOTH * (instant - g_vol);
  } else {
    g_vol *= 0.94f;   // decay when silent
  }

  if (hasWf) {
    drawWaveform(g_wfSamples, WF_BARS, g_vol);
  }
  drawBar(g_vol);
  drawLabel(g_vol);

  delay(15);
}
