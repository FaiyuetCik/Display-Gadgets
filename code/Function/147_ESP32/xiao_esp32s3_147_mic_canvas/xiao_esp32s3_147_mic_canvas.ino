/*
  XIAO ESP32-S3 Plus 1.47 Inch Display
  Big Volume Bar — PDM mic level meter

  Hardware:
    - XIAO ESP32-S3 Plus + 1.47" Display (172x320)
    - PDM microphone (CLK=D0, DATA=D1)

  Required libraries:
    - Seeed_GFX / TFT_eSPI
*/

#include <Arduino.h>
#include "driver.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <driver/i2s_pdm.h>

// ========================= Pins =========================

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

// PDM microphone pins
static constexpr uint8_t PDM_CLK_PIN  = D0;
static constexpr uint8_t PDM_DATA_PIN = D1;

// ========================= Display =========================

TFT_eSPI tft;

static constexpr int SCREEN_W = 172;
static constexpr int SCREEN_H = 320;
static constexpr int CX       = SCREEN_W / 2;
static constexpr int CY       = SCREEN_H / 2;

// ========================= Colours =========================

static constexpr uint16_t C_BLACK    = TFT_BLACK;
static constexpr uint16_t C_WHITE    = TFT_WHITE;
static constexpr uint16_t C_GREEN    = TFT_GREEN;
static constexpr uint16_t C_YELLOW   = TFT_YELLOW;
static constexpr uint16_t C_RED      = TFT_RED;
static constexpr uint16_t C_DARKGREY = TFT_DARKGREY;

// ========================= PDM Mic (I2S PDM mode, ESP-IDF v5) =========================

static constexpr int PDM_SAMPLE_RATE = 16000;
static constexpr int PDM_BUF_SAMPLES = 256;

// DMA buffer for I2S reads
static int16_t g_pdmBuf[PDM_BUF_SAMPLES];

// I2S channel handle (ESP-IDF v5 new driver)
static i2s_chan_handle_t g_i2sRxChan = nullptr;

// I2S PDM configuration and init (ESP-IDF v5 API)
static bool initMic() {
  // Release any previous channel first.
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
  pdmCfg.gpio_cfg.clk = (gpio_num_t)PDM_CLK_PIN;
  pdmCfg.gpio_cfg.din = (gpio_num_t)PDM_DATA_PIN;
  // no clock invert needed for this board
  pdmCfg.gpio_cfg.invert_flags.clk_inv = false;

  err = i2s_channel_init_pdm_rx_mode(g_i2sRxChan, &pdmCfg);
  if (err != ESP_OK) {
    Serial.printf("[MIC] i2s_channel_init_pdm_rx_mode failed: %d\n", (int)err);
    i2s_del_channel(g_i2sRxChan);
    g_i2sRxChan = nullptr;
    return false;
  }

  // Reduce PDM CLK drive strength to cut EMI / coupling noise.
  gpio_set_drive_capability((gpio_num_t)PDM_CLK_PIN, GPIO_DRIVE_CAP_0);

  err = i2s_channel_enable(g_i2sRxChan);
  if (err != ESP_OK) {
    Serial.printf("[MIC] i2s_channel_enable failed: %d\n", (int)err);
    i2s_channel_disable(g_i2sRxChan);
    i2s_del_channel(g_i2sRxChan);
    g_i2sRxChan = nullptr;
    return false;
  }

  Serial.println("[MIC] PDM RX ready (IDF v5)");
  return true;
}

// ========================= Volume bar layout =========================

// Segmented bar layout
static constexpr int BAR_X      = 24;
static constexpr int BAR_W      = SCREEN_W - BAR_X * 2;   // 124
static constexpr int BAR_H      = 220;
static constexpr int BAR_Y_BOT  = SCREEN_H - 28;
static constexpr int BAR_Y_TOP  = BAR_Y_BOT - BAR_H;

static constexpr int SEGS       = 10;
static constexpr int SEG_GAP    = 2;
static constexpr int SEG_H      = (BAR_H - (SEGS - 1) * SEG_GAP) / SEGS;  // ~20px

// ========================= Volume =========================

static constexpr int   VOL_FLOOR = 40;
static constexpr int   VOL_CEIL  = 16000;
static constexpr float SMOOTH    = 0.20f;

float g_vol    = 0.0f;
int   g_lastLit = -1;  // avoid redrawing unchanged segments

static float normalize(int raw) {
  if (raw <= VOL_FLOOR) return 0.0f;
  if (raw >= VOL_CEIL)  return 1.0f;
  return (float)(raw - VOL_FLOOR) / (float)(VOL_CEIL - VOL_FLOOR);
}

// ========================= Drawing =========================

// Color for segment index 0..9 (0=bottom, 9=top).
static uint16_t segColor(int idx) {
  if (idx < 5) return C_GREEN;       // 0-4: green
  if (idx < 9) return C_YELLOW;      // 5-8: yellow
  return C_RED;                       // 9: red
}

// Y-coordinate of the top edge of segment `idx` (0 = bottom).
static int segTop(int idx) {
  return BAR_Y_BOT - (idx + 1) * SEG_H - idx * SEG_GAP;
}

static void drawBarFrame() {
  tft.drawRect(BAR_X - 2, BAR_Y_TOP - 2, BAR_W + 4, BAR_H + 4, C_WHITE);
}

static void drawBar(float vol) {
  int lit = (int)(vol * SEGS + 0.5f);
  if (lit > SEGS) lit = SEGS;
  if (lit == g_lastLit) return;

  // Only redraw segments that changed state.
  for (int i = 0; i < SEGS; i++) {
    int y0 = segTop(i);
    if (i < lit) {
      tft.fillRect(BAR_X, y0, BAR_W, SEG_H, segColor(i));
    } else {
      tft.fillRect(BAR_X, y0, BAR_W, SEG_H, C_BLACK);
    }
  }

  g_lastLit = lit;
}

static int g_lastVolPct = -1;

static void drawLabel(float vol) {
  int pct = (int)(vol * 100.0f + 0.5f);
  if (pct == g_lastVolPct) return;
  g_lastVolPct = pct;

  tft.fillRect(0, BAR_Y_TOP - 40, SCREEN_W, 36, C_BLACK);

  char buf[16];
  snprintf(buf, sizeof(buf), "%d%%", pct);

  tft.setTextDatum(MC_DATUM);
  if (pct < 50)
    tft.setTextColor(C_GREEN, C_BLACK);
  else if (pct < 90)
    tft.setTextColor(C_YELLOW, C_BLACK);
  else
    tft.setTextColor(C_RED, C_BLACK);

  tft.drawString(buf, CX, BAR_Y_TOP - 20, 4);
}

static void drawTitle() {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.drawString("VOLUME", 10, 10, 2);
  tft.setTextColor(C_DARKGREY, C_BLACK);
  tft.drawString("Speak or blow into mic", 10, 32, 1);
}

// ========================= Display init =========================

static void applyFix() {
  tft.writecommand(0x36);
  tft.writedata(0x48);
  delay(10);
}

static void initDisplay() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_SCK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_SCK_PIN, LOW);
  digitalWrite(LCD_MOSI_PIN, LOW);

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);

  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH); delay(20);
  digitalWrite(LCD_RST_PIN, LOW);  delay(80);
  digitalWrite(LCD_RST_PIN, HIGH); delay(180);

  tft.init();
  tft.setRotation(0);
  applyFix();
  tft.invertDisplay(false);
  tft.fillScreen(C_BLACK);
}

// ========================= Setup =========================

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("=== Big Volume Bar (ESP32-S3) ===");

  initDisplay();
  drawTitle();
  drawBarFrame();

  if (!initMic()) {
    tft.fillScreen(C_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(C_RED, C_BLACK);
    tft.drawString("MIC ERROR", CX, CY, 2);
    while (1) delay(1000);
  }

  Serial.println("[MIC] ready — speak or blow into the mic");
}

// ========================= Loop =========================

void loop() {
  // Read PDM samples via I2S (ESP-IDF v5 API)
  size_t bytesRead = 0;
  esp_err_t err = i2s_channel_read(
    g_i2sRxChan,
    (void *)g_pdmBuf,
    sizeof(g_pdmBuf),
    &bytesRead,
    pdMS_TO_TICKS(20)
  );

  int raw = 0;
  if (err == ESP_OK && bytesRead > 0) {
    int samples = bytesRead / sizeof(int16_t);
    int peak = 0;
    for (int i = 0; i < samples; i++) {
      int v = abs((int)g_pdmBuf[i]);
      if (v > peak) peak = v;
    }
    raw = peak;
  }

  if (raw > 0) {
    float instant = normalize(raw);
    g_vol += SMOOTH * (instant - g_vol);
  } else {
    g_vol *= 0.94f;
  }

  drawBar(g_vol);
  drawLabel(g_vol);

  delay(20);
}
