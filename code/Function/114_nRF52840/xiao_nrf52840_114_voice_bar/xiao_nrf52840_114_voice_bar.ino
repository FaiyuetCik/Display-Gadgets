/*
  XIAO nRF52840 Plus 1.14 Inch Display
  Voice Bar — real-time PDM microphone visualization

  Hardware:
    - XIAO nRF52840 Plus + 1.14" Display (135x240 ST7789)
    - PDM microphone (CLK=D0, DATA=D1)

  Features:
    - Real-time equalizer-style waveform
    - 10-segment volume bar (green/yellow/red)
    - Volume percentage display

  Required libraries:
    - Seeed_GFX / TFT_eSPI
    - Adafruit_TinyUSB
    - PDM (bundled with Seeed nRF52 Boards)
*/

#include "driver.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <PDM.h>

// ========================= Pins =========================

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

// ========================= Display =========================

TFT_eSPI tft(135, 240);

static constexpr int LCD_W = 135;
static constexpr int LCD_H = 240;
static constexpr int CX    = LCD_W / 2;

// ========================= PDM Mic =========================

static constexpr int PDM_SAMPLE_RATE = 16000;
static constexpr int PDM_CHANNELS    = 1;
static constexpr int PDM_GAIN        = 30;

int16_t g_pdmBuf[256];
volatile int  g_rawPeak    = 0;
volatile bool g_hasMicData = false;

// Waveform bar buffer — filled by ISR, consumed by loop
static constexpr int WF_BARS = 27;            // 135 px / 5 px per bar
volatile int16_t g_wfSamples[WF_BARS];
volatile bool    g_wfReady = false;

static void onPDMdata() {
  int n = PDM.available();
  if (n <= 0) return;
  if (n > (int)sizeof(g_pdmBuf)) n = sizeof(g_pdmBuf);
  int samples = PDM.read(g_pdmBuf, n) / 2;

  // Compute peak amplitude for the volume bar
  int peak = 0;
  for (int i = 0; i < samples; i++) {
    int v = abs((int)g_pdmBuf[i]);
    if (v > peak) peak = v;
  }
  g_rawPeak    = peak;
  g_hasMicData = true;

  // Down-sample for the waveform visualizer
  int step = max(1, samples / WF_BARS);
  int wi = 0;
  for (int i = 0; i < samples && wi < WF_BARS; i += step) {
    g_wfSamples[wi++] = g_pdmBuf[i];
  }
  g_wfReady = true;
}

// ========================= Layout =========================

// Waveform area (equalizer bars)
static constexpr int WF_Y        = 30;
static constexpr int WF_H        = 65;
static constexpr int WF_BASELINE = WF_Y + WF_H / 2;
static constexpr int WF_AMP      = WF_H / 2 - 3;   // keep 3 px margin top/bottom

// Segmented volume bar
static constexpr int BAR_X      = 32;
static constexpr int BAR_W      = LCD_W - BAR_X * 2;   // 71
static constexpr int BAR_H      = 95;
static constexpr int BAR_Y_BOT  = LCD_H - 14;
static constexpr int BAR_Y_TOP  = BAR_Y_BOT - BAR_H;

static constexpr int SEGS    = 10;
static constexpr int SEG_GAP = 2;
static constexpr int SEG_H   = (BAR_H - (SEGS - 1) * SEG_GAP) / SEGS;  // ~7 px

// ========================= Volume processing =========================

static constexpr int   VOL_FLOOR = 10;
static constexpr int   VOL_CEIL  = 1500;   // very sensitive
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

static void drawWaveform(const int16_t* samples, int count) {
  // Clear the waveform band
  tft.fillRect(0, WF_Y, LCD_W, WF_H, TFT_BLACK);

  // Subtle center baseline
  tft.drawFastHLine(0, WF_BASELINE, LCD_W, tft.color565(45, 45, 45));

  int barW  = LCD_W / count;
  int gap   = max(1, barW / 5);
  int drawW = barW - gap;

  for (int i = 0; i < count; i++) {
    int val = abs((int)samples[i]);
    int h   = (int)((int64_t)val * WF_AMP / 32768);
    if (h < 1)  h = 1;
    if (h > WF_AMP) h = WF_AMP;

    int x = i * barW;

    // Colour by amplitude
    uint16_t color;
    if (val > 24000)       color = TFT_RED;
    else if (val > 14000)  color = TFT_YELLOW;
    else if (val > 6000)   color = TFT_GREEN;
    else                   color = tft.color565(0, 100, 0);

    // Symmetric bar around baseline
    tft.fillRect(x, WF_BASELINE - h, drawW, h * 2, color);
  }
}

// ========================= Draw volume bar =========================

static void drawBarFrame() {
  tft.drawRect(BAR_X - 2, BAR_Y_TOP - 2, BAR_W + 4, BAR_H + 4, TFT_WHITE);
}

static void drawBar(float vol) {
  int lit = (int)(vol * SEGS + 0.5f);
  if (lit > SEGS) lit = SEGS;
  if (lit == g_lastLit) return;

  for (int i = 0; i < SEGS; i++) {
    int y0 = segTop(i);
    if (i < lit)
      tft.fillRect(BAR_X, y0, BAR_W, SEG_H, segColor(i));
    else
      tft.fillRect(BAR_X, y0, BAR_W, SEG_H, TFT_BLACK);
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
  tft.fillRect(0, y0, LCD_W, h, TFT_BLACK);

  char buf[16];
  snprintf(buf, sizeof(buf), "%d%%", pct);

  tft.setTextDatum(MC_DATUM);
  if (pct < 50)
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
  else if (pct < 90)
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  else
    tft.setTextColor(TFT_RED, TFT_BLACK);

  tft.drawString(buf, CX, y0 + h / 2, 4);
}

// ========================= Draw title =========================

static void drawTitle() {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("VOICE BAR", 6, 4, 2);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("Speak or blow into mic", 6, 22, 1);
}

// ========================= Display init =========================

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
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
}

// ========================= Setup =========================

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("=== Voice Bar | XIAO nRF52840 Plus 1.14 ===");

  initDisplay();
  drawTitle();
  drawBarFrame();

  PDM.setPins(D1, D0, -1);
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(sizeof(g_pdmBuf));
  PDM.setGain(PDM_GAIN);

  if (!PDM.begin(PDM_CHANNELS, PDM_SAMPLE_RATE)) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("MIC ERROR", CX, 120, 2);
    while (1) delay(1000);
  }

  Serial.println("[MIC] ready");
}

// ========================= Loop =========================

void loop() {
  // --- Thread-safe copy of ISR data ---
  int     raw = 0;
  int16_t wfCopy[WF_BARS];
  bool    hasWf = false;

  noInterrupts();
  if (g_hasMicData) {
    raw          = g_rawPeak;
    g_hasMicData = false;
  }
  if (g_wfReady) {
    memcpy((void*)wfCopy, (const void*)g_wfSamples, sizeof(wfCopy));
    g_wfReady = false;
    hasWf = true;
  }
  interrupts();

  // --- Smooth volume ---
  float instant = 0.0f;
  if (raw > 0) {
    instant = normalize(raw);
    g_vol += SMOOTH * (instant - g_vol);
  } else {
    g_vol *= 0.94f;   // decay when silent
  }

  // --- Draw ---
  if (hasWf) {
    drawWaveform(wfCopy, WF_BARS);
  }
  drawBar(g_vol);
  drawLabel(g_vol);

  delay(25);
}
