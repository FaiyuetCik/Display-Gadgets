/*
  XIAO nRF52840 Plus 1.47 Inch Display
  Big Volume Bar — minimal PDM mic level meter

  Hardware:
    - XIAO nRF52840 Plus + 1.47" Display (172x320)
    - PDM microphone (CLK=D0, DATA=D1)

  Required libraries:
    - Seeed_GFX / TFT_eSPI
    - Adafruit_TinyUSB
    - PDM
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

TFT_eSPI tft;

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;
static constexpr int CX    = LCD_W / 2;
static constexpr int CY    = LCD_H / 2;

// ========================= PDM Mic =========================

static constexpr int PDM_SAMPLE_RATE = 16000;
static constexpr int PDM_CHANNELS    = 1;
static constexpr int PDM_GAIN        = 30;

int16_t g_pdmBuf[256];
volatile int  g_rawPeak    = 0;
volatile bool g_hasMicData = false;

static void onPDMdata() {
  int n = PDM.available();
  if (n <= 0) return;
  if (n > (int)sizeof(g_pdmBuf)) n = sizeof(g_pdmBuf);
  int samples = PDM.read(g_pdmBuf, n) / 2;
  int peak = 0;
  for (int i = 0; i < samples; i++) {
    int v = abs((int)g_pdmBuf[i]);
    if (v > peak) peak = v;
  }
  g_rawPeak    = peak;
  g_hasMicData = true;
}

// ========================= Volume bar layout =========================

// Segmented bar layout
static constexpr int BAR_X      = 24;
static constexpr int BAR_W      = LCD_W - BAR_X * 2;   // 124
static constexpr int BAR_H      = 220;
static constexpr int BAR_Y_BOT  = LCD_H - 28;
static constexpr int BAR_Y_TOP  = BAR_Y_BOT - BAR_H;

static constexpr int SEGS       = 10;
static constexpr int SEG_GAP    = 2;
static constexpr int SEG_H      = (BAR_H - (SEGS - 1) * SEG_GAP) / SEGS;  // ~20px

// ========================= Volume =========================

static constexpr int   VOL_FLOOR = 40;
static constexpr int   VOL_CEIL  = 16000;
static constexpr float SMOOTH    = 0.20f;

float g_vol = 0.0f;
int   g_lastLit = -1;  // avoid redrawing unchanged segments

static float normalize(int raw) {
  if (raw <= VOL_FLOOR) return 0.0f;
  if (raw >= VOL_CEIL)  return 1.0f;
  return (float)(raw - VOL_FLOOR) / (float)(VOL_CEIL - VOL_FLOOR);
}

// ========================= Drawing =========================

// Color for segment index 0..9 (0=bottom, 9=top).
static uint16_t segColor(int idx) {
  if (idx < 5) return TFT_GREEN;       // 0-4: green
  if (idx < 9) return TFT_YELLOW;      // 5-8: yellow
  return TFT_RED;                       // 9: red
}

// Y-coordinate of the top edge of segment `idx` (0 = bottom).
static int segTop(int idx) {
  return BAR_Y_BOT - (idx + 1) * SEG_H - idx * SEG_GAP;
}

// Y-coordinate of the bottom edge of segment `idx`.
static int segBottom(int idx) {
  return segTop(idx) + SEG_H;
}

static void drawBarFrame() {
  tft.drawRect(BAR_X - 2, BAR_Y_TOP - 2, BAR_W + 4, BAR_H + 4, TFT_WHITE);
}

static void drawBar(float vol) {
  int lit = (int)(vol * SEGS + 0.5f);
  if (lit > SEGS) lit = SEGS;
  if (lit == g_lastLit) return;

  // Only redraw segments that changed state.
  for (int i = 0; i < SEGS; i++) {
    int y0 = segTop(i);
    int y1 = segBottom(i);
    if (i < lit) {
      tft.fillRect(BAR_X, y0, BAR_W, SEG_H, segColor(i));
    } else {
      tft.fillRect(BAR_X, y0, BAR_W, SEG_H, TFT_BLACK);
    }
  }

  g_lastLit = lit;
}

static int g_lastVolPct = -1;

static void drawLabel(float vol) {
  int pct = (int)(vol * 100.0f + 0.5f);
  if (pct == g_lastVolPct) return;
  g_lastVolPct = pct;

  tft.fillRect(0, BAR_Y_TOP - 40, LCD_W, 36, TFT_BLACK);

  char buf[16];
  snprintf(buf, sizeof(buf), "%d%%", pct);

  tft.setTextDatum(MC_DATUM);
  if (pct < 50)
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
  else if (pct < 90)
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  else
    tft.setTextColor(TFT_RED, TFT_BLACK);

  tft.drawString(buf, CX, BAR_Y_TOP - 20, 4);
}

static void drawTitle() {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("VOLUME", 10, 10, 2);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
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
  tft.fillScreen(TFT_BLACK);
}

// ========================= Setup =========================

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("=== Big Volume Bar ===");

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
    tft.drawString("MIC ERROR", CX, CY, 2);
    while (1) delay(1000);
  }

  Serial.println("[MIC] ready");
}

// ========================= Loop =========================

void loop() {
  int raw = 0;
  noInterrupts();
  if (g_hasMicData) {
    raw          = g_rawPeak;
    g_hasMicData = false;
  }
  interrupts();

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
