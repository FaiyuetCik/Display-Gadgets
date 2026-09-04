/*
  XIAO nRF52840 Plus 0.96 Inch Display graphic test.

  Uses Arduino_GFX with software SPI — hardware SPI on nRF52840
  is NOT compatible with this 0.96" ST7789 panel.

  Factory-verified parameters:
    ST7789, 80x160, rotation=2, IPS=true
    col_offset=24, row_offset=0, invert=true

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  Arduino_SWSPI + Arduino_ST7789 bus/panel setup, manual pin init and
  gfx->begin()/invertDisplay(true). Board_XIAO_0inch96_LCD<RST=38,BL=37> +
  Config_Seeed_0inch96_LCD_ST7789 bake 80x160 BGR rot2 invert=false (the
  ST7789 BGR bit now makes raw RGB565 display correctly, so the old R/B
  "looks blue/red" comments no longer apply). Colour constants and tests
  unchanged.
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

// ── Pins ─────────────────────────────────────────────────────────
static constexpr int8_t LCD_RST_PIN = 38;  // XIAO nRF52840 Plus (raw GPIO)
static constexpr int8_t LCD_BL_PIN  = 37;

// ── Display object ───────────────────────────────────────────────
Seeed_GFX display;

// ── Colour constants ─────────────────────────────────────────────
// The 0.96" panel is BGR; Config_Seeed_0inch96_LCD_ST7789 sets the ST7789
// BGR bit, so standard RGB565 values now display with correct colours.
static constexpr uint16_t C_BLACK   = 0x0000;
static constexpr uint16_t C_RED     = 0xF800;
static constexpr uint16_t C_GREEN   = 0x07E0;
static constexpr uint16_t C_BLUE    = 0x001F;
static constexpr uint16_t C_CYAN    = 0x07FF;
static constexpr uint16_t C_MAGENTA = 0xF81F;
static constexpr uint16_t C_YELLOW  = 0xFFE0;
static constexpr uint16_t C_WHITE   = 0xFFFF;
static constexpr uint16_t C_DGREY   = 0x39E7;

// ── Helpers ──────────────────────────────────────────────────────

static uint16_t colorWheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85)      return display.color565(255 - pos * 3, 0, pos * 3);
  if (pos < 170)     { pos -= 85; return display.color565(0, pos * 3, 255 - pos * 3); }
  pos -= 170;
  return display.color565(pos * 3, 255 - pos * 3, 0);
}

// Manual centring (kept 1:1 from the Arduino_GFX original)
static void drawCentre(const char *s, int16_t y, uint8_t size) {
  display.setTextSize(size);
  int16_t tw = (int16_t)strlen(s) * 6 * (int16_t)size;
  int16_t x = (display.width() - tw) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.print(s);
}

static void showTitle(const char *title) {
  display.fillScreen(C_BLACK);
  display.setTextColor(C_CYAN, C_BLACK);
  drawCentre(title, 52, 1);
  display.setTextColor(C_DGREY, C_BLACK);
  drawCentre("nRF52840 + 0.96", 76, 1);
  delay(650);
}

// ── Graphic tests ────────────────────────────────────────────────

static unsigned long testColorBars() {
  unsigned long start = micros();
  const uint16_t colors[] = {
    C_RED, C_GREEN, C_BLUE, C_CYAN, C_MAGENTA, C_YELLOW, C_WHITE, C_BLACK
  };
  int barH = display.height() / 8;
  for (int i = 0; i < 8; ++i)
    display.fillRect(0, i * barH, display.width(), barH, colors[i]);
  return micros() - start;
}

static unsigned long testLines(uint16_t color) {
  unsigned long start = micros();
  int w = display.width(), h = display.height();
  display.fillScreen(C_BLACK);
  for (int x = 0; x < w; x += 5) display.drawLine(0, 0, x, h - 1, color);
  for (int y = 0; y < h; y += 5) display.drawLine(0, 0, w - 1, y, color);
  for (int x = 0; x < w; x += 5) display.drawLine(w - 1, h - 1, x, 0, C_YELLOW);
  for (int y = 0; y < h; y += 5) display.drawLine(w - 1, h - 1, 0, y, C_YELLOW);
  return micros() - start;
}

static unsigned long testFastLines() {
  unsigned long start = micros();
  display.fillScreen(C_BLACK);
  for (int y = 0; y < display.height(); y += 5)
    display.drawFastHLine(0, y, display.width(), C_RED);
  for (int x = 0; x < display.width(); x += 5)
    display.drawFastVLine(x, 0, display.height(), C_BLUE);
  return micros() - start;
}

static unsigned long testRects() {
  unsigned long start = micros();
  display.fillScreen(C_BLACK);
  int cx = display.width() / 2, cy = display.height() / 2;
  int maxSize = min(display.width(), display.height());
  for (int sz = maxSize; sz > 6; sz -= 6)
    display.drawRect(cx - sz / 2, cy - sz / 2, sz, sz, colorWheel(sz * 2));
  return micros() - start;
}

static unsigned long testFilledRects() {
  unsigned long start = micros();
  display.fillScreen(C_BLACK);
  int cx = display.width() / 2, cy = display.height() / 2;
  int maxSize = min(display.width(), display.height());
  for (int sz = maxSize; sz > 6; sz -= 8) {
    display.fillRect(cx - sz / 2, cy - sz / 2, sz, sz, colorWheel(sz * 3));
    display.drawRect(cx - sz / 2, cy - sz / 2, sz, sz, C_WHITE);
  }
  return micros() - start;
}

static unsigned long testCircles() {
  unsigned long start = micros();
  display.fillScreen(C_BLACK);
  int maxR = min(display.width(), display.height()) / 2 - 4;
  for (int r = 4; r <= maxR; r += 4)
    display.drawCircle(display.width() / 2, display.height() / 2, r, colorWheel(r * 2));
  for (int y = 16; y < display.height(); y += 32)
    for (int x = 12; x < display.width(); x += 28)
      display.fillCircle(x, y, 6, colorWheel(x + y));
  return micros() - start;
}

static unsigned long testTriangles() {
  unsigned long start = micros();
  display.fillScreen(C_BLACK);
  int cx = display.width() / 2, cy = display.height() / 2;
  int maxTri = min(display.width(), display.height()) / 2 - 2;
  for (int i = 0; i <= maxTri; i += 6)
    display.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, colorWheel(i * 3));
  for (int i = maxTri; i > 0; i -= 8)
    display.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, colorWheel(200 - i));
  return micros() - start;
}

static unsigned long testRoundRects() {
  unsigned long start = micros();
  display.fillScreen(C_BLACK);
  for (int i = 0; i < 40; i += 5)
    display.drawRoundRect(2 + i / 2, 4 + i, display.width() - 4 - i,
                       display.height() - 8 - i * 2, 5, colorWheel(i * 6));
  return micros() - start;
}

static unsigned long testText() {
  unsigned long start = micros();
  display.fillScreen(C_BLACK);
  display.setTextColor(C_GREEN, C_BLACK);
  drawCentre("Graphic", 6, 1);
  display.setTextColor(C_CYAN, C_BLACK);
  drawCentre("80 x 160 TFT", 22, 1);
  display.setTextColor(C_YELLOW, C_BLACK);
  drawCentre("Seeed_GFX2", 36, 1);
  display.setTextColor(C_WHITE, C_BLACK);
  for (int i = 0; i < 5; ++i) {
    display.setCursor(6, 50 + i * 16);
    display.print("L"); display.print(i + 1);
    display.print(" 0x"); display.print(0x1000 + i * 257, HEX);
  }
  return micros() - start;
}

static unsigned long testGradient() {
  unsigned long start = micros();
  int w = display.width(), h = display.height();
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      display.drawPixel(x, y, display.color565(
          map(x, 0, w - 1, 0, 255),
          map(y, 0, h - 1, 0, 255),
          (x + y) & 0xFF));
  return micros() - start;
}

static void printResult(const char *name, unsigned long us) {
  Serial.print(name); Serial.print(": ");
  Serial.print(us / 1000.0f, 2);
  Serial.println(" ms");
}

static void showResultScreen() {
  display.fillScreen(C_BLACK);
  display.setTextColor(C_GREEN, C_BLACK);
  drawCentre("Done!", 44, 1);
  display.setTextColor(C_WHITE, C_BLACK);
  drawCentre("All tests OK", 64, 1);
  display.setTextColor(C_CYAN, C_BLACK);
  drawCentre("RST to rerun", 84, 1);
  display.drawRoundRect(4, 4, display.width() - 8, display.height() - 8, 4, C_BLUE);
}

// ── Main ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200); delay(800);
  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 0.96 graphic test ===");

  if (!display.begin<Board_XIAO_0inch96_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_Seeed_0inch96_LCD_ST7789>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(TFT_BLACK);
  display.setTextWrap(false);

  Serial.print("LCD: "); Serial.print(display.width());
  Serial.print("x"); Serial.println(display.height());

  showTitle("Graphic Test");

  unsigned long us;
  us = testColorBars();    printResult("Color bars",       us); delay(900);
  us = testLines(C_CYAN);  printResult("Lines",            us); delay(900);
  us = testFastLines();    printResult("Fast lines",       us); delay(900);
  us = testRects();        printResult("Rectangles",       us); delay(900);
  us = testFilledRects();  printResult("Filled rects",     us); delay(900);
  us = testCircles();      printResult("Circles",          us); delay(900);
  us = testTriangles();    printResult("Triangles",        us); delay(900);
  us = testRoundRects();   printResult("Round rects",      us); delay(900);
  us = testText();         printResult("Text",             us); delay(1100);
  us = testGradient();     printResult("Pixel gradient",   us); delay(1200);

  showResultScreen();
  Serial.println("Graphic test finished.");
}

void loop() { delay(1000); }
