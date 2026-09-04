/*
  XIAO ESP32-S3 Plus 0.96 Inch Display graphic test.

  This sketch uses the verified Arduino_GFX setup from the 0.96-inch
  factory dashboard, then runs a compact graphics benchmark:
    - color bars
    - lines
    - rectangles
    - circles
    - triangles
    - rounded rectangles
    - text
    - pixel gradient

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  Arduino_GFX bus + panel construction, manual pins, and
  tft.begin()/setRotation()/invertDisplay(). Config_Seeed_0inch96_LCD_ST7789
  bakes 80x160 BGR rot2. Drawing API carries over 1:1 (tft -> display);
  getTextBounds() (not in Seeed_GFX2) replaced by textWidth()/fontHeight().

  Required libraries:
    - Seeed_GFX2
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>

Seeed_GFX display;

static constexpr int8_t LCD_RST_PIN = 13;  // XIAO ESP32-S3 Plus
static constexpr int8_t LCD_BL_PIN  = 12;

// ── Helpers ────────────────────────────────────────────────────────

static uint16_t colorWheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85)      return display.color565(255 - pos * 3, 0, pos * 3);
  if (pos < 170)     { pos -= 85; return display.color565(0, pos * 3, 255 - pos * 3); }
  pos -= 170;
  return display.color565(pos * 3, 255 - pos * 3, 0);
}

static void drawCenteredText(const char *text, int16_t centerX, int16_t centerY, uint8_t size) {
  display.setTextSize(size);
  int16_t w = display.textWidth(text);
  int16_t h = display.fontHeight();
  display.setCursor(centerX - w / 2, centerY - h / 2);
  display.print(text);
}

static void showTitle(const char *title) {
  display.fillScreen(TFT_BLACK);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  // The 0.96-inch panel is only 80 pixels wide; size 2 makes
  // "Graphic Test" wrap and clip. Keep the title on one line.
  drawCenteredText(title, display.width() / 2, 60, 1);
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  drawCenteredText("ESP32-S3 + 0.96", display.width() / 2, 84, 1);
  delay(650);
}

// ── Graphic tests ──────────────────────────────────────────────────

static unsigned long testColorBars() {
  unsigned long start = micros();
  const uint16_t colors[] = {
    TFT_RED, TFT_GREEN, TFT_BLUE, TFT_CYAN,
    TFT_MAGENTA, TFT_YELLOW, TFT_WHITE, TFT_BLACK
  };
  int barH = display.height() / 8;
  for (int i = 0; i < 8; ++i)
    display.fillRect(0, i * barH, display.width(), barH, colors[i]);
  return micros() - start;
}

static unsigned long testLines(uint16_t color) {
  unsigned long start = micros();
  int w = display.width(), h = display.height();
  display.fillScreen(TFT_BLACK);
  for (int x = 0; x < w; x += 5) display.drawLine(0, 0, x, h - 1, color);
  for (int y = 0; y < h; y += 5) display.drawLine(0, 0, w - 1, y, color);
  for (int x = 0; x < w; x += 5) display.drawLine(w - 1, h - 1, x, 0, TFT_YELLOW);
  for (int y = 0; y < h; y += 5) display.drawLine(w - 1, h - 1, 0, y, TFT_YELLOW);
  return micros() - start;
}

static unsigned long testFastLines() {
  unsigned long start = micros();
  display.fillScreen(TFT_BLACK);
  for (int y = 0; y < display.height(); y += 5)
    display.drawFastHLine(0, y, display.width(), TFT_RED);
  for (int x = 0; x < display.width(); x += 5)
    display.drawFastVLine(x, 0, display.height(), TFT_BLUE);
  return micros() - start;
}

static unsigned long testRects() {
  unsigned long start = micros();
  display.fillScreen(TFT_BLACK);
  int cx = display.width() / 2, cy = display.height() / 2;
  int maxSize = min(display.width(), display.height());
  for (int sz = maxSize; sz > 6; sz -= 6)
    display.drawRect(cx - sz / 2, cy - sz / 2, sz, sz, colorWheel(sz * 2));
  return micros() - start;
}

static unsigned long testFilledRects() {
  unsigned long start = micros();
  display.fillScreen(TFT_BLACK);
  int cx = display.width() / 2, cy = display.height() / 2;
  int maxSize = min(display.width(), display.height());
  for (int sz = maxSize; sz > 6; sz -= 8) {
    display.fillRect(cx - sz / 2, cy - sz / 2, sz, sz, colorWheel(sz * 3));
    display.drawRect(cx - sz / 2, cy - sz / 2, sz, sz, TFT_WHITE);
  }
  return micros() - start;
}

static unsigned long testCircles() {
  unsigned long start = micros();
  display.fillScreen(TFT_BLACK);
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
  display.fillScreen(TFT_BLACK);
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
  display.fillScreen(TFT_BLACK);
  for (int i = 0; i < 40; i += 5)
    display.drawRoundRect(2 + i / 2, 4 + i, display.width() - 4 - i,
                      display.height() - 8 - i * 2, 5, colorWheel(i * 6));
  return micros() - start;
}

static unsigned long testText() {
  unsigned long start = micros();
  display.fillScreen(TFT_BLACK);

  display.setTextColor(TFT_GREEN, TFT_BLACK);
  // Keep the heading within the 80-pixel-wide display.
  drawCenteredText("Graphic Test", display.width() / 2, 10, 1);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.setTextSize(1);
  display.setCursor(6, 28);
  display.print("80 x 160 TFT");
  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  display.setCursor(6, 42);
  display.print("Seeed_GFX2");
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  for (int i = 0; i < 5; ++i) {
    display.setCursor(6, 58 + i * 16);
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

// ── Output ─────────────────────────────────────────────────────────

static void printResult(const char *name, unsigned long us) {
  Serial.print(name);
  Serial.print(": ");
  Serial.print(us / 1000.0f, 2);
  Serial.println(" ms");
}

static void showResultScreen() {
  display.fillScreen(TFT_BLACK);
  display.setTextColor(TFT_GREEN, TFT_BLACK);
  drawCenteredText("Done!", display.width() / 2, 54, 2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  drawCenteredText("All tests OK", display.width() / 2, 76, 1);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  drawCenteredText("RST to rerun", display.width() / 2, 96, 1);
  display.drawRoundRect(4, 4, display.width() - 8, display.height() - 8, 4, TFT_BLUE);
}

// ── Main ───────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus 0.96 graphic test ===");

  if (!display.begin<Board_XIAO_0inch96_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_Seeed_0inch96_LCD_ST7789>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(TFT_BLACK);

  Serial.print("LCD width: ");  Serial.println(display.width());
  Serial.print("LCD height: "); Serial.println(display.height());

  showTitle("Graphic Test");

  unsigned long us;

  us = testColorBars();    printResult("Color bars",       us); delay(900);
  us = testLines(TFT_CYAN); printResult("Lines",           us); delay(900);
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

void loop() {
  delay(1000);
}
