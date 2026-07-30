/*
  XIAO nRF52840 Plus 0.96 Inch Display graphic test.

  Uses Arduino_GFX with software SPI — hardware SPI on nRF52840
  is NOT compatible with this 0.96" ST7789 panel.

  Factory-verified parameters:
    ST7789, 80x160, rotation=2, IPS=true
    col_offset=24, row_offset=0, invert=true
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>

// ── Pins ─────────────────────────────────────────────────────────
static constexpr uint8_t LCD_CS   = D2;
static constexpr uint8_t LCD_DC   = D3;
static constexpr uint8_t LCD_SCK  = D8;
static constexpr uint8_t LCD_MOSI = D10;
static constexpr uint8_t LCD_RST  = D17;
static constexpr uint8_t LCD_BL   = D18;

// ── Display object ───────────────────────────────────────────────
Arduino_GFX *gfx = nullptr;

// ── Colour constants ─────────────────────────────────────────────
// The 0.96" panel has R/B swap, so we alias visually-correct colours
static constexpr uint16_t C_BLACK   = 0x0000;
static constexpr uint16_t C_RED     = 0xF800;   // looks BLUE on screen
static constexpr uint16_t C_GREEN   = 0x07E0;
static constexpr uint16_t C_BLUE    = 0x001F;   // looks RED on screen
static constexpr uint16_t C_CYAN    = 0x07FF;
static constexpr uint16_t C_MAGENTA = 0xF81F;
static constexpr uint16_t C_YELLOW  = 0xFFE0;
static constexpr uint16_t C_WHITE   = 0xFFFF;
static constexpr uint16_t C_DGREY   = 0x39E7;

// ── Helpers ──────────────────────────────────────────────────────

static uint16_t colorWheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85)      return gfx->color565(255 - pos * 3, 0, pos * 3);
  if (pos < 170)     { pos -= 85; return gfx->color565(0, pos * 3, 255 - pos * 3); }
  pos -= 170;
  return gfx->color565(pos * 3, 255 - pos * 3, 0);
}

// Arduino_GFX doesn't have drawString with datum — manual centring
static void drawCentre(const char *s, int16_t y, uint8_t size) {
  gfx->setTextSize(size);
  int16_t tw = (int16_t)strlen(s) * 6 * (int16_t)size;
  int16_t x = (gfx->width() - tw) / 2;
  if (x < 0) x = 0;
  gfx->setCursor(x, y);
  gfx->print(s);
}

static void initDisplay() {
  pinMode(LCD_BL, OUTPUT); digitalWrite(LCD_BL, HIGH); analogWrite(LCD_BL, 255);
  pinMode(LCD_RST, OUTPUT);
  digitalWrite(LCD_RST, HIGH); delay(20);
  digitalWrite(LCD_RST, LOW);  delay(80);
  digitalWrite(LCD_RST, HIGH); delay(180);

  Arduino_DataBus *bus = new Arduino_SWSPI(
      LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
  gfx = new Arduino_ST7789(bus, LCD_RST, 2, true, 80, 160, 24, 0, 24, 0);

  gfx->begin();
  gfx->invertDisplay(true);
  gfx->fillScreen(C_BLACK);
  gfx->setTextWrap(false);
}

static void showTitle(const char *title) {
  gfx->fillScreen(C_BLACK);
  gfx->setTextColor(C_CYAN, C_BLACK);
  drawCentre(title, 52, 1);
  gfx->setTextColor(C_DGREY, C_BLACK);
  drawCentre("nRF52840 + 0.96", 76, 1);
  delay(650);
}

// ── Graphic tests ────────────────────────────────────────────────

static unsigned long testColorBars() {
  unsigned long start = micros();
  const uint16_t colors[] = {
    C_RED, C_GREEN, C_BLUE, C_CYAN, C_MAGENTA, C_YELLOW, C_WHITE, C_BLACK
  };
  int barH = gfx->height() / 8;
  for (int i = 0; i < 8; ++i)
    gfx->fillRect(0, i * barH, gfx->width(), barH, colors[i]);
  return micros() - start;
}

static unsigned long testLines(uint16_t color) {
  unsigned long start = micros();
  int w = gfx->width(), h = gfx->height();
  gfx->fillScreen(C_BLACK);
  for (int x = 0; x < w; x += 5) gfx->drawLine(0, 0, x, h - 1, color);
  for (int y = 0; y < h; y += 5) gfx->drawLine(0, 0, w - 1, y, color);
  for (int x = 0; x < w; x += 5) gfx->drawLine(w - 1, h - 1, x, 0, C_YELLOW);
  for (int y = 0; y < h; y += 5) gfx->drawLine(w - 1, h - 1, 0, y, C_YELLOW);
  return micros() - start;
}

static unsigned long testFastLines() {
  unsigned long start = micros();
  gfx->fillScreen(C_BLACK);
  for (int y = 0; y < gfx->height(); y += 5)
    gfx->drawFastHLine(0, y, gfx->width(), C_RED);
  for (int x = 0; x < gfx->width(); x += 5)
    gfx->drawFastVLine(x, 0, gfx->height(), C_BLUE);
  return micros() - start;
}

static unsigned long testRects() {
  unsigned long start = micros();
  gfx->fillScreen(C_BLACK);
  int cx = gfx->width() / 2, cy = gfx->height() / 2;
  int maxSize = min(gfx->width(), gfx->height());
  for (int sz = maxSize; sz > 6; sz -= 6)
    gfx->drawRect(cx - sz / 2, cy - sz / 2, sz, sz, colorWheel(sz * 2));
  return micros() - start;
}

static unsigned long testFilledRects() {
  unsigned long start = micros();
  gfx->fillScreen(C_BLACK);
  int cx = gfx->width() / 2, cy = gfx->height() / 2;
  int maxSize = min(gfx->width(), gfx->height());
  for (int sz = maxSize; sz > 6; sz -= 8) {
    gfx->fillRect(cx - sz / 2, cy - sz / 2, sz, sz, colorWheel(sz * 3));
    gfx->drawRect(cx - sz / 2, cy - sz / 2, sz, sz, C_WHITE);
  }
  return micros() - start;
}

static unsigned long testCircles() {
  unsigned long start = micros();
  gfx->fillScreen(C_BLACK);
  int maxR = min(gfx->width(), gfx->height()) / 2 - 4;
  for (int r = 4; r <= maxR; r += 4)
    gfx->drawCircle(gfx->width() / 2, gfx->height() / 2, r, colorWheel(r * 2));
  for (int y = 16; y < gfx->height(); y += 32)
    for (int x = 12; x < gfx->width(); x += 28)
      gfx->fillCircle(x, y, 6, colorWheel(x + y));
  return micros() - start;
}

static unsigned long testTriangles() {
  unsigned long start = micros();
  gfx->fillScreen(C_BLACK);
  int cx = gfx->width() / 2, cy = gfx->height() / 2;
  int maxTri = min(gfx->width(), gfx->height()) / 2 - 2;
  for (int i = 0; i <= maxTri; i += 6)
    gfx->drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, colorWheel(i * 3));
  for (int i = maxTri; i > 0; i -= 8)
    gfx->fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, colorWheel(200 - i));
  return micros() - start;
}

static unsigned long testRoundRects() {
  unsigned long start = micros();
  gfx->fillScreen(C_BLACK);
  for (int i = 0; i < 40; i += 5)
    gfx->drawRoundRect(2 + i / 2, 4 + i, gfx->width() - 4 - i,
                       gfx->height() - 8 - i * 2, 5, colorWheel(i * 6));
  return micros() - start;
}

static unsigned long testText() {
  unsigned long start = micros();
  gfx->fillScreen(C_BLACK);
  gfx->setTextColor(C_GREEN, C_BLACK);
  drawCentre("Graphic", 6, 1);
  gfx->setTextColor(C_CYAN, C_BLACK);
  drawCentre("80 x 160 TFT", 22, 1);
  gfx->setTextColor(C_YELLOW, C_BLACK);
  drawCentre("Arduino_GFX", 36, 1);
  gfx->setTextColor(C_WHITE, C_BLACK);
  for (int i = 0; i < 5; ++i) {
    gfx->setCursor(6, 50 + i * 16);
    gfx->print("L"); gfx->print(i + 1);
    gfx->print(" 0x"); gfx->print(0x1000 + i * 257, HEX);
  }
  return micros() - start;
}

static unsigned long testGradient() {
  unsigned long start = micros();
  int w = gfx->width(), h = gfx->height();
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      gfx->drawPixel(x, y, gfx->color565(
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
  gfx->fillScreen(C_BLACK);
  gfx->setTextColor(C_GREEN, C_BLACK);
  drawCentre("Done!", 44, 1);
  gfx->setTextColor(C_WHITE, C_BLACK);
  drawCentre("All tests OK", 64, 1);
  gfx->setTextColor(C_CYAN, C_BLACK);
  drawCentre("RST to rerun", 84, 1);
  gfx->drawRoundRect(4, 4, gfx->width() - 8, gfx->height() - 8, 4, C_BLUE);
}

// ── Main ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200); delay(800);
  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 0.96 graphic test ===");

  initDisplay();

  Serial.print("LCD: "); Serial.print(gfx->width());
  Serial.print("x"); Serial.println(gfx->height());

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
