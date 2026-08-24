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

  Required libraries:
    - GFX Library for Arduino (Arduino_GFX)
*/

#include <Arduino.h>
#include <Arduino_GFX_Library.h>

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

static constexpr int  LCD_W = 80;
static constexpr int  LCD_H = 160;
static constexpr int  LCD_ROTATION = 2;
static constexpr bool LCD_IPS = true;
static constexpr int  LCD_COL_OFFSET_1 = 24;
static constexpr int  LCD_ROW_OFFSET_1 = 0;
static constexpr int  LCD_COL_OFFSET_2 = 24;
static constexpr int  LCD_ROW_OFFSET_2 = 0;

Arduino_DataBus *lcdBus = new Arduino_ESP32SPI(
  LCD_DC_PIN, LCD_CS_PIN, LCD_SCK_PIN, LCD_MOSI_PIN
);
Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus, LCD_RST_PIN, LCD_ROTATION, LCD_IPS, LCD_W, LCD_H,
  LCD_COL_OFFSET_1, LCD_ROW_OFFSET_1, LCD_COL_OFFSET_2, LCD_ROW_OFFSET_2
);
Arduino_GFX &tft = *gfx;

static constexpr uint16_t TFT_BLACK    = RGB565_BLACK;
static constexpr uint16_t TFT_WHITE    = RGB565_WHITE;
static constexpr uint16_t TFT_RED      = RGB565_RED;
static constexpr uint16_t TFT_GREEN    = RGB565_LIGHTGREEN;
static constexpr uint16_t TFT_BLUE     = RGB565_BLUE;
static constexpr uint16_t TFT_CYAN     = RGB565_CYAN;
static constexpr uint16_t TFT_MAGENTA  = RGB565_MAGENTA;
static constexpr uint16_t TFT_YELLOW   = RGB565_YELLOW;
static constexpr uint16_t TFT_DARKGREY = RGB565_DARKGREY;

// ── Display init ──────────────────────────────────────────────────

static void forceBacklightOn() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);
}

static void initDisplay() {
  forceBacklightOn();

  if (!tft.begin(40000000)) {
    Serial.println("[LCD] Arduino_GFX begin failed");
  }
  tft.setRotation(LCD_ROTATION);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
}

// ── Helpers ────────────────────────────────────────────────────────

static uint16_t colorWheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85)      return tft.color565(255 - pos * 3, 0, pos * 3);
  if (pos < 170)     { pos -= 85; return tft.color565(0, pos * 3, 255 - pos * 3); }
  pos -= 170;
  return tft.color565(pos * 3, 255 - pos * 3, 0);
}

static void drawCenteredText(const char *text, int16_t centerX, int16_t centerY, uint8_t size) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.setTextSize(size);
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(centerX - (int16_t)w / 2, centerY - (int16_t)h / 2);
  tft.print(text);
}

static void showTitle(const char *title) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  drawCenteredText(title, tft.width() / 2, 60, 2);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  drawCenteredText("ESP32-S3 + 0.96", tft.width() / 2, 84, 1);
  delay(650);
}

// ── Graphic tests ──────────────────────────────────────────────────

static unsigned long testColorBars() {
  unsigned long start = micros();
  const uint16_t colors[] = {
    TFT_RED, TFT_GREEN, TFT_BLUE, TFT_CYAN,
    TFT_MAGENTA, TFT_YELLOW, TFT_WHITE, TFT_BLACK
  };
  int barH = tft.height() / 8;
  for (int i = 0; i < 8; ++i)
    tft.fillRect(0, i * barH, tft.width(), barH, colors[i]);
  return micros() - start;
}

static unsigned long testLines(uint16_t color) {
  unsigned long start = micros();
  int w = tft.width(), h = tft.height();
  tft.fillScreen(TFT_BLACK);
  for (int x = 0; x < w; x += 5) tft.drawLine(0, 0, x, h - 1, color);
  for (int y = 0; y < h; y += 5) tft.drawLine(0, 0, w - 1, y, color);
  for (int x = 0; x < w; x += 5) tft.drawLine(w - 1, h - 1, x, 0, TFT_YELLOW);
  for (int y = 0; y < h; y += 5) tft.drawLine(w - 1, h - 1, 0, y, TFT_YELLOW);
  return micros() - start;
}

static unsigned long testFastLines() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  for (int y = 0; y < tft.height(); y += 5)
    tft.drawFastHLine(0, y, tft.width(), TFT_RED);
  for (int x = 0; x < tft.width(); x += 5)
    tft.drawFastVLine(x, 0, tft.height(), TFT_BLUE);
  return micros() - start;
}

static unsigned long testRects() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  int cx = tft.width() / 2, cy = tft.height() / 2;
  int maxSize = min(tft.width(), tft.height());
  for (int sz = maxSize; sz > 6; sz -= 6)
    tft.drawRect(cx - sz / 2, cy - sz / 2, sz, sz, colorWheel(sz * 2));
  return micros() - start;
}

static unsigned long testFilledRects() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  int cx = tft.width() / 2, cy = tft.height() / 2;
  int maxSize = min(tft.width(), tft.height());
  for (int sz = maxSize; sz > 6; sz -= 8) {
    tft.fillRect(cx - sz / 2, cy - sz / 2, sz, sz, colorWheel(sz * 3));
    tft.drawRect(cx - sz / 2, cy - sz / 2, sz, sz, TFT_WHITE);
  }
  return micros() - start;
}

static unsigned long testCircles() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  int maxR = min(tft.width(), tft.height()) / 2 - 4;
  for (int r = 4; r <= maxR; r += 4)
    tft.drawCircle(tft.width() / 2, tft.height() / 2, r, colorWheel(r * 2));
  for (int y = 16; y < tft.height(); y += 32)
    for (int x = 12; x < tft.width(); x += 28)
      tft.fillCircle(x, y, 6, colorWheel(x + y));
  return micros() - start;
}

static unsigned long testTriangles() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  int cx = tft.width() / 2, cy = tft.height() / 2;
  int maxTri = min(tft.width(), tft.height()) / 2 - 2;
  for (int i = 0; i <= maxTri; i += 6)
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, colorWheel(i * 3));
  for (int i = maxTri; i > 0; i -= 8)
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, colorWheel(200 - i));
  return micros() - start;
}

static unsigned long testRoundRects() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  for (int i = 0; i < 40; i += 5)
    tft.drawRoundRect(2 + i / 2, 4 + i, tft.width() - 4 - i,
                      tft.height() - 8 - i * 2, 5, colorWheel(i * 6));
  return micros() - start;
}

static unsigned long testText() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(6, 8);
  tft.print("Graphic");
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(6, 28);
  tft.print("80 x 160 TFT");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(6, 42);
  tft.print("Arduino_GFX");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  for (int i = 0; i < 5; ++i) {
    tft.setCursor(6, 58 + i * 16);
    tft.print("L"); tft.print(i + 1);
    tft.print(" 0x"); tft.print(0x1000 + i * 257, HEX);
  }
  return micros() - start;
}

static unsigned long testGradient() {
  unsigned long start = micros();
  int w = tft.width(), h = tft.height();
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      tft.drawPixel(x, y, tft.color565(
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
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  drawCenteredText("Done!", tft.width() / 2, 54, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  drawCenteredText("All tests OK", tft.width() / 2, 76, 1);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  drawCenteredText("RST to rerun", tft.width() / 2, 96, 1);
  tft.drawRoundRect(4, 4, tft.width() - 8, tft.height() - 8, 4, TFT_BLUE);
}

// ── Main ───────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus 0.96 graphic test ===");

  initDisplay();

  Serial.print("LCD width: ");  Serial.println(tft.width());
  Serial.print("LCD height: "); Serial.println(tft.height());

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
