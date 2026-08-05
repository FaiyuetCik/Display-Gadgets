/*
  XIAO nRF52840 Plus 1.14 Inch Display graphic test.

  This sketch uses the same Seeed_GFX / TFT_eSPI setup as the 1.47 version,
  then runs a compact graphics benchmark:
    - color bars
    - lines
    - rectangles
    - circles
    - triangles
    - rounded rectangles
    - text
    - pixel gradient

  1.14 differences from 1.47:
    - 135x240 resolution (vs 172x320)
    - ST7789 IPS panel, no JD9853A fix needed
    - invertDisplay(true)

  Required libraries:
    - Seeed_GFX / TFT_eSPI
    - Adafruit_TinyUSB
*/

#include "driver.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft(135, 240);

static constexpr uint8_t LCD_CS_PIN = D2;
static constexpr uint8_t LCD_DC_PIN = D3;
static constexpr uint8_t LCD_SCK_PIN = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN = D17;
static constexpr uint8_t LCD_BL_PIN = D18;

static void preparePins() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_SCK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);

  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_SCK_PIN, LOW);
  digitalWrite(LCD_MOSI_PIN, LOW);
}

static void forceBacklightOn() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);
}

static void hardResetPanel() {
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(20);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(80);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(180);
}

static void initDisplay() {
  preparePins();
  forceBacklightOn();
  hardResetPanel();

  tft.init();
  tft.setRotation(0);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
}

static void showTitle(const char *title) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString(title, tft.width() / 2, 92, 2);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("XIAO nRF52840 Plus", tft.width() / 2, 118, 1);
  delay(650);
}

static uint16_t colorWheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) {
    return tft.color565(255 - pos * 3, 0, pos * 3);
  }
  if (pos < 170) {
    pos -= 85;
    return tft.color565(0, pos * 3, 255 - pos * 3);
  }
  pos -= 170;
  return tft.color565(pos * 3, 255 - pos * 3, 0);
}

static unsigned long testColorBars() {
  unsigned long start = micros();
  const uint16_t colors[] = {
    TFT_RED,
    TFT_GREEN,
    TFT_BLUE,
    TFT_CYAN,
    TFT_MAGENTA,
    TFT_YELLOW,
    TFT_WHITE,
    TFT_BLACK
  };
  int barH = tft.height() / 8;
  for (int i = 0; i < 8; ++i) {
    tft.fillRect(0, i * barH, tft.width(), barH, colors[i]);
  }
  return micros() - start;
}

static unsigned long testLines(uint16_t color) {
  unsigned long start = micros();
  int w = tft.width();
  int h = tft.height();
  tft.fillScreen(TFT_BLACK);

  for (int x = 0; x < w; x += 8) tft.drawLine(0, 0, x, h - 1, color);
  for (int y = 0; y < h; y += 8) tft.drawLine(0, 0, w - 1, y, color);
  for (int x = 0; x < w; x += 8) tft.drawLine(w - 1, h - 1, x, 0, TFT_YELLOW);
  for (int y = 0; y < h; y += 8) tft.drawLine(w - 1, h - 1, 0, y, TFT_YELLOW);

  return micros() - start;
}

static unsigned long testFastLines() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  for (int y = 0; y < tft.height(); y += 5) {
    tft.drawFastHLine(0, y, tft.width(), TFT_RED);
  }
  for (int x = 0; x < tft.width(); x += 5) {
    tft.drawFastVLine(x, 0, tft.height(), TFT_BLUE);
  }
  return micros() - start;
}

static unsigned long testRects() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  int cx = tft.width() / 2;
  int cy = tft.height() / 2;
  int maxSize = min(tft.width(), tft.height());

  for (int size = maxSize; size > 8; size -= 10) {
    int x = cx - size / 2;
    int y = cy - size / 2;
    tft.drawRect(x, y, size, size, colorWheel(size * 2));
  }
  return micros() - start;
}

static unsigned long testFilledRects() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  int cx = tft.width() / 2;
  int cy = tft.height() / 2;
  int maxSize = min(tft.width(), tft.height());

  for (int size = maxSize; size > 8; size -= 12) {
    int x = cx - size / 2;
    int y = cy - size / 2;
    tft.fillRect(x, y, size, size, colorWheel(size * 3));
    tft.drawRect(x, y, size, size, TFT_WHITE);
  }
  return micros() - start;
}

static unsigned long testCircles() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  int maxR = min(tft.width(), tft.height()) / 2 - 8;
  for (int r = 8; r <= maxR; r += 8) {
    tft.drawCircle(tft.width() / 2, tft.height() / 2, r, colorWheel(r * 2));
  }
  for (int y = 24; y < tft.height(); y += 48) {
    for (int x = 20; x < tft.width(); x += 44) {
      tft.fillCircle(x, y, 10, colorWheel(x + y));
    }
  }
  return micros() - start;
}

static unsigned long testTriangles() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  int cx = tft.width() / 2;
  int cy = tft.height() / 2;
  int maxTri = min(tft.width(), tft.height()) / 2 - 4;

  for (int i = 0; i <= maxTri; i += 10) {
    tft.drawTriangle(
      cx, cy - i,
      cx - i, cy + i,
      cx + i, cy + i,
      colorWheel(i * 3)
    );
  }
  for (int i = maxTri; i > 0; i -= 14) {
    tft.fillTriangle(
      cx, cy - i,
      cx - i, cy + i,
      cx + i, cy + i,
      colorWheel(200 - i)
    );
  }
  return micros() - start;
}

static unsigned long testRoundRects() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  for (int i = 0; i < 70; i += 8) {
    tft.drawRoundRect(
      4 + i / 2,
      8 + i,
      tft.width() - 8 - i,
      tft.height() - 16 - i * 2,
      8,
      colorWheel(i * 4)
    );
  }
  return micros() - start;
}

static unsigned long testText() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Graphic", 8, 14, 4);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("135 x 240 TFT", 10, 62, 2);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("TFT_eSPI", 10, 88, 2);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  for (int i = 0; i < 7; ++i) {
    tft.setCursor(10, 122 + i * 16);
    tft.print("Line ");
    tft.print(i + 1);
    tft.print(" 0x");
    tft.print(0x1000 + i * 137, HEX);
  }
  return micros() - start;
}

static unsigned long testGradient() {
  unsigned long start = micros();
  int w = tft.width();
  int h = tft.height();
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      uint8_t r = map(x, 0, w - 1, 0, 255);
      uint8_t g = map(y, 0, h - 1, 0, 255);
      uint8_t b = (x + y) & 0xFF;
      tft.drawPixel(x, y, tft.color565(r, g, b));
    }
  }
  return micros() - start;
}

static void printResult(const char *name, unsigned long us) {
  Serial.print(name);
  Serial.print(": ");
  Serial.print(us / 1000.0f, 2);
  Serial.println(" ms");
}

static void showResultScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Graphic", tft.width() / 2, 78, 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Finished", tft.width() / 2, 126, 4);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Reset to rerun", tft.width() / 2, 178, 2);
  tft.drawRoundRect(8, 8, tft.width() - 16, tft.height() - 16, 8, TFT_BLUE);
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 1.14 graphic test ===");

  initDisplay();

  Serial.print("LCD width: ");
  Serial.println(tft.width());
  Serial.print("LCD height: ");
  Serial.println(tft.height());

  showTitle("Graphic Test");

  unsigned long us;

  us = testColorBars();
  printResult("Color bars", us);
  delay(900);

  us = testLines(TFT_CYAN);
  printResult("Lines", us);
  delay(900);

  us = testFastLines();
  printResult("Fast lines", us);
  delay(900);

  us = testRects();
  printResult("Rectangles", us);
  delay(900);

  us = testFilledRects();
  printResult("Filled rectangles", us);
  delay(900);

  us = testCircles();
  printResult("Circles", us);
  delay(900);

  us = testTriangles();
  printResult("Triangles", us);
  delay(900);

  us = testRoundRects();
  printResult("Round rectangles", us);
  delay(900);

  us = testText();
  printResult("Text", us);
  delay(1100);

  us = testGradient();
  printResult("Pixel gradient", us);
  delay(1200);

  showResultScreen();
  Serial.println("Graphic test finished.");
}

void loop() {
  delay(1000);
}
