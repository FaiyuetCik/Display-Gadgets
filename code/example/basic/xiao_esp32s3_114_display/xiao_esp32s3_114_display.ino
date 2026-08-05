#include <Arduino.h>
#include "driver.h"
#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft(135, 240);

static constexpr uint8_t LCD_CS_PIN = D2;
static constexpr uint8_t LCD_DC_PIN = D3;
static constexpr uint8_t LCD_SCK_PIN = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN = D17;
static constexpr uint8_t LCD_BL_PIN = D18;

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

static void flashColors() {
  const uint16_t colors[] = {
    TFT_RED,
    TFT_GREEN,
    TFT_BLUE,
    TFT_WHITE,
    TFT_BLACK
  };

  for (uint8_t i = 0; i < 5; i++) {
    tft.fillScreen(colors[i]);
    delay(450);
  }
}

static void drawColorBars() {
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

  const int w = tft.width();
  const int h = tft.height();
  const int barH = h / 8;

  for (uint8_t i = 0; i < 8; i++) {
    tft.fillRect(0, i * barH, w, barH, colors[i]);
  }

  delay(1200);
}

static void drawFinalScreen() {
  const int w = tft.width();
  const int h = tft.height();

  tft.fillScreen(TFT_BLACK);

  tft.drawRoundRect(4, 4, w - 8, h - 8, 8, TFT_DARKGREY);
  tft.drawRoundRect(8, 8, w - 16, h - 16, 6, TFT_BLUE);

  tft.setTextDatum(MC_DATUM);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Seeed_GFX", w / 2, 42, 2);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("1.14 Inch", w / 2, 92, 4);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("XIAO ESP32-S3 Plus", w / 2, 150, 1);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("ST7789 Display", w / 2, 190, 2);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Seeed_GFX 1.14 Inch Display LCD Demo");
  Serial.println("Board: XIAO ESP32-S3 Plus");

  preparePins();
  forceBacklightOn();
  hardResetPanel();

  tft.init();
  tft.setRotation(0);
  tft.invertDisplay(true);

  Serial.print("LCD width: ");
  Serial.println(tft.width());

  Serial.print("LCD height: ");
  Serial.println(tft.height());

  flashColors();
  drawColorBars();
  drawFinalScreen();

  Serial.println("LCD demo finished.");
}

void loop() {
}
