#include "driver.h"
#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft;

// Verified pins for XIAO nRF52840 Plus + 1.47 Inch Touch Display
static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

// JD9853A 172x320 panel fix.
// Current strategy:
// - Use Seeed_GFX / TFT_eSPI ST7789-compatible path.
// - Do not modify generic ST7789 driver.
// - Apply panel-specific MADCTL fix in sketch layer.
static void applyXIAO147PanelFix()
{
  tft.writecommand(0x36);  // MADCTL
  tft.writedata(0x48);     // Verified value for this JD9853A 172x320 panel
  delay(10);
}

static void setXIAO147Rotation(uint8_t rotation)
{
  tft.setRotation(rotation);

  if (rotation == 0) {
    applyXIAO147PanelFix();
  }
}

static void forceBacklightOn()
{
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);
}

static void hardResetPanel()
{
  pinMode(LCD_RST_PIN, OUTPUT);

  digitalWrite(LCD_RST_PIN, HIGH);
  delay(20);

  digitalWrite(LCD_RST_PIN, LOW);
  delay(80);

  digitalWrite(LCD_RST_PIN, HIGH);
  delay(180);
}

static void preparePins()
{
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_SCK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);

  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_SCK_PIN, LOW);
  digitalWrite(LCD_MOSI_PIN, LOW);
}

static void flashColors()
{
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

static void drawColorBars()
{
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

static void drawFinalScreen()
{
  const int w = tft.width();
  const int h = tft.height();

  tft.fillScreen(TFT_BLACK);

  // Border
  tft.drawRoundRect(4, 4, w - 8, h - 8, 10, TFT_DARKGREY);
  tft.drawRoundRect(8, 8, w - 16, h - 16, 8, TFT_BLUE);

  tft.setTextDatum(MC_DATUM);

  // Top label
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Seeed_GFX Test", w / 2, 42, 2);

  // Main title
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Hello XIAO!", w / 2, 98, 4);

  // Product name
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("1.47 Inch", w / 2, 152, 2);
  tft.drawString("Touch Display", w / 2, 178, 2);

  // Powered by
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Powered By", w / 2, 230, 2);
  tft.drawString("XIAO nRF52840 Plus", w / 2, 256, 2);

  // Small footer line
  tft.drawFastHLine(28, 292, w - 56, TFT_DARKGREY);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Seeed_GFX 1.47 Inch Touch Display LCD Demo");
  Serial.println("Board: XIAO nRF52840 Plus");

  preparePins();

  forceBacklightOn();
  hardResetPanel();

  tft.init();
  setXIAO147Rotation(0);

  Serial.print("LCD width: ");
  Serial.println(tft.width());

  Serial.print("LCD height: ");
  Serial.println(tft.height());

  flashColors();
  drawColorBars();
  drawFinalScreen();

  Serial.println("LCD demo finished.");
}

void loop()
{
}