/*
  XIAO nRF52840 Plus + 1.47 Inch Touch Display
  Basic display test extracted from the 147_nRF52840 examples.

  Required library:
    - Arduino_GFX_Library
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Arduino_GFX_Library.h>

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

Arduino_DataBus *lcdBus = new Arduino_SWSPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  LCD_SCK_PIN,
  LCD_MOSI_PIN,
  GFX_NOT_DEFINED
);

Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus,
  LCD_RST_PIN,
  0,
  false,
  172,
  320,
  34,
  0,
  34,
  0
);

static void lcdHardReset() {
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(10);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(30);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(150);
}

static void applyPanelFix() {
  lcdBus->beginWrite();
  lcdBus->writeC8D8(0x36, 0x48);
  lcdBus->endWrite();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, 255);

  lcdHardReset();

  if (!gfx->begin()) {
    Serial.println("[LCD] begin failed");
    while (1) delay(1000);
  }

  applyPanelFix();

  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(RGB565_LIGHTGREEN, RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(8, 18);
  gfx->println("Hello XIAO");

  gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  gfx->setTextSize(1);
  gfx->setCursor(8, 52);
  gfx->println("1.47 display basic");

  gfx->fillRect(8, 82, 36, 40, RGB565_RED);
  gfx->fillRect(48, 82, 36, 40, RGB565_GREEN);
  gfx->fillRect(88, 82, 36, 40, RGB565_BLUE);
  gfx->fillRect(128, 82, 36, 40, RGB565_WHITE);

  Serial.println("[LCD] basic display test ready");
}

void loop() {
  static uint32_t frame = 0;
  gfx->fillRect(8, 150, 156, 18, RGB565_BLACK);
  gfx->setCursor(8, 150);
  gfx->setTextColor(RGB565_CYAN, RGB565_BLACK);
  gfx->print("frame=");
  gfx->print(frame++);
  delay(500);
}
