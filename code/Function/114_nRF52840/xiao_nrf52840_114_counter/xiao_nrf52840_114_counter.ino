/*
  XIAO nRF52840 Plus 1.14 Inch Display
  Counter — Grove Mech Keycap (SKU 111020049) press counter

  Hardware:
    - Grove Mech Keycap → display board's Grove I2C port
    - I2C Grove: Yellow=SCL(D5), White=SDA(D4)
    - Mech Keycap: Yellow=SIG(button), White=NC(LED)
    - → Button is on D5!  Pressed = connects to VCC, ADC jumps 658→940.

  Press the keycap → counter +1.  Max 10, then wraps to 0.

  Required libraries:
    - Seeed_GFX / TFT_eSPI
    - Adafruit_TinyUSB
*/

#include "driver.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <TFT_eSPI.h>

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

static constexpr uint8_t BTN_PIN      = D5;    // I2C SCL = button signal

TFT_eSPI tft(135, 240);
static constexpr int LCD_W = 135;
static constexpr int LCD_H = 240;

static constexpr int COUNT_MAX = 9;
int g_count     = 0;
int g_lastCount = -1;

// Button: analogRead detects voltage jump when pressed
// Not pressed: ADC ~624   Pressed: ADC ~941   Threshold: baseline + 150
static constexpr unsigned long DEBOUNCE_MS = 60;
static constexpr int           ADC_MARGIN  = 150;
unsigned long g_lastPress  = 0;
bool          g_btnPressed = false;
int           g_btnBase    = 0;

static uint16_t countColor(int n) {
  int r = n * 255 / COUNT_MAX;
  int g = 255 - r;
  return tft.color565(r, g, 0);
}

static void drawTitle() {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("COUNTER", 6, 4, 2);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("Press Mech Keycap", 6, 22, 1);
  tft.drawFastHLine(8, 38, LCD_W - 16, tft.color565(55, 55, 55));
}

static void drawNumber(int n) {
  char buf[4];
  snprintf(buf, sizeof(buf), "%d", n);
  uint8_t font = 8;  // big font, single digit only
  tft.fillRect(0, 44, LCD_W, LCD_H - 80, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(countColor(n), TFT_BLACK);
  tft.drawString(buf, LCD_W / 2, LCD_H / 2 - 5, font);
}

static void drawProgressBar(int n) {
  static constexpr int BAR_X = 14, BAR_Y = LCD_H - 34;
  static constexpr int BAR_W = LCD_W - 28, BAR_H = 10;
  tft.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, TFT_WHITE);
  int fillW = n * (BAR_W - 2) / COUNT_MAX;
  if (fillW > 0)
    tft.fillRect(BAR_X + 1, BAR_Y + 1, fillW, BAR_H - 2, countColor(n));
  if (fillW < BAR_W - 2)
    tft.fillRect(BAR_X + 1 + fillW, BAR_Y + 1, BAR_W - 2 - fillW, BAR_H - 2, TFT_BLACK);
}

static void drawHint() {
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("press key:  0 - 9", LCD_W / 2, LCD_H - 6, 1);
}

static void drawAll(int n) {
  if (n == g_lastCount) return;
  drawNumber(n);
  drawProgressBar(n);
  g_lastCount = n;
}

static void initDisplay() {
  pinMode(LCD_CS_PIN, OUTPUT); digitalWrite(LCD_CS_PIN, HIGH);
  pinMode(LCD_DC_PIN, OUTPUT); digitalWrite(LCD_DC_PIN, HIGH);
  pinMode(LCD_SCK_PIN, OUTPUT); digitalWrite(LCD_SCK_PIN, LOW);
  pinMode(LCD_MOSI_PIN, OUTPUT); digitalWrite(LCD_MOSI_PIN, LOW);
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH); analogWrite(LCD_BL_PIN, 255);
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH); delay(20);
  digitalWrite(LCD_RST_PIN, LOW);  delay(80);
  digitalWrite(LCD_RST_PIN, HIGH); delay(180);
  tft.init();
  tft.setRotation(0);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("=== Counter | D5 analogRead ===");

  initDisplay();
  drawTitle();
  drawHint();
  drawAll(g_count);

  // Calibrate baseline (button NOT pressed)
  pinMode(BTN_PIN, INPUT);
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(BTN_PIN); delay(5); }
  g_btnBase = sum / 20;
  Serial.print("[BTN] D5 base="); Serial.print(g_btnBase);
  Serial.print("  threshold="); Serial.println(g_btnBase + ADC_MARGIN);
}

void loop() {
  int adc = analogRead(BTN_PIN);

  static unsigned long g_lastDbg = 0;
  static int g_adcMax = 0;
  if (adc > g_adcMax) g_adcMax = adc;

  if (millis() - g_lastDbg > 500) {
    g_lastDbg = millis();
    Serial.print("ADC="); Serial.print(adc);
    Serial.print("  base="); Serial.print(g_btnBase);
    Serial.print("  max="); Serial.print(g_adcMax);
    Serial.print("  cnt="); Serial.println(g_count);
  }

  bool nowPressed = (adc > g_btnBase + ADC_MARGIN);

  if (nowPressed && !g_btnPressed) {
    unsigned long now = millis();
    if (now - g_lastPress > DEBOUNCE_MS) {
      g_lastPress = now;
      g_count++;
      if (g_count > COUNT_MAX) g_count = 0;
      Serial.print(">>> PRESS! ADC="); Serial.print(adc);
      Serial.print("  count="); Serial.println(g_count);
      drawAll(g_count);
    }
  }

  g_btnPressed = nowPressed;
  delay(10);
}
