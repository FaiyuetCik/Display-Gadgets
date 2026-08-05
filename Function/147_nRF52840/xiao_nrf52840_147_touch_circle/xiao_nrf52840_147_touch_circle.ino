/*
  XIAO nRF52840 Plus 1.47 Inch Touch Display
  Touch Circle Demo

  Tap anywhere on the screen — a small white circle appears at the touch point.
  Circles stay on screen. Tap the "CLEAR" bar at the bottom to erase all circles.

  Hardware:
    - XIAO nRF52840 Plus
    - 1.47" Touch Display (172x320 JD9853A / ST7789)
    - AXS5106L touch controller (I2C addr 0x63)

  Pin map:
    LCD CS   = D2
    LCD DC   = D3
    I2C SDA  = D4
    I2C SCL  = D5
    TOUCH INT= D7
    LCD SCK  = D8
    LCD MOSI = D10
    LCD RST  = D17  (shared with touch RST)
    LCD BL   = D18

  Required libraries:
    - Seeed_GFX / TFT_eSPI
    - Adafruit_TinyUSB
*/

#include "driver.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include "axs5106l_device.h"

// ========================= Pins =========================

static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t LCD_RST_PIN   = D17;
static constexpr uint8_t LCD_BL_PIN    = D18;
static constexpr uint8_t TOUCH_INT_PIN = D7;
static constexpr uint8_t TOUCH_RST_PIN = D17;  // shared with LCD RST

// ========================= Display =========================

TFT_eSPI tft;

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;

// ========================= Touch =========================

volatile bool g_touchIrq = false;
touch_data_t g_touchData;
bool g_wasTouching = false;  // edge-triggered: only add circle on touch-down

void touchIsr() {
  g_touchIrq = true;
}

// ========================= Circles =========================

static constexpr int MAX_CIRCLES = 120;
static constexpr int CIRCLE_RADIUS = 6;

struct Circle {
  int x;
  int y;
};

Circle g_circles[MAX_CIRCLES];
int g_circleCount = 0;

// ========================= Clear area =========================

// Bottom 36 px of the screen is the "CLEAR" zone.
static constexpr int CLEAR_ZONE_TOP = LCD_H - 36;

static bool isInClearZone(int screenY) {
  return screenY >= CLEAR_ZONE_TOP;
}

// ========================= Display helpers =========================

static void applyXIAO147PanelFix() {
  tft.writecommand(0x36);
  tft.writedata(0x48);
  delay(10);
}

static void setBacklight(uint8_t pwm) {
  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, pwm);
}

static void prepareDisplayPins() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_SCK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);

  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_SCK_PIN, LOW);
  digitalWrite(LCD_MOSI_PIN, LOW);
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
  prepareDisplayPins();
  setBacklight(255);
  hardResetPanel();

  tft.init();
  tft.setRotation(0);
  applyXIAO147PanelFix();
  tft.invertDisplay(false);
  tft.fillScreen(TFT_BLACK);
}

// ========================= Touch coordinate mapping =========================
//
// The AXS5106L touch controller reports coordinates already in the
// display's pixel range. The X axis may be mirrored depending on how
// the touch panel is physically mounted relative to the LCD.

static void rawTouchToScreen(uint16_t rawX, uint16_t rawY, int &screenX, int &screenY) {
  // X axis is mirrored: the touch panel reports from the opposite edge.
  // This matches the pattern used by the ESP32 snake game and the commented-
  // out mirror line in the factory dashboard's axs5106l_device driver.
  screenX = LCD_W - 1 - (int)rawX;

  // Y axis is direct.
  screenY = (int)rawY;

  // Clamp to valid screen bounds just in case.
  if (screenX < 0) screenX = 0;
  if (screenX >= LCD_W) screenX = LCD_W - 1;
  if (screenY < 0) screenY = 0;
  if (screenY >= LCD_H) screenY = LCD_H - 1;
}

// ========================= Drawing =========================

static void drawClearBar() {
  tft.fillRect(0, CLEAR_ZONE_TOP, LCD_W, LCD_H - CLEAR_ZONE_TOP, TFT_DARKGREY);
  tft.drawFastHLine(0, CLEAR_ZONE_TOP, LCD_W, TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.drawString("CLEAR", LCD_W / 2, CLEAR_ZONE_TOP + 18, 2);
}

// Safe drawing area: inset by circle radius so circles are fully visible.
static constexpr int DRAW_LEFT   = CIRCLE_RADIUS;
static constexpr int DRAW_RIGHT  = LCD_W - 1 - CIRCLE_RADIUS;
static constexpr int DRAW_TOP    = 56;  // below title bar
static constexpr int DRAW_BOTTOM = CLEAR_ZONE_TOP - 1 - CIRCLE_RADIUS;

static void drawCircleAt(int x, int y, uint16_t color) {
  tft.fillCircle(x, y, CIRCLE_RADIUS, color);
}

// Clamp touch coordinates to the safe drawing area.
static void clampToDrawArea(int &x, int &y) {
  if (x < DRAW_LEFT)   x = DRAW_LEFT;
  if (x > DRAW_RIGHT)  x = DRAW_RIGHT;
  if (y < DRAW_TOP)    y = DRAW_TOP;
  if (y > DRAW_BOTTOM) y = DRAW_BOTTOM;
}

static void drawSafeAreaBorder() {
  // Dim cyan/white border showing where circles will appear fully.
  uint16_t borderColor = tft.color565(80, 80, 80);
  tft.drawRect(DRAW_LEFT - 1, DRAW_TOP - 1,
               DRAW_RIGHT - DRAW_LEFT + 2, DRAW_BOTTOM - DRAW_TOP + 2,
               borderColor);
}

static void drawTitle() {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Touch Circle Demo", 8, 8, 2);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Tap screen to draw", 8, 30, 1);

  char buf[32];
  snprintf(buf, sizeof(buf), "Circles: %d", g_circleCount);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(buf, 8, 46, 1);
}

static void updateCircleCount() {
  tft.fillRect(8, 46, LCD_W - 8, 12, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  char buf[32];
  snprintf(buf, sizeof(buf), "Circles: %d", g_circleCount);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(buf, 8, 46, 1);
}

static void clearAllCircles() {
  // Redraw entire screen — simpler and more reliable than erasing one by one.
  g_circleCount = 0;
  tft.fillScreen(TFT_BLACK);
  drawSafeAreaBorder();
  drawClearBar();
  drawTitle();  // reads g_circleCount=0, shows "Circles: 0"
}

static void addCircle(int screenX, int screenY) {
  // Don't draw circles in the clear bar area.
  if (isInClearZone(screenY)) return;

  // Clamp to safe area so circles are always fully visible.
  clampToDrawArea(screenX, screenY);

  // If buffer is full, remove the oldest circle.
  if (g_circleCount >= MAX_CIRCLES) {
    // Shift everything down — oldest is at index 0.
    for (int i = 0; i < MAX_CIRCLES - 1; i++) {
      g_circles[i] = g_circles[i + 1];
    }
    g_circleCount = MAX_CIRCLES - 1;

    // Redraw everything to cleanly remove the oldest circle.
    tft.fillScreen(TFT_BLACK);
    drawSafeAreaBorder();
    drawClearBar();
    drawTitle();
    for (int i = 0; i < g_circleCount; i++) {
      drawCircleAt(g_circles[i].x, g_circles[i].y, TFT_WHITE);
    }
  }

  // Draw new circle.
  drawCircleAt(screenX, screenY, TFT_WHITE);
  g_circles[g_circleCount].x = screenX;
  g_circles[g_circleCount].y = screenY;
  g_circleCount++;

  updateCircleCount();
}

// ========================= Init =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus Touch Circle Demo ===");

  // Init touch first — the shared RST (D17) will reset both touch and LCD.
  Wire.begin();
  touch_init(&Wire, TOUCH_RST_PIN, TOUCH_INT_PIN);

  // Now init LCD. Its own hard-reset also pulses D17, which is fine.
  initDisplay();

  drawSafeAreaBorder();
  drawClearBar();
  drawTitle();

  Serial.print("LCD: ");
  Serial.print(LCD_W);
  Serial.print("x");
  Serial.println(LCD_H);

  // AXS5106L INT is active-low.
  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TOUCH_INT_PIN), touchIsr, FALLING);

  Serial.println("Touch: AXS5106L ready");
  Serial.println("Tap screen to draw white circles.");
  Serial.println("Tap CLEAR bar at bottom to erase.");
}

// ========================= Loop =========================

void loop() {
  // Edge-triggered: only act on touch-down, not continuously while held.
  bool gotData = get_touch_data(&g_touchData);
  bool touching = gotData && g_touchData.touch_num > 0;

  if (touching && !g_wasTouching) {
    // Finger just touched down — add one circle.
    int screenX, screenY;
    rawTouchToScreen(g_touchData.coords[0].x, g_touchData.coords[0].y,
                     screenX, screenY);

    Serial.print("Touch: raw=(");
    Serial.print(g_touchData.coords[0].x);
    Serial.print(",");
    Serial.print(g_touchData.coords[0].y);
    Serial.print(") -> screen=(");
    Serial.print(screenX);
    Serial.print(",");
    Serial.print(screenY);
    Serial.println(")");

    if (isInClearZone(screenY)) {
      Serial.println("Clear zone tapped — erasing all circles.");
      clearAllCircles();
      // Don't leave a circle on the CLEAR bar.
      g_wasTouching = true;
    } else {
      addCircle(screenX, screenY);
    }
  }

  g_wasTouching = touching;

  delay(10);
}
