/*
  XIAO ESP32-S3 Plus 1.47 Inch Touch Display
  Touch Circle Demo

  Tap anywhere on the screen — a small white circle appears at the touch point.
  Circles stay on screen. Tap the "CLEAR" bar at the bottom to erase all circles.

  Hardware:
    - XIAO ESP32-S3 Plus
    - 1.47" Touch Display (172x320 JD9853A)
    - AXS5106L touch controller (I2C addr 0x63)

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  driver.h + manual pin setup + tft.init()/setRotation(0)/applyXIAO147PanelFix()/
  invertDisplay(false). DROP driver.h + initDisplay() + axs5106l_device.h +
  touch_init()/get_touch_data()/rawTouchToScreen(). The GFX2 Touch layer
  (Touch_AXS5106L + display.attachTouch + display.getTouch) replaces the raw I2C
  touch code; getTouch() already returns X-mirrored screen coordinates, so the
  manual screenX = SCREEN_W-1-rawX mirror is removed. Edge-triggered touch-down
  logic and all circle drawing unchanged.

  Required libraries:
    - Seeed_GFX2
*/

#include <Arduino.h>
#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_JD9853A.h"
#include "panel/Panel_TFT.h"
#include "touch/Touch_AXS5106L.h"
#include <Wire.h>

// ========================= Display =========================

Seeed_GFX display;

static constexpr int8_t LCD_RST_PIN = 13;  // XIAO ESP32-S3 Plus
static constexpr int8_t LCD_BL_PIN = 12;

// Touch: RST shares the LCD RST (already reset by display.begin), INT = D7.
Touch_AXS5106L touch(-1, D7, Wire, 172, 320);

static constexpr int SCREEN_W = 172;
static constexpr int SCREEN_H = 320;

// ========================= Colours =========================

static constexpr uint16_t C_BLACK      = TFT_BLACK;
static constexpr uint16_t C_WHITE      = TFT_WHITE;
static constexpr uint16_t C_GREEN      = TFT_GREEN;
static constexpr uint16_t C_CYAN       = TFT_CYAN;
static constexpr uint16_t C_DARKGREY   = TFT_DARKGREY;
static constexpr uint16_t C_BORDER     = 0x528A;  // dim grey

// ========================= Touch =========================

bool g_wasTouching = false;  // edge-triggered: only add circle on touch-down

// ========================= Circles =========================

static constexpr int MAX_CIRCLES   = 120;
static constexpr int CIRCLE_RADIUS = 6;

struct Circle {
  int x;
  int y;
};

Circle g_circles[MAX_CIRCLES];
int g_circleCount = 0;

// ========================= Clear area =========================

// Bottom 36 px of the screen is the "CLEAR" zone.
static constexpr int CLEAR_ZONE_TOP = SCREEN_H - 36;

static bool isInClearZone(int screenY) {
  return screenY >= CLEAR_ZONE_TOP;
}

// ========================= Drawing =========================

static void drawClearBar() {
  display.fillRect(0, CLEAR_ZONE_TOP, SCREEN_W, SCREEN_H - CLEAR_ZONE_TOP, C_DARKGREY);
  display.drawFastHLine(0, CLEAR_ZONE_TOP, SCREEN_W, C_WHITE);
  display.setTextDatum(MC_DATUM);
  display.setTextColor(C_WHITE, C_DARKGREY);
  display.drawString("CLEAR", SCREEN_W / 2, CLEAR_ZONE_TOP + 18, 2);
}

// Safe drawing area: inset by circle radius so circles are fully visible.
static constexpr int DRAW_LEFT   = CIRCLE_RADIUS;
static constexpr int DRAW_RIGHT  = SCREEN_W - 1 - CIRCLE_RADIUS;
static constexpr int DRAW_TOP    = 56;  // below title bar
static constexpr int DRAW_BOTTOM = CLEAR_ZONE_TOP - 1 - CIRCLE_RADIUS;

static void drawCircleAt(int x, int y, uint16_t color) {
  display.fillCircle(x, y, CIRCLE_RADIUS, color);
}

// Clamp touch coordinates to the safe drawing area.
static void clampToDrawArea(int &x, int &y) {
  if (x < DRAW_LEFT)   x = DRAW_LEFT;
  if (x > DRAW_RIGHT)  x = DRAW_RIGHT;
  if (y < DRAW_TOP)    y = DRAW_TOP;
  if (y > DRAW_BOTTOM) y = DRAW_BOTTOM;
}

static void drawSafeAreaBorder() {
  display.drawRect(DRAW_LEFT - 1, DRAW_TOP - 1,
                   DRAW_RIGHT - DRAW_LEFT + 2, DRAW_BOTTOM - DRAW_TOP + 2,
                   C_BORDER);
}

static void drawTitle() {
  display.setTextDatum(TL_DATUM);
  display.setTextColor(C_CYAN, C_BLACK);
  display.drawString("Touch Circle Demo", 8, 8, 2);

  display.setTextColor(C_WHITE, C_BLACK);
  display.drawString("Tap screen to draw", 8, 30, 1);

  char buf[32];
  snprintf(buf, sizeof(buf), "Circles: %d", g_circleCount);
  display.setTextColor(C_GREEN, C_BLACK);
  display.drawString(buf, 8, 46, 1);
}

static void updateCircleCount() {
  display.fillRect(8, 46, SCREEN_W - 8, 12, C_BLACK);
  display.setTextDatum(TL_DATUM);
  char buf[32];
  snprintf(buf, sizeof(buf), "Circles: %d", g_circleCount);
  display.setTextColor(C_GREEN, C_BLACK);
  display.drawString(buf, 8, 46, 1);
}

static void clearAllCircles() {
  // Redraw entire screen — simpler and more reliable than erasing one by one.
  g_circleCount = 0;
  display.fillScreen(C_BLACK);
  drawSafeAreaBorder();
  drawClearBar();
  drawTitle();
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
    display.fillScreen(C_BLACK);
    drawSafeAreaBorder();
    drawClearBar();
    drawTitle();
    for (int i = 0; i < g_circleCount; i++) {
      drawCircleAt(g_circles[i].x, g_circles[i].y, C_WHITE);
    }
  }

  // Draw new circle.
  drawCircleAt(screenX, screenY, C_WHITE);
  g_circles[g_circleCount].x = screenX;
  g_circles[g_circleCount].y = screenY;
  g_circleCount++;

  updateCircleCount();
}

// ========================= Setup =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Touch Circle Demo ===");

  if (!display.begin<Board_XIAO_1inch47_Touch_Display<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_Seeed_1inch47_Touch_JD9853A>()) {
    Serial.println(display.lastResult().message);
    return;
  }

  if (!display.attachTouch(touch, display.panel().driver().bus())) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(C_BLACK);

  drawSafeAreaBorder();
  drawClearBar();
  drawTitle();

  Serial.print("LCD: ");
  Serial.print(SCREEN_W);
  Serial.print("x");
  Serial.println(SCREEN_H);
  Serial.println("Touch: AXS5106L ready");
  Serial.println("Tap screen to draw white circles.");
  Serial.println("Tap CLEAR bar at bottom to erase.");
}

// ========================= Loop =========================

void loop() {
  // Edge-triggered: only act on touch-down, not continuously while held.
  int32_t x = 0, y = 0;
  bool touching = display.getTouch(&x, &y);

  if (touching && !g_wasTouching) {
    // Finger just touched down — add one circle.
    Serial.print("Touch: (");
    Serial.print((int)x);
    Serial.print(",");
    Serial.print((int)y);
    Serial.println(")");

    if (isInClearZone((int)y)) {
      Serial.println("Clear zone tapped — erasing all circles.");
      clearAllCircles();
    } else {
      addCircle((int)x, (int)y);
    }
  }

  g_wasTouching = touching;

  delay(10);
}
