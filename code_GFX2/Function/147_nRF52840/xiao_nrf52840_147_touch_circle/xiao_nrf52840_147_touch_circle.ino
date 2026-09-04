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

  Ported to Seeed_GFX2: Board_XIAO_1inch47_Touch_Display<38,37> +
  Config_XIAO_1inch47_Touch_JD9853A replace driver.h + manual pin init +
  tft.init()/setRotation(0)/invertDisplay(false)/applyXIAO147PanelFix().
  Touch migrates from raw axs5106l_device.h (touch_init/get_touch_data) to
  Touch_AXS5106L + display.attachTouch()/display.getTouch(). getTouch() already
  mirrors X (JD9853A MADCTL=MX), so rawTouchToScreen() is dropped; the original
  edge-triggered (touching && !wasTouching) logic is kept.

  Required libraries:
    - Seeed_GFX2
    - Adafruit_TinyUSB
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_JD9853A.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include "touch/Touch_AXS5106L.h"

// ========================= Pins =========================

static constexpr int8_t LCD_RST_PIN = 38;  // XIAO nRF52840 Plus
static constexpr int8_t LCD_BL_PIN  = 37;

// ========================= Display =========================

Seeed_GFX display;

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;

// ========================= Touch =========================

// Shared LCD/touch RST is handled by the display board, so touch RST = -1.
// INT = D7 (active-low, handled internally by the touch driver).
Touch_AXS5106L touch(-1, D7, Wire, 172, 320);

bool g_wasTouching = false;  // edge-triggered: only add circle on touch-down

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

// ========================= Drawing =========================

static void drawClearBar() {
  display.fillRect(0, CLEAR_ZONE_TOP, LCD_W, LCD_H - CLEAR_ZONE_TOP, TFT_DARKGREY);
  display.drawFastHLine(0, CLEAR_ZONE_TOP, LCD_W, TFT_WHITE);
  display.setTextDatum(MC_DATUM);
  display.setTextColor(TFT_WHITE, TFT_DARKGREY);
  display.drawString("CLEAR", LCD_W / 2, CLEAR_ZONE_TOP + 18, 2);
}

// Safe drawing area: inset by circle radius so circles are fully visible.
static constexpr int DRAW_LEFT   = CIRCLE_RADIUS;
static constexpr int DRAW_RIGHT  = LCD_W - 1 - CIRCLE_RADIUS;
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
  // Dim cyan/white border showing where circles will appear fully.
  uint16_t borderColor = display.color565(80, 80, 80);
  display.drawRect(DRAW_LEFT - 1, DRAW_TOP - 1,
               DRAW_RIGHT - DRAW_LEFT + 2, DRAW_BOTTOM - DRAW_TOP + 2,
               borderColor);
}

static void drawTitle() {
  display.setTextDatum(TL_DATUM);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.drawString("Touch Circle Demo", 8, 8, 2);

  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString("Tap screen to draw", 8, 30, 1);

  char buf[32];
  snprintf(buf, sizeof(buf), "Circles: %d", g_circleCount);
  display.setTextColor(TFT_GREEN, TFT_BLACK);
  display.drawString(buf, 8, 46, 1);
}

static void updateCircleCount() {
  display.fillRect(8, 46, LCD_W - 8, 12, TFT_BLACK);
  display.setTextDatum(TL_DATUM);
  char buf[32];
  snprintf(buf, sizeof(buf), "Circles: %d", g_circleCount);
  display.setTextColor(TFT_GREEN, TFT_BLACK);
  display.drawString(buf, 8, 46, 1);
}

static void clearAllCircles() {
  // Redraw entire screen — simpler and more reliable than erasing one by one.
  g_circleCount = 0;
  display.fillScreen(TFT_BLACK);
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
    display.fillScreen(TFT_BLACK);
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

  // display.begin() resets the shared LCD/touch RST line and initializes the LCD.
  if (!display.begin<Board_XIAO_1inch47_Touch_Display<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_XIAO_1inch47_Touch_JD9853A>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(TFT_BLACK);

  // Attach the AXS5106L touch controller (touch RST shares the LCD RST already done).
  if (!display.attachTouch(touch, display.panel().driver().bus())) {
    Serial.println(display.lastResult().message);
    return;
  }

  drawSafeAreaBorder();
  drawClearBar();
  drawTitle();

  Serial.print("LCD: ");
  Serial.print(LCD_W);
  Serial.print("x");
  Serial.println(LCD_H);

  Serial.println("Touch: AXS5106L ready");
  Serial.println("Tap screen to draw white circles.");
  Serial.println("Tap CLEAR bar at bottom to erase.");
}

// ========================= Loop =========================

void loop() {
  // Edge-triggered: only act on touch-down, not continuously while held.
  int32_t x = 0, y = 0;
  bool touching = display.getTouch(&x, &y);  // x/y already mirror-corrected screen coords

  if (touching && !g_wasTouching) {
    // Finger just touched down — add one circle.
    int screenX = (int)x;
    int screenY = (int)y;

    // getTouch() already applied the X mirror, so raw coords are no longer available.
    Serial.print("Touch: screen=(");
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
