/*
  XIAO nRF52840 Plus 1.14-inch Display - Getting Started
  Library: Seeed_GFX2 (header: Seeed_GFX.h)
  Show "Hello, XIAO" in large green text on a black background.
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"

Seeed_GFX display;

// Raw GPIO numbers, matching the existing nRF52840 Plus demos.
static constexpr int8_t LCD_RST_PIN = 38;
static constexpr int8_t LCD_BL_PIN = 37;

void setup() {
  Serial.begin(115200);

  // The board configuration initializes SPI, resets the LCD and turns on
  // its backlight. The panel configuration supplies rotation and colors.
  if (!display.begin<Board_XIAO_1inch14_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_Seeed_1inch14_LCD_ST7789>()) {
    Serial.println(display.lastResult().message);
    return;
  }

  display.fillScreen(TFT_BLACK);
  display.setTextColor(TFT_GREEN, TFT_BLACK);
  display.setTextFont(1);
  display.setTextSize(3);
  display.setTextWrap(false);

  // Two centered lines let us use larger text on these narrow displays.
  const char *line1 = "Hello,";
  const char *line2 = "XIAO";
  const int16_t lineHeight = display.fontHeight();
  const int16_t lineGap = lineHeight / 2;
  const int16_t top = (display.height() - 2 * lineHeight - lineGap) / 2;

  display.setCursor((display.width() - display.textWidth(line1)) / 2, top);
  display.print(line1);
  display.setCursor((display.width() - display.textWidth(line2)) / 2,
                    top + lineHeight + lineGap);
  display.print(line2);
}

void loop() {
  // The LCD keeps the greeting visible without redrawing.
  delay(100);
}

