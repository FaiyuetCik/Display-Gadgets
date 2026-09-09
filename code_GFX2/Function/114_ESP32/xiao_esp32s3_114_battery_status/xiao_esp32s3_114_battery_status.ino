/*
  XIAO ESP32-S3 Plus 1.14-inch Display - Voltage Sense Demo
  Library: Seeed_GFX2
  Matches the active voltage UI in 0715_DashBoard_114_ESP32:
  yellow D16/Calc readings, without percentage or power-state inference.
  Calc is D16 voltage multiplied by the hardware divider ratio.
  USB-only readings are displayed too; they do not prove battery presence.
*/
#include <Arduino.h>
#include <math.h>
#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"

Seeed_GFX display;
// Match the local BGR correction used by the other 1.14-inch demos.
struct Config_XIAO_1inch14_LCD_ST7789_BGR {
  using Driver = Driver_ST7789;
  using Panel = Panel_TFT;
  static constexpr uint16_t width = 135;
  static constexpr uint16_t height = 240;
  static constexpr uint8_t colorDepth = 16;
  static constexpr uint8_t rgbOrder = 0x08;
  static constexpr bool invert = true;
};
static constexpr int8_t LCD_RST_PIN = 13;
static constexpr int8_t LCD_BL_PIN = 12;
static constexpr int BAT_ADC_PIN = D16;
static constexpr float DIVIDER_RATIO = (316.0f + 160.0f) / 160.0f;
static constexpr uint8_t SAMPLE_COUNT = 12;
static constexpr uint16_t SAMPLE_DELAY_US = 700;
static constexpr uint32_t UI_MS = 1000;
static constexpr float D16_DELTA_V = 0.02f;
static constexpr float CALC_DELTA_V = 0.05f;

static bool g_displayReady = false;
static float g_shownD16 = -1.0f;
static float g_shownCalc = -1.0f;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
  if (!display.begin<Board_XIAO_1inch14_LCD<LCD_RST_PIN, LCD_BL_PIN>, Config_XIAO_1inch14_LCD_ST7789_BGR>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  g_displayReady = true;
  display.fillScreen(TFT_BLACK);
  display.setTextFont(1);
  display.setTextWrap(false);
}

// All samples contribute, as in the reference Dashboard. No cell-voltage
// thresholds, artificial voltage compensation, or retained battery state.
static float readD16Voltage() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
    sum += analogReadMilliVolts(BAT_ADC_PIN);
    delayMicroseconds(SAMPLE_DELAY_US);
  }
  return (sum / SAMPLE_COUNT) / 1000.0f;
}

static void drawVoltage(float d16, float calc) {
  char line1[32], line2[32];
  snprintf(line1, sizeof(line1), "D16:%.2fV", d16);
  snprintf(line2, sizeof(line2), "Calc:%.2fV", calc);
  // Compact labels fit the 135-pixel panel at fixed 32-pixel font height.
  // Font 2 widths: D16:3.87V = 124px, Calc:3.87V = 128px.
  display.setTextFont(2);
  display.setTextSize(2);
  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  int height = display.fontHeight();
  int gap = 12;
  int y = (display.height() - 2 * height - gap) / 2;
  // Clear the full canvas so changes in value or font scale leave no residue.
  display.fillScreen(TFT_BLACK);
  display.setCursor((display.width() - display.textWidth(line1)) / 2, y);
  display.print(line1);
  display.setCursor((display.width() - display.textWidth(line2)) / 2, y + height + gap);
  display.print(line2);
}

void loop() {
  if (!g_displayReady) { delay(UI_MS); return; }
  float d16 = readD16Voltage();
  float calc = d16 * DIVIDER_RATIO;
  float roundedD16 = roundf(d16 * 100.0f) / 100.0f;
  float roundedCalc = roundf(calc * 100.0f) / 100.0f;
  if (g_shownD16 < 0.0f ||
      fabsf(roundedD16 - g_shownD16) >= D16_DELTA_V ||
      fabsf(roundedCalc - g_shownCalc) >= CALC_DELTA_V) {
    drawVoltage(roundedD16, roundedCalc);
    g_shownD16 = roundedD16;
    g_shownCalc = roundedCalc;
  }
  Serial.print("D16 "); Serial.print(d16, 3);
  Serial.print("V | Calc "); Serial.print(calc, 3);
  Serial.println("V");
  delay(UI_MS);
}
