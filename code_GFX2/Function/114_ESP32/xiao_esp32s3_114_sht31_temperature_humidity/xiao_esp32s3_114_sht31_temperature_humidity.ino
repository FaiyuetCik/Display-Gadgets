/*
  XIAO ESP32-S3 Plus 1.14 Inch Display
  Grove SHT31 temperature and humidity monitor

  Wiring through the 1.14-inch display board Grove I2C port:
    SHT31 SDA -> D4
    SHT31 SCL -> D5
    SHT31 VCC -> 3.3V
    SHT31 GND -> GND

  This test intentionally uses Wire.h directly instead of an SHT31 library,
  so a returned module can be checked for I2C presence, CRC and valid data.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"

Seeed_GFX display;

static constexpr uint8_t I2C_SDA_PIN = D4;
static constexpr uint8_t I2C_SCL_PIN = D5;
static constexpr uint8_t SHT31_ADDR  = 0x44;

static constexpr int LCD_W = 135;
static constexpr int LCD_H = 240;

// The panel is physically wired for BGR color order. Keep this override in
// the sketch; do not modify the Seeed_GFX2 library.
struct Config_XIAO_1inch14_LCD_ST7789_BGR {
  using Driver = Driver_ST7789;
  using Panel = Panel_TFT;
  static constexpr uint16_t width = 135;
  static constexpr uint16_t height = 240;
  static constexpr uint8_t colorDepth = 16;
  static constexpr uint8_t rgbOrder = 0x08;  // BGR
  static constexpr bool invert = true;
};

static uint8_t sht31Crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31)
                         : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static bool sht31Command(uint8_t msb, uint8_t lsb) {
  Wire.beginTransmission(SHT31_ADDR);
  Wire.write(msb);
  Wire.write(lsb);
  return Wire.endTransmission() == 0;
}

static bool readSht31(float &temperature, float &humidity, uint8_t &error) {
  error = 0;

  // High repeatability, single shot, no clock stretching.
  if (!sht31Command(0x24, 0x00)) {
    error = 1;  // command was not acknowledged
    return false;
  }
  delay(20);

  if (Wire.requestFrom((int)SHT31_ADDR, 6) != 6) {
    error = 2;  // wrong response length
    return false;
  }

  uint8_t data[6];
  for (uint8_t i = 0; i < sizeof(data); ++i) data[i] = Wire.read();

  if (sht31Crc8(data, 2) != data[2] || sht31Crc8(data + 3, 2) != data[5]) {
    error = 3;  // CRC failure: wiring or damaged/noisy module
    return false;
  }

  uint16_t rawT = ((uint16_t)data[0] << 8) | data[1];
  uint16_t rawH = ((uint16_t)data[3] << 8) | data[4];
  temperature = -45.0f + 175.0f * ((float)rawT / 65535.0f);
  humidity = 100.0f * ((float)rawH / 65535.0f);
  return true;
}

static void scanI2C() {
  Serial.println("[I2C] scan start");
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    uint8_t result = Wire.endTransmission();
    if (result == 0) {
      Serial.printf("[I2C] found 0x%02X\n", address);
      ++found;
    }
  }
  if (!found) Serial.println("[I2C] no devices found");
  Serial.println("[I2C] scan done");
}

static void drawText(const char *text, int y, uint16_t color, uint8_t font = 2) {
  display.setTextDatum(TC_DATUM);
  display.setTextColor(color, TFT_BLACK);
  display.drawString(text, LCD_W / 2, y, font);
}

static void showWaiting() {
  display.fillScreen(TFT_BLACK);
  drawText("SHT31 SENSOR", 12, TFT_WHITE, 2);
  drawText("I2C address: 0x44", 42, TFT_CYAN, 1);
  drawText("Waiting...", 100, TFT_YELLOW, 2);
}

static void showError(uint8_t error) {
  char line[32];
  snprintf(line, sizeof(line), "SHT31 ERROR %u", error);
  display.fillScreen(TFT_BLACK);
  drawText("SHT31 SENSOR", 12, TFT_WHITE, 2);
  drawText(line, 72, TFT_RED, 2);
  drawText("Check module", 112, TFT_YELLOW, 1);
  drawText("and wiring", 130, TFT_YELLOW, 1);
}

static void showReading(float temperature, float humidity) {
  char line[32];
  display.fillScreen(TFT_BLACK);
  drawText("SHT31 OK", 12, TFT_GREEN, 2);

  snprintf(line, sizeof(line), "T: %.2f C", temperature);
  drawText(line, 72, TFT_YELLOW, 2);
  // Keep a visible gap before the percent sign and identify RH explicitly.
  snprintf(line, sizeof(line), "RH: %.2f  %%", humidity);
  drawText(line, 112, TFT_CYAN, 1);
  drawText("addr 0x44", 176, TFT_DARKGREY, 1);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("=== XIAO ESP32-S3 1.14 SHT31 Temperature/Humidity ===");
  Serial.println("[PIN] SDA=D4 SCL=D5 address=0x44");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  scanI2C();

  if (!display.begin<Board_XIAO_1inch14_LCD<13, 12>,
                     Config_XIAO_1inch14_LCD_ST7789_BGR>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  showWaiting();
}

void loop() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 1000) return;
  lastRead = millis();

  float temperature = 0.0f;
  float humidity = 0.0f;
  uint8_t error = 0;
  if (readSht31(temperature, humidity, error)) {
    Serial.printf("[SHT31] OK T=%.2f C H=%.2f %%\n", temperature, humidity);
    showReading(temperature, humidity);
  } else {
    Serial.printf("[SHT31] read failed error=%u\n", error);
    showError(error);
  }
}
