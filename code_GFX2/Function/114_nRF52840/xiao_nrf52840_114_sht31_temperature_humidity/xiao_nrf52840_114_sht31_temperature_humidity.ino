/*
  XIAO nRF52840 Plus 1.14 Inch Display
  Grove SHT31 temperature and humidity monitor

  Wiring through the 1.14-inch display board Grove I2C port:
    SHT31 SDA -> D4
    SHT31 SCL -> D5
    SHT31 VCC -> 3.3V
    SHT31 GND -> GND

  Uses Wire.h directly so returned SHT31 modules can be checked for:
    - I2C presence at 0x44
    - command acknowledgement
    - valid response length
    - CRC correctness
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>
#include <Wire.h>

static constexpr int8_t LCD_RST_PIN = 38;
static constexpr int8_t LCD_BL_PIN  = 37;
static constexpr uint8_t SHT31_ADDR = 0x44;
static constexpr int LCD_W = 135;

Seeed_GFX display;

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

static bool sendSht31Command(uint8_t msb, uint8_t lsb) {
  Wire.beginTransmission(SHT31_ADDR);
  Wire.write(msb);
  Wire.write(lsb);
  return Wire.endTransmission() == 0;
}

static bool readSht31(float &temperature, float &humidity, uint8_t &error) {
  error = 0;

  // Single-shot, high repeatability, no clock stretching.
  if (!sendSht31Command(0x24, 0x00)) {
    error = 1;
    return false;
  }
  delay(20);

  if (Wire.requestFrom((int)SHT31_ADDR, 6) != 6) {
    error = 2;
    return false;
  }

  uint8_t data[6];
  for (uint8_t i = 0; i < 6; ++i) data[i] = Wire.read();

  if (sht31Crc8(data, 2) != data[2] ||
      sht31Crc8(data + 3, 2) != data[5]) {
    error = 3;
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
    if (Wire.endTransmission() == 0) {
      Serial.print("[I2C] found 0x");
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
      ++found;
    }
  }
  if (!found) Serial.println("[I2C] no devices found");
  Serial.println("[I2C] scan done");
}

static void drawCentered(const char *text, int y, uint16_t color, uint8_t font) {
  display.setTextDatum(TC_DATUM);
  display.setTextColor(color, TFT_BLACK);
  display.drawString(text, LCD_W / 2, y, font);
}

static void showError(uint8_t error) {
  char line[24];
  snprintf(line, sizeof(line), "SHT31 ERROR %u", error);
  display.fillScreen(TFT_BLACK);
  drawCentered("SHT31 SENSOR", 14, TFT_WHITE, 2);
  drawCentered(line, 76, TFT_RED, 2);
  drawCentered("Check module", 116, TFT_YELLOW, 1);
  drawCentered("and wiring", 134, TFT_YELLOW, 1);
}

static void showReading(float temperature, float humidity) {
  char line[32];
  display.fillScreen(TFT_BLACK);
  drawCentered("SHT31 OK", 14, TFT_GREEN, 2);

  snprintf(line, sizeof(line), "T: %.2f C", temperature);
  drawCentered(line, 72, TFT_YELLOW, 2);

  // Keep a clear gap before the percent sign.
  snprintf(line, sizeof(line), "RH: %.2f  %%", humidity);
  drawCentered(line, 112, TFT_CYAN, 1);
  drawCentered("I2C 0x44", 176, TFT_DARKGREY, 1);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("=== XIAO nRF52840 1.14 SHT31 Temperature/Humidity ===");
  Serial.println("[PIN] SDA=D4 SCL=D5 address=0x44");

  // Seeed nRF52 Wire implementation uses the default D4/D5 pins.
  Wire.begin();
  Wire.setClock(100000);
  scanI2C();

  if (!display.begin<Board_XIAO_1inch14_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_XIAO_1inch14_LCD_ST7789_BGR>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(TFT_BLACK);
}

void loop() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 1000) return;
  lastRead = millis();

  float temperature = 0.0f;
  float humidity = 0.0f;
  uint8_t error = 0;
  if (readSht31(temperature, humidity, error)) {
    Serial.print("[SHT31] OK T=");
    Serial.print(temperature, 2);
    Serial.print(" C H=");
    Serial.print(humidity, 2);
    Serial.println(" %");
    showReading(temperature, humidity);
  } else {
    Serial.print("[SHT31] read failed error=");
    Serial.println(error);
    showError(error);
  }
}
