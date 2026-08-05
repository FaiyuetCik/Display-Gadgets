/*
  XIAO ESP32-S3 Plus + 1.14 Inch Display Grove I2C scanner.

  SDA=D4, SCL=D5.
*/

#include <Arduino.h>
#include <Wire.h>

static constexpr uint8_t I2C_SDA_PIN = D4;
static constexpr uint8_t I2C_SCL_PIN = D5;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("=== 1.14 I2C scan basic ===");
  Serial.println("SDA=D4, SCL=D5");
}

void loop() {
  uint8_t count = 0;

  Serial.print("[I2C] scan:");
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(" 0x");
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      ++count;
    }
  }

  if (!count) Serial.print(" none");
  Serial.println();

  delay(2000);
}
