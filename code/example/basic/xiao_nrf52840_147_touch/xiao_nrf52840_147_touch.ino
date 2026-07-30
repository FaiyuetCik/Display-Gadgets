/*
  XIAO nRF52840 Plus touch basic.

  Extracted from CAI_xiao_nrf52840_demo_src.
  Touch controller AXS5106L uses I2C, RST=D17, INT=D7.
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include "axs5106l_device.h"

static constexpr uint8_t TOUCH_RST_PIN = D17;
static constexpr uint8_t TOUCH_INT_PIN = D7;

// Shared result structure filled by get_touch_data().
touch_data_t touchData;

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("=== Touch basic ===");

  // Touch controller is on the board I2C bus.
  Wire.begin();
  touch_init(&Wire, TOUCH_RST_PIN, TOUCH_INT_PIN);
}

void loop() {
  if (get_touch_data(&touchData)) {
    // Print the first touch point. This basic test ignores multi-touch details.
    Serial.print("touches=");
    Serial.print(touchData.touch_num);
    Serial.print(" x=");
    Serial.print(touchData.coords[0].x);
    Serial.print(" y=");
    Serial.println(touchData.coords[0].y);
  }
  delay(50);
}
