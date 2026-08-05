/*
  XIAO nRF52840 Plus touch interrupt basic.

  Touch INT is connected to D7. The interrupt is used as a wake/latch signal;
  touch coordinates are still read over I2C.
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include "axs5106l_device.h"

static constexpr uint8_t TOUCH_RST_PIN = D17;
static constexpr uint8_t TOUCH_INT_PIN = D7;

// Latches set by the D7 interrupt handler.
volatile bool touchIrq = false;
volatile uint32_t touchIrqCount = 0;
touch_data_t touchData;

void touchIsr() {
  // Keep the ISR short; I2C is handled later in loop().
  touchIrq = true;
  touchIrqCount++;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("=== Touch interrupt basic ===");
  Wire.begin();
  touch_init(&Wire, TOUCH_RST_PIN, TOUCH_INT_PIN);

  // AXS5106L INT is active-low in this basic interrupt example.
  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TOUCH_INT_PIN), touchIsr, FALLING);
}

void loop() {
  if (touchIrq) {
    // Copy the interrupt count atomically before printing.
    noInterrupts();
    touchIrq = false;
    uint32_t count = touchIrqCount;
    interrupts();

    Serial.print("touch irq count=");
    Serial.println(count);
  }

  if (get_touch_data(&touchData)) {
    // Coordinates are still read through I2C after the interrupt latch fires.
    Serial.print("touch x=");
    Serial.print(touchData.coords[0].x);
    Serial.print(" y=");
    Serial.println(touchData.coords[0].y);
  }

  delay(20);
}
