/*
  XIAO nRF52840 Plus battery basic.

  Extracted from the dashboard battery code:
    READ_BAT = P0.14, active-low divider enable
    CHG      = P0.17, active-low charging status
    VBAT ADC = PIN_VBAT
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <nrf.h>
#include <nrf_gpio.h>

static constexpr uint8_t READ_BAT_P0_PIN = 14;
static constexpr uint8_t CHG_P0_PIN = 17;

#ifndef PIN_VBAT
// Some Seeed nRF52 BSP versions define PIN_VBAT. Keep a fallback for older cores.
#define PIN_VBAT 35
#endif

// The nRF52840 ADC is configured as 12-bit. The board package maps the ADC
// reference/range so the raw reading is converted with a 3.6 V full scale.
static constexpr int ADC_BITS = 12;
static constexpr int ADC_MAX = (1 << ADC_BITS) - 1;
static constexpr float ADC_FULL_SCALE_V = 3.600f;

// Board divider: R16 = 1M, R17 = 499K. Multiply ADC voltage by this ratio
// to recover the battery-side voltage.
static constexpr float BAT_DIVIDER_RATIO = (1000.0f + 499.0f) / 499.0f;

static void enableBatteryDivider() {
  // READ_BAT is active-low: drive P0.14 low to turn on the divider.
  NRF_P0->OUTCLR = (1UL << READ_BAT_P0_PIN);
  NRF_P0->DIRSET = (1UL << READ_BAT_P0_PIN);
}

static void disableBatteryDivider() {
  // Return P0.14 to high impedance so the divider does not leak current.
  NRF_P0->DIRCLR = (1UL << READ_BAT_P0_PIN);
}

static uint16_t readBatteryRaw() {
  uint32_t sum = 0;

  // Throw away the first few readings after enabling the divider. The ADC input
  // and the high-value divider need a short settling period.
  for (int i = 0; i < 6; ++i) {
    (void)analogRead(PIN_VBAT);
    delay(2);
  }

  // Average several samples to make the serial output stable.
  for (int i = 0; i < 16; ++i) {
    sum += analogRead(PIN_VBAT);
    delay(2);
  }
  return sum / 16;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  analogReadResolution(ADC_BITS);

  // CHG is an open-drain/active-low status signal, so enable an internal pull-up.
  nrf_gpio_cfg_input(CHG_P0_PIN, NRF_GPIO_PIN_PULLUP);
  disableBatteryDivider();

  Serial.println("=== Battery basic ===");
  Serial.print("PIN_VBAT=");
  Serial.println(PIN_VBAT);
}

void loop() {
  // Only enable the divider while measuring VBAT.
  enableBatteryDivider();
  delay(30);
  uint16_t raw = readBatteryRaw();
  disableBatteryDivider();

  // Convert ADC raw -> ADC pin voltage -> battery voltage.
  float vadc = ((float)raw * ADC_FULL_SCALE_V) / (float)ADC_MAX;
  float vbat = vadc * BAT_DIVIDER_RATIO;

  // CHG low means the charger IC reports charging.
  bool charging = (NRF_P0->IN & (1UL << CHG_P0_PIN)) == 0;

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print(" vadc=");
  Serial.print(vadc, 3);
  Serial.print("V vbat=");
  Serial.print(vbat, 3);
  Serial.print("V chg=");
  Serial.println(charging ? "LOW/charging" : "HIGH/not charging");

  delay(1000);
}
