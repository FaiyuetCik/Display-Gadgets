/*
  XIAO nRF52840 Plus battery basic.

  Extracted from the dashboard battery code:
    READ_BAT = P0.14, active-low divider enable
    CHG      = P0.17, active-low charging status
    VBAT ADC = PIN_VBAT
*/

#include <Arduino.h>
#include <nrf.h>
#include <nrf_gpio.h>

static constexpr uint8_t READ_BAT_P0_PIN = 14;
static constexpr uint8_t CHG_P0_PIN = 17;

#ifndef PIN_VBAT
#define PIN_VBAT 35
#endif

static constexpr int ADC_BITS = 12;
static constexpr int ADC_MAX = (1 << ADC_BITS) - 1;
static constexpr float ADC_FULL_SCALE_V = 3.600f;
static constexpr float BAT_DIVIDER_RATIO = (1000.0f + 499.0f) / 499.0f;

static void enableBatteryDivider() {
  NRF_P0->OUTCLR = (1UL << READ_BAT_P0_PIN);
  NRF_P0->DIRSET = (1UL << READ_BAT_P0_PIN);
}

static void disableBatteryDivider() {
  NRF_P0->DIRCLR = (1UL << READ_BAT_P0_PIN);
}

static uint16_t readBatteryRaw() {
  uint32_t sum = 0;
  for (int i = 0; i < 6; ++i) {
    (void)analogRead(PIN_VBAT);
    delay(2);
  }
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
  nrf_gpio_cfg_input(CHG_P0_PIN, NRF_GPIO_PIN_PULLUP);
  disableBatteryDivider();

  Serial.println("=== Battery basic ===");
  Serial.print("PIN_VBAT=");
  Serial.println(PIN_VBAT);
}

void loop() {
  enableBatteryDivider();
  delay(30);
  uint16_t raw = readBatteryRaw();
  disableBatteryDivider();

  float vadc = ((float)raw * ADC_FULL_SCALE_V) / (float)ADC_MAX;
  float vbat = vadc * BAT_DIVIDER_RATIO;
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
