/*
  XIAO ESP32-S3 Plus + 1.14 Inch Display battery ADC basic.

  Schematic:
    VBAT -- R13 316k -- VBAT_ADC(D16) -- R14 160k -- GND

  Divider ratio:
    (316k + 160k) / 160k = 2.975
*/

#include <Arduino.h>

static constexpr uint8_t BAT_ADC_PIN = D16;
static constexpr float BAT_DIVIDER_RATIO = (316.0f + 160.0f) / 160.0f;
static constexpr uint8_t SAMPLE_COUNT = 16;

static uint32_t readBatteryMilliVolts() {
  uint32_t mvSum = 0;

  for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
#if defined(ARDUINO_ARCH_ESP32)
    mvSum += analogReadMilliVolts(BAT_ADC_PIN);
#else
    mvSum += (uint32_t)analogRead(BAT_ADC_PIN) * 3300UL / 4095UL;
#endif
    delay(2);
  }

  uint32_t adcMv = mvSum / SAMPLE_COUNT;
  return (uint32_t)((float)adcMv * BAT_DIVIDER_RATIO);
}

void setup() {
  Serial.begin(115200);
  delay(500);

#if defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(12);
  // 11 dB attenuation gives the ESP32 ADC its widest input range.
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
#endif

  Serial.println();
  Serial.println("=== 1.14 battery basic ===");
  Serial.println("ADC pin: D16");
  Serial.println("Divider: 316k / 160k");
}

void loop() {
  uint32_t batteryMv = readBatteryMilliVolts();

  Serial.print("battery=");
  Serial.print(batteryMv);
  Serial.print("mV ");
  Serial.print((float)batteryMv / 1000.0f, 3);
  Serial.println("V");

  delay(1000);
}
