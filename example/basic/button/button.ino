/*
  XIAO nRF52840 Plus + 1.47 Inch Touch Display
  Basic USR button test extracted from the dashboard pin map.

  USR1 = D19, active low
  USR2 = D15, active low
*/

#include <Arduino.h>

static constexpr uint8_t BTN_A_PIN = D19;
static constexpr uint8_t BTN_B_PIN = D15;
static constexpr uint8_t LCD_BL_PIN = D18;

bool lastA = false;
bool lastB = false;
uint8_t brightness = 255;

static bool pressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(BTN_A_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);
  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, brightness);

  Serial.println("=== Button basic ===");
  Serial.println("USR1(D19): cycle backlight");
  Serial.println("USR2(D15): print press state");
}

void loop() {
  bool nowA = pressed(BTN_A_PIN);
  bool nowB = pressed(BTN_B_PIN);

  if (nowA && !lastA) {
    brightness = (brightness == 255) ? 128 : (brightness == 128) ? 32 : 255;
    analogWrite(LCD_BL_PIN, brightness);
    Serial.print("USR1 pressed, backlight=");
    Serial.println(brightness);
  }

  if (nowB && !lastB) {
    Serial.println("USR2 pressed");
  }

  if (nowA != lastA || nowB != lastB) {
    Serial.print("USR1=");
    Serial.print(nowA ? "PRESSED" : "released");
    Serial.print(" USR2=");
    Serial.println(nowB ? "PRESSED" : "released");
  }

  lastA = nowA;
  lastB = nowB;
  delay(20);
}
