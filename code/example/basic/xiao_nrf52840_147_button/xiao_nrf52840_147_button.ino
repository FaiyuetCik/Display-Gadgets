/*
  XIAO nRF52840 Plus + 1.47 Inch Touch Display
  Basic USR button test extracted from the dashboard pin map.

  USR1 = D19, active low
  USR2 = D15, active low
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

static constexpr uint8_t BTN_A_PIN = D19;
static constexpr uint8_t BTN_B_PIN = D15;
static constexpr uint8_t LCD_BL_PIN = D18;

// Store the previous sampled state so we can detect only the press edge.
bool lastA = false;
bool lastB = false;
uint8_t brightness = 255;

static bool pressed(uint8_t pin) {
  // Both user buttons are wired active-low.
  return digitalRead(pin) == LOW;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  // Pull-ups keep the input HIGH while the button is released.
  pinMode(BTN_A_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);

  // Reuse the LCD backlight as an easy visible output for USR1.
  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, brightness);

  Serial.println("=== Button basic ===");
  Serial.println("USR1(D19): cycle backlight");
  Serial.println("USR2(D15): print press state");
}

void loop() {
  // Poll the two buttons. This is enough for a simple basic test.
  bool nowA = pressed(BTN_A_PIN);
  bool nowB = pressed(BTN_B_PIN);

  if (nowA && !lastA) {
    // USR1 cycles through three brightness levels on each new press.
    brightness = (brightness == 255) ? 128 : (brightness == 128) ? 32 : 255;
    analogWrite(LCD_BL_PIN, brightness);
    Serial.print("USR1 pressed, backlight=");
    Serial.println(brightness);
  }

  if (nowB && !lastB) {
    Serial.println("USR2 pressed");
  }

  if (nowA != lastA || nowB != lastB) {
    // Print state changes only, so the serial monitor stays readable.
    Serial.print("USR1=");
    Serial.print(nowA ? "PRESSED" : "released");
    Serial.print(" USR2=");
    Serial.println(nowB ? "PRESSED" : "released");
  }

  // Save current state for the next edge-detection pass.
  lastA = nowA;
  lastB = nowB;
  delay(20);
}
