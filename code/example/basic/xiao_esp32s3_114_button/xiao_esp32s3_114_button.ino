/*
  XIAO ESP32-S3 Plus + 1.14 Inch Display button basic.

  USR1=D6, USR2=D7, USR3=D19. Buttons are active-low.
*/

#include <Arduino.h>

static constexpr uint8_t USR1_PIN = D6;
static constexpr uint8_t USR2_PIN = D7;
static constexpr uint8_t USR3_PIN = D19;

static int lastUsr1 = HIGH;
static int lastUsr2 = HIGH;
static int lastUsr3 = HIGH;
static uint32_t lastPrintMs = 0;

static void printButtonState(const char *tag, int raw) {
  Serial.print(tag);
  Serial.print('=');
  Serial.print(raw == LOW ? "PRESSED" : "released");
  Serial.print(' ');
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
  pinMode(USR3_PIN, INPUT_PULLUP);

  Serial.println();
  Serial.println("=== 1.14 button basic ===");
  Serial.println("USR1=D6, USR2=D7, USR3=D19, active-low");
}

void loop() {
  int usr1 = digitalRead(USR1_PIN);
  int usr2 = digitalRead(USR2_PIN);
  int usr3 = digitalRead(USR3_PIN);
  bool changed = (usr1 != lastUsr1) || (usr2 != lastUsr2) || (usr3 != lastUsr3);

  uint32_t now = millis();
  if (changed || now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    lastUsr1 = usr1;
    lastUsr2 = usr2;
    lastUsr3 = usr3;

    Serial.print("[BUTTON] ");
    printButtonState("USR1", usr1);
    printButtonState("USR2", usr2);
    printButtonState("USR3", usr3);
    Serial.println();
  }

  delay(20);
}
