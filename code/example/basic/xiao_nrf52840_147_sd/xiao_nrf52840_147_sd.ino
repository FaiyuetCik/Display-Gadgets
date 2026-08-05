/*
  XIAO nRF52840 Plus SD basic.

  Extracted from the SD diagnostic/BMP examples.
  SD shares SPI with LCD hardware pins; this sketch only mounts SD.

  Required library:
    - SdFat
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat.h>

static constexpr uint8_t LCD_CS_PIN = D2;
static constexpr uint8_t SD_CS_PIN = D6;

SdFat SD;

// Remember the first SPI frequency that successfully mounted the card.
uint32_t mountedFreq = 0;

static bool beginSd() {
  // Try fast first, then fall back to slower clocks for marginal cards/wiring.
  const uint32_t freqs[] = {8000000, 4000000, 1000000, 400000};

  // LCD and SD share SPI. Keep LCD deselected while talking to SD.
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  SPI.begin();

  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
    // SHARED_SPI tells SdFat that other devices may share this SPI bus.
    SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, freqs[i], &SPI);
    Serial.print("[SD] try ");
    Serial.print(freqs[i]);
    Serial.print(" Hz ... ");
    if (SD.begin(cfg)) {
      mountedFreq = freqs[i];
      Serial.println("OK");
      return true;
    }
    Serial.println("FAIL");
    delay(100);
  }

  return false;
}

static void listRoot() {
  // Print the SD card root directory as a simple read/write sanity check.
  File32 root;
  if (!root.open("/")) {
    Serial.println("[SD] root open failed");
    return;
  }

  File32 entry;
  while (entry.openNext(&root, O_RDONLY)) {
    char name[64];
    entry.getName(name, sizeof(name));
    Serial.print(entry.isDir() ? "DIR  " : "FILE ");
    Serial.println(name);
    entry.close();
  }
  root.close();
}

void setup() {
  Serial.begin(115200);

  // With TinyUSB, wait for the serial monitor before printing diagnostics.
  while (!Serial) {
    delay(10);
  }
  delay(300);

  Serial.println("=== SD basic ===");

  if (!beginSd()) {
    Serial.println("[SD] mount failed");
    while (1) delay(1000);
  }

  Serial.print("[SD] mounted @ ");
  Serial.print(mountedFreq);
  Serial.println(" Hz");
  listRoot();
}

void loop() {
  delay(1000);
}
