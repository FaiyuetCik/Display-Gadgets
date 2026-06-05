/*
  XIAO nRF52840 Plus PDM microphone basic.

  Extracted from the dashboard/audio examples.
  PDM pins are set as CLK=D0, DIN=D1 in this BSP.
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <PDM.h>

static constexpr int SAMPLE_RATE_HZ = 16000;
static constexpr int CHANNELS = 1;
static constexpr int PDM_GAIN = 30;

// PDM callback writes into this buffer when a chunk of samples is ready.
int16_t pdmBuffer[256];

// Shared between the PDM callback and loop(), so mark as volatile.
volatile int peak = 0;
volatile bool hasSamples = false;

static void onPDMdata() {
  // This callback runs when the PDM driver has bytes available.
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;
  if (bytesAvailable > (int)sizeof(pdmBuffer)) bytesAvailable = sizeof(pdmBuffer);

  // Read signed 16-bit PCM samples from the PDM driver.
  int bytesRead = PDM.read(pdmBuffer, bytesAvailable);
  int samples = bytesRead / 2;
  int localPeak = 0;

  for (int i = 0; i < samples; ++i) {
    int v = abs((int)pdmBuffer[i]);
    if (v > localPeak) localPeak = v;
  }

  // Report peak amplitude as a simple microphone activity indicator.
  peak = localPeak;
  hasSamples = true;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("=== PDM mic basic ===");

  // Seeed nRF52 BSP mapping used by the existing 147_nRF52840 demos.
  PDM.setPins(D1, D0, -1);
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(sizeof(pdmBuffer));
  PDM.setGain(PDM_GAIN);

  if (!PDM.begin(CHANNELS, SAMPLE_RATE_HZ)) {
    Serial.println("[MIC] PDM.begin failed");
    while (1) delay(1000);
  }

  Serial.println("[MIC] started");
}

void loop() {
  if (hasSamples) {
    // Copy and clear the callback result atomically.
    noInterrupts();
    int p = peak;
    hasSamples = false;
    interrupts();

    Serial.print("peak=");
    Serial.println(p);
  }
  delay(100);
}
