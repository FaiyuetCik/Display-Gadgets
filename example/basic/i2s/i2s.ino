/*
  XIAO nRF52840 Plus I2S register bring-up.

  The 147_nRF52840 application code uses PDM for the onboard microphone.
  This file is a minimal nRF52840 I2S clock/data startup skeleton for boards
  or add-ons that expose I2S pins.
*/

#include <Arduino.h>
#include <nrf.h>

static constexpr uint8_t I2S_SCK_PIN = D0;
static constexpr uint8_t I2S_LRCK_PIN = D1;
static constexpr uint8_t I2S_SDOUT_PIN = D2;

static int16_t txBuffer[64];

static void initI2S() {
  for (size_t i = 0; i < sizeof(txBuffer) / sizeof(txBuffer[0]); ++i) {
    txBuffer[i] = (i & 1) ? 12000 : -12000;
  }

  NRF_I2S->ENABLE = 0;
  NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_Master;
  NRF_I2S->CONFIG.RXEN = I2S_CONFIG_RXEN_RXEN_Disabled;
  NRF_I2S->CONFIG.TXEN = I2S_CONFIG_TXEN_TXEN_Enabled;
  NRF_I2S->CONFIG.MCKEN = I2S_CONFIG_MCKEN_MCKEN_Disabled;
  NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_32X;
  NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
  NRF_I2S->CONFIG.ALIGN = I2S_CONFIG_ALIGN_ALIGN_Left;
  NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S;
  NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Left;

  NRF_I2S->PSEL.SCK = g_ADigitalPinMap[I2S_SCK_PIN];
  NRF_I2S->PSEL.LRCK = g_ADigitalPinMap[I2S_LRCK_PIN];
  NRF_I2S->PSEL.SDOUT = g_ADigitalPinMap[I2S_SDOUT_PIN];
  NRF_I2S->PSEL.SDIN = 0xFFFFFFFF;
  NRF_I2S->PSEL.MCK = 0xFFFFFFFF;

  NRF_I2S->TXD.PTR = (uint32_t)txBuffer;
  NRF_I2S->RXD.PTR = 0;
  NRF_I2S->TXD.MAXCNT = sizeof(txBuffer) / sizeof(uint32_t);
  NRF_I2S->RXD.MAXCNT = 0;

  NRF_I2S->ENABLE = 1;
  NRF_I2S->TASKS_START = 1;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("=== I2S basic ===");
  Serial.println("Starting nRF52840 I2S TX skeleton");
  Serial.println("Default pins: SCK=D0 LRCK=D1 SDOUT=D2");

  initI2S();
  Serial.println("[I2S] started");
}

void loop() {
  if (NRF_I2S->EVENTS_TXPTRUPD) {
    NRF_I2S->EVENTS_TXPTRUPD = 0;
    NRF_I2S->TXD.PTR = (uint32_t)txBuffer;
  }
  delay(10);
}
