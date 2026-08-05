/*
  XIAO nRF52840 Plus + MAX98357A I2S speaker demo.

  Wiring:
    D11 -> MAX98357A DIN
    D12 -> MAX98357A BCLK
    D13 -> MAX98357A LRC / WS
    3V3 -> VIN
    GND -> GND

  Press USR1 to play a low-volume ~441 Hz test tone for 1 second.
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <nrf.h>

static constexpr uint8_t I2S_DIN_PIN = D11;
static constexpr uint8_t I2S_BCLK_PIN = D12;
static constexpr uint8_t I2S_LRCLK_PIN = D13;
static constexpr uint8_t USR1_PIN = D19;

// 32 MHz / 63 / 32 = ~15873 Hz LRCLK. 18 samples per half-cycle gives ~441 Hz.
static constexpr size_t SAMPLE_FRAMES = 128;
static constexpr int16_t TONE_AMPLITUDE = 800;
static constexpr uint32_t PLAY_DURATION_MS = 1000;
static uint32_t txBuffer[SAMPLE_FRAMES];

static bool isPlaying = false;
static bool lastUsr1State = HIGH;
static uint32_t playStartedMs = 0;

static void fillToneBuffer() {
  for (size_t frame = 0; frame < SAMPLE_FRAMES; ++frame) {
    int16_t sample = ((frame / 18) & 1) ? TONE_AMPLITUDE : -TONE_AMPLITUDE;
    txBuffer[frame] = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;
  }
}

static void initI2S() {
  NRF_I2S->TASKS_STOP = 1;
  NRF_I2S->ENABLE = 0;

  NRF_I2S->EVENTS_RXPTRUPD = 0;
  NRF_I2S->EVENTS_TXPTRUPD = 0;
  NRF_I2S->EVENTS_STOPPED = 0;

  NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_Master;
  NRF_I2S->CONFIG.RXEN = I2S_CONFIG_RXEN_RXEN_Disabled;
  NRF_I2S->CONFIG.TXEN = I2S_CONFIG_TXEN_TXEN_Enabled;
  // MAX98357A does not need an MCLK pin, but nRF I2S master still needs the
  // internal master clock generator to derive BCLK and LRCLK.
  NRF_I2S->CONFIG.MCKEN = I2S_CONFIG_MCKEN_MCKEN_Enabled;
  NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV63;
  NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_32X;
  NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
  NRF_I2S->CONFIG.ALIGN = I2S_CONFIG_ALIGN_ALIGN_Left;
  NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S;
  NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Stereo;

  NRF_I2S->PSEL.MCK = 0xFFFFFFFF;
  NRF_I2S->PSEL.SCK = g_ADigitalPinMap[I2S_BCLK_PIN];
  NRF_I2S->PSEL.LRCK = g_ADigitalPinMap[I2S_LRCLK_PIN];
  NRF_I2S->PSEL.SDIN = 0xFFFFFFFF;
  NRF_I2S->PSEL.SDOUT = g_ADigitalPinMap[I2S_DIN_PIN];

  NRF_I2S->TXD.PTR = (uint32_t)txBuffer;
  NRF_I2S->RXD.PTR = 0;
  NRF_I2S->RXTXD.MAXCNT = SAMPLE_FRAMES;

  NRF_I2S->ENABLE = 1;
  NRF_I2S->TASKS_START = 1;
}

static void serviceI2S() {
  if (NRF_I2S->EVENTS_TXPTRUPD) {
    NRF_I2S->EVENTS_TXPTRUPD = 0;
    NRF_I2S->TXD.PTR = (uint32_t)txBuffer;
  }
}

static void stopI2S() {
  NRF_I2S->TASKS_STOP = 1;

  uint32_t startMs = millis();
  while (!NRF_I2S->EVENTS_STOPPED && millis() - startMs < 50) {
    delay(1);
  }

  NRF_I2S->EVENTS_STOPPED = 0;
  NRF_I2S->ENABLE = 0;
  isPlaying = false;
}

static void startI2SPlayback() {
  initI2S();
  isPlaying = true;
  playStartedMs = millis();
}

static void handleUsr1Button() {
  bool usr1State = digitalRead(USR1_PIN);

  if (lastUsr1State == HIGH && usr1State == LOW && !isPlaying) {
    delay(20);
    if (digitalRead(USR1_PIN) == LOW) {
      startI2SPlayback();
    }
  }

  lastUsr1State = usr1State;
}

void setup() {
  pinMode(USR1_PIN, INPUT_PULLUP);
  fillToneBuffer();
}

void loop() {
  handleUsr1Button();

  if (!isPlaying) return;

  serviceI2S();

  if (millis() - playStartedMs >= PLAY_DURATION_MS) {
    stopI2S();
  }
}
