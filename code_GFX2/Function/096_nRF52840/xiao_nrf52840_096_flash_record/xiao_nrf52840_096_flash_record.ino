/*
  XIAO nRF52840 Plus + 0.96 Inch Display Flash recorder.

  Operation:
    USR1 records ~0.7 seconds from the onboard PDM microphone.
    The latest recording is saved to the internal Flash filesystem
    (InternalFS) as /REC_RAW.WAV.
    USR2 plays the saved WAV through a MAX98357A on D11/D12/D13.

  nRF52840 note:
    The internal filesystem is only ~28 KB, so this demo records at
    16 kHz and fits a ~0.7 s clip (11200 samples = ~22 KB PCM).
    The ESP32 versions record 5 s at 16 kHz because LittleFS is much
    larger there.

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  Arduino_SWSPI + Arduino_ST7789 bus/panel setup, manual pin init (CS/DC/
  SCK/MOSI/RST/BL) and gfx->begin()/invertDisplay(true).
  Board_XIAO_0inch96_LCD<RST=38,BL=37> + Config_Seeed_0inch96_LCD_ST7789 bake
  80x160 BGR rot2 invert=false. PDM capture (ISR), InternalFS save/load and
  NRF_I2S playback are unchanged.

  Required display library:
    - Seeed_GFX2

  Audio and filesystem libraries (bundled with the Seeed nRF52 core):
    - Adafruit_TinyUSB
    - PDM
    - Adafruit_LittleFS + InternalFileSystem
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <PDM.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <nrf.h>

using namespace Adafruit_LittleFS_Namespace;

static constexpr uint8_t PDM_CLK_PIN   = D0;
static constexpr uint8_t PDM_DATA_PIN  = D1;
static constexpr uint8_t USR1_PIN      = D6;
static constexpr uint8_t USR2_PIN      = D7;
static constexpr uint8_t I2S_DOUT_PIN  = D11;
static constexpr uint8_t I2S_BCLK_PIN  = D12;
static constexpr uint8_t I2S_LRCLK_PIN = D13;
static constexpr int8_t  LCD_RST_PIN   = 38;  // XIAO nRF52840 Plus (raw GPIO)
static constexpr int8_t  LCD_BL_PIN    = 37;

static constexpr int LCD_W = 80;
static constexpr int LCD_H = 160;

// --- Audio config ----------------------------------------------------

static constexpr uint32_t SAMPLE_RATE_HZ = 16000;
static constexpr uint16_t CHANNELS       = 1;
static constexpr int      PDM_GAIN       = 30;

// 0.7 s @ 16 kHz -> 11200 samples -> 22400 bytes PCM. Fits InternalFS (~28 KB).
static constexpr uint32_t RECORD_SAMPLES = 11200;
static constexpr uint32_t RECORD_BYTES   = RECORD_SAMPLES * sizeof(int16_t);

static constexpr size_t   PDM_CHUNK_SAMPLES    = 256;
static constexpr size_t   I2S_PLAYBACK_FRAMES  = 256;
static constexpr float    PLAYBACK_GAIN        = 0.75f;

static const char WAV_PATH[] = "/REC_RAW.WAV";

Seeed_GFX display;

static constexpr uint16_t C_BLACK     = 0x0000;
static constexpr uint16_t C_RED       = 0xF800;
static constexpr uint16_t C_GREEN     = 0x07E0;
static constexpr uint16_t C_CYAN      = 0x07FF;
static constexpr uint16_t C_YELLOW    = 0xFFE0;
static constexpr uint16_t C_WHITE     = 0xFFFF;
static constexpr uint16_t C_LIGHTGREY = 0xD69A;
static constexpr uint16_t C_DARKGREY  = 0x39E7;

static int16_t recordBuffer[RECORD_SAMPLES];
static int16_t pdmChunk[PDM_CHUNK_SAMPLES];
static uint32_t i2sBufferA[I2S_PLAYBACK_FRAMES];
static uint32_t i2sBufferB[I2S_PLAYBACK_FRAMES];

volatile uint32_t capturedSamples = 0;
volatile bool     captureEnabled  = false;
volatile bool     captureComplete = false;

static bool hasRecording = false;
static bool lastUsr1 = HIGH;
static bool lastUsr2 = HIGH;

struct WavHeader {
  char riff[4];
  uint32_t chunkSize;
  char wave[4];
  char fmt[4];
  uint32_t subchunk1Size;
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;
  char data[4];
  uint32_t subchunk2Size;
};

static WavHeader makeWavHeader(uint32_t dataBytes) {
  WavHeader h = {
    {'R', 'I', 'F', 'F'}, 36 + dataBytes, {'W', 'A', 'V', 'E'},
    {'f', 'm', 't', ' '}, 16, 1, 1, SAMPLE_RATE_HZ,
    SAMPLE_RATE_HZ * 2, 2, 16, {'d', 'a', 't', 'a'}, dataBytes
  };
  return h;
}

// --- Display ---------------------------------------------------------

// Compact 80x160 UI.
static void drawCentered(const char *text, int16_t y, uint8_t size) {
  display.setTextSize(size);
  int16_t x = (LCD_W - (int16_t)strlen(text) * 6 * size) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.print(text);
}

static void screen(const char *title, const char *line1, const char *line2, uint16_t color) {
  display.fillScreen(C_BLACK);
  display.setTextColor(color, C_BLACK);
  drawCentered(title, 6, 1);
  display.drawFastHLine(4, 20, LCD_W - 8, color);
  display.setTextSize(1);
  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(4, 40);
  display.print(line1);
  display.setTextColor(C_LIGHTGREY, C_BLACK);
  display.setCursor(4, 58);
  display.print(line2);
}

static void readyScreen() {
  screen("Recorder", "USR1: record", hasRecording ? "USR2: play" : "No recording", C_CYAN);
}

static void progressScreen(uint32_t samples) {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 150 && samples < RECORD_SAMPLES) return;
  lastMs = millis();

  uint32_t percent = min((uint32_t)100, samples * 100 / RECORD_SAMPLES);

  display.fillRect(0, 22, LCD_W, LCD_H - 22, C_BLACK);
  display.setTextColor(C_RED, C_BLACK);
  drawCentered("REC", 40, 2);

  char text[24];
  snprintf(text, sizeof(text), "%lu%%", (unsigned long)percent);
  display.setTextColor(C_WHITE, C_BLACK);
  drawCentered(text, 76, 2);

  snprintf(text, sizeof(text), "%lu.%lus",
           (unsigned long)(samples / SAMPLE_RATE_HZ),
           (unsigned long)((samples % SAMPLE_RATE_HZ) / (SAMPLE_RATE_HZ / 10)));
  display.setTextColor(C_LIGHTGREY, C_BLACK);
  drawCentered(text, 104, 1);

  int x = 8, y = 130, w = LCD_W - 16, h = 10;
  display.drawRect(x, y, w, h, C_DARKGREY);
  display.fillRect(x + 1, y + 1, (w - 2) * (int)percent / 100, h - 2, C_RED);
}

// --- PDM capture (ISR) ----------------------------------------------

static void onPdmData() {
  int n = PDM.available();
  if (n <= 0) return;
  if (n > (int)sizeof(pdmChunk)) n = sizeof(pdmChunk);

  int bytesRead = PDM.read(pdmChunk, n);
  if (bytesRead <= 0) return;

  uint32_t count = (uint32_t)bytesRead / sizeof(int16_t);
  if (!captureEnabled) return;

  uint32_t offset = capturedSamples;
  uint32_t remaining = RECORD_SAMPLES - offset;
  if (count > remaining) count = remaining;
  if (count > 0) {
    memcpy(&recordBuffer[offset], pdmChunk, count * sizeof(int16_t));
    capturedSamples = offset + count;
  }
  if (capturedSamples >= RECORD_SAMPLES) {
    captureEnabled = false;
    captureComplete = true;
  }
}

// --- InternalFS save/load -------------------------------------------

static bool saveRecording() {
  InternalFS.remove(WAV_PATH);

  Adafruit_LittleFS_Namespace::File f(InternalFS);
  if (!f.open(WAV_PATH, FILE_O_WRITE)) return false;

  WavHeader h = makeWavHeader(RECORD_BYTES);
  bool ok = f.write((const uint8_t *)&h, sizeof(h)) == sizeof(h);
  ok = ok && f.write((const uint8_t *)recordBuffer, RECORD_BYTES) == RECORD_BYTES;
  f.close();
  return ok;
}

static bool loadRecording() {
  Adafruit_LittleFS_Namespace::File f(InternalFS);
  if (!f.open(WAV_PATH, FILE_O_READ)) return false;
  f.seek(sizeof(WavHeader));
  size_t n = f.read((uint8_t *)recordBuffer, RECORD_BYTES);
  f.close();
  return n == RECORD_BYTES;
}

// --- Record ----------------------------------------------------------

static bool recordToFlash() {
  screen("Recorder", "Starting mic...", "", C_YELLOW);

  capturedSamples = 0;
  captureEnabled = false;
  captureComplete = false;

  PDM.setPins(PDM_DATA_PIN, PDM_CLK_PIN, -1);
  PDM.onReceive(onPdmData);
  PDM.setBufferSize(sizeof(pdmChunk));

  if (!PDM.begin(CHANNELS, SAMPLE_RATE_HZ)) {
    screen("Error", "Mic init failed", "Check mic", C_RED);
    return false;
  }
  PDM.setGain(PDM_GAIN);

  captureEnabled = true;
  uint32_t start = millis();
  while (!captureComplete) {
    progressScreen(capturedSamples);
    if (millis() - start > 3000) break;   // timeout guard
    delay(1);
  }
  captureEnabled = false;
  PDM.end();

  screen("Recorder", "Saving...", WAV_PATH, C_YELLOW);
  hasRecording = saveRecording();
  screen(hasRecording ? "Done" : "Error",
         hasRecording ? "Saved WAV" : "Write failed",
         hasRecording ? "USR2: play" : "Check flash", hasRecording ? C_GREEN : C_RED);
  return hasRecording;
}

// --- I2S playback (NRF_I2S peripheral) ------------------------------

static uint32_t packStereoSample(int16_t sample) {
  int32_t scaled = (int32_t)((float)sample * PLAYBACK_GAIN);
  if (scaled > 32767) scaled = 32767;
  if (scaled < -32768) scaled = -32768;
  uint16_t s = (uint16_t)(int16_t)scaled;
  return ((uint32_t)s << 16) | s;
}

static size_t fillI2SBuffer(uint32_t *buffer, uint32_t startSample) {
  size_t frames = 0;
  while (frames < I2S_PLAYBACK_FRAMES && startSample + frames < RECORD_SAMPLES) {
    buffer[frames] = packStereoSample(recordBuffer[startSample + frames]);
    frames++;
  }
  while (frames < I2S_PLAYBACK_FRAMES) buffer[frames++] = 0;
  return frames;
}

static void initI2SPlayback(uint32_t *firstBuffer) {
  NRF_I2S->TASKS_STOP = 1;
  NRF_I2S->ENABLE = 0;
  NRF_I2S->EVENTS_RXPTRUPD = 0;
  NRF_I2S->EVENTS_TXPTRUPD = 0;
  NRF_I2S->EVENTS_STOPPED = 0;

  NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_Master;
  NRF_I2S->CONFIG.RXEN = I2S_CONFIG_RXEN_RXEN_Disabled;
  NRF_I2S->CONFIG.TXEN = I2S_CONFIG_TXEN_TXEN_Enabled;
  NRF_I2S->CONFIG.MCKEN = I2S_CONFIG_MCKEN_MCKEN_Enabled;
  NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV63;
  NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_32X;   // ~16 kHz LRCK for 16 kHz audio
  NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
  NRF_I2S->CONFIG.ALIGN = I2S_CONFIG_ALIGN_ALIGN_Left;
  NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S;
  NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Stereo;

  NRF_I2S->PSEL.MCK = 0xFFFFFFFF;
  NRF_I2S->PSEL.SCK = g_ADigitalPinMap[I2S_BCLK_PIN];
  NRF_I2S->PSEL.LRCK = g_ADigitalPinMap[I2S_LRCLK_PIN];
  NRF_I2S->PSEL.SDIN = 0xFFFFFFFF;
  NRF_I2S->PSEL.SDOUT = g_ADigitalPinMap[I2S_DOUT_PIN];

  NRF_I2S->TXD.PTR = (uint32_t)firstBuffer;
  NRF_I2S->RXD.PTR = 0;
  NRF_I2S->RXTXD.MAXCNT = I2S_PLAYBACK_FRAMES;

  NRF_I2S->ENABLE = 1;
  NRF_I2S->TASKS_START = 1;
}

static void stopI2SPlayback() {
  NRF_I2S->TASKS_STOP = 1;
  uint32_t startMs = millis();
  while (!NRF_I2S->EVENTS_STOPPED && millis() - startMs < 100) delay(1);
  NRF_I2S->EVENTS_STOPPED = 0;
  NRF_I2S->ENABLE = 0;
}

static void playRecording() {
  if (!hasRecording) {
    screen("Playback", "No recording", "Press USR1 first", C_YELLOW);
    delay(900);
    readyScreen();
    return;
  }

  screen("Playback", "Loading...", WAV_PATH, C_GREEN);
  if (!loadRecording()) {
    screen("Error", "Load failed", "Record again", C_RED);
    delay(1200);
    readyScreen();
    return;
  }

  screen("Playback", "Playing...", "Please wait", C_GREEN);

  uint32_t nextSample = 0;
  uint8_t drainBuffers = 0;
  bool useA = false;
  uint32_t lastI2SEventMs = millis();

  fillI2SBuffer(i2sBufferA, nextSample);
  nextSample += I2S_PLAYBACK_FRAMES;
  initI2SPlayback(i2sBufferA);

  while (nextSample < RECORD_SAMPLES || drainBuffers < 2) {
    if (NRF_I2S->EVENTS_TXPTRUPD) {
      NRF_I2S->EVENTS_TXPTRUPD = 0;
      lastI2SEventMs = millis();

      uint32_t *nextBuffer = useA ? i2sBufferA : i2sBufferB;
      if (nextSample < RECORD_SAMPLES) {
        fillI2SBuffer(nextBuffer, nextSample);
        nextSample += I2S_PLAYBACK_FRAMES;
      } else {
        memset(nextBuffer, 0, I2S_PLAYBACK_FRAMES * sizeof(uint32_t));
        drainBuffers++;
      }
      NRF_I2S->TXD.PTR = (uint32_t)nextBuffer;
      useA = !useA;
    }
    if (millis() - lastI2SEventMs > 500) break;
  }

  stopI2SPlayback();
  screen("Playback", "Finished", "USR1: rec  USR2: play", C_GREEN);
}

// --- Setup / Loop ----------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);

  if (!display.begin<Board_XIAO_0inch96_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_Seeed_0inch96_LCD_ST7789>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(TFT_BLACK);
  display.setTextWrap(false);

  hasRecording = InternalFS.begin() && InternalFS.exists(WAV_PATH);
  readyScreen();
}

void loop() {
  bool usr1 = digitalRead(USR1_PIN);
  bool usr2 = digitalRead(USR2_PIN);

  if (lastUsr1 == HIGH && usr1 == LOW) {
    delay(25);
    recordToFlash();
    readyScreen();
  }
  if (lastUsr2 == HIGH && usr2 == LOW) {
    delay(25);
    playRecording();
    delay(600);
    readyScreen();
  }

  lastUsr1 = usr1;
  lastUsr2 = usr2;
  delay(10);
}
