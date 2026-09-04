/*
  XIAO nRF52840 Plus offline RAM recorder and raw playback example.

  Flow:
    IDLE -> PREPARE_SYSTEM -> QUIET_RADIO -> PREPARE_PERIPHERALS
    -> START_HFCLK -> START_PDM -> DISCARD_WARMUP -> CAPTURE_RAM
    -> STOP_PDM -> SAVE_RAW -> DONE

  nRF52840 RAM note:
    5 seconds * 16000 samples/s * 2 bytes = 160000 bytes.
    Ten seconds would require 320000 bytes and does not fit in 256 KB RAM.

  Output:
    /REC_001_RAW.WAV

  Operation:
    Insert a FAT-formatted SD card, power on the board, then press USR1.
    The LCD shows the current recorder state and capture progress.
    After a successful recording, press USR2 to play the latest raw audio
    through a MAX98357A module connected to D11/D12/D13.

  Ported to Seeed_GFX2: Board_XIAO_1inch47_Touch_Display<38,37> +
  Config_XIAO_1inch47_Touch_JD9853A replace driver.h + manual pin init +
  tft.init()/setRotation(0)/invertDisplay(false)/setSwapBytes(true) +
  applyPanelFix(). Keeps SdFat (shared SPI with LCD), the PDM capture /
  I2S raw playback / WAV writer, and acquireForLcd/acquireForSd shared-SPI
  arbitration (LCD CS=D2 owned by Seeed_GFX; only SD CS=D6 is idled here).
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_JD9853A.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <PDM.h>
#include <SPI.h>
#include <SdFat.h>
#include <nrf.h>

// ========================= Pin map =========================

static constexpr uint8_t PDM_CLK_PIN = D0;
static constexpr uint8_t PDM_DATA_PIN = D1;

// Shared-SPI arbitration pins (Board template owns RST/BL + the LCD bus).
static constexpr uint8_t LCD_CS_PIN = D2;
static constexpr uint8_t SD_CS_PIN = D6;

static constexpr int8_t LCD_RST_PIN = 38;  // XIAO nRF52840 Plus
static constexpr int8_t LCD_BL_PIN  = 37;

static constexpr uint8_t USR1_PIN = D19;
static constexpr uint8_t USR2_PIN = D15;
static constexpr uint8_t I2S_DIN_PIN = D11;
static constexpr uint8_t I2S_BCLK_PIN = D12;
static constexpr uint8_t I2S_LRCLK_PIN = D13;

static constexpr uint32_t SAMPLE_RATE_HZ = 16000;
static constexpr uint16_t CHANNELS = 1;
static constexpr uint16_t BITS_PER_SAMPLE = 16;
static constexpr uint32_t RECORD_SECONDS = 5;
static constexpr uint32_t RECORD_SAMPLES = SAMPLE_RATE_HZ * RECORD_SECONDS;
static constexpr uint32_t RECORD_BYTES = RECORD_SAMPLES * sizeof(int16_t);
static constexpr uint32_t WARMUP_MS = 300;
static constexpr size_t PDM_CHUNK_SAMPLES = 256;
static constexpr int PDM_GAIN = 30;

static constexpr size_t I2S_PLAYBACK_FRAMES = 256;
static constexpr float PLAYBACK_GAIN = 0.75f;

Seeed_GFX display;

enum RecorderState {
  STATE_IDLE,
  STATE_PREPARE_SYSTEM,
  STATE_QUIET_RADIO,
  STATE_PREPARE_PERIPHERALS,
  STATE_START_HFCLK,
  STATE_START_PDM,
  STATE_DISCARD_WARMUP,
  STATE_CAPTURE_RAM,
  STATE_STOP_PDM,
  STATE_SAVE_RAW,
  STATE_DONE,
  STATE_ERROR
};

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

SdFat sdCard;

// Static allocation makes the RAM cost visible at link time.
static int16_t recordBuffer[RECORD_SAMPLES];
static int16_t pdmChunk[PDM_CHUNK_SAMPLES];
static uint32_t i2sBufferA[I2S_PLAYBACK_FRAMES];
static uint32_t i2sBufferB[I2S_PLAYBACK_FRAMES];

volatile uint32_t capturedSamples = 0;
volatile uint32_t discardedSamples = 0;
volatile bool captureEnabled = false;
volatile bool captureComplete = false;

RecorderState recorderState = STATE_IDLE;
uint32_t lastProgressDrawMs = 0;
const char *errorReason = "unknown";
bool displayReady = false;
bool recordingBusy = false;
bool hasRecording = false;
bool lastUsr1State = HIGH;
bool lastUsr2State = HIGH;

char rawPath[32];

static const char *stateName(RecorderState state);

// ========================= Bus helpers =========================
// SD and LCD share the SPI bus. Seeed_GFX Bus_SPI owns LCD_CS (D2) and parks it
// HIGH between transactions; only SD_CS (D6) is idled here to keep the SD card off
// the shared bus before the other device uses it.

static void acquireForLcd() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void acquireForSd() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void drawHeader(const char *title, uint16_t color) {
  if (!displayReady) return;
  acquireForLcd();
  display.fillScreen(TFT_BLACK);
  display.setTextDatum(TL_DATUM);
  display.setTextColor(color, TFT_BLACK);
  display.drawString(title, 8, 10, 2);
  display.drawFastHLine(0, 32, display.width(), color);
}

static void showScreen(
  const char *title,
  const char *line1,
  const char *line2,
  uint16_t color
) {
  if (!displayReady) return;
  drawHeader(title, color);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(line1, 8, 54, 2);
  if (line2 && line2[0]) {
    display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    display.drawString(line2, 8, 78, 2);
  }
}

static void showReadyScreen() {
  showScreen("RAM Recorder", "USR1: record", "USR2: play last", TFT_CYAN);
}

static void showStateScreen(RecorderState state) {
  if (!displayReady) return;
  char line[48];
  snprintf(line, sizeof(line), "State: %s", stateName(state));
  showScreen("Recorder", line, "Please wait...", TFT_YELLOW);
}

static void showRecordingProgress(uint32_t elapsedMs) {
  if (!displayReady) return;
  uint32_t now = millis();
  if (now - lastProgressDrawMs < 200) return;
  lastProgressDrawMs = now;

  uint32_t percent = min((uint32_t)100, elapsedMs * 100 / (RECORD_SECONDS * 1000UL));
  int barX = 8;
  int barY = 118;
  int barW = display.width() - 16;
  int barH = 14;
  int fillW = barW * percent / 100;

  acquireForLcd();
  display.fillRect(0, 42, display.width(), 116, TFT_BLACK);
  display.setTextColor(TFT_RED, TFT_BLACK);
  display.drawString("Recording", 8, 54, 4);
  display.setTextColor(TFT_WHITE, TFT_BLACK);

  char line[40];
  snprintf(line, sizeof(line), "%lu.%01lus / %lus",
           (unsigned long)(elapsedMs / 1000),
           (unsigned long)((elapsedMs % 1000) / 100),
           (unsigned long)RECORD_SECONDS);
  display.drawString(line, 8, 92, 2);

  display.drawRect(barX, barY, barW, barH, TFT_DARKGREY);
  if (fillW > 2) {
    display.fillRect(barX + 1, barY + 1, fillW - 2, barH - 2, TFT_RED);
  }
}

static void showDoneScreen() {
  if (!displayReady) return;
  drawHeader("Done", TFT_GREEN);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(rawPath, 8, 52, 2);
  display.drawString("Saved raw WAV", 8, 78, 2);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.drawString("USR1: record", 8, 128, 2);
  display.drawString("USR2: play raw", 8, 150, 2);
}

static void showErrorScreen() {
  if (!displayReady) return;
  showScreen("Error", errorReason, "Check Serial Monitor", TFT_RED);
}

static void showNoRecordingScreen() {
  showScreen("Playback", "No recording yet", "Press USR1 first", TFT_YELLOW);
}

static void showPlaybackDoneScreen() {
  showScreen("Playback", "Finished", "USR1: record  USR2: play", TFT_GREEN);
}

static void showPlaybackLoadError() {
  showScreen("Playback", "RAW file load failed", "Record again or check SD", TFT_RED);
}

static const char *stateName(RecorderState state) {
  switch (state) {
    case STATE_IDLE: return "IDLE";
    case STATE_PREPARE_SYSTEM: return "PREPARE_SYSTEM";
    case STATE_QUIET_RADIO: return "QUIET_RADIO";
    case STATE_PREPARE_PERIPHERALS: return "PREPARE_PERIPHERALS";
    case STATE_START_HFCLK: return "START_HFCLK";
    case STATE_START_PDM: return "START_PDM";
    case STATE_DISCARD_WARMUP: return "DISCARD_WARMUP";
    case STATE_CAPTURE_RAM: return "CAPTURE_RAM";
    case STATE_STOP_PDM: return "STOP_PDM";
    case STATE_SAVE_RAW: return "SAVE_RAW";
    case STATE_DONE: return "DONE";
    default: return "ERROR";
  }
}

static void enterState(RecorderState state) {
  recorderState = state;
  Serial.print("[STATE] ");
  Serial.println(stateName(state));
  if (state != STATE_CAPTURE_RAM && state != STATE_ERROR && state != STATE_DONE) {
    showStateScreen(state);
  }
}

static bool failWithReason(const char *reason) {
  errorReason = reason;
  Serial.print("[FAIL] ");
  Serial.println(reason);
  Serial.flush();
  return false;
}

static bool beginSd() {
  acquireForSd();
  SPI.begin();

  const uint32_t freqs[] = {8000000, 4000000, 1000000, 400000};
  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
    SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, freqs[i], &SPI);
    if (sdCard.begin(cfg)) {
      return true;
    }
    delay(100);
  }
  return false;
}

static void selectOutputPaths() {
  for (int i = 1; i < 1000; ++i) {
    snprintf(rawPath, sizeof(rawPath), "/REC_%03d_RAW.WAV", i);
    if (!sdCard.exists(rawPath)) return;
  }

  snprintf(rawPath, sizeof(rawPath), "/REC_999_RAW.WAV");
}

static void quietRadio() {
  // This sketch never initializes Bluefruit/BLE. Ensure the RADIO peripheral
  // is disabled before the timing-sensitive capture section.
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  uint32_t start = millis();
  while (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled &&
         millis() - start < 20) {
    delay(1);
  }
}

static bool startExternalHfclk() {
  uint32_t current = NRF_CLOCK->HFCLKSTAT;
  bool alreadyRunning =
    ((current & CLOCK_HFCLKSTAT_STATE_Msk) >> CLOCK_HFCLKSTAT_STATE_Pos) ==
    CLOCK_HFCLKSTAT_STATE_Running;
  bool alreadyXtal =
    ((current & CLOCK_HFCLKSTAT_SRC_Msk) >> CLOCK_HFCLKSTAT_SRC_Pos) ==
    CLOCK_HFCLKSTAT_SRC_Xtal;
  if (alreadyRunning && alreadyXtal) return true;

  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  uint32_t start = millis();
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED) {
    if (millis() - start > 100) return false;
  }

  uint32_t stat = NRF_CLOCK->HFCLKSTAT;
  bool running =
    ((stat & CLOCK_HFCLKSTAT_STATE_Msk) >> CLOCK_HFCLKSTAT_STATE_Pos) ==
    CLOCK_HFCLKSTAT_STATE_Running;
  bool xtal =
    ((stat & CLOCK_HFCLKSTAT_SRC_Msk) >> CLOCK_HFCLKSTAT_SRC_Pos) ==
    CLOCK_HFCLKSTAT_SRC_Xtal;
  return running && xtal;
}

static void onPdmData() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;
  if (bytesAvailable > (int)sizeof(pdmChunk)) {
    bytesAvailable = sizeof(pdmChunk);
  }

  int bytesRead = PDM.read(pdmChunk, bytesAvailable);
  if (bytesRead <= 0) return;

  uint32_t count = (uint32_t)bytesRead / sizeof(int16_t);
  if (!captureEnabled) {
    discardedSamples += count;
    return;
  }

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

static bool startPdm() {
  capturedSamples = 0;
  discardedSamples = 0;
  captureEnabled = false;
  captureComplete = false;

  NVIC_EnableIRQ(PDM_IRQn);
  PDM.setPins(PDM_DATA_PIN, PDM_CLK_PIN, -1);
  PDM.onReceive(onPdmData);
  PDM.setBufferSize(sizeof(pdmChunk));
  PDM.setGain(PDM_GAIN);
  return PDM.begin(CHANNELS, SAMPLE_RATE_HZ);
}

static bool stopPdmCleanly() {
  captureEnabled = false;

  // Request a peripheral stop and wait for END/STOPPED before PDM.end()
  // disconnects the pins and disables the IRQ.
  NVIC_DisableIRQ(PDM_IRQn);
  NRF_PDM->EVENTS_END = 0;
  NRF_PDM->EVENTS_STOPPED = 0;
  NRF_PDM->TASKS_STOP = 1;

  uint32_t start = millis();
  while (!NRF_PDM->EVENTS_STOPPED && millis() - start < 100) {
    delay(1);
  }
  bool stopped = NRF_PDM->EVENTS_STOPPED != 0;
  PDM.end();
  return stopped;
}

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
  while (frames < I2S_PLAYBACK_FRAMES) {
    buffer[frames++] = 0;
  }
  return frames;
}

static bool loadRawRecordingFromSd() {
  acquireForSd();

  File32 file;
  if (!file.open(rawPath, O_RDONLY)) return false;
  if (!file.seekSet(44)) {
    file.close();
    return false;
  }

  uint8_t *dst = reinterpret_cast<uint8_t *>(recordBuffer);
  uint32_t remaining = RECORD_BYTES;
  while (remaining > 0) {
    uint32_t chunk = min((uint32_t)4096, remaining);
    int bytesRead = file.read(dst, chunk);
    if (bytesRead <= 0) {
      file.close();
      return false;
    }
    dst += bytesRead;
    remaining -= (uint32_t)bytesRead;
  }

  file.close();
  return true;
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

  NRF_I2S->TXD.PTR = (uint32_t)firstBuffer;
  NRF_I2S->RXD.PTR = 0;
  NRF_I2S->RXTXD.MAXCNT = I2S_PLAYBACK_FRAMES;

  NRF_I2S->ENABLE = 1;
  NRF_I2S->TASKS_START = 1;
}

static void stopI2SPlayback() {
  NRF_I2S->TASKS_STOP = 1;
  uint32_t startMs = millis();
  while (!NRF_I2S->EVENTS_STOPPED && millis() - startMs < 100) {
    delay(1);
  }
  NRF_I2S->EVENTS_STOPPED = 0;
  NRF_I2S->ENABLE = 0;
}

static void playLatestRecording() {
  if (!hasRecording) {
    showNoRecordingScreen();
    delay(900);
    showReadyScreen();
    return;
  }

  showScreen("Playback", "Loading RAW audio...", rawPath, TFT_GREEN);
  if (!loadRawRecordingFromSd()) {
    showPlaybackLoadError();
    delay(1200);
    showDoneScreen();
    return;
  }

  Serial.println("[PLAY] latest RAW audio");
  showScreen("Playback", "Playing RAW audio", "Please wait...", TFT_GREEN);

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

    if (millis() - lastI2SEventMs > 500) {
      Serial.println("[PLAY] I2S timeout");
      break;
    }
  }

  stopI2SPlayback();
  showPlaybackDoneScreen();
  delay(700);
  showDoneScreen();
  Serial.println("[PLAY] finished");
}

static void fillWavHeader(WavHeader &header, uint32_t pcmBytes) {
  memcpy(header.riff, "RIFF", 4);
  memcpy(header.wave, "WAVE", 4);
  memcpy(header.fmt, "fmt ", 4);
  memcpy(header.data, "data", 4);
  header.chunkSize = 36 + pcmBytes;
  header.subchunk1Size = 16;
  header.audioFormat = 1;
  header.numChannels = CHANNELS;
  header.sampleRate = SAMPLE_RATE_HZ;
  header.bitsPerSample = BITS_PER_SAMPLE;
  header.blockAlign = CHANNELS * BITS_PER_SAMPLE / 8;
  header.byteRate = SAMPLE_RATE_HZ * header.blockAlign;
  header.subchunk2Size = pcmBytes;
}

static bool writeBufferToWav(
  const char *path,
  const int16_t *samples,
  uint32_t sampleCount
) {
  acquireForSd();
  File32 file;
  if (!file.open(path, O_WRONLY | O_CREAT | O_TRUNC)) return false;

  uint32_t pcmBytes = sampleCount * sizeof(int16_t);
  WavHeader header;
  fillWavHeader(header, pcmBytes);
  if (file.write(
        reinterpret_cast<const uint8_t *>(&header),
        sizeof(header)) != sizeof(header)) {
    file.close();
    sdCard.remove(path);
    return false;
  }

  const uint8_t *data = reinterpret_cast<const uint8_t *>(samples);
  uint32_t offset = 0;
  while (offset < pcmBytes) {
    uint32_t chunk = min((uint32_t)4096, pcmBytes - offset);
    size_t written = file.write(data + offset, chunk);
    if (written == 0) {
      file.close();
      sdCard.remove(path);
      return false;
    }
    offset += written;
  }

  file.flush();
  file.close();
  return true;
}

static bool runRecorder() {
  errorReason = "unknown";
  lastProgressDrawMs = 0;
  enterState(STATE_PREPARE_SYSTEM);

  enterState(STATE_QUIET_RADIO);
  quietRadio();

  enterState(STATE_PREPARE_PERIPHERALS);
  if (!beginSd()) return failWithReason("SD mount failed");
  selectOutputPaths();
  digitalWrite(SD_CS_PIN, HIGH);

  enterState(STATE_START_HFCLK);
  if (!startExternalHfclk()) return failWithReason("external HFCLK start failed");

  enterState(STATE_START_PDM);
  if (!startPdm()) return failWithReason("PDM begin failed");

  enterState(STATE_DISCARD_WARMUP);
  uint32_t warmupStart = millis();
  while (millis() - warmupStart < WARMUP_MS) delay(1);

  enterState(STATE_CAPTURE_RAM);
  capturedSamples = 0;
  captureComplete = false;
  captureEnabled = true;
  uint32_t captureStart = millis();
  while (!captureComplete) {
    showRecordingProgress(millis() - captureStart);
    if (millis() - captureStart > (RECORD_SECONDS + 2) * 1000UL) {
      captureEnabled = false;
      return failWithReason("PDM capture timeout");
    }
    delay(1);
  }
  enterState(STATE_STOP_PDM);
  if (!stopPdmCleanly()) return failWithReason("PDM stop failed");

  enterState(STATE_SAVE_RAW);
  if (!writeBufferToWav(rawPath, recordBuffer, RECORD_SAMPLES)) {
    return failWithReason("save RAW WAV failed");
  }

  enterState(STATE_DONE);
  showDoneScreen();
  return true;
}

static void haltWithError() {
  enterState(STATE_ERROR);
  showErrorScreen();
  uint32_t lastPrintMs = 0;
  while (1) {
    if (Serial && millis() - lastPrintMs >= 1000) {
      lastPrintMs = millis();
      Serial.print("[ERROR] state=");
      Serial.print(stateName(recorderState));
      Serial.print(" reason=");
      Serial.println(errorReason);
      Serial.flush();
    }
    delay(240);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);

  if (!display.begin<Board_XIAO_1inch47_Touch_Display<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_XIAO_1inch47_Touch_JD9853A>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(TFT_BLACK);
  displayReady = true;

  showReadyScreen();

  uint32_t serialStartMs = millis();
  while (!Serial && millis() - serialStartMs < 2000) delay(10);

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus RAM PDM recorder ===");
  Serial.print("[RAM] record buffer bytes=");
  Serial.println(RECORD_BYTES);
  Serial.println("[RADIO] BLE is not initialized by this sketch");
  Serial.println("[PDM] library uses EasyDMA double buffering");

  enterState(STATE_IDLE);
  showReadyScreen();
}

void loop() {
  bool usr1State = digitalRead(USR1_PIN);
  bool usr2State = digitalRead(USR2_PIN);

  if (lastUsr1State == HIGH && usr1State == LOW && !recordingBusy) {
    delay(20);
    if (digitalRead(USR1_PIN) == LOW) {
      recordingBusy = true;
      if (!runRecorder()) haltWithError();
      hasRecording = true;

      Serial.print("[SAVE] ");
      Serial.println(rawPath);

      recordingBusy = false;
      showDoneScreen();
    }
  }

  if (lastUsr2State == HIGH && usr2State == LOW && !recordingBusy) {
    delay(20);
    if (digitalRead(USR2_PIN) == LOW) {
      playLatestRecording();
    }
  }

  lastUsr1State = usr1State;
  lastUsr2State = usr2State;
  delay(20);
}
