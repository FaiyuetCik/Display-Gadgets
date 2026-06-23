/*
  XIAO nRF52840 Plus offline PDM recorder.

  Operation:
    1. Insert a FAT-formatted SD card.
    2. Power on or reset the board.
    3. After a 2-second preparation delay, the board records for 10 seconds.
    4. The recording is saved as /REC_001.WAV, /REC_002.WAV, and so on.

  WAV format:
    - 16000 Hz
    - 16-bit signed PCM
    - Mono

  Hardware:
    PDM microphone DATA = D1, CLK = D0
    SD CS = D6

  Required libraries:
    - PDM
    - SdFat from the Seeed nRF52 board package
    - Adafruit_TinyUSB
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <PDM.h>
#include <SPI.h>
#include <SdFat.h>

static constexpr uint8_t PDM_CLK_PIN = D0;
static constexpr uint8_t PDM_DATA_PIN = D1;
static constexpr uint8_t LCD_CS_PIN = D2;
static constexpr uint8_t SD_CS_PIN = D6;

static constexpr uint32_t SAMPLE_RATE_HZ = 16000;
static constexpr uint16_t CHANNELS = 1;
static constexpr uint16_t BITS_PER_SAMPLE = 16;
static constexpr uint32_t RECORD_SECONDS = 10;
static constexpr int PDM_GAIN = 30;

static constexpr size_t PDM_CHUNK_SAMPLES = 256;
static constexpr size_t RING_SAMPLES = 4096;

SdFat sdCard;
File32 recordFile;

volatile int16_t ringBuffer[RING_SAMPLES];
volatile uint32_t ringWrite = 0;
volatile uint32_t ringRead = 0;
volatile uint32_t droppedSamples = 0;

int16_t pdmChunk[PDM_CHUNK_SAMPLES];
int16_t writeChunk[PDM_CHUNK_SAMPLES];

static void setStatusLed(bool on) {
  // XIAO nRF52840 built-in LED is active-low.
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);
}

static void blinkSuccess() {
  for (int i = 0; i < 3; ++i) {
    setStatusLed(true);
    delay(180);
    setStatusLed(false);
    delay(180);
  }
}

static void haltWithError() {
  while (1) {
    setStatusLed(true);
    delay(120);
    setStatusLed(false);
    delay(120);
  }
}

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

static bool beginSd() {
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin();

  const uint32_t freqs[] = {8000000, 4000000, 1000000, 400000};
  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
    SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, freqs[i], &SPI);
    Serial.print("[SD] try ");
    Serial.print(freqs[i]);
    Serial.print(" Hz ... ");
    if (sdCard.begin(cfg)) {
      Serial.println("OK");
      return true;
    }
    Serial.println("FAIL");
    delay(100);
  }
  return false;
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

static bool writeWavHeader(File32 &file, uint32_t pcmBytes) {
  WavHeader header;
  fillWavHeader(header, pcmBytes);
  if (!file.seekSet(0)) return false;
  return file.write(
           reinterpret_cast<const uint8_t *>(&header),
           sizeof(header)) == sizeof(header);
}

static void nextRecordPath(char *path, size_t pathSize) {
  for (int i = 1; i < 1000; ++i) {
    snprintf(path, pathSize, "/REC_%03d.WAV", i);
    if (!sdCard.exists(path)) return;
  }
  snprintf(path, pathSize, "/REC_999.WAV");
}

static inline uint32_t ringCount(uint32_t writePos, uint32_t readPos) {
  return writePos >= readPos
           ? writePos - readPos
           : RING_SAMPLES - (readPos - writePos);
}

static inline uint32_t ringFree(uint32_t writePos, uint32_t readPos) {
  return (RING_SAMPLES - 1) - ringCount(writePos, readPos);
}

static void resetRing() {
  noInterrupts();
  ringWrite = 0;
  ringRead = 0;
  droppedSamples = 0;
  interrupts();
}

static void pushSamples(const int16_t *samples, size_t count) {
  noInterrupts();
  uint32_t writePos = ringWrite;
  uint32_t readPos = ringRead;

  for (size_t i = 0; i < count; ++i) {
    if (ringFree(writePos, readPos) == 0) {
      droppedSamples++;
      continue;
    }
    ringBuffer[writePos] = samples[i];
    writePos = (writePos + 1) % RING_SAMPLES;
  }

  ringWrite = writePos;
  interrupts();
}

static size_t popSamples(int16_t *samples, size_t maxCount) {
  size_t count = 0;
  noInterrupts();
  while (count < maxCount && ringRead != ringWrite) {
    samples[count++] = ringBuffer[ringRead];
    ringRead = (ringRead + 1) % RING_SAMPLES;
  }
  interrupts();
  return count;
}

static void onPdmData() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;
  if (bytesAvailable > (int)sizeof(pdmChunk)) {
    bytesAvailable = sizeof(pdmChunk);
  }

  int bytesRead = PDM.read(pdmChunk, bytesAvailable);
  if (bytesRead > 0) {
    pushSamples(pdmChunk, (size_t)bytesRead / sizeof(int16_t));
  }
}

static bool startMicrophone() {
  resetRing();
  PDM.setPins(PDM_DATA_PIN, PDM_CLK_PIN, -1);
  PDM.onReceive(onPdmData);
  PDM.setBufferSize(sizeof(pdmChunk));
  PDM.setGain(PDM_GAIN);

  if (!PDM.begin(CHANNELS, SAMPLE_RATE_HZ)) return false;
  delay(50);
  return true;
}

static bool writeSamples(const int16_t *samples, size_t count) {
  size_t bytes = count * sizeof(int16_t);
  return recordFile.write(
           reinterpret_cast<const uint8_t *>(samples),
           bytes) == bytes;
}

static bool recordWav() {
  char path[24];
  nextRecordPath(path, sizeof(path));

  if (!recordFile.open(path, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.print("[REC] open failed: ");
    Serial.println(path);
    return false;
  }

  if (!writeWavHeader(recordFile, 0)) {
    Serial.println("[REC] initial WAV header failed");
    recordFile.close();
    sdCard.remove(path);
    return false;
  }

  if (!startMicrophone()) {
    Serial.println("[MIC] PDM.begin failed");
    recordFile.close();
    sdCard.remove(path);
    return false;
  }

  Serial.print("[REC] recording ");
  Serial.print(RECORD_SECONDS);
  Serial.print(" seconds to ");
  Serial.println(path);
  setStatusLed(true);

  uint32_t pcmBytes = 0;
  uint32_t startMs = millis();
  uint32_t lastStatusMs = startMs;
  bool writeOk = true;

  while (millis() - startMs < RECORD_SECONDS * 1000UL) {
    size_t count = popSamples(writeChunk, PDM_CHUNK_SAMPLES);
    if (count == 0) {
      delay(1);
      continue;
    }

    if (!writeSamples(writeChunk, count)) {
      Serial.println("[REC] SD write failed");
      writeOk = false;
      break;
    }
    pcmBytes += count * sizeof(int16_t);

    uint32_t now = millis();
    if (now - lastStatusMs >= 1000) {
      lastStatusMs = now;
      Serial.print("[REC] seconds=");
      Serial.print((now - startMs) / 1000);
      Serial.print(" bytes=");
      Serial.print(pcmBytes);
      Serial.print(" dropped=");
      Serial.println((uint32_t)droppedSamples);
    }
  }

  PDM.end();
  setStatusLed(false);

  if (writeOk) {
    while (true) {
      size_t count = popSamples(writeChunk, PDM_CHUNK_SAMPLES);
      if (count == 0) break;
      if (!writeSamples(writeChunk, count)) {
        writeOk = false;
        break;
      }
      pcmBytes += count * sizeof(int16_t);
    }
  }

  if (writeOk && !writeWavHeader(recordFile, pcmBytes)) {
    Serial.println("[REC] final WAV header failed");
    writeOk = false;
  }

  recordFile.flush();
  recordFile.close();

  if (!writeOk) {
    sdCard.remove(path);
    return false;
  }

  Serial.print("[REC] saved: ");
  Serial.println(path);
  Serial.print("[REC] PCM bytes: ");
  Serial.println(pcmBytes);
  Serial.print("[REC] dropped samples: ");
  Serial.println((uint32_t)droppedSamples);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  pinMode(LED_BUILTIN, OUTPUT);
  setStatusLed(false);

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus offline WAV recorder ===");
  Serial.println("PDM: DATA=D1 CLK=D0");
  Serial.println("WAV: 16000 Hz, 16-bit, mono");

  if (!beginSd()) {
    Serial.println("[SD] mount failed");
    haltWithError();
  }

  Serial.println("[REC] starts in 2 seconds");
  delay(2000);

  if (!recordWav()) {
    Serial.println("[REC] failed");
    haltWithError();
  }

  Serial.println("[REC] complete; reset to record another file");
  blinkSuccess();
}

void loop() {
  delay(1000);
}
