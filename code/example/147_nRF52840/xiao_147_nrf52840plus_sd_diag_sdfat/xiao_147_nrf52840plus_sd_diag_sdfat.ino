/*
  XIAO nRF52840 Plus + 1.47 display
  LCD + SD + audio (conservative v1)

  Based on the WORKING LCD+SD baseline:
  - LCD: explicit SW SPI ST7789 path
  - SD : built-in board-package SdFat, SHARED_SPI
  - MIC: PDM on D0/D1
  - No IMU / No Touch

  Design goal of this version:
  1) Keep the proven LCD path unchanged
  2) Keep the proven SD shared-SPI path unchanged
  3) Add audio with the LEAST extra bus pressure
  4) No frequent LCD refresh during recording
  5) First prioritize stable WAV recording to SD
*/

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <PDM.h>
#include <Arduino_GFX_Library.h>
#include <stdarg.h>
#include <math.h>

// ---------- pin map ----------
static constexpr uint8_t PDM_CLK_PIN   = D0;
static constexpr uint8_t PDM_DATA_PIN  = D1;
static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t SD_CS_PIN     = D6;
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t SD_MISO_PIN   = D9;   // informational
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t LCD_RST_PIN   = D17;
static constexpr uint8_t LCD_BL_PIN    = D18;

// ---------- audio config ----------
static constexpr int SAMPLE_RATE_HZ = 16000;
static constexpr int CHANNELS = 1;
static constexpr int RECORD_SECONDS = 5;
static constexpr int PDM_GAIN = 30;
static constexpr size_t PDM_CHUNK_SAMPLES = 256;
static constexpr size_t RING_SAMPLES = 4096;

// ---------- LCD bus: keep the SUCCESSFUL path ----------
Arduino_DataBus *lcdBus = new Arduino_SWSPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  LCD_SCK_PIN,
  LCD_MOSI_PIN,
  GFX_NOT_DEFINED
);

Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus,
  LCD_RST_PIN,
  0,
  false,
  172, 320,
  34, 0,
  34, 0
);

// ---------- SD ----------
SdFat SD;
File recFile;

// ---------- audio buffer ----------
volatile int16_t g_ring[RING_SAMPLES];
volatile uint32_t g_ringWrite = 0;
volatile uint32_t g_ringRead = 0;
volatile uint32_t g_droppedSamples = 0;
volatile bool g_overflow = false;

int16_t g_pdmChunk[PDM_CHUNK_SAMPLES];
int16_t g_writeChunk[PDM_CHUNK_SAMPLES];

// ---------- colors ----------
static constexpr uint16_t C_BLACK = RGB565_BLACK;
static constexpr uint16_t C_WHITE = RGB565_WHITE;
static constexpr uint16_t C_GREEN = RGB565_LIGHTGREEN;
static constexpr uint16_t C_RED   = RGB565_RED;
static constexpr uint16_t C_CYAN  = RGB565_CYAN;
static constexpr uint16_t C_BLUE  = RGB565_BLUE;
static constexpr uint16_t C_YELL  = RGB565_YELLOW;

// ---------- WAV ----------
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

// ---------- utils ----------
static void logf(const char *fmt, ...) {
  char buf[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
}

static void fillWavHeader(WavHeader &h, uint32_t pcmBytes) {
  memcpy(h.riff, "RIFF", 4);
  memcpy(h.wave, "WAVE", 4);
  memcpy(h.fmt,  "fmt ", 4);
  memcpy(h.data, "data", 4);
  h.subchunk1Size = 16;
  h.audioFormat = 1;
  h.numChannels = CHANNELS;
  h.sampleRate = SAMPLE_RATE_HZ;
  h.bitsPerSample = 16;
  h.byteRate = SAMPLE_RATE_HZ * CHANNELS * (h.bitsPerSample / 8);
  h.blockAlign = CHANNELS * (h.bitsPerSample / 8);
  h.subchunk2Size = pcmBytes;
  h.chunkSize = 36 + pcmBytes;
}

static bool writeEmptyWavHeader(File &file) {
  WavHeader h;
  fillWavHeader(h, 0);
  return file.write(reinterpret_cast<const uint8_t *>(&h), sizeof(h)) == sizeof(h);
}

static bool patchWavHeader(File &file, uint32_t pcmBytes) {
  WavHeader h;
  fillWavHeader(h, pcmBytes);
  if (!file.seek(0)) return false;
  return file.write(reinterpret_cast<const uint8_t *>(&h), sizeof(h)) == sizeof(h);
}

// ---------- shared bus helpers ----------
static void lcdHardReset() {
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(10);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(30);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(150);
}

static void gpioIdleState() {
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  pinMode(LCD_DC_PIN, OUTPUT);
  digitalWrite(LCD_DC_PIN, HIGH);

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
}

static void acquireForLcd() {
  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(LCD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void acquireForSd() {
  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(SD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void lcdWriteMadctlFix() {
  acquireForLcd();
  lcdBus->beginWrite();
  lcdBus->writeC8D8(0x36, 0x48);
  lcdBus->endWrite();
}

// ---------- LCD ----------
static bool initLcd() {
  gpioIdleState();

  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, 255);

  lcdHardReset();

  if (!gfx->begin()) {
    Serial.println("[LCD] gfx->begin() failed");
    return false;
  }

  lcdWriteMadctlFix();
  gfx->fillScreen(C_BLACK);
  return true;
}

static void lcdLine(int y, uint16_t color, const String &text, int size = 1) {
  acquireForLcd();
  int h = (size == 1) ? 16 : 24;
  gfx->fillRect(0, y, 172, h, C_BLACK);
  gfx->setCursor(0, y);
  gfx->setTextSize(size);
  gfx->setTextColor(color, C_BLACK);
  gfx->print(text);
}

static void lcdTitle(const String &title, const String &sub) {
  acquireForLcd();
  gfx->fillScreen(C_BLACK);
  gfx->drawRect(0, 0, 172, 320, C_YELL);

  gfx->setCursor(0, 22);
  gfx->setTextSize(2);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->println(title);

  gfx->setCursor(0, 50);
  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->println(sub);
}

static void lcdColorBars() {
  acquireForLcd();
  gfx->fillRect(0,   90, 43, 18, C_RED);
  gfx->fillRect(43,  90, 43, 18, C_GREEN);
  gfx->fillRect(86,  90, 43, 18, C_BLUE);
  gfx->fillRect(129, 90, 43, 18, C_WHITE);
}

// ---------- SD ----------
static bool initSdSharedSpi(uint32_t &okFreq) {
  const uint32_t freqs[] = {400000, 1000000, 4000000, 8000000};

  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
    acquireForSd();
    SPI.begin();
    delay(5);

    logf("[SD] try init @ %lu Hz ... ", (unsigned long)freqs[i]);

    SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, freqs[i], &SPI);
    if (SD.begin(cfg)) {
      Serial.println("OK");
      okFreq = freqs[i];
      return true;
    }

    Serial.println("FAIL");
    if (SD.card()) {
      logf("  code=0x%02X data=0x%02X\n",
           SD.card()->errorCode(),
           SD.card()->errorData());
    }
    delay(120);
  }

  return false;
}

static String nextRecordFileName() {
  char name[24];
  for (int i = 1; i < 1000; ++i) {
    snprintf(name, sizeof(name), "/REC_%03d.WAV", i);
    acquireForSd();
    if (!SD.exists(name)) return String(name);
  }
  return String("/REC_999.WAV");
}

// ---------- audio ring ----------
static inline uint32_t ringCountNoLock(uint32_t w, uint32_t r) {
  return (w >= r) ? (w - r) : (RING_SAMPLES - (r - w));
}

static inline uint32_t ringFreeNoLock(uint32_t w, uint32_t r) {
  return (RING_SAMPLES - 1) - ringCountNoLock(w, r);
}

static void resetRing() {
  noInterrupts();
  g_ringWrite = 0;
  g_ringRead = 0;
  g_droppedSamples = 0;
  g_overflow = false;
  interrupts();
}

static void pushSamplesToRing(const int16_t *src, size_t count) {
  noInterrupts();
  uint32_t w = g_ringWrite;
  uint32_t r = g_ringRead;
  for (size_t i = 0; i < count; ++i) {
    if (ringFreeNoLock(w, r) == 0) {
      g_overflow = true;
      ++g_droppedSamples;
      continue;
    }
    g_ring[w] = src[i];
    w = (w + 1) % RING_SAMPLES;
  }
  g_ringWrite = w;
  interrupts();
}

static size_t popSamplesFromRing(int16_t *dst, size_t maxCount) {
  size_t n = 0;
  noInterrupts();
  while (n < maxCount && g_ringRead != g_ringWrite) {
    dst[n++] = g_ring[g_ringRead];
    g_ringRead = (g_ringRead + 1) % RING_SAMPLES;
  }
  interrupts();
  return n;
}

static void onPDMdata() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;
  if (bytesAvailable > (int)sizeof(g_pdmChunk)) bytesAvailable = sizeof(g_pdmChunk);
  int bytesRead = PDM.read(reinterpret_cast<void *>(g_pdmChunk), bytesAvailable);
  if (bytesRead > 0) pushSamplesToRing(g_pdmChunk, static_cast<size_t>(bytesRead / 2));
}

static bool startPDM() {
  PDM.setPins(D1, D0, -1);
  resetRing();
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(sizeof(g_pdmChunk));
  PDM.setGain(PDM_GAIN);
  if (!PDM.begin(CHANNELS, SAMPLE_RATE_HZ)) return false;
  delay(50);
  return true;
}

static void stopPDM() {
  PDM.end();
}

static int32_t calcPeak(const int16_t *samples, size_t count) {
  int32_t peak = 0;
  for (size_t i = 0; i < count; ++i) {
    int32_t a = abs((int32_t)samples[i]);
    if (a > peak) peak = a;
  }
  return peak;
}

// ---------- record ----------
static bool recordWavToSd(String &savedName, uint32_t &pcmBytesOut, int32_t &peakOut) {
  savedName = nextRecordFileName();

  acquireForSd();
  if (!recFile.open(savedName.c_str(), O_RDWR | O_CREAT | O_TRUNC)) {
    logf("[SD] open failed: %s\n", savedName.c_str());
    return false;
  }

  acquireForSd();
  if (!writeEmptyWavHeader(recFile)) {
    Serial.println("[SD] write header failed");
    recFile.close();
    return false;
  }

  if (!startPDM()) {
    Serial.println("[MIC] PDM.begin failed");
    recFile.close();
    acquireForSd();
    SD.remove(savedName.c_str());
    return false;
  }

  lcdWriteMadctlFix();
  lcdTitle("Recording...", savedName);
  lcdColorBars();
  lcdLine(120, C_WHITE, "conservative v1");
  lcdLine(138, C_CYAN, "no live meter");
  lcdLine(156, C_WHITE, "recording 5 sec");

  logf("[REC] start -> %s\n", savedName.c_str());

  const uint32_t startMs = millis();
  uint32_t lastLogMs = 0;
  uint32_t pcmBytes = 0;
  int32_t sessionPeak = 0;

  while (millis() - startMs < RECORD_SECONDS * 1000UL) {
    size_t n = popSamplesFromRing(g_writeChunk, PDM_CHUNK_SAMPLES);
    if (n == 0) {
      delay(1);
      continue;
    }

    int32_t peak = calcPeak(g_writeChunk, n);
    if (peak > sessionPeak) sessionPeak = peak;

    size_t bytes = n * sizeof(int16_t);

    acquireForSd();
    if (recFile.write(reinterpret_cast<const uint8_t *>(g_writeChunk), bytes) != bytes) {
      Serial.println("[SD] write failed during record");
      stopPDM();
      recFile.close();
      return false;
    }

    pcmBytes += bytes;

    uint32_t now = millis();
    if (now - lastLogMs > 500) {
      lastLogMs = now;
      logf("[MIC] t=%lu ms peak=%ld dropped=%lu bytes=%lu\n",
           (unsigned long)(now - startMs),
           (long)peak,
           (unsigned long)g_droppedSamples,
           (unsigned long)pcmBytes);
    }
  }

  stopPDM();

  uint32_t drainStart = millis();
  while (millis() - drainStart < 300) {
    size_t n = popSamplesFromRing(g_writeChunk, PDM_CHUNK_SAMPLES);
    if (n == 0) break;

    size_t bytes = n * sizeof(int16_t);
    acquireForSd();
    if (recFile.write(reinterpret_cast<const uint8_t *>(g_writeChunk), bytes) != bytes) {
      Serial.println("[SD] write failed while draining");
      recFile.close();
      return false;
    }

    pcmBytes += bytes;
  }

  acquireForSd();
  if (!patchWavHeader(recFile, pcmBytes)) {
    Serial.println("[SD] patch WAV header failed");
    recFile.close();
    return false;
  }

  acquireForSd();
  recFile.flush();
  recFile.close();

  pcmBytesOut = pcmBytes;
  peakOut = sessionPeak;

  logf("[REC] done. bytes=%lu peak=%ld dropped=%lu\n",
       (unsigned long)pcmBytesOut,
       (long)peakOut,
       (unsigned long)g_droppedSamples);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 1.47 LCD + SD + audio ===");
  Serial.println("LCD baseline = working SW SPI ST7789 path");
  Serial.println("SD backend   = board-package SdFat, SHARED_SPI");
  Serial.println("Audio        = PDM D0/D1, conservative v1");
  Serial.println("No IMU / No Touch");

  if (!initLcd()) {
    Serial.println("[FAIL] LCD init failed");
    return;
  }

  lcdTitle("LCD OK", "starting SD...");
  lcdColorBars();
  delay(600);

  uint32_t okFreq = 0;
  if (!initSdSharedSpi(okFreq)) {
    Serial.println("[FAIL] SD mount failed");
    lcdTitle("SD FAIL", "mount failed");
    lcdLine(74, C_RED, "shared SPI mount fail");
    return;
  }

  Serial.print("[OK] SD mounted @ ");
  Serial.print(okFreq);
  Serial.println(" Hz");

  lcdWriteMadctlFix();
  lcdTitle("SD OK", String("freq=") + okFreq);
  lcdColorBars();
  lcdLine(120, C_WHITE, "next: record WAV");
  delay(600);

  String savedName;
  uint32_t pcmBytes = 0;
  int32_t peak = 0;

  if (!recordWavToSd(savedName, pcmBytes, peak)) {
    Serial.println("[FAIL] recording failed");
    lcdWriteMadctlFix();
    lcdTitle("REC FAIL", "recording failed");
    lcdLine(120, C_RED, "check serial log");
    return;
  }

  Serial.println("[OK] recording success");

  lcdWriteMadctlFix();
  lcdTitle("REC PASS", savedName);
  lcdColorBars();
  lcdLine(120, C_WHITE, String("bytes=") + pcmBytes);
  lcdLine(138, C_WHITE, String("peak=") + peak);
  lcdLine(156, C_WHITE, String("drop=") + (unsigned long)g_droppedSamples);
  lcdLine(174, C_CYAN, "Reset to record again");

  Serial.println("[DONE] LCD + SD + audio finished");
}

void loop() {
  delay(1000);
}
