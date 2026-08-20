/*
  XIAO ESP32-S3 Plus + 1.47 Touch Display
  SD BMP Reader Diagnostic v0.8

  Why this version exists:
    Previous versions proved SD.begin() is OK, but SD.open("test.bmp") did not find/open
    the image reliably. This version keeps the SD session active and does all SD work
    before switching back to LCD drawing.

  Test target:
    Put a BMP file in the SD card root directory.

  Preferred file names:
    /test.bmp
    /TEST.BMP
    /image.bmp
    /IMAGE.BMP
    /image001.bmp
    /IMAGE001.BMP

  Preferred BMP:
    24-bit uncompressed BMP
    172 x 320

  Hardware pin map:
    LCD CS       = D2
    LCD DC       = D3
    SD CS        = D6
    LCD/SD SCK   = D8
    SD MISO      = D9
    LCD/SD MOSI  = D10
    LCD RST      = D17
    LCD BL       = D18
*/

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
#error "This firmware is for XIAO ESP32-S3 Plus. Please select an ESP32-S3 board."
#endif

#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include <ctype.h>

// ========================= Pin map =========================

static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t SD_CS_PIN     = D6;
static constexpr uint8_t SPI_SCK_PIN   = D8;
static constexpr uint8_t SD_MISO_PIN   = D9;
static constexpr uint8_t SPI_MOSI_PIN  = D10;
static constexpr uint8_t LCD_RST_PIN   = D17;
static constexpr uint8_t LCD_BL_PIN    = D18;

// ========================= LCD =========================

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;

Arduino_DataBus *lcdBus = new Arduino_SWSPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  SPI_SCK_PIN,
  SPI_MOSI_PIN,
  GFX_NOT_DEFINED
);

Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus,
  LCD_RST_PIN,
  0,
  false,
  LCD_W,
  LCD_H,
  34, 0,
  34, 0
);

// ========================= Buffers =========================

static constexpr int BMP_MAX_SRC_W = 360;
static uint8_t rowBuf[BMP_MAX_SRC_W * 4 + 8];
static uint16_t frameBuf[LCD_W * LCD_H];

static char g_loadedPath[64] = "";
static uint32_t g_sdFreq = 0;
static uint32_t g_readMs = 0;
static uint32_t g_totalMs = 0;
static bool g_sdMounted = false;
static bool g_bmpLoaded = false;

// ========================= Colors =========================

static constexpr uint16_t C_BLACK  = RGB565_BLACK;
static constexpr uint16_t C_WHITE  = RGB565_WHITE;
static constexpr uint16_t C_GREEN  = RGB565_LIGHTGREEN;
static constexpr uint16_t C_CYAN   = RGB565_CYAN;
static constexpr uint16_t C_YELLOW = RGB565_YELLOW;
static constexpr uint16_t C_RED    = RGB565_RED;
static constexpr uint16_t C_GRAY   = 0x8410;

// ========================= Bus helpers =========================

static void spiDevicesIdle() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(SD_CS_PIN, HIGH);
}

static void restoreLcdSwSpiPins() {
  pinMode(SPI_SCK_PIN, OUTPUT);
  pinMode(SPI_MOSI_PIN, OUTPUT);
  digitalWrite(SPI_SCK_PIN, LOW);
  digitalWrite(SPI_MOSI_PIN, LOW);
}

static void acquireForLcd() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  restoreLcdSwSpiPins();
  delayMicroseconds(2);
}

static void acquireForSd() {
  spiDevicesIdle();
  pinMode(SD_MISO_PIN, INPUT_PULLUP);
  SPI.begin(SPI_SCK_PIN, SD_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
  delayMicroseconds(2);
}

static void endSdAndReturnToLcd() {
  if (g_sdMounted) {
    SD.end();
    g_sdMounted = false;
    delay(30);
  }
  SPI.end();
  spiDevicesIdle();
  restoreLcdSwSpiPins();
}

static void lcdWriteMadctlFix() {
  acquireForLcd();
  lcdBus->beginWrite();
  lcdBus->writeC8D8(0x36, 0x48);
  lcdBus->endWrite();
}

static bool initLcd() {
  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, 255);

  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(10);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(30);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(150);

  acquireForLcd();

  if (!gfx->begin()) {
    Serial.println("[LCD] begin failed");
    return false;
  }

  lcdWriteMadctlFix();
  gfx->fillScreen(C_BLACK);
  return true;
}

static void drawTextScreen(const char *line1, const char *line2, const char *line3) {
  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(8, 16);
  gfx->print("Hello,XIAO!");

  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(8, 52);
  gfx->print("ESP32-S3 1.47 BMP Test");

  gfx->drawFastHLine(8, 72, 156, C_GRAY);

  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(10, 100);
  gfx->print(line1);

  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(10, 122);
  gfx->print(line2);

  gfx->setCursor(10, 144);
  gfx->print(line3);
}

static void drawLoadedImage() {
  acquireForLcd();
  gfx->draw16bitRGBBitmap(0, 0, frameBuf, LCD_W, LCD_H);

  gfx->fillRect(0, 0, LCD_W, 18, C_BLACK);
  gfx->drawFastHLine(0, 18, LCD_W, C_GREEN);
  gfx->setTextSize(1);

  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(2, 3);
  gfx->print("BMP OK");

  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(56, 3);
  gfx->print(g_readMs);
  gfx->print("ms");

  gfx->fillRect(0, LCD_H - 14, LCD_W, 14, C_BLACK);
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(2, LCD_H - 11);
  gfx->print(g_loadedPath);
}

// ========================= SD =========================

static bool mountSd() {
  if (g_sdMounted) return true;

  acquireForSd();
  delay(30);

  const uint32_t freqs[] = {4000000, 1000000, 400000};

  for (uint8_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
    spiDevicesIdle();

    Serial.printf("[SD] Trying %lu Hz...\n", (unsigned long)freqs[i]);
    Serial.flush();

    if (SD.begin(SD_CS_PIN, SPI, freqs[i])) {
      g_sdMounted = true;
      g_sdFreq = freqs[i];

      uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);
      Serial.printf("[SD] OK card=%llu MB freq=%lu Hz\n",
                    cardSizeMB, (unsigned long)freqs[i]);
      return true;
    }

    Serial.printf("[SD] failed at %lu Hz\n", (unsigned long)freqs[i]);
  }

  Serial.println("[SD] Mount failed");
  return false;
}

static void sdWriteReadProbe() {
  // This is a quick proof that the filesystem can open/write/read a normal file.
  // If this fails, the problem is not BMP format; it is SD filesystem/open path.
  const char *probe = "/SDPROBE.TXT";

  Serial.println("[PROBE] write /SDPROBE.TXT");
  Serial.flush();

  SD.remove(probe);
  File w = SD.open(probe, FILE_WRITE);
  if (!w) {
    Serial.println("[PROBE] write open FAILED");
    return;
  }

  w.println("XIAO ESP32-S3 SD probe OK");
  w.flush();
  w.close();
  Serial.println("[PROBE] write OK");

  Serial.println("[PROBE] read /SDPROBE.TXT");
  Serial.flush();

  File r = SD.open(probe, FILE_READ);
  if (!r) {
    Serial.println("[PROBE] read open FAILED");
    return;
  }

  String firstLine = r.readStringUntil('\n');
  r.close();

  Serial.print("[PROBE] read OK: ");
  Serial.println(firstLine);
}

// ========================= BMP =========================

static uint16_t readLE16(File &f) {
  uint8_t b[2];
  if (f.read(b, 2) != 2) return 0;
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static uint32_t readLE32(File &f) {
  uint8_t b[4];
  if (f.read(b, 4) != 4) return 0;
  return (uint32_t)b[0] |
         ((uint32_t)b[1] << 8) |
         ((uint32_t)b[2] << 16) |
         ((uint32_t)b[3] << 24);
}

static uint16_t rgb888To565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) |
         ((g & 0xFC) << 3) |
         (b >> 3);
}

static bool decodeBmpFile(File &f, const char *path) {
  uint32_t t0 = millis();

  for (int i = 0; i < LCD_W * LCD_H; i++) {
    frameBuf[i] = C_BLACK;
  }

  uint16_t signature = readLE16(f);
  if (signature != 0x4D42) {
    Serial.print("[BMP] not BMP signature: 0x");
    Serial.println(signature, HEX);
    return false;
  }

  (void)readLE32(f); // file size
  (void)readLE16(f); // reserved 1
  (void)readLE16(f); // reserved 2
  uint32_t dataOffset = readLE32(f);

  uint32_t headerSize = readLE32(f);
  if (headerSize < 40) {
    Serial.println("[BMP] unsupported header");
    return false;
  }

  int32_t srcW = (int32_t)readLE32(f);
  int32_t srcHRaw = (int32_t)readLE32(f);
  uint16_t planes = readLE16(f);
  uint16_t bpp = readLE16(f);
  uint32_t compression = readLE32(f);

  if (planes != 1 || compression != 0) {
    Serial.println("[BMP] only uncompressed BI_RGB BMP supported");
    return false;
  }

  if (!(bpp == 24 || bpp == 32 || bpp == 16)) {
    Serial.print("[BMP] unsupported bpp=");
    Serial.println(bpp);
    return false;
  }

  bool topDown = srcHRaw < 0;
  int32_t srcH = topDown ? -srcHRaw : srcHRaw;

  if (srcW <= 0 || srcH <= 0) {
    Serial.println("[BMP] bad size");
    return false;
  }

  if (srcW > BMP_MAX_SRC_W) {
    Serial.print("[BMP] width too large=");
    Serial.println(srcW);
    return false;
  }

  uint32_t rowSize = ((uint32_t)srcW * bpp + 31) / 32 * 4;
  if (rowSize > sizeof(rowBuf)) {
    Serial.println("[BMP] row buffer too small");
    return false;
  }

  int drawW = min((int)srcW, LCD_W);
  int drawH = min((int)srcH, LCD_H);
  int cropX = srcW > LCD_W ? (srcW - LCD_W) / 2 : 0;
  int cropY = srcH > LCD_H ? (srcH - LCD_H) / 2 : 0;
  int dstX = srcW < LCD_W ? (LCD_W - srcW) / 2 : 0;
  int dstY = srcH < LCD_H ? (LCD_H - srcH) / 2 : 0;

  Serial.printf("[BMP] header OK path=%s size=%ldx%ld bpp=%u row=%lu offset=%lu\n",
                path, (long)srcW, (long)srcH, bpp,
                (unsigned long)rowSize, (unsigned long)dataOffset);
  Serial.flush();

  for (int y = 0; y < drawH; y++) {
    int srcY = cropY + y;
    int fileY = topDown ? srcY : (srcH - 1 - srcY);
    uint32_t rowOffset = dataOffset + (uint32_t)fileY * rowSize;

    if (!f.seek(rowOffset)) {
      Serial.print("[BMP] seek failed y=");
      Serial.println(y);
      return false;
    }

    int n = f.read(rowBuf, rowSize);
    if (n != (int)rowSize) {
      Serial.print("[BMP] row read failed y=");
      Serial.print(y);
      Serial.print(" got=");
      Serial.println(n);
      return false;
    }

    for (int x = 0; x < drawW; x++) {
      int srcX = cropX + x;
      uint16_t px;

      if (bpp == 24) {
        int p = srcX * 3;
        uint8_t b = rowBuf[p + 0];
        uint8_t g = rowBuf[p + 1];
        uint8_t r = rowBuf[p + 2];
        px = rgb888To565(r, g, b);
      } else if (bpp == 32) {
        int p = srcX * 4;
        uint8_t b = rowBuf[p + 0];
        uint8_t g = rowBuf[p + 1];
        uint8_t r = rowBuf[p + 2];
        px = rgb888To565(r, g, b);
      } else {
        int p = srcX * 2;
        px = (uint16_t)rowBuf[p] | ((uint16_t)rowBuf[p + 1] << 8);
      }

      frameBuf[(dstY + y) * LCD_W + (dstX + x)] = px;
    }

    if ((y & 0x1F) == 0) yield();
  }

  g_readMs = millis() - t0;
  strncpy(g_loadedPath, path, sizeof(g_loadedPath) - 1);
  g_loadedPath[sizeof(g_loadedPath) - 1] = 0;

  return true;
}

static bool tryOpenBmpPath(const char *path) {
  Serial.print("[IMG] open start ");
  Serial.println(path);
  Serial.flush();

  File f = SD.open(path, FILE_READ);

  Serial.print("[IMG] open done  ");
  Serial.println(path);
  Serial.flush();

  if (!f) {
    Serial.print("[IMG] miss ");
    Serial.println(path);
    return false;
  }

  if (f.isDirectory()) {
    Serial.print("[IMG] is directory ");
    Serial.println(path);
    f.close();
    return false;
  }

  Serial.print("[IMG] file size=");
  Serial.println((unsigned long)f.size());

  bool ok = decodeBmpFile(f, path);
  f.close();

  if (ok) {
    Serial.print("[IMG] BMP loaded ");
    Serial.println(path);
    return true;
  }

  Serial.print("[IMG] BMP decode failed ");
  Serial.println(path);
  return false;
}

static bool loadFirstBmpFromSd() {
  if (!mountSd()) return false;

  // Keep this probe. It tells us whether File open/read/write works at all.
  sdWriteReadProbe();

  const char *paths[] = {
    "/test.bmp",
    "/TEST.BMP",
    "/image.bmp",
    "/IMAGE.BMP",
    "/image001.bmp",
    "/IMAGE001.BMP",
    "/image002.bmp",
    "/IMAGE002.BMP",
    "/1.bmp",
    "/1.BMP",
    "/2.bmp",
    "/2.BMP",
    "test.bmp",
    "TEST.BMP",
    "image.bmp",
    "IMAGE.BMP",
    "image001.bmp",
    "IMAGE001.BMP",
    "image002.bmp",
    "IMAGE002.BMP",
    "1.bmp",
    "1.BMP",
    "2.bmp",
    "2.BMP"
  };

  for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
    if (tryOpenBmpPath(paths[i])) return true;
    yield();
  }

  Serial.println("[IMG] No BMP file loaded");
  return false;
}

// ========================= Arduino =========================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus 1.47 SD BMP Reader Diagnostic v0.8 ===");
  Serial.println("[PIN] LCD CS=D2 DC=D3 SCK=D8 MOSI=D10 RST=D17 BL=D18");
  Serial.println("[PIN] SD  CS=D6 SCK=D8 MISO=D9 MOSI=D10");
  Serial.println("[IMG] Put /test.bmp in SD root");

  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  spiDevicesIdle();

  if (!initLcd()) {
    Serial.println("[LCD] failed");
    while (1) delay(1000);
  }

  drawTextScreen("Mounting SD...", "Do not remove card", "");

  uint32_t t0 = millis();
  g_bmpLoaded = loadFirstBmpFromSd();
  g_totalMs = millis() - t0;

  endSdAndReturnToLcd();

  if (g_bmpLoaded) {
    Serial.printf("[DONE] BMP loaded path=%s readMs=%lu totalMs=%lu\n",
                  g_loadedPath, (unsigned long)g_readMs, (unsigned long)g_totalMs);
    drawLoadedImage();
  } else {
    Serial.println("[DONE] No BMP loaded");
    drawTextScreen("No BMP loaded", "Check Serial Monitor", "Use /test.bmp");
  }
}

void loop() {
  delay(1000);
}
