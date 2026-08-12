/*
  XIAO ESP32-S3 Plus 1.47 Inch Display — SD Photo Frame

  Reads 24-bit uncompressed BMP images from the SD card and displays them
  on the 172x320 LCD. Images auto-advance every 2 seconds.

  Usage:
    1. Format a microSD card as FAT32.
    2. Put 24-bit uncompressed BMP files in the root directory.
       Recommended size: 172x320 pixels (exact full-screen fit).
       Larger images up to 360px wide are center-cropped.
    3. Insert the SD card, power on the board.

  Required Arduino libraries:
    - TFT_eSPI / Seeed_GFX
    - SdFat

  Hardware pins (same as nRF52840 version):
    LCD CS   = D2
    LCD DC   = D3
    SD CS    = D6
    SCK      = D8
    MISO     = D9
    MOSI     = D10
    LCD RST  = D17
    LCD BL   = D18
*/

#include <Arduino.h>
#include "driver.h"
#include <SPI.h>
#include <SdFat.h>
#include <TFT_eSPI.h>
#include <cstring>

// ========================= Pin map =========================

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t SD_CS_PIN    = D6;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t SD_MISO_PIN  = D9;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

// ========================= Display =========================

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;

TFT_eSPI tft;

// ========================= SD card =========================

SdFat sdCard;
static uint32_t g_sdFreq = 400000;  // stored after successful mount

// SD frequencies are tried low-to-high in beginSd().

// ========================= BMP config =========================

static constexpr int MAX_IMAGES   = 24;
static constexpr int MAX_PATH_LEN = 64;

// Row buffer sized for 360px-wide 32-bit BMP rows.
static constexpr int BMP_MAX_SRC_W = 360;
static uint8_t  rowBuf[BMP_MAX_SRC_W * 4 + 8];
static uint16_t lineBuf[LCD_W];

char  g_imagePaths[MAX_IMAGES][MAX_PATH_LEN];
int   g_imageCount = 0;
int   g_imageIndex = 0;

// ========================= Colors =========================

static constexpr uint16_t C_BLACK  = TFT_BLACK;
static constexpr uint16_t C_WHITE  = TFT_WHITE;
static constexpr uint16_t C_GREEN  = TFT_GREEN;
static constexpr uint16_t C_CYAN   = TFT_CYAN;
static constexpr uint16_t C_YELLOW = TFT_YELLOW;
static constexpr uint16_t C_RED    = TFT_RED;
static constexpr uint16_t C_GRAY   = 0x8410;

// ========================= Bus helpers =========================

// LCD and SD card share a single SPI bus.  Only one chip-select
// may be active at a time.  These helpers assert both CS pins high
// before pulling the correct one low, avoiding bus contention.

// IMPORTANT: Do NOT call pinMode() here!  On ESP32-S3, pinMode() can
// disconnect the pin from the SPI controller (e.g. GPIO6 = FSPICS1),
// breaking SdFat's CS control.  Pins are already set to OUTPUT in
// preparePins() during startup.  We ONLY deselect via digitalWrite(HIGH).
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

// ========================= LCD init =========================

static void preparePins() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_SCK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_SCK_PIN, LOW);
  digitalWrite(LCD_MOSI_PIN, LOW);
  digitalWrite(SD_CS_PIN, HIGH);
}

static void hardResetPanel() {
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(20);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(80);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(180);
}

static void applyPanelFix() {
  // The JD9853A panel on the 1.47-inch board needs MADCTL=0x48
  // for correct colour and orientation.
  acquireForLcd();
  tft.writecommand(0x36);
  tft.writedata(0x48);
  delay(10);
}

static void initLcd() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);
  hardResetPanel();
  tft.init();
  tft.setSwapBytes(true);
  tft.setRotation(0);
  applyPanelFix();
  tft.invertDisplay(false);
  tft.fillScreen(C_BLACK);
}

// ========================= UI helpers =========================

static void showMessage(const char *title, const String &line1,
                        const String &line2 = "") {
  acquireForLcd();
  tft.fillScreen(C_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(C_CYAN, C_BLACK);
  tft.setCursor(4, 4);
  tft.print(title);
  tft.drawFastHLine(0, 18, LCD_W, C_CYAN);
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(4, 34);
  tft.print(line1);
  if (line2.length()) {
    tft.setCursor(4, 50);
    tft.print(line2);
  }
}

static void drawStatusBar(const char *path, bool ok, uint32_t renderMs) {
  SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  tft.fillRect(0, 0, LCD_W, 18, C_BLACK);
  tft.drawFastHLine(0, 18, LCD_W, ok ? C_GREEN : C_RED);
  tft.setTextSize(1);
  tft.setTextColor(ok ? C_GREEN : C_RED, C_BLACK);
  tft.setCursor(2, 3);
  tft.print(ok ? "SD OK" : "SD ERR");

  tft.setTextColor(C_YELLOW, C_BLACK);
  tft.setCursor(80, 3);
  tft.print(renderMs);
  tft.print("ms");

  // File name at the bottom.
  tft.fillRect(0, LCD_H - 14, LCD_W, 14, C_BLACK);
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(2, LCD_H - 11);
  String name = String(path);
  int slash = name.lastIndexOf('/');
  if (slash >= 0) name = name.substring(slash + 1);
  if (name.length() > 20) name = name.substring(0, 20);
  tft.print(name);
  SPI.endTransaction();
}

// ========================= SD card =========================

static bool beginSd() {
  Serial.println("[SD] acquireForSd()...");
  acquireForSd();

  Serial.println("[SD] SPI.begin()...");
  SPI.begin();
  Serial.println("[SD] SPI.begin() done");

  // Print pin states for debugging
  Serial.print("[SD] SD_CS_PIN(D6)=");
  Serial.print(SD_CS_PIN);
  Serial.print("  LCD_CS_PIN(D2)=");
  Serial.print(LCD_CS_PIN);
  Serial.print("  MISO=D9  MOSI=D10  SCK=D8");
  Serial.println();

  // Dump SD_CS pin value
  pinMode(SD_CS_PIN, INPUT);
  int sdCsState = digitalRead(SD_CS_PIN);
  Serial.print("[SD] SD_CS pin state (before begin): ");
  Serial.println(sdCsState);

  // Try SHARED_SPI — DEDICATED_SPI on ESP32-S3 hangs during f.read()
  // for unknown driver-level reasons. SHARED_SPI uses proper SPI
  // transactions, which should work since TFT_eSPI has
  // SUPPORT_TRANSACTIONS=0 (no mutex contention).
  const uint32_t freqs[] = {400000, 1000000, 4000000, 8000000};
  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
    SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, freqs[i], &SPI);
    Serial.print("[SD] trying ");
    Serial.print(freqs[i]);
    Serial.print(" Hz (");
    Serial.print(i+1);
    Serial.print("/");
    Serial.print(sizeof(freqs)/sizeof(freqs[0]));
    Serial.print(")... ");

    // Try to begin — if this hangs, we'll at least know which freq
    bool ok = sdCard.begin(cfg);

    if (ok) {
      Serial.print("OK! mounted @ ");
      Serial.print(freqs[i]);
      Serial.println(" Hz");
      g_sdFreq = freqs[i];   // remember for later acquireForSd()
      return true;
    } else {
      Serial.println("FAILED");
    }
    delay(30);
  }

  Serial.println("[SD] All frequencies failed.");
  return false;
}

// ========================= File scanning =========================

static bool endsWithBmp(const char *name) {
  size_t n = strlen(name);
  if (n < 4) return false;
  return tolower(name[n - 4]) == '.' &&
         tolower(name[n - 3]) == 'b' &&
         tolower(name[n - 2]) == 'm' &&
         tolower(name[n - 1]) == 'p';
}

static void scanImages() {
  g_imageCount = 0;
  File32 root;
  if (!root.open("/")) return;

  File32 entry;
  while (g_imageCount < MAX_IMAGES && entry.openNext(&root, O_RDONLY)) {
    if (!entry.isDir()) {
      char name[MAX_PATH_LEN];
      entry.getName(name, sizeof(name));
      if (endsWithBmp(name)) {
        snprintf(g_imagePaths[g_imageCount], MAX_PATH_LEN, "/%s", name);
        Serial.print("[IMG] ");
        Serial.println(g_imagePaths[g_imageCount]);
        g_imageCount++;
      }
    }
    entry.close();
  }
  root.close();
}

// ========================= BMP reader =========================

static uint16_t readLE16(File32 &f) {
  uint8_t b[2];
  if (f.read(b, 2) != 2) return 0;
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static uint32_t readLE32(File32 &f) {
  uint8_t b[4];
  if (f.read(b, 4) != 4) return 0;
  return (uint32_t)b[0] |
         ((uint32_t)b[1] << 8) |
         ((uint32_t)b[2] << 16) |
         ((uint32_t)b[3] << 24);
}

static uint16_t rgb888To565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static bool drawBmp(const char *path) {
  Serial.println("[BMP] opening file...");
  acquireForSd();   // switch SPI bus to SD before reading file
  File32 f;
  if (!f.open(path, O_RDONLY)) {
    Serial.print("[BMP] open failed: ");
    Serial.println(path);
    return false;
  }
  Serial.println("[BMP] file opened OK");
  Serial.print("[BMP] file size: ");
  Serial.println(f.size());

  uint32_t t0 = millis();

  // --- Read BMP header ---
  Serial.println("[BMP] reading header...");
  f.seekSet(0);  // ensure we're at the start of the file
  Serial.println("[BMP] seekSet(0) done, now reading...");
  // Read BMP header — inline reads instead of readLE16/readLE32
  // (workaround for SdFat File32::read() hang when called through helper funcs)
  uint32_t dataOffset, headerSize, compression;
  int32_t  srcW, srcHRaw;
  uint16_t planes, bpp;
  uint8_t  hdr_b[4];
  int n;

  // "BM" signature
  n = f.read(hdr_b, 2);
  // Compare bytes directly to avoid any integer-promotion / shift bugs.
  if (n != 2 || hdr_b[0] != 0x42 || hdr_b[1] != 0x4D) {
    Serial.print("[BMP] not a BM file (n=");
    Serial.print(n);
    Serial.print(" b0=0x");
    Serial.print(hdr_b[0], HEX);
    Serial.print(" b1=0x");
    Serial.print(hdr_b[1], HEX);
    Serial.println(")");
    f.close();
    return false;
  }
  Serial.println("[BMP] BM signature OK");

  // file size (skip) + reserved 1+2 (skip)
  n = f.read(hdr_b, 4);
  n = f.read(hdr_b, 4);

  // data offset
  n = f.read(hdr_b, 4);
  dataOffset = (uint32_t)hdr_b[0] | ((uint32_t)hdr_b[1] << 8) |
               ((uint32_t)hdr_b[2] << 16) | ((uint32_t)hdr_b[3] << 24);

  // header size
  n = f.read(hdr_b, 4);
  headerSize = (uint32_t)hdr_b[0] | ((uint32_t)hdr_b[1] << 8) |
               ((uint32_t)hdr_b[2] << 16) | ((uint32_t)hdr_b[3] << 24);

  if (headerSize < 40) {
    Serial.println("[BMP] unsupported header size");
    f.close();
    return false;
  }

  // width, height
  n = f.read(hdr_b, 4);
  srcW = (int32_t)((uint32_t)hdr_b[0] | ((uint32_t)hdr_b[1] << 8) |
                   ((uint32_t)hdr_b[2] << 16) | ((uint32_t)hdr_b[3] << 24));
  n = f.read(hdr_b, 4);
  srcHRaw = (int32_t)((uint32_t)hdr_b[0] | ((uint32_t)hdr_b[1] << 8) |
                      ((uint32_t)hdr_b[2] << 16) | ((uint32_t)hdr_b[3] << 24));

  // planes + bpp
  n = f.read(hdr_b, 2);
  planes = (uint16_t)hdr_b[0] | ((uint16_t)hdr_b[1] << 8);
  n = f.read(hdr_b, 2);
  bpp = (uint16_t)hdr_b[0] | ((uint16_t)hdr_b[1] << 8);

  // compression
  n = f.read(hdr_b, 4);
  compression = (uint32_t)hdr_b[0] | ((uint32_t)hdr_b[1] << 8) |
                ((uint32_t)hdr_b[2] << 16) | ((uint32_t)hdr_b[3] << 24);

  if (planes != 1 || compression != 0) {
    Serial.println("[BMP] only uncompressed BMP supported");
    f.close();
    return false;
  }

  if (!(bpp == 16 || bpp == 24 || bpp == 32)) {
    Serial.print("[BMP] unsupported bpp: ");
    Serial.println(bpp);
    f.close();
    return false;
  }

  bool topDown = (srcHRaw < 0);
  int32_t srcH = topDown ? -srcHRaw : srcHRaw;

  Serial.print("[BMP] ");
  Serial.print(srcW);
  Serial.print("x");
  Serial.print(srcH);
  Serial.print(" bpp=");
  Serial.print(bpp);
  Serial.println(" header OK");

  if (srcW <= 0 || srcH <= 0 || srcW > BMP_MAX_SRC_W) {
    Serial.print("[BMP] bad size: ");
    Serial.print(srcW);
    Serial.print("x");
    Serial.println(srcH);
    f.close();
    return false;
  }

  uint32_t rowSize = ((uint32_t)srcW * bpp + 31) / 32 * 4;
  Serial.print("[BMP] dataOffset="); Serial.print(dataOffset);
  Serial.print(" rowSize="); Serial.println(rowSize);
  if (rowSize > sizeof(rowBuf)) {
    Serial.println("[BMP] row buffer overflow");
    f.close();
    return false;
  }

  // --- Compute crop / centering ---
  int drawW = min((int)srcW, LCD_W);
  int drawH = min((int)srcH, LCD_H);
  int cropX = (srcW > LCD_W) ? (srcW - LCD_W) / 2 : 0;
  int cropY = (srcH > LCD_H) ? (srcH - LCD_H) / 2 : 0;
  int dstX  = (srcW < LCD_W) ? (LCD_W - srcW) / 2 : 0;
  int dstY  = (srcH < LCD_H) ? (LCD_H - srcH) / 2 : 0;

  Serial.println("[BMP] fillScreen BLACK...");
  SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  tft.fillScreen(C_BLACK);
  SPI.endTransaction();
  Serial.println("[BMP] fillScreen done, starting row loop");

  // --- Draw row by row ---
  for (int y = 0; y < drawH; y++) {
    int srcY  = cropY + y;
    int fileY = topDown ? srcY : (srcH - 1 - srcY);
    uint32_t rowOffset = dataOffset + (uint32_t)fileY * rowSize;

    if (y == 0) {
      Serial.print("[BMP] row 0: fileY="); Serial.print(fileY);
      Serial.print(" rowOffset="); Serial.println(rowOffset);
    }

    acquireForSd();   // switch SPI back to SD before reading row data
    if (y == 0) Serial.println("[BMP] seekSet...");
    if (!f.seekSet(rowOffset)) {
      Serial.print("[BMP] seek failed at row "); Serial.println(y);
      f.close();
      return false;
    }
    if (y == 0) Serial.println("[BMP] seekSet OK, reading row...");
    if (f.read(rowBuf, rowSize) != (int)rowSize) {
      Serial.print("[BMP] row read failed at row "); Serial.println(y);
      f.close();
      return false;
    }
    if (y == 0) Serial.println("[BMP] row read OK, converting pixels...");

    // Convert one row from BMP pixel format to RGB565.
    for (int x = 0; x < drawW; x++) {
      int srcX = cropX + x;
      if (bpp == 24) {
        int p  = srcX * 3;
        uint8_t b = rowBuf[p];
        uint8_t g = rowBuf[p + 1];
        uint8_t r = rowBuf[p + 2];
        lineBuf[x] = rgb888To565(r, g, b);
      } else if (bpp == 32) {
        int p  = srcX * 4;
        uint8_t b = rowBuf[p];
        uint8_t g = rowBuf[p + 1];
        uint8_t r = rowBuf[p + 2];
        lineBuf[x] = rgb888To565(r, g, b);
      } else {
        // 16-bit BMP — assume RGB565.
        int p = srcX * 2;
        lineBuf[x] = (uint16_t)rowBuf[p] | ((uint16_t)rowBuf[p + 1] << 8);
      }
    }

    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    tft.pushImage(dstX, dstY + y, drawW, 1, lineBuf);
    SPI.endTransaction();
  }

  Serial.println("[BMP] row loop complete");
  f.close();
  Serial.println("[BMP] file closed, drawing status bar...");

  uint32_t renderMs = millis() - t0;
  Serial.print("[BMP] ");
  Serial.print(path);
  Serial.print("  ");
  Serial.print(srcW);
  Serial.print("x");
  Serial.print(srcH);
  Serial.print(" bpp=");
  Serial.print(bpp);
  Serial.print("  ");
  Serial.print(renderMs);
  Serial.println(" ms");

  drawStatusBar(path, true, renderMs);
  Serial.println("[BMP] done");
  return true;
}

// ========================= Setup / loop =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus 1.47 SD Photo Frame ===");
  Serial.print("[SETUP] Chip: ");
  Serial.println(ESP.getChipModel());
  Serial.print("[SETUP] Flash size: ");
  Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
  Serial.println(" MB");
  Serial.print("[SETUP] PSRAM: ");
  Serial.print(ESP.getPsramSize() / 1024 / 1024);
  Serial.println(" MB");

  Serial.println("[SETUP] Step 1/6: preparePins()...");
  preparePins();
  Serial.println("[SETUP] Step 1/6: done");

  Serial.println("[SETUP] Step 2/6: initLcd()...");
  initLcd();
  Serial.println("[SETUP] Step 2/6: done");

  Serial.println("[SETUP] Step 3/6: showMessage()...");
  showMessage("SD Photo Frame", "Mounting SD card...");
  Serial.println("[SETUP] Step 3/6: done");

  Serial.println("[SETUP] Step 4/6: beginSd()...");
  if (!beginSd()) {
    Serial.println("[SETUP] Step 4/6: FAILED - SD card mount failed");
    showMessage("SD Photo Frame", "SD card mount failed",
                "Insert FAT32 card with BMP files");
    while (1) delay(1000);
  }
  Serial.println("[SETUP] Step 4/6: done - SD mounted OK");

  Serial.println("[SETUP] Step 5/6: scanImages()...");
  scanImages();
  Serial.println("[SETUP] Step 5/6: done");

  // === RAW SECTOR READ TEST ===
  // Bypass the FAT layer entirely — test if low-level SPI reads work.
  Serial.println("[TEST] Raw sector read test (bypassing FAT)...");
  uint8_t rawBuf[512];
  memset(rawBuf, 0, 512);
  uint32_t testSectors[] = {0, 1, 100, 1000};
  for (int ti = 0; ti < 4; ti++) {
    Serial.printf("[TEST]   reading sector %u... ", testSectors[ti]);
    if (sdCard.card()->readSector(testSectors[ti], rawBuf)) {
      Serial.printf("OK (first byte: 0x%02X)\n", rawBuf[0]);
    } else {
      Serial.println("FAILED");
    }
  }
  Serial.println("[TEST] Raw sector test complete");
  // === END RAW SECTOR TEST ===

  // === FILE READ TEST ===
  // Now test if File32::read() works on a specific file.
  // === PRECISE REPLICATION of drawBmp's readLE16 ===
  // Same file, same read size, same seekSet(0) before read.
  Serial.println("[TEST] Replicating drawBmp readLE16 exactly...");
  {
    acquireForSd();  // exactly what drawBmp does
    File32 testFile;
    Serial.println("[TEST]   opening /Atest.bmp...");
    if (testFile.open("/Atest.bmp", O_RDONLY)) {
      Serial.print("[TEST]   size=");
      Serial.print(testFile.size());
      Serial.print(" avail=");
      Serial.println(testFile.available());

      Serial.println("[TEST]   seekSet(0)...");
      testFile.seekSet(0);
      Serial.println("[TEST]   seekSet(0) done");

      Serial.println("[TEST]   reading 2 bytes (like readLE16)...");
      uint8_t b[2];
      int n = testFile.read(b, 2);
      Serial.print("[TEST]   read returned: ");
      Serial.print(n);
      Serial.print(" bytes: 0x");
      Serial.print(b[0], HEX);
      Serial.print(" 0x");
      Serial.println(b[1], HEX);

      testFile.close();
    } else {
      Serial.println("[TEST]   open FAILED");
    }
  }
  Serial.println("[TEST] Replication test complete");
  // === END FILE READ TEST ===

  // === DIRECTORY-ENTRY READ TEST ===
  // Same approach as scanImages — open via root directory, then read.
  Serial.println("[TEST] Dir-entry read test...");
  File32 root2;
  if (root2.open("/")) {
    File32 entry;
    while (entry.openNext(&root2, O_RDONLY)) {
      char name[64];
      entry.getName(name, sizeof(name));
      if (strstr(name, ".bmp") || strstr(name, ".BMP")) {
        Serial.print("[TEST]   found via dir: ");
        Serial.println(name);
        Serial.print("[TEST]   size=");
        Serial.print(entry.size());
        Serial.print(" avail=");
        Serial.println(entry.available());
        uint8_t b;
        Serial.println("[TEST]   calling read(&b,1)...");
        int n = entry.read(&b, 1);
        Serial.print("[TEST]   read returned: ");
        Serial.println(n);
        entry.close();
        break;  // just test first file
      }
      entry.close();
    }
    root2.close();
  }
  Serial.println("[TEST] Dir-entry read test complete");
  // === END DIRECTORY-ENTRY READ TEST ===

  Serial.println("[SETUP] Step 6/6: check results...");
  if (g_imageCount == 0) {
    showMessage("SD Photo Frame", "No BMP files found",
                "Put 24-bit BMP images in SD root");
    Serial.println("[SETUP] No BMP files found");
  } else {
    Serial.print("[IMG] found ");
    Serial.print(g_imageCount);
    Serial.println(" images");
  }
  // === TEST: Call drawBmp from setup() to see if it works here ===
  if (g_imageCount > 0) {
    Serial.println("[SETUP] Testing drawBmp() from setup()...");
    bool ok = drawBmp(g_imagePaths[0]);
    Serial.print("[SETUP] drawBmp() from setup returned: ");
    Serial.println(ok ? "TRUE" : "FALSE");
  }
  Serial.println("[SETUP] Setup() complete, entering loop()");
}

void loop() {
  Serial.println("[LOOP] loop() iteration start");
  if (g_imageCount == 0) {
    delay(1000);
    return;
  }

  const char *path = g_imagePaths[g_imageIndex];
  Serial.print("[LOOP] drawing: ");
  Serial.println(path);

  bool ok = drawBmp(path);
  if (!ok) {
    // Show error on screen so it's clear something went wrong.
    showMessage("SD Photo Frame", "BMP load failed", path);
    delay(2000);
  } else {
    // Wait 2 seconds, then advance to the next image.
    delay(2000);
  }

  g_imageIndex = (g_imageIndex + 1) % g_imageCount;
}
