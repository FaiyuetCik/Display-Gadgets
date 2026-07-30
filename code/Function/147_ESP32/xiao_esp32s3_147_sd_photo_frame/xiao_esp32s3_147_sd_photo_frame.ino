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

// ESP32-S3 SPI handles 8 MHz reliably without the gradual
// frequency fallback that nRF52840 needs.
static constexpr uint32_t SD_FREQ = 8000000;

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

static void acquireForLcd() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static void acquireForSd() {
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  pinMode(SD_CS_PIN, OUTPUT);
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
  acquireForLcd();
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
}

// ========================= SD card =========================

static bool beginSd() {
  acquireForSd();
  // ESP32 SPI::begin() skips re-init if already started (the _spi guard).
  // TFT_eSPI started SPI with MISO = -1, so we must end() first to force
  // a fresh begin() that includes the MISO pin the SD card needs.
  SPI.end();
  delay(5);
  SPI.begin(LCD_SCK_PIN, SD_MISO_PIN, LCD_MOSI_PIN);

  // ESP32-S3 SPI is stable at 8 MHz — try once.
  SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, SD_FREQ, &SPI);
  if (sdCard.begin(cfg)) {
    Serial.print("[SD] mounted @ ");
    Serial.print(SD_FREQ);
    Serial.println(" Hz");
    return true;
  }

  // Fallback: try lower frequencies.
  const uint32_t fallback[] = {4000000, 1000000, 400000};
  for (size_t i = 0; i < sizeof(fallback) / sizeof(fallback[0]); ++i) {
    SdSpiConfig fb(SD_CS_PIN, SHARED_SPI, fallback[i], &SPI);
    if (sdCard.begin(fb)) {
      Serial.print("[SD] mounted @ ");
      Serial.print(fallback[i]);
      Serial.println(" Hz (fallback)");
      return true;
    }
    delay(50);
  }

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
  File32 f;
  if (!f.open(path, O_RDONLY)) {
    Serial.print("[BMP] open failed: ");
    Serial.println(path);
    return false;
  }

  uint32_t t0 = millis();

  // --- Read BMP header ---
  if (readLE16(f) != 0x4D42) {       // "BM" signature
    Serial.println("[BMP] not a BM file");
    f.close();
    return false;
  }

  (void)readLE32(f);                  // file size
  (void)readLE16(f);                  // reserved 1
  (void)readLE16(f);                  // reserved 2
  uint32_t dataOffset = readLE32(f);
  uint32_t headerSize = readLE32(f);

  if (headerSize < 40) {
    Serial.println("[BMP] unsupported header size");
    f.close();
    return false;
  }

  int32_t  srcW        = (int32_t)readLE32(f);
  int32_t  srcHRaw     = (int32_t)readLE32(f);
  uint16_t planes      = readLE16(f);
  uint16_t bpp         = readLE16(f);
  uint32_t compression = readLE32(f);

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

  if (srcW <= 0 || srcH <= 0 || srcW > BMP_MAX_SRC_W) {
    Serial.print("[BMP] bad size: ");
    Serial.print(srcW);
    Serial.print("x");
    Serial.println(srcH);
    f.close();
    return false;
  }

  uint32_t rowSize = ((uint32_t)srcW * bpp + 31) / 32 * 4;
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

  acquireForLcd();
  tft.fillScreen(C_BLACK);

  // --- Draw row by row ---
  for (int y = 0; y < drawH; y++) {
    int srcY  = cropY + y;
    int fileY = topDown ? srcY : (srcH - 1 - srcY);
    uint32_t rowOffset = dataOffset + (uint32_t)fileY * rowSize;

    if (!f.seekSet(rowOffset)) {
      Serial.println("[BMP] seek failed");
      f.close();
      return false;
    }

    if (f.read(rowBuf, rowSize) != (int)rowSize) {
      Serial.println("[BMP] row read failed");
      f.close();
      return false;
    }

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

    acquireForLcd();
    tft.pushImage(dstX, dstY + y, drawW, 1, lineBuf);
  }

  f.close();

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
  return true;
}

// ========================= Setup / loop =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus 1.47 SD Photo Frame ===");

  preparePins();
  initLcd();
  showMessage("SD Photo Frame", "Mounting SD card...");

  if (!beginSd()) {
    showMessage("SD Photo Frame", "SD card mount failed",
                "Insert FAT32 card with BMP files");
    while (1) delay(1000);
  }

  scanImages();

  if (g_imageCount == 0) {
    showMessage("SD Photo Frame", "No BMP files found",
                "Put 24-bit BMP images in SD root");
  } else {
    Serial.print("[IMG] found ");
    Serial.print(g_imageCount);
    Serial.println(" images");
  }
}

void loop() {
  if (g_imageCount == 0) {
    delay(1000);
    return;
  }

  const char *path = g_imagePaths[g_imageIndex];
  drawBmp(path);

  // Wait 2 seconds, then advance to the next image.
  delay(2000);
  g_imageIndex = (g_imageIndex + 1) % g_imageCount;
}
