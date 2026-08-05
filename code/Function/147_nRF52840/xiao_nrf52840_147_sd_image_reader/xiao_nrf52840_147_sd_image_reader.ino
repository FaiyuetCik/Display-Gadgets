/*
  XIAO nRF52840 Plus SD image reader.

  Direct display support:
    - BMP: uncompressed 16/24/32-bit, center-cropped to the 172x320 LCD.

  Put image files in the SD card root directory.
*/

#include "driver.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat.h>
#include <TFT_eSPI.h>

static constexpr uint8_t LCD_CS_PIN = D2;
static constexpr uint8_t LCD_DC_PIN = D3;
static constexpr uint8_t SD_CS_PIN = D6;
static constexpr uint8_t LCD_SCK_PIN = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN = D17;
static constexpr uint8_t LCD_BL_PIN = D18;

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;
static constexpr int MAX_FILES = 24;
static constexpr int MAX_PATH_LEN = 64;
static constexpr int BMP_MAX_SRC_W = 360;

TFT_eSPI tft;
SdFat sdCard;

char imagePaths[MAX_FILES][MAX_PATH_LEN];
int imageCount = 0;
int imageIndex = 0;
uint32_t mountedFreq = 0;

static uint8_t rowBuf[BMP_MAX_SRC_W * 4 + 8];
static uint16_t lineBuf[LCD_W];

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
  tft.fillScreen(TFT_BLACK);
}

static void showMessage(const char *title, const String &line1, const String &line2 = "") {
  acquireForLcd();
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(4, 4);
  tft.print(title);
  tft.drawFastHLine(0, 18, LCD_W, TFT_CYAN);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(4, 34);
  tft.print(line1);
  if (line2.length()) {
    tft.setCursor(4, 50);
    tft.print(line2);
  }
}

static bool beginSd() {
  acquireForSd();
  SPI.begin();
  const uint32_t freqs[] = {8000000, 4000000, 1000000, 400000};
  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
    SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, freqs[i], &SPI);
    if (sdCard.begin(cfg)) {
      mountedFreq = freqs[i];
      return true;
    }
    delay(100);
  }
  return false;
}

static bool endsWithNoCase(const char *name, const char *ext) {
  size_t n = strlen(name);
  size_t e = strlen(ext);
  if (n < e) return false;
  for (size_t i = 0; i < e; ++i) {
    if (tolower(name[n - e + i]) != tolower(ext[i])) return false;
  }
  return true;
}

static bool isBmpFile(const char *name) {
  return endsWithNoCase(name, ".bmp");
}

static void scanImages() {
  imageCount = 0;
  File32 root;
  if (!root.open("/")) return;

  File32 entry;
  while (imageCount < MAX_FILES && entry.openNext(&root, O_RDONLY)) {
    if (!entry.isDir()) {
      char name[MAX_PATH_LEN];
      entry.getName(name, sizeof(name));
      if (isBmpFile(name)) {
        snprintf(imagePaths[imageCount], MAX_PATH_LEN, "/%s", name);
        Serial.print("[IMAGE] ");
        Serial.println(imagePaths[imageCount]);
        imageCount++;
      }
    }
    entry.close();
  }
  root.close();
}

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
  if (!f.open(path, O_RDONLY)) return false;
  if (readLE16(f) != 0x4D42) {
    f.close();
    return false;
  }

  (void)readLE32(f);
  (void)readLE16(f);
  (void)readLE16(f);
  uint32_t dataOffset = readLE32(f);
  uint32_t headerSize = readLE32(f);
  int32_t srcW = (int32_t)readLE32(f);
  int32_t srcHRaw = (int32_t)readLE32(f);
  uint16_t planes = readLE16(f);
  uint16_t bpp = readLE16(f);
  uint32_t compression = readLE32(f);

  if (headerSize < 40 || planes != 1 || compression != 0 ||
      srcW <= 0 || srcW > BMP_MAX_SRC_W ||
      !(bpp == 16 || bpp == 24 || bpp == 32)) {
    f.close();
    return false;
  }

  bool topDown = srcHRaw < 0;
  int32_t srcH = topDown ? -srcHRaw : srcHRaw;
  if (srcH <= 0) {
    f.close();
    return false;
  }

  uint32_t rowSize = ((uint32_t)srcW * bpp + 31) / 32 * 4;
  if (rowSize > sizeof(rowBuf)) {
    f.close();
    return false;
  }

  int drawW = min((int)srcW, LCD_W);
  int drawH = min((int)srcH, LCD_H);
  int cropX = srcW > LCD_W ? (srcW - LCD_W) / 2 : 0;
  int cropY = srcH > LCD_H ? (srcH - LCD_H) / 2 : 0;
  int dstX = srcW < LCD_W ? (LCD_W - srcW) / 2 : 0;
  int dstY = srcH < LCD_H ? (LCD_H - srcH) / 2 : 0;

  acquireForLcd();
  tft.fillScreen(TFT_BLACK);

  for (int y = 0; y < drawH; ++y) {
    int srcY = cropY + y;
    int fileY = topDown ? srcY : srcH - 1 - srcY;
    uint32_t rowOffset = dataOffset + (uint32_t)fileY * rowSize;
    if (!f.seekSet(rowOffset) || f.read(rowBuf, rowSize) != (int)rowSize) {
      f.close();
      return false;
    }

    for (int x = 0; x < drawW; ++x) {
      int srcX = cropX + x;
      if (bpp == 24) {
        int p = srcX * 3;
        lineBuf[x] = rgb888To565(rowBuf[p + 2], rowBuf[p + 1], rowBuf[p]);
      } else if (bpp == 32) {
        int p = srcX * 4;
        lineBuf[x] = rgb888To565(rowBuf[p + 2], rowBuf[p + 1], rowBuf[p]);
      } else {
        int p = srcX * 2;
        lineBuf[x] = (uint16_t)rowBuf[p] | ((uint16_t)rowBuf[p + 1] << 8);
      }
    }

    acquireForLcd();
    tft.pushImage(dstX, dstY + y, drawW, 1, lineBuf);
  }

  f.close();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  preparePins();
  initLcd();
  showMessage("SD image reader", "Mounting SD...");

  if (!beginSd()) {
    showMessage("SD image reader", "SD mount failed");
    while (1) delay(1000);
  }

  scanImages();
  Serial.print("[SD] mounted @ ");
  Serial.println(mountedFreq);
  if (imageCount == 0) showMessage("SD image reader", "No BMP found");
}

void loop() {
  if (imageCount == 0) {
    delay(1000);
    return;
  }

  const char *path = imagePaths[imageIndex];
  if (!drawBmp(path)) showMessage(path, "BMP decode failed");

  delay(2000);
  imageIndex = (imageIndex + 1) % imageCount;
}
