/*
  XIAO nRF52840 Plus + 1.47 Inch Touch Display
  SD BMP Image Viewer / SD + LCD Stress Test v0.1

  Purpose:
    - Verify SD card image reading.
    - Verify SD card and high-frequency LCD refresh can coexist stably.
    - This is a standalone storage/display validation firmware.

  Important update from hardware/BSP sync:
    - D17 / D19 mapping issue has been fixed.
    - This sketch uses the corrected definition:
        LCD_RST_PIN = D17
      D19 is not used here.

  SD image format:
    - Put BMP files in the SD card root directory.
    - Recommended: 24-bit uncompressed BMP.
    - Recommended resolution: 172x320 for exact full-screen display.
    - 240x320 BMP also works; this sketch center-crops to 172x320.
    - File examples:
        /test.bmp
        /image001.bmp
        /image002.bmp

  Libraries:
    - Arduino_GFX_Library
    - SdFat

  Hardware pin map:
    LCD CS   = D2
    LCD DC   = D3
    SD CS    = D6
    LCD/SD SCK  = D8
    SD MISO     = D9
    LCD/SD MOSI = D10
    LCD RST  = D17   // corrected
    LCD BL   = D18
*/

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <Arduino_GFX_Library.h>

// ========================= Pin map =========================

static constexpr uint8_t LCD_CS_PIN    = D2;
static constexpr uint8_t LCD_DC_PIN    = D3;
static constexpr uint8_t SD_CS_PIN     = D6;
static constexpr uint8_t LCD_SCK_PIN   = D8;
static constexpr uint8_t SD_MISO_PIN   = D9;
static constexpr uint8_t LCD_MOSI_PIN  = D10;
static constexpr uint8_t LCD_RST_PIN   = D17; // corrected mapping
static constexpr uint8_t LCD_BL_PIN    = D18;

// ========================= Display =========================

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;

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
  LCD_W,
  LCD_H,
  34,
  0,
  34,
  0
);

// ========================= SD =========================

SdFat SD;

static constexpr uint32_t SD_FREQ_INIT  = 400000;
static constexpr uint32_t SD_FREQ_RUN_1 = 4000000;
static constexpr uint32_t SD_FREQ_RUN_2 = 8000000;

static uint32_t g_sdFreq = 0;

// ========================= BMP config =========================

static constexpr int MAX_IMAGES = 16;
static constexpr int MAX_PATH_LEN = 64;

// Enough for 360px wide 32bpp BMP rows.
// 172x320 / 240x320 images are the recommended cases.
static constexpr int BMP_MAX_SRC_W = 360;
static uint8_t rowBuf[BMP_MAX_SRC_W * 4 + 8];
static uint16_t lineBuf[LCD_W];

char g_imagePaths[MAX_IMAGES][MAX_PATH_LEN];
int g_imageCount = 0;
int g_imageIndex = 0;

uint32_t g_frame = 0;
uint32_t g_lastFrameMs = 0;
uint32_t g_lastRenderMs = 0;
uint32_t g_failCount = 0;

// Continuous full image read mode.
// true  = stress test: keep re-opening + reading BMP + drawing LCD.
// false = slideshow: delay between frames.
static constexpr bool STRESS_READ_EVERY_FRAME = true;
static constexpr uint32_t SLIDESHOW_DELAY_MS = 1200;

// ========================= Colors =========================

static constexpr uint16_t C_BLACK  = RGB565_BLACK;
static constexpr uint16_t C_WHITE  = RGB565_WHITE;
static constexpr uint16_t C_GREEN  = RGB565_LIGHTGREEN;
static constexpr uint16_t C_CYAN   = RGB565_CYAN;
static constexpr uint16_t C_YELLOW = RGB565_YELLOW;
static constexpr uint16_t C_RED    = RGB565_RED;
static constexpr uint16_t C_GRAY   = 0x8410;
static constexpr uint16_t C_BLUE   = RGB565_BLUE;

// ========================= Bus helpers =========================

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

static void lcdHardReset() {
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(10);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(30);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(150);
}

static void lcdWriteMadctlFix() {
  acquireForLcd();
  lcdBus->beginWrite();
  lcdBus->writeC8D8(0x36, 0x48);
  lcdBus->endWrite();
}

static void setBacklight(uint8_t pwm) {
  pinMode(LCD_BL_PIN, OUTPUT);
  analogWrite(LCD_BL_PIN, pwm);
}

// ========================= Basic UI =========================

static void printFixed(int x, int y, uint16_t color, const String &s, int chars) {
  String out = s;
  while ((int)out.length() < chars) out += ' ';
  if ((int)out.length() > chars) out = out.substring(0, chars);

  acquireForLcd();
  gfx->setTextSize(1);
  gfx->setTextColor(color, C_BLACK);
  gfx->setCursor(x, y);
  gfx->print(out);
}

static void drawNoImageScreen(const char *msg) {
  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(8, 16);
  gfx->print("Hello,XIAO!");

  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(10, 46);
  gfx->print("1.47 SD BMP Reader");

  gfx->drawFastHLine(8, 64, 156, C_GRAY);

  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(10, 88);
  gfx->print("Put BMP files in SD root");

  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(10, 112);
  gfx->print("/test.bmp");
  gfx->setCursor(10, 128);
  gfx->print("/image001.bmp");
  gfx->setCursor(10, 144);
  gfx->print("/image002.bmp");

  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(10, 176);
  gfx->print("Format:");
  gfx->setCursor(10, 192);
  gfx->print("24-bit uncompressed BMP");
  gfx->setCursor(10, 208);
  gfx->print("172x320 recommended");

  gfx->setTextColor(C_RED, C_BLACK);
  gfx->setCursor(10, 250);
  gfx->print(msg);
}

static void drawStatusBar(const char *path, bool ok, uint32_t renderMs) {
  acquireForLcd();

  gfx->fillRect(0, 0, LCD_W, 18, C_BLACK);
  gfx->drawFastHLine(0, 18, LCD_W, ok ? C_GREEN : C_RED);

  gfx->setTextSize(1);
  gfx->setTextColor(ok ? C_GREEN : C_RED, C_BLACK);
  gfx->setCursor(2, 3);
  gfx->print(ok ? "SD IMG" : "IMG ERR");

  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(50, 3);
  gfx->print("#");
  gfx->print((unsigned long)g_frame);

  gfx->setTextColor(C_YELLOW, C_BLACK);
  gfx->setCursor(100, 3);
  gfx->print(renderMs);
  gfx->print("ms");

  gfx->fillRect(0, LCD_H - 14, LCD_W, 14, C_BLACK);
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(2, LCD_H - 11);

  String name = String(path);
  int slash = name.lastIndexOf('/');
  if (slash >= 0) name = name.substring(slash + 1);
  if (name.length() > 20) name = name.substring(0, 20);
  gfx->print(name);
}

// ========================= SD init/list =========================

static bool beginSd() {
  acquireForSd();
  SPI.begin();

  uint32_t freqs[] = {SD_FREQ_RUN_2, SD_FREQ_RUN_1, SD_FREQ_INIT};

  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
    SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, freqs[i], &SPI);
    if (SD.begin(cfg)) {
      g_sdFreq = freqs[i];
      Serial.print("[SD] begin OK freq=");
      Serial.println(g_sdFreq);
      return true;
    }

    Serial.print("[SD] begin failed freq=");
    Serial.println(freqs[i]);
    delay(50);
  }

  return false;
}

static bool endsWithBmp(const char *name) {
  size_t n = strlen(name);
  if (n < 4) return false;

  char a = tolower(name[n - 4]);
  char b = tolower(name[n - 3]);
  char c = tolower(name[n - 2]);
  char d = tolower(name[n - 1]);

  return a == '.' && b == 'b' && c == 'm' && d == 'p';
}

static void addImagePath(const char *name) {
  if (g_imageCount >= MAX_IMAGES) return;
  if (!endsWithBmp(name)) return;

  char path[MAX_PATH_LEN];
  if (name[0] == '/') {
    snprintf(path, sizeof(path), "%s", name);
  } else {
    snprintf(path, sizeof(path), "/%s", name);
  }

  strncpy(g_imagePaths[g_imageCount], path, MAX_PATH_LEN - 1);
  g_imagePaths[g_imageCount][MAX_PATH_LEN - 1] = 0;

  Serial.print("[IMG] found ");
  Serial.println(g_imagePaths[g_imageCount]);

  g_imageCount++;
}

static void scanBmpFiles() {
  g_imageCount = 0;

  File32 root;
  if (!root.open("/")) {
    Serial.println("[SD] root open failed");
    return;
  }

  File32 entry;
  while (entry.openNext(&root, O_RDONLY)) {
    if (!entry.isDir()) {
      char name[MAX_PATH_LEN];
      entry.getName(name, sizeof(name));
      addImagePath(name);
    }
    entry.close();
  }

  root.close();

  // Fallback candidates, useful if LFN listing behaves differently.
  if (g_imageCount == 0) {
    const char *candidates[] = {
      "/test.bmp",
      "/image.bmp",
      "/image001.bmp",
      "/image002.bmp",
      "/1.bmp",
      "/2.bmp"
    };

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
      File32 f;
      if (f.open(candidates[i], O_RDONLY)) {
        f.close();
        addImagePath(candidates[i]);
      }
    }
  }

  Serial.print("[IMG] count=");
  Serial.println(g_imageCount);
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
  return ((r & 0xF8) << 8) |
         ((g & 0xFC) << 3) |
         (b >> 3);
}

static bool drawBmpFromSd(const char *path) {
  File32 f;
  if (!f.open(path, O_RDONLY)) {
    Serial.print("[BMP] open failed ");
    Serial.println(path);
    return false;
  }

  uint32_t t0 = millis();

  uint16_t signature = readLE16(f);
  if (signature != 0x4D42) {
    Serial.println("[BMP] not BM signature");
    f.close();
    return false;
  }

  (void)readLE32(f); // file size
  (void)readLE16(f); // reserved 1
  (void)readLE16(f); // reserved 2
  uint32_t dataOffset = readLE32(f);

  uint32_t headerSize = readLE32(f);
  if (headerSize < 40) {
    Serial.println("[BMP] unsupported header");
    f.close();
    return false;
  }

  int32_t srcW = (int32_t)readLE32(f);
  int32_t srcHRaw = (int32_t)readLE32(f);
  uint16_t planes = readLE16(f);
  uint16_t bpp = readLE16(f);
  uint32_t compression = readLE32(f);

  if (planes != 1 || compression != 0) {
    Serial.println("[BMP] only BI_RGB uncompressed BMP supported");
    f.close();
    return false;
  }

  if (!(bpp == 24 || bpp == 32 || bpp == 16)) {
    Serial.print("[BMP] unsupported bpp=");
    Serial.println(bpp);
    f.close();
    return false;
  }

  bool topDown = srcHRaw < 0;
  int32_t srcH = topDown ? -srcHRaw : srcHRaw;

  if (srcW <= 0 || srcH <= 0) {
    Serial.println("[BMP] bad size");
    f.close();
    return false;
  }

  if (srcW > BMP_MAX_SRC_W) {
    Serial.print("[BMP] width too large for buffer: ");
    Serial.println(srcW);
    f.close();
    return false;
  }

  uint32_t rowSize = ((uint32_t)srcW * bpp + 31) / 32 * 4;
  if (rowSize > sizeof(rowBuf)) {
    Serial.println("[BMP] row buffer too small");
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
  gfx->fillScreen(C_BLACK);

  for (int y = 0; y < drawH; y++) {
    int srcY = cropY + y;
    int fileY = topDown ? srcY : (srcH - 1 - srcY);
    uint32_t rowOffset = dataOffset + (uint32_t)fileY * rowSize;

    if (!f.seekSet(rowOffset)) {
      Serial.println("[BMP] seek failed");
      f.close();
      return false;
    }

    int n = f.read(rowBuf, rowSize);
    if (n != (int)rowSize) {
      Serial.println("[BMP] row read failed");
      f.close();
      return false;
    }

    for (int x = 0; x < drawW; x++) {
      int srcX = cropX + x;

      if (bpp == 24) {
        int p = srcX * 3;
        uint8_t b = rowBuf[p + 0];
        uint8_t g = rowBuf[p + 1];
        uint8_t r = rowBuf[p + 2];
        lineBuf[x] = rgb888To565(r, g, b);
      } else if (bpp == 32) {
        int p = srcX * 4;
        uint8_t b = rowBuf[p + 0];
        uint8_t g = rowBuf[p + 1];
        uint8_t r = rowBuf[p + 2];
        lineBuf[x] = rgb888To565(r, g, b);
      } else {
        int p = srcX * 2;
        uint16_t px = (uint16_t)rowBuf[p] | ((uint16_t)rowBuf[p + 1] << 8);
        // Most generated 16-bit BMPs used here are RGB565. If your file is RGB555,
        // use 24-bit BMP instead for this validation firmware.
        lineBuf[x] = px;
      }
    }

    acquireForLcd();
    gfx->draw16bitRGBBitmap(dstX, dstY + y, lineBuf, drawW, 1);
  }

  f.close();

  g_lastRenderMs = millis() - t0;

  Serial.print("[BMP] draw OK path=");
  Serial.print(path);
  Serial.print(" size=");
  Serial.print(srcW);
  Serial.print("x");
  Serial.print(srcH);
  Serial.print(" bpp=");
  Serial.print(bpp);
  Serial.print(" renderMs=");
  Serial.println(g_lastRenderMs);

  return true;
}

// ========================= Init =========================

static bool initLcd() {
  setBacklight(255);
  lcdHardReset();

  if (!gfx->begin()) {
    Serial.println("[LCD] begin failed");
    return false;
  }

  lcdWriteMadctlFix();

  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  Serial.println("[LCD] OK, RST=D17 corrected");
  return true;
}

// ========================= Arduino =========================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== XIAO nRF52840 Plus 1.47 SD BMP Reader v0.1 ===");
  Serial.println("LCD_RST uses corrected D17 definition. D19 is not used.");

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);

  if (!initLcd()) {
    while (1) delay(1000);
  }

  drawNoImageScreen("Mounting SD...");

  if (!beginSd()) {
    Serial.println("[SD] mount failed");
    drawNoImageScreen("SD mount failed");
    while (1) delay(1000);
  }

  scanBmpFiles();

  if (g_imageCount <= 0) {
    drawNoImageScreen("No BMP found");
  }
}

void loop() {
  if (g_imageCount <= 0) {
    // Keep a tiny animation so LCD refresh is still visible.
    static int x = 0;
    static int dir = 1;

    acquireForLcd();
    gfx->fillRect(10, 286, 152, 8, C_BLACK);
    gfx->fillRect(10 + x, 286, 20, 8, C_CYAN);

    x += dir * 3;
    if (x <= 0 || x >= 132) dir = -dir;

    delay(40);
    return;
  }

  const char *path = g_imagePaths[g_imageIndex];

  bool ok = drawBmpFromSd(path);
  g_frame++;

  if (!ok) {
    g_failCount++;
  }

  drawStatusBar(path, ok, g_lastRenderMs);

  Serial.print("[STAT] frame=");
  Serial.print((unsigned long)g_frame);
  Serial.print(" image=");
  Serial.print(g_imageIndex + 1);
  Serial.print("/");
  Serial.print(g_imageCount);
  Serial.print(" fail=");
  Serial.print((unsigned long)g_failCount);
  Serial.print(" sdFreq=");
  Serial.println((unsigned long)g_sdFreq);

  g_imageIndex++;
  if (g_imageIndex >= g_imageCount) g_imageIndex = 0;

  if (!STRESS_READ_EVERY_FRAME) {
    delay(SLIDESHOW_DELAY_MS);
  } else {
    delay(5);
  }
}
