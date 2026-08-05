/*
  XIAO nRF52840 Plus SD text reader.

  Supported files in the SD root:
    - TXT, LOG, CSV

  Each file is displayed page by page on the 172x320 LCD.
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
static constexpr int TEXT_COLUMNS = 27;
static constexpr int TEXT_LINES = 24;

TFT_eSPI tft;
SdFat sdCard;

char textPaths[MAX_FILES][MAX_PATH_LEN];
int textCount = 0;
int textIndex = 0;

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

static void initLcd() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);

  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(20);
  digitalWrite(LCD_RST_PIN, LOW);
  delay(80);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(180);

  tft.init();
  tft.setSwapBytes(true);
  tft.setRotation(0);
  acquireForLcd();
  tft.writecommand(0x36);
  tft.writedata(0x48);
  tft.invertDisplay(false);
  tft.fillScreen(TFT_BLACK);
}

static void drawHeader(const char *title, uint16_t color) {
  acquireForLcd();
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(3, 3);
  String name = title;
  if (name.length() > 26) name = name.substring(0, 26);
  tft.print(name);
  tft.drawFastHLine(0, 17, LCD_W, color);
}

static bool beginSd() {
  acquireForSd();
  SPI.begin();
  const uint32_t freqs[] = {8000000, 4000000, 1000000, 400000};
  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i) {
    SdSpiConfig cfg(SD_CS_PIN, SHARED_SPI, freqs[i], &SPI);
    if (sdCard.begin(cfg)) return true;
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

static bool isTextFile(const char *name) {
  return endsWithNoCase(name, ".txt") ||
         endsWithNoCase(name, ".log") ||
         endsWithNoCase(name, ".csv");
}

static void scanTextFiles() {
  textCount = 0;
  File32 root;
  if (!root.open("/")) return;

  File32 entry;
  while (textCount < MAX_FILES && entry.openNext(&root, O_RDONLY)) {
    if (!entry.isDir()) {
      char name[MAX_PATH_LEN];
      entry.getName(name, sizeof(name));
      if (isTextFile(name)) {
        snprintf(textPaths[textCount], MAX_PATH_LEN, "/%s", name);
        Serial.print("[TEXT] ");
        Serial.println(textPaths[textCount]);
        textCount++;
      }
    }
    entry.close();
  }
  root.close();
}

static void drawLine(int lineNumber, const char *text) {
  acquireForLcd();
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(3, 23 + lineNumber * 12);
  tft.print(text);
}

static void showTextFile(const char *path) {
  File32 f;
  if (!f.open(path, O_RDONLY)) {
    drawHeader(path, TFT_RED);
    drawLine(0, "Open failed");
    return;
  }

  uint32_t page = 1;
  bool done = false;
  while (!done) {
    drawHeader(path, TFT_CYAN);
    acquireForLcd();
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(132, 3);
    tft.print("P");
    tft.print(page);

    char line[TEXT_COLUMNS + 1];
    int column = 0;
    int row = 0;

    while (row < TEXT_LINES) {
      int c = f.read();
      if (c < 0) {
        done = true;
        if (column > 0) {
          line[column] = 0;
          drawLine(row++, line);
        }
        break;
      }

      if (c == '\r') continue;
      if (c == '\n' || column >= TEXT_COLUMNS) {
        line[column] = 0;
        drawLine(row++, line);
        Serial.println(line);
        column = 0;
        if (c != '\n' && c >= 32 && c <= 126) line[column++] = (char)c;
      } else if (c == '\t') {
        do {
          line[column++] = ' ';
        } while (column < TEXT_COLUMNS && (column % 4) != 0);
      } else if (c >= 32 && c <= 126) {
        line[column++] = (char)c;
      }
    }

    if (!done) {
      delay(2500);
      page++;
    }
  }

  f.close();
  delay(1800);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  preparePins();
  initLcd();
  drawHeader("SD text reader", TFT_YELLOW);
  drawLine(0, "Mounting SD...");

  if (!beginSd()) {
    drawHeader("SD text reader", TFT_RED);
    drawLine(0, "SD mount failed");
    while (1) delay(1000);
  }

  scanTextFiles();
  if (textCount == 0) {
    drawHeader("SD text reader", TFT_YELLOW);
    drawLine(0, "No TXT/LOG/CSV found");
  }
}

void loop() {
  if (textCount == 0) {
    delay(1000);
    return;
  }

  showTextFile(textPaths[textIndex]);
  textIndex = (textIndex + 1) % textCount;
}
