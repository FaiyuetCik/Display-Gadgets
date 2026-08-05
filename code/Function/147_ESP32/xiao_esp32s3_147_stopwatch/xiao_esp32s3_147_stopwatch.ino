/*
  XIAO ESP32-S3 Plus 1.47 Inch Display — Stopwatch

  Left  btn (USR1, D19): Start / Stop
  Right btn (USR2, D15): Lap / Reset

  Runs at microsecond precision using esp_timer_get_time().
*/

#include <Arduino.h>
#include "driver.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "esp_timer.h"
#include <Wire.h>
#include "axs5106l_device.h"

// ========================= Pins =========================

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;
static constexpr uint8_t USR1_PIN     = D19;   // Left  btn — Start / Stop
static constexpr uint8_t USR2_PIN     = D15;   // Right btn — Lap / Reset
static constexpr uint8_t TOUCH_RST    = D17;   // shared with LCD reset
static constexpr uint8_t TOUCH_INT    = D7;

// ========================= Layout =========================

static constexpr int SCREEN_W = 172;
static constexpr int SCREEN_H = 320;
static constexpr int TIME_Y   = 80;
static constexpr int TIME_W   = 160;
static constexpr int TIME_H   = 40;

// ========================= Colours =========================

static constexpr uint16_t C_BLACK  = TFT_BLACK;
static constexpr uint16_t C_WHITE  = TFT_WHITE;
static constexpr uint16_t C_GREEN  = TFT_GREEN;
static constexpr uint16_t C_RED    = TFT_RED;
static constexpr uint16_t C_YELLOW = TFT_YELLOW;
static constexpr uint16_t C_CYAN   = TFT_CYAN;
static constexpr uint16_t C_GRAY   = 0x8410;
static constexpr uint16_t C_DARK   = 0x2104;
static constexpr uint16_t C_ORANGE = 0xFD20;

// ========================= Display =========================

TFT_eSPI tft;

// ========================= Stopwatch state =========================

enum State { IDLE, RUNNING, STOPPED };
enum TouchZone { TZ_NONE, TZ_LEFT, TZ_RIGHT };

struct ButtonState {
  bool usr1 : 1;
  bool usr2 : 1;
};

State state = IDLE;

int64_t  elapsedUs    = 0;        // accumulated time (excluding current run)
int64_t  runStartUs   = 0;        // when current run started
int64_t  lapStartUs   = 0;        // when current lap started

static constexpr int MAX_LAPS = 4;
int64_t  lapAbs[MAX_LAPS] = {};  // absolute times at each lap point
int      lapCount         = 0;

uint32_t lastRefresh  = 0;
uint32_t lastBtnMs    = 0;
uint32_t lastTouchMs  = 0;
touch_data_t touchData;
bool wasTouching      = false;
static constexpr uint32_t TOUCH_COOLDOWN = 200;

// ========================= Display init =========================

static void preparePins() {
  pinMode(LCD_CS_PIN, OUTPUT);   digitalWrite(LCD_CS_PIN, HIGH);
  pinMode(LCD_DC_PIN, OUTPUT);   digitalWrite(LCD_DC_PIN, HIGH);
  pinMode(LCD_SCK_PIN, OUTPUT);  digitalWrite(LCD_SCK_PIN, LOW);
  pinMode(LCD_MOSI_PIN, OUTPUT); digitalWrite(LCD_MOSI_PIN, LOW);
}

static void forceBacklight() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 200);
}

static void hardResetPanel() {
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH); delay(20);
  digitalWrite(LCD_RST_PIN, LOW);  delay(80);
  digitalWrite(LCD_RST_PIN, HIGH); delay(180);
}

static void applyXIAO147PanelFix() {
  tft.writecommand(0x36);
  tft.writedata(0x48);
  delay(10);
}

static void initLcd() {
  preparePins();
  forceBacklight();
  hardResetPanel();
  tft.init();
  tft.setRotation(0);
  applyXIAO147PanelFix();
  tft.invertDisplay(false);
  tft.fillScreen(C_BLACK);
}

// ========================= Time helpers =========================

static int64_t nowUs() {
  return esp_timer_get_time();  // microseconds since boot
}

static int64_t currentDisplayUs() {
  if (state == RUNNING) {
    return elapsedUs + (nowUs() - runStartUs);
  }
  return elapsedUs;
}

// Format: "MM:SS.mm" or "M:SS.mm" or "HH:MM:SS"
static void formatTime(int64_t us, char *buf, size_t len) {
  int64_t cs = us / 10000;  // centiseconds
  int c = cs % 100;
  int s = (cs / 100) % 60;
  int m = cs / 6000;

  if (m < 60) {
    snprintf(buf, len, "%02d:%02d.%02d", m, s, c);
  } else {
    int h = m / 60;
    m %= 60;
    snprintf(buf, len, "%d:%02d:%02d", h, m, s);
  }
}

// ========================= Drawing =========================

static void drawLayout() {
  tft.fillScreen(C_BLACK);

  // Title bar
  tft.fillRect(0, 0, SCREEN_W, 26, C_DARK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_ORANGE, C_DARK);
  tft.drawString("STOPWATCH", 6, 4, 2);

  // Divider
  tft.drawFastHLine(0, 26, SCREEN_W, C_CYAN);

  // Touch buttons
  static constexpr int BTN_Y = 208;
  static constexpr int BTN_H = 70;
  static constexpr int BTN_X_L = 4;
  static constexpr int BTN_X_R = 90;

  // Left touch button — USR1
  tft.fillRoundRect(BTN_X_L, BTN_Y, 78, BTN_H, 6, C_DARK);
  tft.drawRoundRect(BTN_X_L, BTN_Y, 78, BTN_H, 6, C_GREEN);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_GREEN, C_DARK);
  tft.drawString("USR1", BTN_X_L + 39, BTN_Y + 26, 2);
  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_DARK);
  tft.drawString("Start / Stop", BTN_X_L + 39, BTN_Y + 52, 1);

  // Right touch button — USR2
  tft.fillRoundRect(BTN_X_R, BTN_Y, 78, BTN_H, 6, C_DARK);
  tft.drawRoundRect(BTN_X_R, BTN_Y, 78, BTN_H, 6, C_YELLOW);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_YELLOW, C_DARK);
  tft.drawString("USR2", BTN_X_R + 39, BTN_Y + 26, 2);
  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_DARK);
  tft.drawString("Lap / Reset", BTN_X_R + 39, BTN_Y + 52, 1);

  // Button hints at bottom
  tft.fillRect(0, SCREEN_H - 28, SCREEN_W, 28, C_DARK);
  tft.drawFastHLine(0, SCREEN_H - 28, SCREEN_W, C_CYAN);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(C_GREEN, C_DARK);
  tft.drawString("USR1  Start / Stop", SCREEN_W / 2, SCREEN_H - 22, 1);
  tft.setTextColor(C_YELLOW, C_DARK);
  tft.drawString("USR2  Lap / Reset",  SCREEN_W / 2, SCREEN_H - 12, 1);

  // Initial time
  char buf[16];
  formatTime(0, buf, sizeof(buf));
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.drawString(buf, SCREEN_W / 2, TIME_Y, 4);
}

// In-place time update area

static void updateTime() {
  char buf[16];
  formatTime(currentDisplayUs(), buf, sizeof(buf));

  tft.fillRect((SCREEN_W - TIME_W) / 2, TIME_Y - TIME_H / 2, TIME_W, TIME_H, C_BLACK);
  tft.setTextDatum(MC_DATUM);
  uint16_t c = (state == RUNNING) ? C_GREEN : (state == STOPPED ? C_YELLOW : C_WHITE);
  tft.setTextColor(c, C_BLACK);
  tft.drawString(buf, SCREEN_W / 2, TIME_Y, 4);
}

static void updateLaps() {
  tft.fillRect(12, 120, SCREEN_W - 24, 80, C_BLACK);

  if (lapCount == 0) return;

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_CYAN, C_BLACK);
  tft.drawString("LAPS", 16, 122, 1);

  for (int i = 0; i < lapCount; ++i) {
    int64_t prev = (i > 0) ? lapAbs[i - 1] : 0;
    int64_t dur  = lapAbs[i] - prev;

    char buf[24], tBuf[16];
    formatTime(dur, tBuf, sizeof(tBuf));
    snprintf(buf, sizeof(buf), "#%d  %s", i + 1, tBuf);

    uint16_t c = (i == lapCount - 1) ? C_WHITE : C_CYAN;
    tft.setTextColor(c, C_BLACK);
    tft.setCursor(20, 138 + i * 18);
    tft.print(buf);
  }
}

static void drawStateIndicator() {
  // Clear old indicator area
  tft.fillRect(SCREEN_W - 36, 4, 30, 16, C_DARK);

  const char *label = "";
  uint16_t c = C_GRAY;
  switch (state) {
    case IDLE:    label = "IDLE"; c = C_GRAY;   break;
    case RUNNING: label = "RUN";  c = C_GREEN;  break;
    case STOPPED: label = "STOP"; c = C_YELLOW; break;
  }
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(c, C_DARK);
  tft.drawString(label, SCREEN_W - 6, 4, 2);
}

// ========================= Buttons =========================

static void initButtons() {
  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
}

static ButtonState readButtons() {
  static bool last1 = true, last2 = true;
  static uint32_t t1 = 0, t2 = 0;
  uint32_t now = millis();

  ButtonState s = {false, false};

  if (digitalRead(USR1_PIN) == LOW && last1 == HIGH && now - t1 > 50) {
    s.usr1 = true; t1 = now;
  }
  if (digitalRead(USR2_PIN) == LOW && last2 == HIGH && now - t2 > 50) {
    s.usr2 = true; t2 = now;
  }
  last1 = digitalRead(USR1_PIN);
  last2 = digitalRead(USR2_PIN);
  return s;
}

// ========================= Touch =========================

static void initTouch() {
  Wire.begin();
  touch_init(&Wire, TOUCH_RST, TOUCH_INT);
  pinMode(TOUCH_INT, INPUT_PULLUP);
}

static TouchZone readTouch() {
  bool touching = get_touch_data(&touchData) && touchData.touch_num > 0;

  if (!touching) {
    wasTouching = false;
    return TZ_NONE;
  }
  if (wasTouching) return TZ_NONE;       // only fire on first frame
  if (millis() - lastTouchMs < TOUCH_COOLDOWN) return TZ_NONE;

  wasTouching   = true;
  lastTouchMs   = millis();

  uint16_t tx = touchData.coords[0].x;
  uint16_t ty = touchData.coords[0].y;

  // Touch zone: y in [208..278], split left/right at x=86
  // X-axis is mirrored: high raw X = left side of screen
  if (ty >= 208 && ty <= 278) {
    if (tx > 86) return TZ_LEFT;
    else         return TZ_RIGHT;
  }
  return TZ_NONE;
}

static void flashTouchBtn(bool left) {
  int bx = left ? 4 : 90;
  int by = 208;
  tft.fillRoundRect(bx, by, 78, 70, 6, 0x4208);  // brighter flash
  delay(80);
  uint16_t border = left ? C_GREEN : C_YELLOW;
  tft.fillRoundRect(bx, by, 78, 70, 6, C_DARK);
  tft.drawRoundRect(bx, by, 78, 70, 6, border);
  // Redraw label
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(border, C_DARK);
  tft.drawString(left ? "USR1" : "USR2", bx + 39, by + 26, 2);
  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_DARK);
  tft.drawString(left ? "Start / Stop" : "Lap / Reset", bx + 39, by + 52, 1);
}

// ========================= Actions =========================

static void doStart() {
  runStartUs = nowUs();
  lapStartUs = runStartUs;
  state = RUNNING;
  drawStateIndicator();
}

static void doStop() {
  elapsedUs += (nowUs() - runStartUs);
  state = STOPPED;
  drawStateIndicator();
}

static void doReset() {
  elapsedUs = 0;
  lapCount = 0;
  memset(lapAbs, 0, sizeof(lapAbs));
  state = IDLE;
  drawStateIndicator();
  updateLaps();
  updateTime();
}

static void doLap() {
  if (lapCount >= MAX_LAPS) return;
  lapAbs[lapCount] = elapsedUs + (nowUs() - runStartUs);
  lapCount++;
  updateLaps();
}

// ========================= Setup / loop =========================

void setup() {
  Serial.begin(115200);
  delay(500);

  initTouch();      // reset D17 first (shared with LCD)
  initLcd();        // reset D17 again, then configure LCD registers
  initButtons();

  drawLayout();
  drawStateIndicator();
}

void loop() {
  ButtonState btns = readButtons();
  TouchZone   tz   = readTouch();
  uint32_t    now  = millis();

  // Combine physical buttons + virtual touch buttons
  bool usr1 = btns.usr1 || (tz == TZ_LEFT);
  bool usr2 = btns.usr2 || (tz == TZ_RIGHT);

  // Touch feedback flash
  if (tz == TZ_LEFT)  flashTouchBtn(true);
  if (tz == TZ_RIGHT) flashTouchBtn(false);

  switch (state) {

    case IDLE:
      if (usr1) {        // Start
        doStart();
      }
      break;

    case RUNNING:
      if (usr1) {        // Stop
        doStop();
      } else if (usr2) { // Lap
        doLap();
      }
      // Refresh display at ~30 fps while running
      if (now - lastRefresh >= 33) {
        lastRefresh = now;
        updateTime();
      }
      break;

    case STOPPED:
      if (usr1) {        // Resume
        doStart();
      } else if (usr2) { // Reset
        doReset();
      }
      break;
  }

  delay(5);
}
