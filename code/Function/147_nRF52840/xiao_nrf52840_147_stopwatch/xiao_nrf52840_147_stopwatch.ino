/*
  XIAO nRF52840 Plus 1.47 Inch Display
  Premium Stopwatch — minimalist chronograph with lap timing

  Hardware:
    - XIAO nRF52840 Plus + 1.47" Display (172x320)
    - USR1 (D19): Start / Stop
    - USR2 (D15): Reset (when stopped) / Lap (when running)

  Design:
    - Blued-steel & silver palette on deep black
    - Centered time with stacked centiseconds
    - Breathing pulse indicator when running
    - Scrollable lap table

  Required libraries:
    - Seeed_GFX / TFT_eSPI
    - Adafruit_TinyUSB
*/

#include "driver.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <TFT_eSPI.h>

// ========================= Pins =========================

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;
static constexpr uint8_t USR1_PIN     = D19;  // Start / Stop
static constexpr uint8_t USR2_PIN     = D15;  // Reset / Lap

// ========================= Display =========================

TFT_eSPI tft;

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;
static constexpr int CX    = LCD_W / 2;

// ========================= Design Tokens =========================
//
// Palette inspired by fine chronograph watches:
// blued-steel hands, silvered dials, brushed titanium.

//  RGB565 values computed from target sRGB:
//    R5 = round(R * 31 / 255), G6 = round(G * 63 / 255), B5 = round(B * 31 / 255)
static constexpr uint16_t COLOR_BG       = TFT_BLACK;
static constexpr uint16_t COLOR_TIME     = 0xEF3B;  // warm paper-white
static constexpr uint16_t COLOR_CENTISEC = 0xA4F1;  // muted taupe
static constexpr uint16_t COLOR_RUNNING  = 0x5DBD;  // blued steel
static constexpr uint16_t COLOR_RUN_DIM  = 0x4CD9;  // blued steel (dim)
static constexpr uint16_t COLOR_STOPPED  = 0x736D;  // pewter gray
static constexpr uint16_t COLOR_LAP_NUM  = 0x52CB;  // blue-gray
static constexpr uint16_t COLOR_LAP_TIME = 0x94B1;  // warm silver
static constexpr uint16_t COLOR_DIVIDER  = 0x2104;  // deep ink
static constexpr uint16_t COLOR_LABEL    = 0x5AEB;  // muted slate
static constexpr uint16_t COLOR_HINT     = 0x3186;  // dark graphite

// ========================= Typography =========================

static constexpr uint8_t FONT_TIME     = 4;   // MM:SS  — 24x32px proportional
static constexpr uint8_t FONT_STATUS   = 2;   // status — 12x16px
static constexpr uint8_t FONT_CAPTION  = 1;   // laps, hints — 6x8px

// ========================= Layout =========================

// Title
static constexpr int TITLE_Y       = 10;
static constexpr int DIVIDER_TOP_Y = 32;

// Time display
static constexpr int TIME_Y        = 100;   // center-y of MM:SS (font 4)
static constexpr int CENTISEC_Y    = 134;   // center-y of .CC  (font 2)

// Status indicator
static constexpr int STATUS_Y      = 178;
static constexpr int STATUS_X      = 20;
static constexpr int STATUS_DOT_R  = 4;
static constexpr int STATUS_DOT_CX = STATUS_X + STATUS_DOT_R;
static constexpr int STATUS_DOT_CY = STATUS_Y + 6;

// Dividers
static constexpr int DIVIDER_MID_Y = 206;
static constexpr int DIVIDER_BOT_Y = 308;

// Lap table
static constexpr int LAP_START_Y   = 218;
static constexpr int LAP_ROW_H     = 17;
static constexpr int LAP_NUM_X     = 20;
static constexpr int LAP_TIME_X    = LCD_W - 20;
static constexpr int MAX_VIS_LAPS  = 5;

// Button hints
static constexpr int HINTS_Y       = 312;

// ========================= Stopwatch State =========================

enum State : uint8_t {
  STOPPED = 0,
  RUNNING = 1,
};

static State      g_state         = STOPPED;
static uint32_t   g_startMillis   = 0;       // when current run began
static uint32_t   g_accumulated   = 0;       // time accumulated before current run (ms)

// Lap storage
static constexpr uint8_t MAX_LAPS = 30;
static uint32_t   g_laps[MAX_LAPS];
static uint8_t    g_lapCount      = 0;

// Display cache — only redraw when values change
static uint32_t   g_lastDrawnCs   = 0xFFFFFFFF;  // last time rendered (centiseconds)
static uint8_t    g_lastLapCount  = 0xFF;
static State      g_lastState     = (State)0xFF;
static bool       g_lastPulsePhase = false;
static bool       g_needsFullRedraw = true;

// Pulse animation
static uint32_t   g_lastPulseMs   = 0;
static bool       g_pulsePhase    = false;    // toggles every 500ms when RUNNING

// ========================= Types & Helpers =========================

// Time components — defined before any functions so Arduino's
// auto-generated prototypes can reference it.
struct TimeBreakdown {
  uint8_t mins;
  uint8_t secs;
  uint8_t centis;
};

// Returns current stopwatch time in milliseconds.
static uint32_t currentMillis() {
  if (g_state == RUNNING) {
    return g_accumulated + (millis() - g_startMillis);
  }
  return g_accumulated;
}

// Format time components from milliseconds.
static TimeBreakdown breakTime(uint32_t ms) {
  uint32_t totalCentis = ms / 10;
  TimeBreakdown t;
  t.mins   = (totalCentis / 6000) % 100;   // 0–99 min
  t.secs   = (totalCentis / 100) % 60;
  t.centis = totalCentis % 100;
  return t;
}

// ========================= Display Init =========================

static void applyFix() {
  tft.writecommand(0x36);
  tft.writedata(0x48);
  delay(10);
}

static void initDisplay() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_SCK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_SCK_PIN, LOW);
  digitalWrite(LCD_MOSI_PIN, LOW);

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);

  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH); delay(20);
  digitalWrite(LCD_RST_PIN, LOW);  delay(80);
  digitalWrite(LCD_RST_PIN, HIGH); delay(180);

  tft.init();
  tft.setRotation(0);
  applyFix();
  tft.invertDisplay(false);
  tft.fillScreen(COLOR_BG);
}

// ========================= Drawing =========================

// --- Zone A: Title + top divider (static) ---

static void drawTitle() {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  tft.drawString("STOPWATCH", 10, TITLE_Y, 2);
}

static void drawTopDivider() {
  tft.drawFastHLine(10, DIVIDER_TOP_Y, LCD_W - 20, COLOR_DIVIDER);
}

// --- Zone B: Time display ---

static void clearTimeZone() {
  // Erase both MM:SS and .CC in one rect to prevent proportional-font ghosting.
  // Font 4 (32px) at TIME_Y + Font 2 (16px) at CENTISEC_Y ≈ 80px span.
  tft.fillRect(0, TIME_Y - 30, LCD_W, 80, COLOR_BG);
}

static void drawTime(uint32_t ms) {
  TimeBreakdown t = breakTime(ms);

  char bufMain[8];
  snprintf(bufMain, sizeof(bufMain), "%02d:%02d", t.mins, t.secs);

  char bufSub[8];
  snprintf(bufSub, sizeof(bufSub), ".%02d", t.centis);

  // Main time: MM:SS
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_TIME, COLOR_BG);
  tft.drawString(bufMain, CX, TIME_Y, FONT_TIME);

  // Centiseconds: .CC
  tft.setTextColor(COLOR_CENTISEC, COLOR_BG);
  tft.drawString(bufSub, CX, CENTISEC_Y, FONT_STATUS);
}

// --- Zone C: Status indicator ---

static void drawStatusDot(bool running, bool pulsePhase) {
  int cx = STATUS_DOT_CX;
  int cy = STATUS_DOT_CY;
  int r  = STATUS_DOT_R;

  // Clear dot area
  tft.fillRect(cx - r - 2, cy - r - 2, (r + 2) * 2, (r + 2) * 2, COLOR_BG);

  if (running) {
    if (pulsePhase) {
      // Filled — solid blued steel
      tft.fillCircle(cx, cy, r, COLOR_RUNNING);
    } else {
      // Outline — breathing out
      tft.drawCircle(cx, cy, r, COLOR_RUN_DIM);
    }
  } else {
    // Stopped — open circle, pewter
    tft.drawCircle(cx, cy, r, COLOR_STOPPED);
  }
}

static void drawStatusText(bool running) {
  // Clear text area
  tft.fillRect(STATUS_X + 18, STATUS_Y, 120, 18, COLOR_BG);

  tft.setTextDatum(TL_DATUM);
  const char* label = running ? "RUNNING" : "STOPPED";
  uint16_t color = running ? COLOR_RUNNING : COLOR_STOPPED;
  tft.setTextColor(color, COLOR_BG);
  tft.drawString(label, STATUS_X + 18, STATUS_Y, FONT_STATUS);
}

static void drawStatus(bool running, bool pulsePhase) {
  drawStatusDot(running, pulsePhase);
  drawStatusText(running);
}

// --- Zone D: Middle divider ---

static void drawMidDivider() {
  tft.drawFastHLine(10, DIVIDER_MID_Y, LCD_W - 20, COLOR_DIVIDER);
}

// --- Zone E: Lap table ---

static void clearLapZone() {
  tft.fillRect(0, LAP_START_Y, LCD_W, DIVIDER_BOT_Y - LAP_START_Y, COLOR_BG);
}

static void drawLapTable() {
  clearLapZone();

  if (g_lapCount == 0) {
    // Empty state — subtle hint
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COLOR_LABEL, COLOR_BG);
    int midY = LAP_START_Y + (DIVIDER_BOT_Y - LAP_START_Y) / 2;
    tft.drawString("No laps recorded", CX, midY, FONT_CAPTION);
    return;
  }

  // Show the most recent laps (up to MAX_VIS_LAPS), newest at top.
  uint8_t startIdx = (g_lapCount > MAX_VIS_LAPS) ? (g_lapCount - MAX_VIS_LAPS) : 0;
  uint8_t showCount = (g_lapCount > MAX_VIS_LAPS) ? MAX_VIS_LAPS : g_lapCount;

  for (uint8_t i = 0; i < showCount; i++) {
    uint8_t lapIdx = g_lapCount - showCount + i;   // 0-based index into g_laps
    uint32_t lapMs = g_laps[lapIdx];
    TimeBreakdown t = breakTime(lapMs);

    int y = LAP_START_Y + i * LAP_ROW_H;

    // Lap number — left-aligned
    char numBuf[6];
    snprintf(numBuf, sizeof(numBuf), "%d", lapIdx + 1);

    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(COLOR_LAP_NUM, COLOR_BG);
    tft.drawString(numBuf, LAP_NUM_X, y, FONT_CAPTION);

    // Lap time — right-aligned
    char timeBuf[12];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d.%02d", t.mins, t.secs, t.centis);

    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(COLOR_LAP_TIME, COLOR_BG);
    tft.drawString(timeBuf, LAP_TIME_X, y, FONT_CAPTION);
  }
}

// --- Zone F: Button hints ---

static void drawHints() {
  // Clear hints area
  tft.fillRect(0, HINTS_Y - 2, LCD_W, LCD_H - HINTS_Y + 2, COLOR_BG);

  // Bottom hairline
  tft.drawFastHLine(10, DIVIDER_BOT_Y, LCD_W - 20, COLOR_DIVIDER);

  // Left hint — primary action
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_HINT, COLOR_BG);
  if (g_state == STOPPED) {
    tft.drawString("START", 24, HINTS_Y, FONT_CAPTION);
  } else {
    tft.drawString("STOP", 26, HINTS_Y, FONT_CAPTION);
  }

  // Right hint — secondary action (context-dependent)
  tft.setTextDatum(TR_DATUM);
  if (g_state == STOPPED && g_accumulated > 0) {
    tft.setTextColor(COLOR_HINT, COLOR_BG);
    tft.drawString("RESET", LCD_W - 24, HINTS_Y, FONT_CAPTION);
  } else if (g_state == RUNNING) {
    tft.setTextColor(COLOR_RUNNING, COLOR_BG);
    tft.drawString("LAP", LCD_W - 28, HINTS_Y, FONT_CAPTION);
  }
  // Stopped at zero: right side intentionally blank — nothing to reset.
}

// --- Full redraw ---

static void drawFullUI() {
  tft.fillScreen(COLOR_BG);

  drawTitle();
  drawTopDivider();
  drawTime(currentMillis());
  drawStatus(g_state == RUNNING, g_pulsePhase);
  drawMidDivider();
  drawLapTable();
  drawHints();

  g_lastDrawnCs   = currentMillis() / 10;
  g_lastLapCount  = g_lapCount;
  g_lastState     = g_state;
  g_lastPulsePhase = g_pulsePhase;
  g_needsFullRedraw = false;
}

// --- Incremental update ---

static void updateDisplay() {
  if (g_needsFullRedraw) {
    drawFullUI();
    return;
  }

  uint32_t nowMs = currentMillis();
  uint32_t nowCs = nowMs / 10;

  // Time: redraw only when centiseconds change (~10ms granularity)
  if (nowCs != g_lastDrawnCs) {
    clearTimeZone();
    drawTime(nowMs);
    g_lastDrawnCs = nowCs;
  }

  // Status: redraw when state or pulse phase changes
  bool running = (g_state == RUNNING);
  if (g_state != g_lastState || g_pulsePhase != g_lastPulsePhase) {
    drawStatus(running, g_pulsePhase);
    g_lastState = g_state;
    g_lastPulsePhase = g_pulsePhase;
  }

  // Lap list: redraw when laps are added or cleared
  if (g_lapCount != g_lastLapCount) {
    drawLapTable();
    g_lastLapCount = g_lapCount;
  }

  // Hints are only redrawn on full-redraw paths (state transitions, RESET).
  // No incremental hint update needed — state changes always trigger full redraws.
}

// ========================= Button Handling =========================

static bool readButton(uint8_t pin) {
  if (digitalRead(pin) == LOW) {
    delay(30);  // debounce
    if (digitalRead(pin) == LOW) {
      // Wait for release
      while (digitalRead(pin) == LOW) { delay(5); }
      return true;
    }
  }
  return false;
}

static void handleStartStop() {
  if (g_state == STOPPED) {
    // START
    g_state = RUNNING;
    g_startMillis = millis();
    Serial.println("[STOPWATCH] START");
  } else {
    // STOP
    g_accumulated = g_accumulated + (millis() - g_startMillis);
    g_state = STOPPED;
    Serial.print("[STOPWATCH] STOP  time=");
    Serial.println(g_accumulated);
  }
  // Full redraw handles hints, status, and time zone in one pass.
  g_needsFullRedraw = true;
}

static void handleResetLap() {
  if (g_state == RUNNING) {
    // LAP — record current time
    if (g_lapCount < MAX_LAPS) {
      uint32_t lapTime = g_accumulated + (millis() - g_startMillis);
      g_laps[g_lapCount] = lapTime;
      g_lapCount++;
      Serial.print("[STOPWATCH] LAP #");
      Serial.print(g_lapCount);
      Serial.print("  time=");
      Serial.println(lapTime);
    } else {
      Serial.println("[STOPWATCH] LAP  full — ignoring");
    }
  } else {
    // RESET — zero everything
    if (g_accumulated > 0) {
      g_accumulated = 0;
      g_lapCount = 0;
      g_needsFullRedraw = true;  // time, laps, and hints all change
      Serial.println("[STOPWATCH] RESET");
    }
  }
}

static void handleButtons() {
  if (readButton(USR1_PIN)) {
    handleStartStop();
  }

  if (readButton(USR2_PIN)) {
    handleResetLap();
  }
}

// ========================= Animation =========================

static void updatePulseAnimation(uint32_t now) {
  if (g_state != RUNNING) {
    // When stopped, always show the open circle (phase doesn't matter).
    if (g_pulsePhase != false) {
      g_pulsePhase = false;
    }
    return;
  }

  if (now - g_lastPulseMs >= 500) {
    g_lastPulseMs = now;
    g_pulsePhase = !g_pulsePhase;
  }
}

// ========================= Setup =========================

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println();
  Serial.println("=== Premium Stopwatch ===");
  Serial.println("USR1: Start/Stop   USR2: Reset/Lap");

  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);

  initDisplay();
  drawFullUI();

  Serial.println("[STOPWATCH] ready");
}

// ========================= Loop =========================

void loop() {
  uint32_t now = millis();

  handleButtons();

  // After button handling, state may have changed.
  // Always compute the latest time for display.
  updatePulseAnimation(now);
  updateDisplay();

  delay(15);  // ~60Hz refresh — smooth centisecond updates
}
