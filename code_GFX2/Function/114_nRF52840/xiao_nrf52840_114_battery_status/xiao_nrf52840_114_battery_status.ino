/*
  XIAO nRF52840 Plus 1.14-inch Display - Battery Status Demo
  Library: Seeed_GFX2
  Battery icon states: USB PWR (no battery) / percentage / charging.
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include <nrf.h>
#include <nrf_gpio.h>
#include <math.h>

// This 1.14-inch panel needs BGR: RGB swaps red/blue and yellow/cyan.
// Override only color order; inherit the panel geometry and inversion setting.
struct BatteryPanelConfig : Config_Seeed_1inch14_LCD_ST7789 {
  static constexpr uint8_t rgbOrder = TFT_MAD_BGR;
};

Seeed_GFX display;

static constexpr int8_t LCD_RST_PIN = 38;
static constexpr int8_t LCD_BL_PIN = 37;

static uint16_t colorByPercent(int pct) {
  if (pct <= 15) return TFT_RED;
  if (pct <= 35) return TFT_YELLOW;
  return TFT_GREEN;
}

static int lipoPercent(float v) {
  struct Point { float v; int p; };
  static const Point table[] = {
    {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.92f, 70}, {3.85f, 60},
    {3.79f, 50},  {3.72f, 40}, {3.66f, 30}, {3.58f, 20}, {3.50f, 10},
    {3.30f, 5},   {3.10f, 1},  {3.00f, 0}
  };
  const int N = sizeof(table) / sizeof(table[0]);
  if (v >= table[0].v) return 100;
  if (v <= table[N - 1].v) return 0;
  for (int i = 0; i < N - 1; ++i) {
    if (v <= table[i].v && v >= table[i + 1].v) {
      float f = (v - table[i + 1].v) / (table[i].v - table[i + 1].v);
      return table[i + 1].p + (int)roundf(f * (table[i].p - table[i + 1].p));
    }
  }
  return 0;
}

static constexpr int SCALE = 2;
static constexpr int BAT_W = 22 * SCALE;
static constexpr int BAT_H = 12 * SCALE;
static constexpr int CHG_W = 12 * SCALE;
static constexpr int CHG_H = 14 * SCALE;

static void drawBatteryIcon(int x, int y, bool valid, int percent, bool charging) {
  display.fillRect(x - 2 * SCALE, y - 2 * SCALE, BAT_W + 8 * SCALE, BAT_H + 4 * SCALE, TFT_BLACK);

  uint16_t outline = valid ? TFT_WHITE : TFT_DARKGREY;
  uint16_t fillColor = valid ? colorByPercent(percent) : TFT_RED;
  if (charging && valid) fillColor = TFT_CYAN;

  display.drawRoundRect(x, y, BAT_W, BAT_H, 2 * SCALE, outline);
  display.fillRect(x + BAT_W, y + 4 * SCALE, 3 * SCALE, 4 * SCALE, outline);

  if (!valid) {
    display.drawLine(x + 4 * SCALE, y + 3 * SCALE, x + BAT_W - 4 * SCALE, y + BAT_H - 4 * SCALE, TFT_RED);
    display.drawLine(x + BAT_W - 4 * SCALE, y + 3 * SCALE, x + 4 * SCALE, y + BAT_H - 4 * SCALE, TFT_RED);
    return;
  }

  int fillW = map(percent, 0, 100, 0, BAT_W - 4 * SCALE);
  if (fillW < 0) fillW = 0;
  if (fillW > BAT_W - 4 * SCALE) fillW = BAT_W - 4 * SCALE;
  if (fillW > 0) display.fillRect(x + 2 * SCALE, y + 2 * SCALE, fillW, BAT_H - 4 * SCALE, fillColor);
}

static void drawChargeIcon(int x, int y, bool charging) {
  display.fillRect(x, y, CHG_W, CHG_H, TFT_BLACK);
  if (!charging) return;

  display.fillTriangle(x + 6 * SCALE, y + 0 * SCALE, x + 1 * SCALE, y + 7 * SCALE, x + 6 * SCALE, y + 7 * SCALE, TFT_YELLOW);
  display.fillTriangle(x + 5 * SCALE, y + 6 * SCALE, x + 11 * SCALE, y + 6 * SCALE, x + 4 * SCALE, y + 13 * SCALE, TFT_YELLOW);

  display.drawLine(x + 6 * SCALE, y + 0 * SCALE, x + 1 * SCALE, y + 7 * SCALE, TFT_ORANGE);
  display.drawLine(x + 1 * SCALE, y + 7 * SCALE, x + 6 * SCALE, y + 7 * SCALE, TFT_ORANGE);
  display.drawLine(x + 11 * SCALE, y + 6 * SCALE, x + 4 * SCALE, y + 13 * SCALE, TFT_ORANGE);
}

// ---- nRF52840 battery circuit ----
static constexpr uint8_t READ_BAT_P0_PIN = 14;  // P0.14, active-low divider enable
static constexpr uint8_t CHG_P0_PIN      = 17;  // P0.17, active-low charging
#ifndef PIN_VBAT
#define PIN_VBAT 35
#endif
// Battery detection adapted from Dashboard using measured OFF/ON transitions.
static constexpr float BAT_DIVIDER_RATIO = (1000.0f + 499.0f) / 499.0f;
static constexpr float BAT_CAL_FACTOR    = 1.000f;
static constexpr int ADC_BITS            = 12;
static constexpr int ADC_MAX             = (1 << ADC_BITS) - 1;
static constexpr float ADC_FULL_SCALE_V  = 3.600f;

// A real Li-ion cell is a low-impedance source, so samples are usually stable.
// With no battery inserted but USB plugged in, the charger/VBAT node can float near
// 4V and still produce a stable fake voltage. Spread alone cannot detect absence.
static constexpr uint16_t BAT_PRESENT_MIN_RAW = 80;
static constexpr uint16_t BAT_FLOAT_RANGE_RAW = 80;

// Avoid high-impedance ADC noise causing visible NO BAT flicker.
// Once a real battery has been detected, noisy-but-plausible readings are held/filtered.
// Missing battery is declared only after many consecutive bad batches.
static constexpr uint8_t BAT_INVALID_CONFIRM_COUNT = 3;
static constexpr uint8_t BAT_NOISY_HOLD_CONFIRM_COUNT = 3;
static constexpr float BAT_VALID_MIN_V = 2.80f;
static constexpr float BAT_VALID_MAX_V = 4.60f;
// If spread is noisy but the averaged voltage is close to the last good cell voltage,
// keep it as a real battery. If the average jumps far while spread is huge, treat it
// as battery removed / charger floating.
static constexpr float BAT_NOISY_CLOSE_DELTA_V = 0.08f;

// Event-driven battery presence detection.
// We do NOT trust static VBAT voltage under USB-C, because the charger BAT node can
// look like a real 3.7V Li-ion cell even with no battery attached.
// Instead, under USB-C we first learn a USB-only baseline, then confirm battery insert
// only after a sustained downward VBAT shift. CHG can be LOW in both states.
static constexpr uint16_t BAT_STABLE_PRESENT_SPREAD_RAW = 30;
static constexpr uint8_t BAT_INSERT_CONFIRM_COUNT = 2;
static constexpr uint8_t BAT_REMOVE_CONFIRM_COUNT = 4;
static constexpr uint8_t BAT_BASELINE_CONFIRM_COUNT = 2;
static constexpr float BAT_BASELINE_CLOSE_V = 0.03f;
static constexpr float BAT_INSERT_DELTA_V = 0.10f;
static constexpr float BAT_REMOVE_DELTA_V = 0.14f;

// When USB-C is plugged into an already battery-powered board, charger STAT and
// VBAT ADC can glitch for a few refresh cycles. Do not classify this as battery removal.
static constexpr uint32_t BAT_CHG_TRANSIENT_HOLD_MS = 900;
static constexpr uint8_t BAT_PRESENT_NOISY_HOLD_COUNT = 4;

// Charger status filter.
// ~CHG is active-low. Set charging immediately when LOW is stable.
// Clear it after a few stable HIGH samples so unplugging USB-C is reflected quickly.
static constexpr uint8_t CHG_SAMPLE_COUNT = 9;
static constexpr uint8_t CHG_HIGH_CLEAR_COUNT = 1;

struct BatteryState {
  uint16_t raw = 0;
  uint16_t rawMin = 0;
  uint16_t rawMax = 0;
  float vadc = 0.0f;
  float vbat = 0.0f;
  int percent = 0;
  bool charging = false;
  bool valid = false;
};
BatteryState g_bat;
BatteryState g_lastGoodBat;
bool g_haveLastGoodBat = false;
uint8_t g_batInvalidStreak = 0;
uint8_t g_batValidStreak = 0;
uint8_t g_batNoisyStreak = 0;
const char *g_batFilterState = "BOOT";

bool g_chgRawLow = false;
bool g_chgState = false;
uint8_t g_chgHighStreak = 0;
uint32_t g_lastChgLowMs = 0;

enum BatteryPresenceState {
  BAT_BOOT = 0,
  BAT_USB_ONLY,
  BAT_INSERT_CANDIDATE,
  BAT_PRESENT,
  BAT_REMOVE_CANDIDATE
};

BatteryPresenceState g_batState = BAT_BOOT;
bool g_batPhysicallyConfirmed = false;

bool g_usbBaselineValid = false;
float g_usbBaselineVbat = 0.0f;
uint16_t g_usbBaselineRaw = 0;
uint16_t g_usbBaselineSpread = 0;
float g_usbBaselineCandidateVbat = 0.0f;
uint8_t g_usbBaselineCandidateCount = 0;
// Cold USB-only starts at 1.66V in the measured trace, not always 4.14V.
// Remember a sustained low-voltage absence separately from the high plateau.
uint8_t g_usbLowAbsentCount = 0;
bool g_usbLowAbsentConfirmed = false;
// 1.14-inch trace: USB-only spread~150/CHG HIGH, cell spread<10/CHG LOW.
uint8_t g_usbNoisyAbsentCount = 0;
bool g_usbNoisyAbsentConfirmed = false;
float g_insertCandidateVbat = 0.0f;

bool g_usbPresent = false;
bool g_usbInitialized = false;
uint32_t g_lastUsbChangeMs = 0;
bool g_chgRawInitialized = false;
uint32_t g_lastChgRawChangeMs = 0;
uint8_t g_insertCandidateStreak = 0;
uint8_t g_removeCandidateStreak = 0;

static const char *batteryPresenceStateName(BatteryPresenceState s);
static void setBatteryAbsentUsb(const BatteryState &measured, const char *filterState);
static void confirmBatteryPresent(const BatteryState &measured, const char *filterState);
static void updateUsbOnlyBaseline(const BatteryState &m, uint16_t spread);

static void enableBatteryDivider() {
  // READ_BAT is P0.14. Enable by sinking it to GND.
  NRF_P0->OUTCLR = (1UL << READ_BAT_P0_PIN);
  NRF_P0->DIRSET = (1UL << READ_BAT_P0_PIN);
}

static void disableBatteryDivider() {
  // High impedance = divider off, reduce leakage.
  NRF_P0->DIRCLR = (1UL << READ_BAT_P0_PIN);
}

static bool sampleChargingRawLow() {
  uint8_t lowCount = 0;

  for (uint8_t i = 0; i < CHG_SAMPLE_COUNT; i++) {
    if ((NRF_P0->IN & (1UL << CHG_P0_PIN)) == 0) {
      lowCount++;
    }
    delayMicroseconds(400);
  }

  // Majority vote. A single noisy low should not trigger charging.
  bool newRawLow = (lowCount >= ((CHG_SAMPLE_COUNT / 2) + 1));
  uint32_t now = millis();

  if (!g_chgRawInitialized) {
    g_chgRawInitialized = true;
    g_lastChgRawChangeMs = now;
  } else if (newRawLow != g_chgRawLow) {
    g_lastChgRawChangeMs = now;
  }

  g_chgRawLow = newRawLow;
  return g_chgRawLow;
}

static bool updateChargingState(bool batValid, float vbat) {
  (void)vbat;

  bool rawLow = sampleChargingRawLow();

  if (!batValid) {
    g_chgState = false;
    g_chgHighStreak = 0;
    return false;
  }

  if (rawLow) {
    g_chgState = true;
    g_lastChgLowMs = millis();
    g_chgHighStreak = 0;
    return true;
  }

  if (g_chgHighStreak < 255) g_chgHighStreak++;

  // Quick clear: when USB-C is removed, ~CHG should remain HIGH.
  // CHG_HIGH_CLEAR_COUNT selects the number of refreshes before clearing.
  if (g_chgHighStreak >= CHG_HIGH_CLEAR_COUNT) {
    g_chgState = false;
  }

  return g_chgState;
}

static const char *batteryPresenceStateName(BatteryPresenceState s) {
  switch (s) {
    case BAT_BOOT:             return "BOOT";
    case BAT_USB_ONLY:         return "USB_ONLY";
    case BAT_INSERT_CANDIDATE: return "INSERT?";
    case BAT_PRESENT:          return "PRESENT";
    case BAT_REMOVE_CANDIDATE: return "REMOVE?";
    default:                   return "UNKNOWN";
  }
}

static void setBatteryAbsentUsb(const BatteryState &measured, const char *filterState) {
  // Keep the established plateau across invalid/noisy readings. A stable
  // higher removal voltage can qualify a new plateau without erasing history.
  g_usbBaselineCandidateCount = 0;
  updateUsbOnlyBaseline(measured, measured.rawMax - measured.rawMin);
  g_insertCandidateStreak = 0;
  g_batState = BAT_USB_ONLY;
  g_batPhysicallyConfirmed = false;
  g_haveLastGoodBat = false;
  g_chgState = false;
  g_chgHighStreak = 0;
  g_batFilterState = filterState;
}

static void confirmBatteryPresent(const BatteryState &measured, const char *filterState) {
  g_batState = BAT_PRESENT;
  g_batPhysicallyConfirmed = true;
  g_usbLowAbsentConfirmed = false;
  g_usbLowAbsentCount = 0;
  g_usbNoisyAbsentConfirmed = false;
  g_usbNoisyAbsentCount = 0;
  g_insertCandidateStreak = 0;
  g_removeCandidateStreak = 0;
  g_batFilterState = filterState;

  g_bat = measured;
  g_bat.valid = true;
  g_lastGoodBat = g_bat;
  g_haveLastGoodBat = true;
}

static void updateUsbOnlyBaseline(const BatteryState &m, uint16_t spread) {
  // Startup readings and floating/noisy batches must not establish a baseline.
  if (m.vbat <= BAT_VALID_MIN_V || m.vbat >= BAT_VALID_MAX_V ||
      m.raw <= BAT_PRESENT_MIN_RAW || spread > BAT_FLOAT_RANGE_RAW) {
    g_usbBaselineCandidateCount = 0;
    return;
  }

  // Keep the highest confirmed stable plateau within this USB session.
  // Following a falling voltage would absorb slow battery insertion and leave
  // USB PWR latched. A later OFF plateau can recover an initially low baseline.
  if (g_usbBaselineValid && m.vbat <= g_usbBaselineVbat + BAT_BASELINE_CLOSE_V) {
    g_usbBaselineCandidateCount = 0;
    return;
  }

  if (g_usbBaselineCandidateCount == 0 ||
      fabsf(m.vbat - g_usbBaselineCandidateVbat) > BAT_BASELINE_CLOSE_V) {
    g_usbBaselineCandidateVbat = m.vbat;
    g_usbBaselineCandidateCount = 1;
    return;
  }

  g_usbBaselineCandidateVbat = (g_usbBaselineCandidateVbat + m.vbat) * 0.5f;
  if (++g_usbBaselineCandidateCount >= BAT_BASELINE_CONFIRM_COUNT) {
    g_usbBaselineValid = true;
    g_usbBaselineVbat = g_usbBaselineCandidateVbat;
    g_usbBaselineRaw = m.raw;
    g_usbBaselineSpread = spread;
    g_usbBaselineCandidateCount = 0;
  }
}

static uint16_t readBatteryRawAvg(uint8_t samples, uint16_t &rawMin, uint16_t &rawMax) {
  if (samples > 32) samples = 32;

  uint16_t buf[32];

  for (uint8_t i = 0; i < 6; i++) {
    (void)analogRead(PIN_VBAT);
    delay(2);
  }

  for (uint8_t i = 0; i < samples; i++) {
    buf[i] = analogRead(PIN_VBAT);
    delay(2);
  }

  // Sort small buffer and use a trimmed average to reject charger/floating spikes.
  for (uint8_t i = 0; i < samples; i++) {
    for (uint8_t j = i + 1; j < samples; j++) {
      if (buf[j] < buf[i]) {
        uint16_t t = buf[i];
        buf[i] = buf[j];
        buf[j] = t;
      }
    }
  }

  uint8_t trim = samples >= 16 ? 4 : 1;
  uint32_t sum = 0;
  uint8_t count = 0;

  // Report a robust range, not the absolute min/max, so one spike does not
  // make a real battery look like NO BAT.
  rawMin = buf[trim];
  rawMax = buf[samples - 1 - trim];

  for (uint8_t i = trim; i < samples - trim; i++) {
    sum += buf[i];
    count++;
  }

  return count ? (uint16_t)(sum / count) : buf[samples / 2];
}

static void updateBattery() {
  BatteryState measured;

  enableBatteryDivider();
  delay(30);

  measured.raw = readBatteryRawAvg(28, measured.rawMin, measured.rawMax);
  measured.vadc = ((float)measured.raw * ADC_FULL_SCALE_V) / (float)ADC_MAX;
  measured.vbat = measured.vadc * BAT_DIVIDER_RATIO * BAT_CAL_FACTOR;
  measured.percent = lipoPercent(measured.vbat);

  uint16_t spread = measured.rawMax - measured.rawMin;
  bool voltagePlausible = (measured.raw > BAT_PRESENT_MIN_RAW &&
                           measured.vbat > BAT_VALID_MIN_V &&
                           measured.vbat < BAT_VALID_MAX_V);
  bool stableBatch = voltagePlausible && (spread <= BAT_FLOAT_RANGE_RAW);

  // VBUS detection is independent of charger CHG and USB enumeration.
  bool usbNow = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
  if (!g_usbInitialized || usbNow != g_usbPresent) {
    g_lastUsbChangeMs = millis();
    // An old USB-only baseline is invalid after a USB power cycle.
    g_usbBaselineValid = false;
    g_usbBaselineCandidateCount = 0;
    g_usbLowAbsentCount = 0;
    g_usbLowAbsentConfirmed = false;
    g_usbNoisyAbsentCount = 0;
    g_usbNoisyAbsentConfirmed = false;
    g_insertCandidateStreak = 0;
    g_removeCandidateStreak = 0;
  }
  g_usbInitialized = true;
  g_usbPresent = usbNow;
  bool chargingNow = updateChargingState(voltagePlausible && usbNow, measured.vbat);

  bool closeToLastGood = g_haveLastGoodBat &&
                         fabsf(measured.vbat - g_lastGoodBat.vbat) <= BAT_NOISY_CLOSE_DELTA_V;
  // User trace: OFF=4.14V, ON=3.87V, both CHG LOW and stable.
  // Only an upward jump suggests removal on this circuit. A downward jump
  // must not discard the newly connected battery.
  bool roseFromLastGood = g_haveLastGoodBat &&
                         (measured.vbat - g_lastGoodBat.vbat) >= BAT_REMOVE_DELTA_V;

  disableBatteryDivider();

  // Remember sustained, stable below-cell voltage on USB. Do not use a
  // single bad ADC sample or an overvoltage as evidence for first insertion.
  if (usbNow && measured.raw > BAT_PRESENT_MIN_RAW &&
      measured.vbat < BAT_VALID_MIN_V && spread <= BAT_FLOAT_RANGE_RAW) {
    if (g_usbLowAbsentCount < BAT_INVALID_CONFIRM_COUNT) g_usbLowAbsentCount++;
    if (g_usbLowAbsentCount >= BAT_INVALID_CONFIRM_COUNT) {
      g_usbLowAbsentConfirmed = true;
    }
  } else {
    g_usbLowAbsentCount = 0;
  }

  // This board can have a plausible average even without a cell. Preserve
  // the sustained noisy/HIGH signature separately from a voltage baseline.
  // A later stable/LOW plateau supplies insertion evidence even if the
  // voltage drop is less than BAT_INSERT_DELTA_V.
  if (usbNow && voltagePlausible && spread > BAT_FLOAT_RANGE_RAW && !g_chgRawLow) {
    if (g_usbNoisyAbsentCount < BAT_BASELINE_CONFIRM_COUNT) g_usbNoisyAbsentCount++;
    if (g_usbNoisyAbsentCount >= BAT_BASELINE_CONFIRM_COUNT) {
      g_usbNoisyAbsentConfirmed = true;
    }
  } else {
    g_usbNoisyAbsentCount = 0;
  }

  // Case 1: not even a plausible Li-ion voltage.
  if (!voltagePlausible) {
    g_batValidStreak = 0;
    g_batNoisyStreak = 0;
    if (g_batInvalidStreak < 255) g_batInvalidStreak++;

    if (g_batPhysicallyConfirmed && g_batInvalidStreak < BAT_INVALID_CONFIRM_COUNT) {
      g_bat = g_lastGoodBat;
      g_bat.valid = true;
      g_bat.charging = chargingNow;
      g_batFilterState = "HOLD";
      return;
    }

    setBatteryAbsentUsb(measured, "MISS");
    g_bat = measured;
    g_bat.valid = false;
    g_bat.charging = false;
    return;
  }

  g_batInvalidStreak = 0;

  // With VBUS absent, a stable valid cell voltage is a real battery path.
  if (!usbNow && stableBatch) {
    measured.valid = true;
    measured.charging = false;
    confirmBatteryPresent(measured, "BAT_ONLY");
    return;
  }

  // Keep a known battery through USB insertion and its charging-voltage lift.
  if (usbNow && g_batPhysicallyConfirmed && stableBatch &&
      (millis() - g_lastUsbChangeMs) < BAT_CHG_TRANSIENT_HOLD_MS) {
    measured.valid = true;
    measured.charging = chargingNow;
    confirmBatteryPresent(measured, "USB_TR");
    return;
  }

  // Case 2: already confirmed battery present.
  if (g_batPhysicallyConfirmed) {
    uint32_t nowMs = millis();
    bool recentChgTransition = (nowMs - g_lastChgRawChangeMs) < BAT_CHG_TRANSIENT_HOLD_MS;

    // In a real USB unplug case:
    //   CHG goes HIGH quickly, but VBAT remains stable because the real battery is still attached.
    //   => stableBatch is true, so we should NOT remove the battery.
    //
    // In a real battery removal while USB is present:
    //   CHG usually goes HIGH and the high-impedance VBAT ADC becomes noisy.
    //   => !stableBatch + CHG HIGH persists for several refreshes.
    //
    // In USB plug-in transient:
    //   spread may glitch briefly. Hold PRESENT for a short guard window.
    bool noisyOrJump = (!stableBatch) || (usbNow && roseFromLastGood);
    bool chgHighNow = !g_chgRawLow;
    bool likelyRemoved = noisyOrJump && chgHighNow;

    if (noisyOrJump) {
      if (likelyRemoved) {
        if (g_removeCandidateStreak < 255) g_removeCandidateStreak++;
        g_batState = BAT_REMOVE_CANDIDATE;

        // Product UI fix:
        // As soon as we see the clear removal signature
        //   CHG HIGH + noisy/jumped VBAT
        // do NOT show the stale battery percentage with charging cleared.
        // That one-frame state looked like "battery inserted but not charging".
        // Instead, switch UI to USB PWR immediately while the internal confirm
        // counter decides whether to fully clear batConf.
        g_bat = measured;
        g_bat.valid = false;
        g_bat.charging = false;

        if (g_removeCandidateStreak >= BAT_REMOVE_CONFIRM_COUNT) {
          setBatteryAbsentUsb(measured, "REMOVED");
          g_batFilterState = "REMOVED";
        } else {
          g_batFilterState = "RM_UI";
        }
        return;
      }

      // Noisy ADC while CHG is still LOW: usually charging startup / LCD/SD ADC glitch.
      if (closeToLastGood && (recentChgTransition || g_removeCandidateStreak < BAT_PRESENT_NOISY_HOLD_COUNT)) {
        if (g_removeCandidateStreak < 255) g_removeCandidateStreak++;
        g_batState = BAT_PRESENT;

        BatteryState filtered = g_lastGoodBat;
        filtered.valid = true;
        filtered.charging = chargingNow;

        g_bat = filtered;
        g_batFilterState = recentChgTransition ? "USB_TR" : "NOISY_H";
        return;
      }

      // Last fallback for noisy state that does not look like a clean USB unplug.
      if (g_removeCandidateStreak < 255) g_removeCandidateStreak++;
      g_batState = BAT_REMOVE_CANDIDATE;

      if (g_removeCandidateStreak >= BAT_REMOVE_CONFIRM_COUNT) {
        setBatteryAbsentUsb(measured, "REMOVED");
        g_bat = measured;
        g_bat.valid = false;
        g_bat.charging = false;
        return;
      }

      g_bat = measured;
      g_bat.valid = false;
      g_bat.charging = false;
      g_batFilterState = "RM_UI";
      return;
    }

    g_removeCandidateStreak = 0;

    if (stableBatch) {
      measured.valid = true;
      measured.charging = chargingNow;
      confirmBatteryPresent(measured, "STABLE");
      return;
    }

    if (closeToLastGood) {
      BatteryState filtered = measured;
      filtered.valid = true;
      filtered.vbat = g_lastGoodBat.vbat * 0.80f + measured.vbat * 0.20f;
      filtered.vadc = filtered.vbat / (BAT_DIVIDER_RATIO * BAT_CAL_FACTOR);
      filtered.percent = lipoPercent(filtered.vbat);
      filtered.charging = chargingNow;
      confirmBatteryPresent(filtered, "NOISY");
      return;
    }
  }

  // Case 3: USB present, battery not confirmed. Neither a static LOW on
  // CHG nor an upward voltage change proves insertion. Require a stable
  // downward shift from the high USB-only baseline, OR a recovery from a
  // confirmed low USB-only voltage (trace: 1.66V -> 3.87V). The latter must
  // be checked before the new cell voltage can become an empty-node baseline.
  // Static USB+battery at boot remains ambiguous without an observed transition.
  bool baselineDrop = g_usbBaselineValid &&
                      (g_usbBaselineVbat - measured.vbat) >= BAT_INSERT_DELTA_V;
  bool riseFromLowAbsent = g_usbLowAbsentConfirmed;
  bool stableAfterNoisyAbsent = g_usbNoisyAbsentConfirmed && g_chgRawLow &&
                               spread <= BAT_STABLE_PRESENT_SPREAD_RAW;
  bool insertCandidate = usbNow && stableBatch &&
                         (baselineDrop || riseFromLowAbsent || stableAfterNoisyAbsent);

  if (insertCandidate) {
    // Require a settled plateau across batches as well as low within-batch noise.
    if (g_insertCandidateStreak > 0 &&
        fabsf(measured.vbat - g_insertCandidateVbat) > BAT_BASELINE_CLOSE_V) {
      g_insertCandidateStreak = 0;
    }
    g_insertCandidateVbat = measured.vbat;
    if (g_insertCandidateStreak < 255) g_insertCandidateStreak++;
    g_batState = BAT_INSERT_CANDIDATE;
    g_batFilterState = stableAfterNoisyAbsent ? "INS_STABLE" : (riseFromLowAbsent ? "INS_LOW" : "INS_DROP");

    if (g_insertCandidateStreak >= BAT_INSERT_CONFIRM_COUNT) {
      measured.valid = true;
      measured.charging = chargingNow;
      confirmBatteryPresent(measured, stableAfterNoisyAbsent ? "INSERT_STABLE" : (riseFromLowAbsent ? "INSERT_LOW" : (chargingNow ? "INSERT_CHG" : "INSERT")));
      return;
    }
  } else {
    g_insertCandidateStreak = 0;
    g_batState = BAT_USB_ONLY;
    updateUsbOnlyBaseline(measured, spread);
    g_batFilterState = g_usbBaselineValid ? "USBVBAT" : "BASE_WAIT";
  }

  g_bat = measured;
  g_bat.valid = false;
  g_bat.charging = false;
}

void setup() {
  Serial.begin(115200);

  nrf_gpio_cfg_input(CHG_P0_PIN, NRF_GPIO_PIN_PULLUP);
  analogReadResolution(12);
  NRF_P0->DIRCLR = (1UL << READ_BAT_P0_PIN);
  // Start acquiring the USB-only plateau before display initialization so an
  // early insertion is less likely to become the first observed voltage.
  for (uint8_t i = 0; i < BAT_BASELINE_CONFIRM_COUNT; ++i) updateBattery();

  if (!display.begin<Board_XIAO_1inch14_LCD<LCD_RST_PIN, LCD_BL_PIN>, BatteryPanelConfig>()) {
    Serial.println(display.lastResult().message);
    return;
  }
  display.fillScreen(TFT_BLACK);



}

void loop() {

  updateBattery();
  float vbat = g_bat.vbat;
  bool valid = g_bat.valid;
  bool charging = valid && g_bat.charging;
  int percent = valid ? g_bat.percent : 0;

  int cx = display.width() / 2;
  int batX = cx - BAT_W - 2 * SCALE;
  int chgX = cx + 2 * SCALE;
  int iconY = display.height() / 2 - 40;

  drawBatteryIcon(batX, iconY, valid, percent, charging);
  drawChargeIcon(chgX, iconY, charging);

  display.setTextFont(1);
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setTextWrap(false);

  char buf[40];
  int line1Y = iconY + BAT_H + 16;
  int line2Y = line1Y + 24;

  display.fillRect(0, line1Y - 4, display.width(), 52, TFT_BLACK);

  if (!valid) {
    snprintf(buf, sizeof(buf), "USB PWR");
  } else {
    snprintf(buf, sizeof(buf), "%d%%", percent);
  }
  display.setCursor((display.width() - display.textWidth(buf)) / 2, line1Y);
  display.print(buf);

  if (valid) {
    snprintf(buf, sizeof(buf), "%.2fV", vbat);
    display.setCursor((display.width() - display.textWidth(buf)) / 2, line2Y);
    display.print(buf);
  }

  Serial.print("VBAT "); Serial.print(vbat); Serial.print("V  ");
  Serial.print(charging ? "charging" : "not charging"); Serial.print("  ");
  Serial.print(valid ? percent : -1);
  Serial.print("  spread="); Serial.print(g_bat.rawMax - g_bat.rawMin);
  Serial.print("  usb="); Serial.print(g_usbPresent ? "ON" : "OFF");
  Serial.print("  base="); Serial.print(g_usbBaselineVbat, 3);
  Serial.print("  baseValid="); Serial.print(g_usbBaselineValid ? "Y" : "N");
  Serial.print("  baseCount="); Serial.print(g_usbBaselineCandidateCount);
  Serial.print("  noisyAbsent="); Serial.print(g_usbNoisyAbsentConfirmed ? "Y" : "N");
  Serial.print("  lowAbsent="); Serial.print(g_usbLowAbsentConfirmed ? "Y" : "N");
  Serial.print("  insertCount="); Serial.print(g_insertCandidateStreak);
  Serial.print("  chgRaw="); Serial.print(g_chgRawLow ? "LOW" : "HIGH");
  Serial.print("  state="); Serial.print(batteryPresenceStateName(g_batState));
  Serial.print("  filter="); Serial.print(g_batFilterState);
  Serial.print("  removeCount="); Serial.println(g_removeCandidateStreak);

  delay(500);

}
