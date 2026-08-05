/*
  ============================================================================
  LVGL v9 Minimal Demo — XIAO 1.47 Inch Touch Display (nRF52840 Plus)
  ============================================================================

  A self-contained LVGL v9 demo using the OFFICIAL LVGL library (v9.5.0),
  a 1/10-screen buffer (~11 KB) for partial rendering, and a working
  AXS5106L touch driver.

  WHY v9?
  – Seeed's LVGL v8 fork has a partial-render bug: only the first strip
    flushes.  We worked around it with a full-screen buffer (107 KB), but
    that likely clashes with the nRF52840's USB CDC DMA buffers, killing
    Serial output (and touch) once lv_timer_handler() starts.
  – LVGL v9's partial renderer works correctly out of the box, so we can
    use a tiny 1/10-screen buffer (~11 KB) and keep far away from the USB
    stack's memory.

  WHAT IT SHOWS
  – A title label at the top ("LVGL v9 Demo")
  – A "Click Me!" button in the centre
  – A counter label below the button
  – Each click increments the counter
  – A footer hint at the bottom

  ============================
  HOW TO SET UP (BEFORE UPLOAD)
  ============================

  1. Install Seeed nRF52 Boards in Arduino IDE (Board Manager).

  2. Install Seeed_GFX from:
     https://github.com/Seeed-Studio/Seeed_GFX
     !! Remove the original TFT_eSPI library first, or they will conflict.

  3. Install the OFFICIAL LVGL library (v9.5.x):
     – Arduino Library Manager → search "lvgl" → install "lvgl" by LVGL
       (version 9.5.0 or later).
     – !! REMOVE SeeedStudio_lvgl first — the two WILL conflict.
     – The official library includes lv_conf.h; no manual copying needed
       for basic use.

  4. lv_conf.h settings (the library's own lv_conf.h, OR copy it to
     Arduino/libraries/ if you want to customise):
     – LV_COLOR_DEPTH 16          (already default)
     – LV_TICK_CUSTOM 0            (we use lv_tick_set_cb() instead)
     – LV_MEM_SIZE at least 48 KB  (adjust if memory is tight)
     – Everything else at defaults is fine.

  5. If Serial does not appear after upload, install Adafruit TinyUSB
     from Library Manager.  The sketch already #includes it.

  6. Board: "Seeed XIAO nRF52840 Plus"
     Port:  select the correct COM port
     Click Upload.

  ============================
  TROUBLESHOOTING
  ============================

  – "fatal error: lvgl.h: No such file or directory"
      The official LVGL library is not installed.  Install "lvgl" from
      Library Manager (NOT SeeedStudio_lvgl).

  – Colours look inverted (black↔white, red↔cyan)?
      Flip tft.invertDisplay(false) → tft.invertDisplay(true), or vice-versa.

  – Colours are wrong but not inverted (e.g. red↔blue)?
      Flip LV_COLOR_16_SWAP in lv_conf.h (0↔1), OR flip
      tft.setSwapBytes(true↔false) below.

  – Touch does not respond:
      Open Serial Monitor (115200 baud).  You should see "Touch ID: xx xx xx"
      on startup.  If not, check the I2C connection (D4=SDA, D5=SCL).
      If the ID prints but touch still doesn't work, look for the
      "[TOUCH DIAG]" lines in Serial — they print every ~3 seconds and
      tell you whether the touch callback is being invoked and whether
      the controller is reporting touches.
*/

#include "driver.h"
#include "axs5106l_device.h"

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <lvgl.h>

// ============================================================================
// Display — 1.47" 172×320 JD9853A (ST7789-compatible)
// ============================================================================

static constexpr uint16_t SCREEN_W = 172;
static constexpr uint16_t SCREEN_H = 320;

// 1/10 screen buffer = ~11 KB (vs 107 KB full-screen in the v8 demo).
// This stays well clear of the nRF52840 USB CDC DMA region.
static constexpr uint32_t BUF_SIZE = SCREEN_W * SCREEN_H / 10;

static constexpr uint8_t LCD_CS_PIN  = D2;
static constexpr uint8_t LCD_DC_PIN  = D3;
static constexpr uint8_t LCD_SCK_PIN = D8;
static constexpr uint8_t LCD_MOS_PIN = D10;
static constexpr uint8_t LCD_RST_PIN = D17;
static constexpr uint8_t LCD_BL_PIN  = D18;

TFT_eSPI tft;

// The 1.47" panel needs this MADCTL value for correct orientation.
static void applyPanelFix() {
  tft.writecommand(0x36);     // MADCTL
  tft.writedata(0x48);
  delay(10);
}

static void setRotation(uint8_t rot) {
  tft.setRotation(rot);
  if (rot == 0) applyPanelFix();   // rotation 0 resets MADCTL; re-apply
}

// ============================================================================
// Hardware helpers
// ============================================================================

static void preparePins() {
  pinMode(LCD_CS_PIN,  OUTPUT); digitalWrite(LCD_CS_PIN,  HIGH);
  pinMode(LCD_DC_PIN,  OUTPUT); digitalWrite(LCD_DC_PIN,  HIGH);
  pinMode(LCD_SCK_PIN, OUTPUT); digitalWrite(LCD_SCK_PIN, LOW);
  pinMode(LCD_MOS_PIN, OUTPUT); digitalWrite(LCD_MOS_PIN, LOW);
}

static void backlightOn() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);
}

static void hardResetPanel() {
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH); delay(20);
  digitalWrite(LCD_RST_PIN, LOW);  delay(80);
  digitalWrite(LCD_RST_PIN, HIGH); delay(180);
}

static void initLCD() {
  preparePins();
  backlightOn();
  hardResetPanel();

  tft.init();
  setRotation(0);
  tft.invertDisplay(false);        // JD9853A: inversion OFF = normal colours
  tft.setSwapBytes(true);          // pushColors expects RGB565 bytes swapped
  tft.fillScreen(TFT_BLACK);
}

// ============================================================================
// LVGL v9 display flush callback
// ============================================================================
// v9 signature: (lv_display_t *, const lv_area_t *, uint8_t *)
// v8 signature: (lv_disp_drv_t *, const lv_area_t *, lv_color_t *)
//
// LVGL calls this whenever it has rendered a dirty area.  We push that area
// to the LCD through TFT_eSPI's pushColors().

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area,
                          uint8_t *pixelmap) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)pixelmap, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);        // v9: takes lv_display_t*
}

// ============================================================================
// LVGL v9 touch input device callback
// ============================================================================
// v9 signature: (lv_indev_t *, lv_indev_data_t *)
// v8 signature: (lv_indev_drv_t *, lv_indev_data_t *)
//
// LVGL calls this periodically (~30 ms).  We read the AXS5106L via I2C and
// feed the first touch point back.  No touch → state = RELEASED.

static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  static touch_data_t td;
  static uint32_t call_count = 0;
  call_count++;

  bool has_touch = get_touch_data(&td);

  // Print diagnostic every ~3 seconds (LVGL polls touch ~30 times/sec).
  if (call_count % 100 == 0) {
    Serial.print("[TOUCH DIAG] call#=");
    Serial.print(call_count);
    Serial.print("  has_touch=");
    Serial.print(has_touch);
    Serial.print("  touch_num=");
    Serial.println(td.touch_num);
  }

  if (has_touch && td.touch_num > 0) {
    int16_t x = td.coords[0].x;
    int16_t y = td.coords[0].y;

    // Print raw coordinates on every touch (comment out if too noisy).
    Serial.print("Touch raw: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");

    // Clamp to screen bounds.
    if (x < 0) x = 0; if (x >= SCREEN_W) x = SCREEN_W - 1;
    if (y < 0) y = 0; if (y >= SCREEN_H) y = SCREEN_H - 1;

    data->state   = LV_INDEV_STATE_PRESSED;  // v9: PRESSED (not PR)
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;   // v9: RELEASED (not REL)
  }
}

// ============================================================================
// LVGL tick callback
// ============================================================================
// v9 uses lv_tick_set_cb() instead of LV_TICK_CUSTOM / lv_tick_inc().
// This keeps us independent of lv_conf.h customisation.

static uint32_t my_tick_get_cb(void) {
  return millis();
}

// ============================================================================
// UI — a simple counter with a button
// ============================================================================

static lv_obj_t *countLabel;    // "Count: 0" label, updated on click

static void onButtonClicked(lv_event_t *e) {
  static uint32_t counter = 0;
  counter++;
  lv_label_set_text_fmt(countLabel, "Count: %u", counter);
  Serial.print("Button clicked!  count=");
  Serial.println(counter);
}

static void createUI() {
  lv_obj_t *scr = lv_screen_active();   // v9: lv_screen_active() (was lv_scr_act())

  // ---- title ----
  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "LVGL v9 Demo");
  lv_obj_set_style_text_color(title, lv_color_hex(0x00FFFF), 0);  // cyan
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

  // ---- button (explicit bright style so it stands out) ----
  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_set_size(btn, 140, 56);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, -24);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x0066CC), 0);       // blue button
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x004499), LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn, onButtonClicked, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *btnLabel = lv_label_create(btn);
  lv_label_set_text(btnLabel, "Click Me!");
  lv_obj_set_style_text_color(btnLabel, lv_color_hex(0xFFFFFF), 0);
  lv_obj_center(btnLabel);

  // ---- counter label ----
  countLabel = lv_label_create(scr);
  lv_label_set_text(countLabel, "Count: 0");
  lv_obj_set_style_text_color(countLabel, lv_color_hex(0xFFFF00), 0); // yellow
  lv_obj_align(countLabel, LV_ALIGN_CENTER, 0, 60);

  // ---- footer hint ----
  lv_obj_t *hint = lv_label_create(scr);
  lv_label_set_text(hint, "XIAO nRF52840 + 1.47\" Touch");
  lv_obj_set_style_text_color(hint, lv_color_hex(0x888888), 0);
  lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -12);
}

// ============================================================================
// Setup & Loop
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);   // let USB serial stabilise

  Serial.println();
  Serial.println("=== LVGL v9 Minimal Demo ===");
  Serial.print("LVGL version: ");
  Serial.print(lv_version_major());
  Serial.print(".");
  Serial.print(lv_version_minor());
  Serial.print(".");
  Serial.println(lv_version_patch());
  Serial.print("Buffer: 1/10 screen = ");
  Serial.print(BUF_SIZE);
  Serial.print(" pixels (");
  Serial.print(BUF_SIZE * sizeof(lv_color_t));
  Serial.println(" bytes)");
  Serial.println("Board: XIAO nRF52840 Plus + 1.47\" Touch Display");

  // --- 1. init touch (FIRST — D17 is shared, touch reset also resets LCD) ---
  Serial.println("[1/4] Initialising touch (AXS5106L)...");
  Wire.begin();
  touch_init(&Wire, LCD_RST_PIN, D7);

  // --- 2. init LCD (this also resets touch via hardResetPanel) ---
  Serial.println("[2/4] Initialising LCD...");
  initLCD();
  Serial.print("      size=");
  Serial.print(tft.width());
  Serial.print("x");
  Serial.println(tft.height());

  // --- 2b. re-awaken touch (hardResetPanel killed it; touch auto-boots after reset) ---
  // Just reset the I2C bus state and give the controller time to finish booting.
  Wire.end();
  delay(50);
  Wire.begin();
  delay(300);   // AXS5106L needs ~300ms after reset to start scanning
  Serial.println("[2b] Touch re-awakened");

  // --- 3. init LVGL v9 ---
  Serial.println("[3/4] Initialising LVGL v9...");
  lv_init();

  // Tick: use the v9 callback API instead of LV_TICK_CUSTOM.
  lv_tick_set_cb(my_tick_get_cb);

  // Display: create with 1/10-screen buffer + partial render mode.
  static lv_color_t buf1[BUF_SIZE];
  memset(buf1, 0, sizeof(buf1));

  lv_display_t *disp = lv_display_create(SCREEN_W, SCREEN_H);
  lv_display_set_buffers(
      disp, buf1, NULL,
      BUF_SIZE * sizeof(lv_color_t),
      LV_DISPLAY_RENDER_MODE_PARTIAL   // partial render — only dirty areas flushed
  );
  lv_display_set_flush_cb(disp, lvgl_flush_cb);

  // Input device: pointer (touch).
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lvgl_touch_cb);

  // --- 4. create UI ---
  Serial.println("[4/4] Creating UI...");
  createUI();

  Serial.println("Setup done.  Touch the button!");
}

void loop() {
  // v9: lv_timer_handler() takes a timeout in ms (0 = run once, don't block).
  // Returns the time until the next timer expires, or UINT32_MAX if idle.
  uint32_t delay_ms = lv_timer_handler();
  if (delay_ms > 5) delay_ms = 5;
  delay(delay_ms);
}
