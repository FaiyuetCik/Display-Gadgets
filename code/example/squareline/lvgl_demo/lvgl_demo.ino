/*
  ============================================================================
  LVGL Minimal Demo — XIAO 1.47 Inch Touch Display (nRF52840 Plus)
  ============================================================================

  A self-contained LVGL demo that proves display + touch + LVGL all work
  together.  No SquareLine Studio required — the UI is built with raw LVGL
  widgets.

  Uses LVGL **v8.3** (the version Seeed's ecosystem is tested against).

  WHAT IT SHOWS
  – A title label at the top
  – A "Click Me!" button in the centre
  – A counter label below the button
  – Each click increments the counter

  ============================
  HOW TO SET UP (BEFORE UPLOAD)
  ============================

  1. Install Seeed nRF52 Boards in Arduino IDE (Board Manager).

  2. Install Seeed_GFX from:
     https://github.com/Seeed-Studio/Seeed_GFX
     !! Remove the original TFT_eSPI library first, or they will conflict.

  3. Install SeeedStudio_lvgl (LVGL v8.3, Seeed's fork):
     https://github.com/Seeed-Projects/SeeedStudio_lvgl
     — Download the .zip and extract it into your Arduino/libraries folder.
     — The folder must be named "SeeedStudio_lvgl" (rename if needed).
     — !! Remove any other LVGL library first (the official one will
          conflict with Seeed's fork).

  4. Move lv_conf.h to the libraries root:
     — Copy SeeedStudio_lvgl/lv_conf.h → Arduino/libraries/lv_conf.h
     (This is required by the Seeed LVGL library; see its README.)

  5. If Serial does not appear after upload, install Adafruit TinyUSB
     from Library Manager.  The sketch already #includes it.

  6. Board: "Seeed XIAO nRF52840 Plus"
     Port:  select the correct COM port
     Click Upload.

  ============================
  TROUBLESHOOTING
  ============================

  – Colours look inverted (black↔white, red↔cyan)?
      Flip tft.invertDisplay(false) → tft.invertDisplay(true), or vice-versa.
      (See README tip 8.)

  – Colours are wrong but not inverted (e.g. red↔blue)?
      Flip LV_COLOR_16_SWAP (0↔1) in lv_conf.h, OR flip
      tft.setSwapBytes(true↔false) below.

  – "fatal error: lvgl.h: No such file or directory"
      LVGL library is not installed.  Make sure SeeedStudio_lvgl is in
      Arduino/libraries/ and lv_conf.h is in Arduino/libraries/.

  – Touch does not respond:
      Open Serial Monitor (115200 baud).  You should see "Touch ID: xx xx xx"
      on startup.  If not, check the I2C connection (D4=SDA, D5=SCL).
      If the ID prints but touch still doesn't work, try adjusting the
      coordinate mapping in my_touchpad_read().
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

static constexpr uint8_t LCD_CS_PIN  = D2;
static constexpr uint8_t LCD_DC_PIN  = D3;
static constexpr uint8_t LCD_SCK_PIN = D8;
static constexpr uint8_t LCD_MOS_PIN = D10;
static constexpr uint8_t LCD_RST_PIN = D17;
static constexpr uint8_t LCD_BL_PIN  = D18;

TFT_eSPI tft;   // global — shared by initLCD() and the flush callback

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
// Hardware helpers (from the factory display basic example)
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
// LVGL display flush callback (v8 API)
// ============================================================================
// LVGL calls this whenever it has rendered a dirty area.  We push that area
// to the LCD through TFT_eSPI's pushColors().
//
// v8 signature: (lv_disp_drv_t *, const lv_area_t *, lv_color_t *)
// v9 signature: (lv_display_t *, const lv_area_t *, uint8_t *)

static void lvgl_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area,
                          lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);        // Seeed LVGL v8: takes lv_disp_drv_t*
}

// ============================================================================
// LVGL touch input device callback (v8 API)
// ============================================================================
// LVGL calls this periodically (~30 ms).  We read the AXS5106L and feed the
// first touch point back.  No touch → state = RELEASED.
//
// v8 signature: (lv_indev_drv_t *, lv_indev_data_t *)
// v9 signature: (lv_indev_t *, lv_indev_data_t *)

static void lvgl_touch_cb(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  static touch_data_t td;
  static uint32_t call_count = 0;
  call_count++;

  bool has_touch = get_touch_data(&td);

  // Print diagnostic every ~3 seconds (LVGL polls touch ~30 times/sec)
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

    Serial.print("Touch raw: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");

    if (x < 0) x = 0; if (x >= SCREEN_W) x = SCREEN_W - 1;
    if (y < 0) y = 0; if (y >= SCREEN_H) y = SCREEN_H - 1;

    data->state   = LV_INDEV_STATE_PR;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
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
  lv_obj_t *scr = lv_scr_act();

  // ---- title ----
  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "LVGL Demo");
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
  Serial.println("=== LVGL Minimal Demo (v8.3) ===");
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

  // --- 3. init LVGL ---
  Serial.println("[3/4] Initialising LVGL...");
  lv_init();

  // Full-screen buffer (~107 KB): only full_refresh=1 works with Seeed's LVGL fork.
  // Partial-render mode (full_refresh=0) has a multi-strip bug — only 1st strip flushes.
  // This is the minimum working config for now.
  static lv_color_t buf1[SCREEN_W * SCREEN_H];
  memset(buf1, 0, sizeof(buf1));
  static lv_disp_draw_buf_t draw_buf;
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, SCREEN_W * SCREEN_H);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res  = SCREEN_W;
  disp_drv.ver_res  = SCREEN_H;
  disp_drv.flush_cb = lvgl_flush_cb;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.full_refresh = 1;    // MUST: Seeed's partial-render is broken
  lv_disp_drv_register(&disp_drv);

  // Input device: pointer (touch).
  static lv_indev_drv_t indev_drv;                              // v8 driver
  lv_indev_drv_init(&indev_drv);
  indev_drv.type    = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lvgl_touch_cb;
  lv_indev_drv_register(&indev_drv);                            // v8: register

  // Tick: handled by LV_TICK_CUSTOM (millis()) in lv_conf.h — no manual setup needed.

  // --- 4. create UI ---
  Serial.println("[4/4] Creating UI...");
  createUI();

  Serial.println("Setup done.  Touch the button!");

  // Force full-screen initial render, then switch to partial mode so
  // lv_timer_handler() won't re-render the whole screen every call.
  for (int i = 0; i < 30; i++) {
    lv_timer_handler();
    delay(1);
  }
  // Now disable full_refresh — only dirty areas (e.g. counter label) will redraw.
  lv_disp_t *disp = lv_disp_get_default();
  disp->driver->full_refresh = 0;

  Serial.println("Full refresh OFF — touch should work now.");
}

void loop() {
  lv_timer_handler();
  delay(2);
}
