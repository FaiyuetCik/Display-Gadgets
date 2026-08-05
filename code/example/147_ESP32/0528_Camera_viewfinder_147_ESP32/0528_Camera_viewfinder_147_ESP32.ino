/*
  XIAO ESP32-S3 Plus + 1.47 Inch Display
  JPEG Camera Viewfinder v0.4.4

  Why this version:
    - Camera-only JPEG diagnostic is stable on your board:
        JPEG QVGA, fmt=4, fps≈12.5, psram≈8.3MB
    - Previous RGB565 camera mode crashed inside cam_task.
    - Therefore this version keeps the camera in stable JPEG mode,
      then decodes JPEG to RGB565 for LCD preview.

  Function:
    - Camera: JPEG QVGA 320x240
    - Decode: JPG_SCALE_2X -> RGB565 160x120
    - Display: rotate 160x120 to 120x160, centered on 1.47 LCD
    - USR1 short: freeze / return live
    - USR1 long : force live

  Important pin conflict:
    - CAM_SDA = GPIO40
    - Previous 1.47 LCD_RST = D17/GPIO40
    - So LCD_RST is disabled in this demo. LCD relies on power-on reset.

  Arduino IDE:
    - Tools > PSRAM > OPI PSRAM
*/

#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include <Arduino_GFX_Library.h>

// ========================= Camera pins =========================

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15

#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// ========================= 1.47 LCD pins =========================

static constexpr int LCD_CS_PIN   = 3;   // D2
static constexpr int LCD_DC_PIN   = 4;   // D3
static constexpr int LCD_SCK_PIN  = 7;   // D8
static constexpr int LCD_MOSI_PIN = 9;   // D10
static constexpr int LCD_RST_PIN  = GFX_NOT_DEFINED; // GPIO40 conflicts with CAM_SDA
static constexpr int LCD_BL_PIN   = 41;  // D18
static constexpr int USR1_PIN     = 42;  // D19

// USR2 is disabled in this demo because previous mapping likely conflicts with CAM_VSYNC/GPIO38.

// ========================= LCD config =========================

static constexpr int LCD_W = 172;
static constexpr int LCD_H = 320;
static constexpr uint32_t LCD_SPI_HZ = 80000000;  // If panel becomes unstable, change to 40000000.

static constexpr int LCD_ROTATION = 0;
static constexpr bool LCD_IPS = false;
static constexpr bool LCD_INVERT_COLORS = false;
static constexpr int LCD_COL_OFFSET_1 = 34;
static constexpr int LCD_ROW_OFFSET_1 = 0;
static constexpr int LCD_COL_OFFSET_2 = 34;
static constexpr int LCD_ROW_OFFSET_2 = 0;

Arduino_DataBus *lcdBus = new Arduino_ESP32SPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  LCD_SCK_PIN,
  LCD_MOSI_PIN,
  GFX_NOT_DEFINED
);

Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus,
  LCD_RST_PIN,
  LCD_ROTATION,
  LCD_IPS,
  LCD_W,
  LCD_H,
  LCD_COL_OFFSET_1,
  LCD_ROW_OFFSET_1,
  LCD_COL_OFFSET_2,
  LCD_ROW_OFFSET_2
);

// ========================= Colors =========================

static constexpr uint16_t C_BLACK  = RGB565_BLACK;
static constexpr uint16_t C_WHITE  = RGB565_WHITE;
static constexpr uint16_t C_GREEN  = RGB565_LIGHTGREEN;
static constexpr uint16_t C_RED    = RGB565_RED;
static constexpr uint16_t C_BLUE   = RGB565_BLUE;
static constexpr uint16_t C_CYAN   = RGB565_CYAN;
static constexpr uint16_t C_YELLOW = RGB565_YELLOW;
static constexpr uint16_t C_GRAY   = 0x8410;
static constexpr uint16_t C_DIM    = 0x2104;
static constexpr uint16_t C_LINE   = 0x39E7;

// 1.47 has normal color order in our previous Dashboard.
static constexpr uint16_t V_RED    = C_RED;
static constexpr uint16_t V_BLUE   = C_BLUE;
static constexpr uint16_t V_YELLOW = C_YELLOW;
static constexpr uint16_t V_CYAN   = C_CYAN;
static constexpr uint16_t V_GREEN  = C_GREEN;
static constexpr uint16_t V_WHITE  = C_WHITE;

// ========================= Preview config =========================

// v0.4.2 clean/sharp preview:
// Camera uses JPEG QVGA 320x240, then decodes at full size.
// We center-crop 320x240 -> 172x240 and push one complete display buffer.
// Compared with 240x240, QVGA keeps more horizontal detail and avoids the
// soft square-frame resampling look.
static constexpr int SRC_W = 320;
static constexpr int SRC_H = 240;
static constexpr int RGB_BYTES = SRC_W * SRC_H * 2;

static constexpr bool PREVIEW_ROTATE_CW = false;
static constexpr bool PREVIEW_MIRROR_X = false;
static constexpr bool PREVIEW_MIRROR_Y = false;

static constexpr int PREVIEW_W = LCD_W;  // 172
static constexpr int PREVIEW_H = 240;
static constexpr int PREVIEW_X = 0;
static constexpr int PREVIEW_Y = 34;
static constexpr int PREVIEW_CROP_X = (SRC_W - PREVIEW_W) / 2; // 74
static constexpr int PREVIEW_CROP_Y = (SRC_H - PREVIEW_H) / 2; // 0
static constexpr int PREVIEW_BYTES = PREVIEW_W * PREVIEW_H * 2;

// If preview colors look wrong, toggle this.
static constexpr bool DECODED_RGB565_BYTE_SWAP = false;

// Screen OSD is static only. Dynamic on-screen FPS/heap text causes visible
// shimmer on this small SPI LCD while live video is refreshing.
static constexpr bool SHOW_DYNAMIC_SCREEN_STATUS = false;

// v0.4.4 anti-flicker / anti-overexposure strategy:
// Do not rely on auto exposure for live preview. Indoor LED light + AE/AGC
// pumping is a major source of visible brightness flicker.
// Start with a manual exposure profile, and long-press USR1 to cycle presets.
static constexpr bool LOCK_EXPOSURE_AFTER_WARMUP = false;

// JPEG quality: lower number = better quality but lower FPS.
// 8 is sharper than 10. If FPS drops too much, change it back to 10 or 12.
static constexpr int JPEG_QUALITY_PREVIEW = 8;

// Manual exposure presets. Smaller AEC = darker / shorter exposure.
// Preset 0 is intentionally conservative to avoid overexposure.
// Long-press USR1 cycles: 0 -> 1 -> 2 -> 3 -> 0.
static const uint16_t AEC_PRESETS[] = {120, 180, 240, 320};
static const uint8_t  AGC_PRESETS[] = {0,   0,   1,   2};
static constexpr uint8_t EXPOSURE_PRESET_COUNT =
    sizeof(AEC_PRESETS) / sizeof(AEC_PRESETS[0]);
uint8_t g_exposurePresetIndex = 1;

// Mild sensor tuning. Keep these conservative to avoid noisy preview.
static constexpr int SENSOR_CONTRAST = 1;
static constexpr int SENSOR_BRIGHTNESS = -1;
static constexpr int SENSOR_SATURATION = 0;
static constexpr int SENSOR_SHARPNESS = 2;
static constexpr int SENSOR_DENOISE = 1;


// ========================= Buttons =========================

static constexpr uint32_t BTN_DEBOUNCE_MS = 35;
static constexpr uint32_t BTN_LONG_MS = 700;

static constexpr int BTN_NONE  = 0;
static constexpr int BTN_SHORT = 1;
static constexpr int BTN_LONG  = 2;

struct ButtonState {
  bool lastRawPressed = false;
  bool stablePressed = false;
  uint32_t lastChangeMs = 0;
  uint32_t pressStartMs = 0;
};

ButtonState g_usr1;

// ========================= Runtime =========================

enum UiMode {
  MODE_LIVE = 0,
  MODE_LAST = 1,
};

UiMode g_mode = MODE_LIVE;

bool g_lcdOk = false;
bool g_cameraOk = false;
bool g_hasLastFrame = false;
bool g_setupBlocked = false;
bool g_redrawChrome = true;

uint8_t *g_rgbFrame = nullptr;
uint8_t *g_displayFrame = nullptr;
uint8_t *g_lastFrame = nullptr;

uint32_t g_frameCount = 0;
uint32_t g_totalFrames = 0;
uint32_t g_lastFpsMs = 0;
float g_fps = 0.0f;

bool g_exposureLocked = false;
uint16_t g_lockedAecValue = 0;
uint8_t g_lockedAgcGain = 0;

String g_lastStatus = "";
String g_lastHint = "";
String g_lastHeap = "";
uint32_t g_lastStatusUpdateMs = 0;

// ========================= Exposure / Flicker control =========================

static void applyExposurePreset(uint8_t idx) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;

  if (idx >= EXPOSURE_PRESET_COUNT) idx = 0;
  g_exposurePresetIndex = idx;

  g_lockedAecValue = AEC_PRESETS[g_exposurePresetIndex];
  g_lockedAgcGain = AGC_PRESETS[g_exposurePresetIndex];

  // Manual exposure/gain: this is the main anti-flicker path.
  // It also prevents the camera from easily blowing out highlights.
  s->set_exposure_ctrl(s, 0);
  s->set_gain_ctrl(s, 0);
  s->set_aec_value(s, g_lockedAecValue);
  s->set_agc_gain(s, g_lockedAgcGain);

  // Keep AWB on for now; it stabilizes color without changing exposure time.
  s->set_whitebal(s, 1);

  g_exposureLocked = true;
  g_totalFrames = 0;

  Serial.print("[CAM] manual exposure preset=");
  Serial.print(g_exposurePresetIndex);
  Serial.print(" aec=");
  Serial.print(g_lockedAecValue);
  Serial.print(" agc=");
  Serial.println(g_lockedAgcGain);
}

static void cycleExposurePreset() {
  uint8_t next = (g_exposurePresetIndex + 1) % EXPOSURE_PRESET_COUNT;
  applyExposurePreset(next);
}

static void unlockCameraExposureForRelock() {
  // Kept for compatibility with earlier versions.
  // In v0.4.4, long press cycles manual exposure presets instead.
  cycleExposurePreset();
}

static void maybeLockCameraExposure() {
  // v0.4.4 uses manual exposure from startup.
  // No warm-up lock is needed.
}

// ========================= Helpers =========================

static void acquireForLcd() {
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  delayMicroseconds(2);
}

static uint16_t fixRgb565(uint16_t c) {
  if (DECODED_RGB565_BYTE_SWAP) {
    return (uint16_t)((c << 8) | (c >> 8));
  }
  return c;
}

static String padRight(String s, int width) {
  while ((int)s.length() < width) s += " ";
  if ((int)s.length() > width) s = s.substring(0, width);
  return s;
}

static void printTextFixed(int x, int y, uint16_t color, String s, int widthChars) {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->setTextSize(1);
  gfx->setTextColor(color, C_BLACK);
  gfx->setCursor(x, y);
  gfx->print(padRight(s, widthChars));
}

static const char *modeName() {
  return (g_mode == MODE_LIVE) ? "LIVE" : "LAST";
}

static uint8_t *allocPsramOrHeap(size_t bytes, const char *name) {
  uint8_t *p = nullptr;

  if (psramFound()) {
    p = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }

  if (!p) {
    p = (uint8_t *)malloc(bytes);
  }

  Serial.print("[MEM] ");
  Serial.print(name);
  Serial.print(" ");
  Serial.print(bytes);
  Serial.print(" bytes -> ");
  Serial.println(p ? "OK" : "FAILED");

  return p;
}

static bool allocBuffers() {
  if (!g_rgbFrame) {
    g_rgbFrame = allocPsramOrHeap(RGB_BYTES, "rgbFrame240");
  }
  if (!g_displayFrame) {
    g_displayFrame = allocPsramOrHeap(PREVIEW_BYTES, "displayFrame172x240");
  }
  if (!g_lastFrame) {
    g_lastFrame = allocPsramOrHeap(PREVIEW_BYTES, "lastFrame172x240");
  }

  return g_rgbFrame && g_displayFrame && g_lastFrame;
}

// ========================= LCD UI =========================

static void lcdWriteMadctlFix() {
  // This is copied from the proven 0519 Dashboard driver code:
  // ST7789-compatible init + bus-level MADCTL fix for JD9853A 172x320.
  acquireForLcd();
  lcdBus->beginWrite();
  lcdBus->writeCommand(0x36);
  lcdBus->write(0x48);
  lcdBus->endWrite();

  Serial.println("[LCD] MADCTL bus write: 0x36 = 0x48");
}

static bool initLcd() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);

  acquireForLcd();

  if (!gfx->begin(LCD_SPI_HZ)) {
    g_lcdOk = false;
    Serial.println("[LCD] begin failed");
    return false;
  }

  // Do not call invertDisplay() here. The proven 0519 Dashboard path did not use it.
  lcdWriteMadctlFix();

  acquireForLcd();
  gfx->fillScreen(C_BLACK);
  gfx->setTextWrap(false);

  g_lcdOk = true;
  Serial.println("[LCD] OK 1.47 ST7789-compatible JD9853A path, no LCD_RST");
  return true;
}

static void drawChrome() {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  // Minimal static UI. Keep the screen quiet: no dynamic text over the video.
  gfx->setTextSize(2);
  gfx->setTextColor(V_GREEN, C_BLACK);
  gfx->setCursor(8, 8);
  gfx->print("XIAO Cam");

  gfx->setTextSize(1);
  gfx->setTextColor(V_CYAN, C_BLACK);
  gfx->setCursor(125, 16);
  gfx->print("LIVE");

  gfx->drawFastHLine(0, 31, LCD_W, C_LINE);
  gfx->drawFastHLine(0, PREVIEW_Y + PREVIEW_H + 2, LCD_W, C_LINE);

  // Bottom hint is intentionally tiny/static to avoid refresh flicker.
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(8, 291);
  gfx->print("USR1 shot  hold:EXP");

  g_lastStatus = "";
  g_lastHint = "";
  g_lastHeap = "";
  g_lastStatusUpdateMs = 0;
  g_redrawChrome = false;
}

static void updateStatusText() {
  if (!g_lcdOk || !SHOW_DYNAMIC_SCREEN_STATUS) return;

  uint32_t nowStatus = millis();
  if (nowStatus - g_lastStatusUpdateMs < 1000) {
    return;
  }
  g_lastStatusUpdateMs = nowStatus;

  String status = String(modeName()) + "  FPS " + String(g_fps, 1);
  if (status != g_lastStatus) {
    g_lastStatus = status;
    printTextFixed(8, 263, (g_mode == MODE_LIVE) ? V_GREEN : V_YELLOW, status, 22);
  }

  String heap = String("H ") + String(ESP.getFreeHeap() / 1024) + "K";
  heap += String(" P ") + String(ESP.getFreePsram() / 1024) + "K";

  if (heap != g_lastHeap) {
    g_lastHeap = heap;
    printTextFixed(8, 300, C_GRAY, heap, 22);
  }
}

static void drawErrorScreen(const char *title, const char *line1, const char *line2) {
  if (!g_lcdOk) return;

  acquireForLcd();
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(V_RED, C_BLACK);
  gfx->setCursor(12, 80);
  gfx->print(title);

  gfx->setTextSize(1);
  gfx->setTextColor(V_WHITE, C_BLACK);
  gfx->setCursor(12, 120);
  gfx->print(line1);
  gfx->setCursor(12, 136);
  gfx->print(line2);
}

// ========================= Frame rendering =========================

static void buildPreviewFrame(const uint8_t *srcBuf, uint8_t *dstBuf) {
  if (!srcBuf || !dstBuf) return;

  const uint16_t *src = (const uint16_t *)srcBuf;
  uint16_t *dst = (uint16_t *)dstBuf;

  for (int y = 0; y < PREVIEW_H; y++) {
    int sy0 = y + PREVIEW_CROP_Y;
    if (sy0 < 0) sy0 = 0;
    if (sy0 >= SRC_H) sy0 = SRC_H - 1;
    int sy = PREVIEW_MIRROR_Y ? (SRC_H - 1 - sy0) : sy0;

    uint16_t *row = dst + y * PREVIEW_W;

    for (int x = 0; x < PREVIEW_W; x++) {
      int sx0 = x + PREVIEW_CROP_X;
      if (sx0 < 0) sx0 = 0;
      if (sx0 >= SRC_W) sx0 = SRC_W - 1;
      int sx = PREVIEW_MIRROR_X ? (SRC_W - 1 - sx0) : sx0;

      uint16_t c = src[sy * SRC_W + sx];
      row[x] = fixRgb565(c);
    }
  }
}

static void renderDecodedFrame(const uint8_t *buf) {
  if (!g_lcdOk || !buf || !g_displayFrame) return;

  buildPreviewFrame(buf, g_displayFrame);

  acquireForLcd();

  // One full bitmap push. This is much less flickery than pushing many
  // individual scanline bands.
  gfx->draw16bitRGBBitmap(PREVIEW_X, PREVIEW_Y, (uint16_t *)g_displayFrame, PREVIEW_W, PREVIEW_H);
}

static void renderPreviewFrameBuffer(const uint8_t *displayBuf) {
  if (!g_lcdOk || !displayBuf) return;
  acquireForLcd();
  gfx->draw16bitRGBBitmap(PREVIEW_X, PREVIEW_Y, (uint16_t *)displayBuf, PREVIEW_W, PREVIEW_H);
}

static bool decodeJpegToRgb565(camera_fb_t *fb, uint8_t *out) {
  if (!fb || !out) return false;

  bool ok = jpg2rgb565(fb->buf, fb->len, out, JPG_SCALE_NONE);
  if (!ok) {
    Serial.println("[JPEG] jpg2rgb565 failed");
  }
  return ok;
}

// ========================= Camera =========================

static bool initCameraJpeg() {
  Serial.println("[CAM] initCameraJpeg()");
  Serial.print("[CAM] psram=");
  Serial.println(psramFound() ? "YES" : "NO");

  if (!psramFound()) {
    Serial.println("[FATAL] PSRAM disabled/not detected");
    return false;
  }

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
#else
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
#endif

  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA; // 320x240, full decode then center crop
  config.jpeg_quality = JPEG_QUALITY_PREVIEW;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  Serial.print("[CAM] JPEG QVGA 320x240 quality="); Serial.print(JPEG_QUALITY_PREVIEW); Serial.println(" XCLK 10MHz fb=2 PSRAM latest");

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.print("[CAM] init failed err=0x");
    Serial.println((uint32_t)err, HEX);
    g_cameraOk = false;
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    Serial.print("[CAM] sensor PID=0x");
    Serial.println(s->id.PID, HEX);

    // Preview tuning.
    // Most visible flicker was from auto exposure/gain pumping, so we switch
    // to manual exposure immediately after basic sensor setup.
    s->set_quality(s, JPEG_QUALITY_PREVIEW);
    s->set_brightness(s, SENSOR_BRIGHTNESS);
    s->set_contrast(s, SENSOR_CONTRAST);
    s->set_saturation(s, SENSOR_SATURATION);
    s->set_whitebal(s, 1);
    s->set_lenc(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    // If your board package reports no set_denoise(), comment out the next line.
    s->set_denoise(s, SENSOR_DENOISE);
    // If your board package reports no set_sharpness(), comment out the next line.
    s->set_sharpness(s, SENSOR_SHARPNESS);
    s->set_gainceiling(s, GAINCEILING_2X);

    applyExposurePreset(g_exposurePresetIndex);
  }

  g_cameraOk = true;
  Serial.println("[CAM] OK");
  return true;
}

// ========================= Buttons =========================

static int updateUsr1Button() {
  bool rawPressed = (digitalRead(USR1_PIN) == LOW);
  uint32_t now = millis();

  if (rawPressed != g_usr1.lastRawPressed) {
    g_usr1.lastRawPressed = rawPressed;
    g_usr1.lastChangeMs = now;
  }

  if ((now - g_usr1.lastChangeMs) < BTN_DEBOUNCE_MS) {
    return BTN_NONE;
  }

  if (rawPressed != g_usr1.stablePressed) {
    g_usr1.stablePressed = rawPressed;

    if (g_usr1.stablePressed) {
      g_usr1.pressStartMs = now;
    } else {
      uint32_t held = now - g_usr1.pressStartMs;
      return (held >= BTN_LONG_MS) ? BTN_LONG : BTN_SHORT;
    }
  }

  return BTN_NONE;
}

static void handleButtons() {
  int e1 = updateUsr1Button();

  if (e1 == BTN_SHORT) {
    if (g_mode == MODE_LIVE) {
      if (g_hasLastFrame) {
        g_mode = MODE_LAST;
        g_redrawChrome = true;
        Serial.println("[BTN] USR1 short -> show frozen frame");
      } else {
        Serial.println("[BTN] USR1 short -> no frozen frame yet");
      }
    } else {
      g_mode = MODE_LIVE;
      g_redrawChrome = true;
      Serial.println("[BTN] USR1 short -> live");
    }
  } else if (e1 == BTN_LONG) {
    g_mode = MODE_LIVE;
    g_redrawChrome = true;
    unlockCameraExposureForRelock();
    Serial.println("[BTN] USR1 long -> cycle exposure preset");
  }
}

// ========================= Arduino =========================

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("=== XIAO ESP32-S3 Plus + 1.47 JPEG Camera Viewfinder v0.4.4 ===");
  Serial.println("[INFO] Camera stays in stable JPEG mode; LCD displays decoded RGB565");
  Serial.println("[INFO] v0.4.4 uses manual exposure presets to reduce LED/AE flicker and overexposure");
  Serial.println("[INFO] LCD_RST disabled because GPIO40 is CAM_SDA");
  Serial.println("[LCD] using proven Dashboard path: Arduino_ST7789, IPS=false, MADCTL=0x48 via lcdBus");
  Serial.print("[VIEW] rotateCW=");
  Serial.print(PREVIEW_ROTATE_CW ? "Y" : "N");
  Serial.print(" mirrorX=");
  Serial.print(PREVIEW_MIRROR_X ? "Y" : "N");
  Serial.print(" mirrorY=");
  Serial.print(PREVIEW_MIRROR_Y ? "Y" : "N");
  Serial.print(" preview=");
  Serial.print(PREVIEW_W);
  Serial.print("x");
  Serial.print(PREVIEW_H);
  Serial.print(" src=");
  Serial.print(SRC_W);
  Serial.print("x");
  Serial.print(SRC_H);
  Serial.print(" dynamicScreenStatus=");
  Serial.println(SHOW_DYNAMIC_SCREEN_STATUS ? "Y" : "N");

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  pinMode(USR1_PIN, INPUT_PULLUP);

  if (!psramFound()) {
    if (initLcd()) {
      drawErrorScreen("PSRAM", "Enable OPI PSRAM", "Tools menu");
    }
    g_setupBlocked = true;
    return;
  }

  if (!allocBuffers()) {
    if (initLcd()) {
      drawErrorScreen("MEM ERR", "RGB buffer failed", "Check PSRAM");
    }
    g_setupBlocked = true;
    return;
  }

  // Camera first because CAM_SDA/GPIO40 conflicts with LCD_RST.
  if (!initCameraJpeg()) {
    if (initLcd()) {
      drawErrorScreen("CAM ERR", "Camera init failed", "Check FPC");
    }
    g_setupBlocked = true;
    return;
  }

  delay(200);

  if (!initLcd()) {
    Serial.println("[LCD] init failed");
    g_setupBlocked = true;
    return;
  }

  g_lastFpsMs = millis();
  drawChrome();
  updateStatusText();
}

void loop() {
  if (g_setupBlocked) {
    delay(500);
    return;
  }

  if (!g_cameraOk || !g_lcdOk) {
    delay(200);
    return;
  }

  handleButtons();

  if (g_redrawChrome) {
    drawChrome();
    updateStatusText();

    if (g_mode == MODE_LAST && g_hasLastFrame) {
      renderPreviewFrameBuffer(g_lastFrame);
    }
  }

  if (g_mode == MODE_LAST && g_hasLastFrame) {
    updateStatusText();
    delay(20);
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAM] fb_get failed");
    delay(30);
    return;
  }

  bool ok = decodeJpegToRgb565(fb, g_rgbFrame);

  if (ok) {
    renderDecodedFrame(g_rgbFrame);
    memcpy(g_lastFrame, g_displayFrame, PREVIEW_BYTES);
    g_hasLastFrame = true;
  }

  esp_camera_fb_return(fb);

  g_frameCount++;
  g_totalFrames++;
  maybeLockCameraExposure();

  uint32_t now = millis();
  if (now - g_lastFpsMs >= 1000) {
    g_fps = (float)g_frameCount * 1000.0f / (float)(now - g_lastFpsMs);
    g_frameCount = 0;
    g_lastFpsMs = now;

    Serial.print("[LIVE] fps=");
    Serial.print(g_fps, 1);
    Serial.print(" heap=");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" psram=");
    Serial.print(ESP.getFreePsram());
    Serial.print(" mode=");
    Serial.print(modeName());
    Serial.print(" expPreset="); Serial.print(g_exposurePresetIndex); Serial.print(" aeLock=");
    Serial.print(g_exposureLocked ? "Y" : "N");
    Serial.print(" aec=");
    Serial.print(g_lockedAecValue);
    Serial.print(" agc=");
    Serial.println(g_lockedAgcGain);
  }

  updateStatusText();
}
