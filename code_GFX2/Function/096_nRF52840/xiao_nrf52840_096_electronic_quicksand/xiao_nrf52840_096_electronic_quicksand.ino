/*
  XIAO nRF52840 Plus 0.96 Inch Display electronic quicksand demo.

  Uses Arduino_GFX with software SPI — hardware SPI on nRF52840
  is NOT compatible with this 0.96" ST7789 panel.

  0.96 panel: 80x160 ST7789, rotation=2, invertDisplay=true, colstart=24.
  These values match the factory firmware:
    example/096_nRF52840/0526_DashBoard_096_nRF52840.ino

  Ported to Seeed_GFX2: the Board/Config templates replace the original
  Arduino_SWSPI + Arduino_ST7789 bus/panel setup, forceBacklightOn(),
  hardResetPanel() and gfx->begin()/invertDisplay(true).
  Board_XIAO_0inch96_LCD<RST=38,BL=37> + Config_Seeed_0inch96_LCD_ST7789 bake
  80x160 BGR rot2 invert=false. The rgb565() helper no longer swaps R/B (the
  config's ST7789 BGR bit now handles it). IMU (LSM6DS3) and particle physics
  unchanged.
*/

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_ST7789.h"
#include "panel/Panel_TFT.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include "LSM6DS3.h"
#include <math.h>

// ========================= Pins =========================

static constexpr int8_t LCD_RST_PIN = 38;  // XIAO nRF52840 Plus (raw GPIO)
static constexpr int8_t LCD_BL_PIN  = 37;

// ========================= Display =========================

Seeed_GFX display;

// ========================= Quicksand params =========================

static constexpr uint8_t GRID_W = 13;
static constexpr uint8_t GRID_H = 26;
static constexpr uint8_t CELL_SIZE = 6;
static constexpr uint16_t SAND_PARTICLES = 65;
static constexpr float GRAVITY = 0.34f;
static constexpr float DAMPING = 0.88f;
static constexpr float MAX_VELOCITY = 1.25f;
static constexpr float TOP_MOBILITY = 1.35f;
static constexpr float BOTTOM_MOBILITY = 0.52f;
static constexpr uint8_t FRAME_INTERVAL_MS = 8;

// ========================= Colors =========================

static constexpr uint16_t C_BLACK = 0x0000;
static constexpr uint16_t C_WHITE = 0xFFFF;

// ========================= IMU =========================

LSM6DS3 imu(I2C_MODE, 0x6A);

// ========================= Particle system =========================

struct Particle {
  float x, y, vx, vy;
  int8_t cellX, cellY;
  int8_t oldCellX, oldCellY;
  uint16_t color;
};

Particle particles[SAND_PARTICLES];
bool occupied[GRID_W][GRID_H];
uint16_t particleOrder[SAND_PARTICLES];

float accelX = 0, accelY = 0;
float gravityX = 0, gravityY = 1.0f;
float depthMinScore = 0, depthSpan = 1.0f;
int16_t gridOriginX = 0, gridOriginY = 0;
uint32_t lastFrameMs = 0;

// ========================= Color helpers =========================

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  // Config_Seeed_0inch96_LCD_ST7789 sets the ST7789 BGR bit, so standard
  // RGB565 packing now displays correct colours (R/B swap removed).
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static uint16_t blendColor(uint16_t a, uint16_t b, uint8_t amount) {
  uint8_t ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
  uint8_t br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
  uint8_t rr = ar + ((int16_t)(br - ar) * amount) / 255;
  uint8_t rg = ag + ((int16_t)(bg - ag) * amount) / 255;
  uint8_t rb = ab + ((int16_t)(bb - ab) * amount) / 255;
  return (rr << 11) | (rg << 5) | rb;
}

// ========================= LCD init =========================

static bool initDisplay() {
  if (!display.begin<Board_XIAO_0inch96_LCD<LCD_RST_PIN, LCD_BL_PIN>,
                     Config_Seeed_0inch96_LCD_ST7789>()) {
    Serial.println(display.lastResult().message);
    return false;
  }
  display.fillScreen(TFT_BLACK);

  Serial.print("LCD w=");  Serial.print(display.width());
  Serial.print(" h=");     Serial.println(display.height());

  gridOriginX = (display.width()  - GRID_W * CELL_SIZE) / 2;
  gridOriginY = (display.height() - GRID_H * CELL_SIZE) / 2;
  return true;
}

static void drawBackground() {
  display.fillScreen(C_BLACK);
}

static void showMessage(const char *title, const char *line) {
  display.fillScreen(C_BLACK);
  display.setTextColor(rgb565(255, 220, 80), C_BLACK);
  display.setTextSize(1);
  display.setCursor(4, 44);  display.print(title);
  display.setTextColor(C_WHITE, C_BLACK);
  display.setCursor(8, 72);  display.print(line);
}

// ========================= Cell drawing =========================

static void drawCell(int8_t cx, int8_t cy, uint16_t color) {
  int16_t px = gridOriginX + cx * CELL_SIZE + 1;
  int16_t py = gridOriginY + cy * CELL_SIZE + 1;
  display.fillRect(px, py, CELL_SIZE - 2, CELL_SIZE - 2, color);
}

static void clearCell(int8_t cx, int8_t cy) {
  if (cx < 0 || cx >= GRID_W || cy < 0 || cy >= GRID_H) return;
  drawCell(cx, cy, C_BLACK);
}

// ========================= Particle physics (same as 1.14) =========================

static float randUnit() {
  return random(-1000, 1001) * 0.001f;
}

static void clearOccupancy() {
  memset(occupied, 0, sizeof(occupied));
}

static void resetParticles() {
  const uint16_t deep   = rgb565(196, 128, 18);
  const uint16_t bright = rgb565(255, 236, 92);
  uint16_t idx = 0;
  clearOccupancy();
  for (int8_t y = GRID_H - 5; y < GRID_H && idx < SAND_PARTICLES; ++y)
    for (int8_t x = 0; x < GRID_W && idx < SAND_PARTICLES; ++x) {
      Particle &p = particles[idx++];
      p.x = x; p.y = y;
      p.vx = randUnit() * 0.2f; p.vy = randUnit() * 0.2f;
      p.cellX = x; p.cellY = y; p.oldCellX = -1; p.oldCellY = -1;
      p.color = blendColor(deep, bright, random(50, 230));
      occupied[x][y] = true;
    }
  for (uint16_t i = 0; i < SAND_PARTICLES; ++i) particleOrder[i] = i;
}

static void readGravity() {
  float x = imu.readFloatAccelX(), y = imu.readFloatAccelY();
  accelX = accelX * 0.78f + x * 0.22f;
  accelY = accelY * 0.78f + y * 0.22f;
  float len = sqrtf(accelX*accelX + accelY*accelY);
  if (len < 0.06f) return;
  gravityX = gravityX * 0.72f + (accelX/len) * 0.28f;
  gravityY = gravityY * 0.72f + (accelY/len) * 0.28f;
}

static void updateDepthRange() {
  float minS = 100000, maxS = -100000;
  const float corners[4][2] = {{0,0},{(float)(GRID_W-1),0},{0,(float)(GRID_H-1)},{(float)(GRID_W-1),(float)(GRID_H-1)}};
  for (int i = 0; i < 4; ++i) {
    float s = corners[i][0]*gravityX + corners[i][1]*gravityY;
    if (s < minS) minS = s;
    if (s > maxS) maxS = s;
  }
  depthMinScore = minS; depthSpan = maxS - minS;
  if (depthSpan < 0.001f) depthSpan = 1.0f;
}

static float particleDepth(const Particle &p) {
  return constrain((p.cellX*gravityX + p.cellY*gravityY - depthMinScore)/depthSpan, 0.0f, 1.0f);
}

static float particleMobility(const Particle &p) {
  float d = particleDepth(p);
  return TOP_MOBILITY + (BOTTOM_MOBILITY - TOP_MOBILITY)*d;
}

static float clampV(float v, float lim) { return (v > lim) ? lim : ((v < -lim) ? -lim : v); }

static void constrainToScreen(Particle &p) {
  if (p.x < 0) { p.x = 0; if (p.vx < 0) p.vx *= -0.25f; }
  else if (p.x > GRID_W-1) { p.x = GRID_W-1; if (p.vx > 0) p.vx *= -0.25f; }
  if (p.y < 0) { p.y = 0; if (p.vy < 0) p.vy *= -0.25f; }
  else if (p.y > GRID_H-1) { p.y = GRID_H-1; if (p.vy > 0) p.vy *= -0.25f; }
}

static bool pickCell(Particle &p, int8_t wX, int8_t wY, int8_t &oX, int8_t &oY, float mob) {
  float best = 100000; bool found = false;
  for (int8_t r = 0; r <= 1; ++r)
    for (int8_t dy = -r; dy <= r; ++dy)
      for (int8_t dx = -r; dx <= r; ++dx) {
        if (abs(dx)+abs(dy) != r) continue;
        int8_t cx = wX+dx, cy = wY+dy;
        if (cx < 0 || cx >= GRID_W || cy < 0 || cy >= GRID_H) continue;
        if (occupied[cx][cy]) continue;
        float px = (float)cx - p.x, py = (float)cy - p.y;
        float s = px*px + py*py - (px*gravityX + py*gravityY)*(0.40f + mob*0.28f);
        if (s < best) { best = s; oX = cx; oY = cy; found = true; }
      }
  return found;
}

static void updateParticles() {
  clearOccupancy();
  for (uint16_t i = 1; i < SAND_PARTICLES; ++i) {
    uint16_t key = particleOrder[i];
    float keyS = particles[key].cellX*gravityX + particles[key].cellY*gravityY;
    int16_t j = i-1;
    while (j >= 0 && particles[particleOrder[j]].cellX*gravityX + particles[particleOrder[j]].cellY*gravityY < keyS)
      { particleOrder[j+1] = particleOrder[j]; --j; }
    particleOrder[j+1] = key;
  }
  for (uint16_t i = 0; i < SAND_PARTICLES; ++i) {
    Particle &p = particles[particleOrder[i]];
    float mob = particleMobility(p);
    p.vx = clampV((p.vx + gravityX*GRAVITY*mob)*(0.74f + mob*0.15f), MAX_VELOCITY*(0.52f + mob*0.42f));
    p.vy = clampV((p.vy + gravityY*GRAVITY*mob)*(0.74f + mob*0.15f), MAX_VELOCITY*(0.52f + mob*0.42f));
    p.x += p.vx; p.y += p.vy; constrainToScreen(p);
    int8_t wX = constrain((int)roundf(p.x),0,GRID_W-1), wY = constrain((int)roundf(p.y),0,GRID_H-1);
    int8_t cX = p.cellX, cY = p.cellY;
    if (!pickCell(p, wX, wY, cX, cY, mob)) { cX = p.cellX; cY = p.cellY; p.vx *= -0.18f; p.vy *= -0.18f; }
    p.x = p.x*0.35f + cX*0.65f; p.y = p.y*0.35f + cY*0.65f;
    p.oldCellX = p.cellX; p.oldCellY = p.cellY;
    p.cellX = cX; p.cellY = cY;
    occupied[cX][cY] = true;
  }
}

static void drawParticles() {
  for (uint16_t i = 0; i < SAND_PARTICLES; ++i) {
    Particle &p = particles[i];
    if (p.oldCellX != p.cellX || p.oldCellY != p.cellY) clearCell(p.oldCellX, p.oldCellY);
  }
  for (uint16_t i = 0; i < SAND_PARTICLES; ++i) {
    Particle &p = particles[i];
    if (p.oldCellX != p.cellX || p.oldCellY != p.cellY) drawCell(p.cellX, p.cellY, p.color);
  }
}

// ========================= Setup / Loop =========================

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!initDisplay()) {
    while (1) delay(1000);
  }
  showMessage("Quicksand", "Starting IMU...");

  Wire.begin();
  int ret = imu.begin();
  Serial.println();
  Serial.println("=== Electronic Quicksand 0.96 ===");
  Serial.print("imu.begin="); Serial.println(ret);

  if (ret != 0) { showMessage("IMU Error", "Check LSM6DS3"); while (1) delay(1000); }

  randomSeed((uint32_t)micros());
  drawBackground();
  resetParticles();
  for (uint16_t i = 0; i < SAND_PARTICLES; ++i)
    drawCell(particles[i].cellX, particles[i].cellY, particles[i].color);
  lastFrameMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastFrameMs < FRAME_INTERVAL_MS) return;
  lastFrameMs = now;
  readGravity();
  updateDepthRange();
  updateParticles();
  drawParticles();
}
