/*
  XIAO ESP32-S3 Plus 0.96 Inch Display electronic quicksand demo.

  This is the 0.96-inch ESP32-S3 version of the electronic quicksand.
  It uses the same low-resolution occupancy-grid trick, with a smaller
  grid that fits the 80x160 ST7789 panel.

  IMU auto-detection: QMI8658 (0x6A/0x6B) or LSM6-compatible (0x6A/0x6B).

  Required libraries:
    - GFX Library for Arduino (Arduino_GFX)
*/

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <math.h>

// ── Pins ───────────────────────────────────────────────────────────

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

static constexpr int  LCD_W = 80;
static constexpr int  LCD_H = 160;
static constexpr int  LCD_ROTATION = 2;
static constexpr bool LCD_IPS = true;
static constexpr int  LCD_COL_OFFSET_1 = 24;
static constexpr int  LCD_ROW_OFFSET_1 = 0;
static constexpr int  LCD_COL_OFFSET_2 = 24;
static constexpr int  LCD_ROW_OFFSET_2 = 0;

static constexpr uint8_t I2C_SDA_PIN  = D4;
static constexpr uint8_t I2C_SCL_PIN  = D5;
static constexpr uint8_t IMU_INT_PIN  = D14;

// ── Quicksand params (0.96" — 80x160) ─────────────────────────────

static constexpr uint8_t  GRID_W             = 13;
static constexpr uint8_t  GRID_H             = 26;
static constexpr uint8_t  CELL_SIZE          = 6;
static constexpr uint16_t SAND_PARTICLES     = 65;
static constexpr float    GRAVITY            = 0.34f;
static constexpr float    DAMPING            = 0.88f;
static constexpr float    MAX_VELOCITY       = 1.25f;
static constexpr float    TOP_MOBILITY       = 1.35f;
static constexpr float    BOTTOM_MOBILITY    = 0.52f;
static constexpr uint8_t  FRAME_INTERVAL_MS  = 8;

// ── Display ────────────────────────────────────────────────────────

Arduino_DataBus *lcdBus = new Arduino_ESP32SPI(
  LCD_DC_PIN, LCD_CS_PIN, LCD_SCK_PIN, LCD_MOSI_PIN
);
Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus, LCD_RST_PIN, LCD_ROTATION, LCD_IPS, LCD_W, LCD_H,
  LCD_COL_OFFSET_1, LCD_ROW_OFFSET_1, LCD_COL_OFFSET_2, LCD_ROW_OFFSET_2
);
Arduino_GFX &tft = *gfx;

static constexpr uint16_t TFT_BLACK = RGB565_BLACK;
static constexpr uint16_t TFT_WHITE = RGB565_WHITE;

// ── IMU types ──────────────────────────────────────────────────────

enum ImuType {
  IMU_NONE = 0,
  IMU_QMI8658,
  IMU_LSM6
};

// ── Particle system ────────────────────────────────────────────────

struct Particle {
  float    x, y, vx, vy;
  int8_t   cellX, cellY;
  int8_t   oldCellX, oldCellY;
  uint16_t color;
};

Particle  particles[SAND_PARTICLES];
bool      occupied[GRID_W][GRID_H];
uint16_t  particleOrder[SAND_PARTICLES];

ImuType   imuType       = IMU_NONE;
uint8_t   imuAddr       = 0;
float     accelX        = 0.0f;
float     accelY        = 0.0f;
float     gravityX      = 0.0f;
float     gravityY      = 1.0f;
float     depthMinScore = 0.0f;
float     depthSpan     = 1.0f;
int16_t   gridOriginX   = 0;
int16_t   gridOriginY   = 0;
uint32_t  lastFrameMs   = 0;

// ── Color helpers ──────────────────────────────────────────────────

// The verified 0.96-inch panel displays red and blue channels swapped.
// Pre-swap R/B here so requested RGB colors appear correctly on the panel.
static uint16_t panelRgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3);
}

static uint16_t blendColor(uint16_t a, uint16_t b, uint8_t amount) {
  uint8_t ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
  uint8_t br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
  uint8_t rr = ar + ((int16_t)(br - ar) * amount) / 255;
  uint8_t rg = ag + ((int16_t)(bg - ag) * amount) / 255;
  uint8_t rb = ab + ((int16_t)(bb - ab) * amount) / 255;
  return (rr << 11) | (rg << 5) | rb;
}

// ── I2C helpers ────────────────────────────────────────────────────

static int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t *val) {
  return i2cRead(addr, reg, val, 1);
}

// ── IMU init ───────────────────────────────────────────────────────

static bool initQmi(uint8_t addr) {
  uint8_t who = 0;
  if (!i2cRead8(addr, 0x00, &who)) return false;
  if (who == 0x00 || who == 0xFF) return false;

  i2cWrite8(addr, 0x02, 0x60);
  i2cWrite8(addr, 0x03, 0x03);
  i2cWrite8(addr, 0x04, 0x53);
  i2cWrite8(addr, 0x08, 0x03);
  delay(20);

  uint8_t data[12] = {};
  if (!i2cRead(addr, 0x35, data, sizeof(data))) return false;

  imuType = IMU_QMI8658;
  imuAddr = addr;
  Serial.printf("[IMU] QMI8658-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

static bool initLsm(uint8_t addr) {
  uint8_t who = 0;
  if (!i2cRead8(addr, 0x0F, &who)) return false;
  if (who != 0x69 && who != 0x6A && who != 0x6C) return false;

  i2cWrite8(addr, 0x10, 0x60);
  i2cWrite8(addr, 0x11, 0x60);
  delay(20);

  imuType = IMU_LSM6;
  imuAddr = addr;
  Serial.printf("[IMU] LSM6-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

static bool initImu() {
  imuType = IMU_NONE;
  imuAddr = 0;

  bool ok = initQmi(0x6B) || initQmi(0x6A) || initLsm(0x6A) || initLsm(0x6B);
  if (!ok) Serial.println("[IMU] not found");
  return ok;
}

// ── IMU read ───────────────────────────────────────────────────────

static bool readAccel(float &x, float &y) {
  if (imuType == IMU_QMI8658) {
    uint8_t d[6] = {};
    if (!i2cRead(imuAddr, 0x35, d, sizeof(d))) return false;
    x = le16(&d[0]) / 16384.0f;
    y = le16(&d[2]) / 16384.0f;
    return true;
  }
  if (imuType == IMU_LSM6) {
    uint8_t a[6] = {};
    if (!i2cRead(imuAddr, 0x28, a, sizeof(a))) return false;
    x = le16(&a[0]) * 0.000061f;
    y = le16(&a[2]) * 0.000061f;
    return true;
  }
  return false;
}

// ── Display init ───────────────────────────────────────────────────

static void forceBacklightOn() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  analogWrite(LCD_BL_PIN, 255);
}

static void initDisplay() {
  forceBacklightOn();

  if (!tft.begin(40000000)) {
    Serial.println("[LCD] Arduino_GFX begin failed");
  }
  tft.setRotation(LCD_ROTATION);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);

  gridOriginX = (tft.width()  - GRID_W * CELL_SIZE) / 2;
  gridOriginY = (tft.height() - GRID_H * CELL_SIZE) / 2;
}

// ── UI helpers ─────────────────────────────────────────────────────

static void showMessage(const char *title, const char *line) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(panelRgb565(255, 220, 80), TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(6, 48);  tft.print(title);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 74); tft.print(line);
}

// ── Cell drawing ───────────────────────────────────────────────────

static void drawCell(int8_t x, int8_t y, uint16_t color) {
  int16_t px = gridOriginX + x * CELL_SIZE + 1;
  int16_t py = gridOriginY + y * CELL_SIZE + 1;
  tft.fillRect(px, py, CELL_SIZE - 2, CELL_SIZE - 2, color);
}

static void clearCell(int8_t x, int8_t y) {
  if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) return;
  drawCell(x, y, TFT_BLACK);
}

// ── Particle physics ───────────────────────────────────────────────

static float randUnit() {
  return random(-1000, 1001) * 0.001f;
}

static void clearOccupancy() {
  memset(occupied, 0, sizeof(occupied));
}

static void resetParticles() {
  const uint16_t deep   = panelRgb565(196, 128, 18);
  const uint16_t bright = panelRgb565(255, 236, 92);
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
  float x = 0.0f, y = 0.0f;
  if (!readAccel(x, y)) return;

  accelX = accelX * 0.78f + x * 0.22f;
  accelY = accelY * 0.78f + y * 0.22f;

  float len = sqrtf(accelX * accelX + accelY * accelY);
  if (len < 0.06f) return;

  gravityX = gravityX * 0.72f + (accelX / len) * 0.28f;
  gravityY = gravityY * 0.72f + (accelY / len) * 0.28f;
}

static void updateDepthRange() {
  float minS = 100000.0f, maxS = -100000.0f;
  const float corners[4][2] = {
    {0.0f, 0.0f}, {(float)(GRID_W - 1), 0.0f},
    {0.0f, (float)(GRID_H - 1)}, {(float)(GRID_W - 1), (float)(GRID_H - 1)}
  };
  for (int i = 0; i < 4; ++i) {
    float s = corners[i][0] * gravityX + corners[i][1] * gravityY;
    if (s < minS) minS = s;
    if (s > maxS) maxS = s;
  }
  depthMinScore = minS; depthSpan = maxS - minS;
  if (depthSpan < 0.001f) depthSpan = 1.0f;
}

static float particleDepth(const Particle &p) {
  float cur = p.cellX * gravityX + p.cellY * gravityY;
  return constrain((cur - depthMinScore) / depthSpan, 0.0f, 1.0f);
}

static float particleMobility(const Particle &p) {
  float d = particleDepth(p);
  return TOP_MOBILITY + (BOTTOM_MOBILITY - TOP_MOBILITY) * d;
}

static float clampV(float v, float lim) {
  return v > lim ? lim : (v < -lim ? -lim : v);
}

static void constrainToScreen(Particle &p) {
  if (p.x < 0.0f)       { p.x = 0.0f;       if (p.vx < 0.0f) p.vx *= -0.25f; }
  else if (p.x > GRID_W - 1) { p.x = GRID_W - 1; if (p.vx > 0.0f) p.vx *= -0.25f; }
  if (p.y < 0.0f)       { p.y = 0.0f;       if (p.vy < 0.0f) p.vy *= -0.25f; }
  else if (p.y > GRID_H - 1) { p.y = GRID_H - 1; if (p.vy > 0.0f) p.vy *= -0.25f; }
}

static bool pickCell(Particle &p, int8_t wX, int8_t wY,
                     int8_t &oX, int8_t &oY, float mob) {
  float best = 100000.0f; bool found = false;
  for (int8_t r = 0; r <= 1; ++r)
    for (int8_t dy = -r; dy <= r; ++dy)
      for (int8_t dx = -r; dx <= r; ++dx) {
        if (abs(dx) + abs(dy) != r) continue;
        int8_t cx = wX + dx, cy = wY + dy;
        if (cx < 0 || cx >= GRID_W || cy < 0 || cy >= GRID_H || occupied[cx][cy]) continue;
        float px = (float)cx - p.x, py = (float)cy - p.y;
        float s = px * px + py * py - (px * gravityX + py * gravityY) * (0.40f + mob * 0.28f);
        if (s < best) { best = s; oX = cx; oY = cy; found = true; }
      }
  return found;
}

static void updateParticles() {
  clearOccupancy();

  // Insertion sort by depth (back-to-front)
  for (uint16_t i = 1; i < SAND_PARTICLES; ++i) {
    uint16_t key = particleOrder[i];
    float keyS = particles[key].cellX * gravityX + particles[key].cellY * gravityY;
    int16_t j = i - 1;
    while (j >= 0) {
      Particle &pj = particles[particleOrder[j]];
      float sj = pj.cellX * gravityX + pj.cellY * gravityY;
      if (sj >= keyS) break;
      particleOrder[j + 1] = particleOrder[j];
      --j;
    }
    particleOrder[j + 1] = key;
  }

  for (uint16_t i = 0; i < SAND_PARTICLES; ++i) {
    Particle &p = particles[particleOrder[i]];
    float mob = particleMobility(p);
    float localDamping = 0.74f + mob * 0.15f;
    float localGravity = GRAVITY * mob;
    float localMaxV    = MAX_VELOCITY * (0.52f + mob * 0.42f);

    p.vx = clampV((p.vx + gravityX * localGravity) * localDamping, localMaxV);
    p.vy = clampV((p.vy + gravityY * localGravity) * localDamping, localMaxV);
    p.x += p.vx; p.y += p.vy;
    constrainToScreen(p);

    int8_t wX = constrain((int)roundf(p.x), 0, GRID_W - 1);
    int8_t wY = constrain((int)roundf(p.y), 0, GRID_H - 1);
    int8_t cX = p.cellX, cY = p.cellY;

    if (!pickCell(p, wX, wY, cX, cY, mob)) {
      cX = p.cellX; cY = p.cellY;
      p.vx *= -0.18f; p.vy *= -0.18f;
    }

    p.x = p.x * 0.35f + cX * 0.65f;
    p.y = p.y * 0.35f + cY * 0.65f;
    p.oldCellX = p.cellX; p.oldCellY = p.cellY;
    p.cellX = cX; p.cellY = cY;
    occupied[cX][cY] = true;
  }
}

static void drawParticles() {
  for (uint16_t i = 0; i < SAND_PARTICLES; ++i) {
    Particle &p = particles[i];
    if (p.oldCellX != p.cellX || p.oldCellY != p.cellY)
      clearCell(p.oldCellX, p.oldCellY);
  }
  for (uint16_t i = 0; i < SAND_PARTICLES; ++i) {
    Particle &p = particles[i];
    if (p.oldCellX != p.cellX || p.oldCellY != p.cellY)
      drawCell(p.cellX, p.cellY, p.color);
  }
}

// ── Setup / Loop ───────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);

  initDisplay();
  showMessage("Quicksand", "Starting IMU...");

  pinMode(IMU_INT_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("=== Electronic Quicksand 0.96 ===");

  if (!initImu()) {
    showMessage("IMU Error", "Check I2C");
    while (1) delay(1000);
  }

  randomSeed((uint32_t)micros());
  tft.fillScreen(TFT_BLACK);
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
