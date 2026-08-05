/*
  XIAO ESP32-S3 Plus 1.47 Inch Display — Snake Game

  Tilt the board (IMU) or press buttons to steer the snake.
  Eat red food to grow and increase your score.

  Controls:
    IMU tilt  — steer snake (whichever axis is tilted more)
    USR1 D15  — rotate direction counter-clockwise
    USR2 D19  — rotate direction clockwise

  Required libraries:
    - Seeed_GFX / TFT_eSPI
*/

#include <Arduino.h>
#include "driver.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include "axs5106l_device.h"
#include <math.h>

// ========================= Pins =========================

static constexpr uint8_t LCD_CS_PIN   = D2;
static constexpr uint8_t LCD_DC_PIN   = D3;
static constexpr uint8_t I2C_SDA_PIN  = D4;
static constexpr uint8_t I2C_SCL_PIN  = D5;
static constexpr uint8_t LCD_SCK_PIN  = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN  = D17;
static constexpr uint8_t LCD_BL_PIN   = D18;

static constexpr uint8_t USR1_PIN     = D19;   // Left  btn (hw USR1)
static constexpr uint8_t USR2_PIN     = D15;   // Right btn (hw USR2)
static constexpr uint8_t TOUCH_RST    = D17;   // shared with LCD reset
static constexpr uint8_t TOUCH_INT    = D7;

// ========================= Grid layout =========================

static constexpr int SCREEN_W   = 172;
static constexpr int SCREEN_H   = 320;
static constexpr int TOP_BAR_H  = 28;
static constexpr int CELL_SIZE  = 10;
static constexpr int GRID_COLS  = 17;
static constexpr int GRID_ROWS  = 18;   // shorter — bottom area for d-pad
static constexpr int DPAD_Y     = TOP_BAR_H + GRID_ROWS * CELL_SIZE;
static constexpr int DPAD_H     = SCREEN_H - DPAD_Y;
static constexpr int PLAY_X     = (SCREEN_W - GRID_COLS * CELL_SIZE) / 2;   // 1
static constexpr int PLAY_Y     = TOP_BAR_H;
static constexpr int MAX_LENGTH = GRID_COLS * GRID_ROWS;

// ========================= Colours =========================

static constexpr uint16_t C_BLACK       = TFT_BLACK;
static constexpr uint16_t C_WHITE       = TFT_WHITE;
static constexpr uint16_t C_GREEN       = TFT_GREEN;
static constexpr uint16_t C_RED         = TFT_RED;
static constexpr uint16_t C_YELLOW      = TFT_YELLOW;
static constexpr uint16_t C_CYAN        = TFT_CYAN;
static constexpr uint16_t C_DARK_GREEN  = 0x04C0;
static constexpr uint16_t C_MID_GREEN   = 0x0660;
static constexpr uint16_t C_HEAD_GREEN  = 0x07E0;
static constexpr uint16_t C_FOOD_GLOW   = 0xF9E0;
static constexpr uint16_t C_BG          = 0x10A2;
static constexpr uint16_t C_GRID        = 0x2104;
static constexpr uint16_t C_TOP_BG      = 0x0861;
static constexpr uint16_t C_GRAY        = TFT_WHITE;  // was 0x8410, now white for readability
static constexpr uint16_t C_ORANGE      = TFT_ORANGE;

// ========================= Game timing =========================

static constexpr uint32_t TICK_START_MS  = 500;
static constexpr uint32_t TICK_MIN_MS    = 150;
static constexpr uint32_t TICK_STEP_MS   = 8;
static constexpr uint32_t IMU_INTERVAL   = 50;
static constexpr uint32_t FOOD_PULSE_MS  = 300;
static constexpr uint32_t DEBOUNCE_MS    = 50;
static constexpr uint32_t GAME_OVER_COOLDOWN = 1000;
static constexpr float    TILT_THRESHOLD = 0.35f;
static constexpr float    IMU_ALPHA     = 0.20f;

// ========================= Types =========================

enum Direction { DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT };
enum GameState { STATE_TITLE, STATE_PLAYING, STATE_PAUSED, STATE_GAME_OVER };

struct Point {
  int8_t x, y;
};

struct BtnRect { int x1, y1, x2, y2; };

struct ButtonState {
  bool usr1 : 1;
  bool usr2 : 1;
};

// ========================= Display =========================

TFT_eSPI tft;

// ========================= Snake state =========================

Point    body[MAX_LENGTH];
uint16_t snakeHead   = 0;
uint16_t snakeTail   = 0;
uint16_t snakeLen    = 3;
Direction dir        = DIR_RIGHT;
Direction nextDir    = DIR_RIGHT;
bool     occupied[GRID_COLS][GRID_ROWS];

Point    food;
uint16_t currentFoodColor = C_RED;

GameState state        = STATE_TITLE;
uint32_t  score        = 0;
uint32_t  tickInterval = TICK_START_MS;
uint32_t  lastTick     = 0;
uint32_t  lastImu      = 0;
uint32_t  lastPulse    = 0;
uint32_t  deathTime    = 0;
bool      foodEaten    = false;

// ========================= IMU state =========================

uint8_t imuAddr   = 0;
bool    imuOk     = false;
float   smoothX   = 0.0f;
float   smoothY   = 0.0f;
bool    imuIsQmi  = false;

// ========================= Touch state =========================

touch_data_t touchData;
bool     wasTouching = false;
uint32_t lastTouchMs = 0;
static constexpr uint32_t TOUCH_COOLDOWN = 150;

// ========================= I2C helpers =========================

static int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool write8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

static bool read8(uint8_t addr, uint8_t reg, uint8_t *val) {
  return readBytes(addr, reg, val, 1);
}

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
  analogWrite(LCD_BL_PIN, 255);
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

// ========================= Drawing helpers =========================

static void fillCell(int col, int row, uint16_t color) {
  int x = PLAY_X + col * CELL_SIZE;
  int y = PLAY_Y + row * CELL_SIZE;
  tft.fillRect(x, y, CELL_SIZE, CELL_SIZE, color);
}

static void drawCellBorder(int col, int row) {
  int x = PLAY_X + col * CELL_SIZE;
  int y = PLAY_Y + row * CELL_SIZE;
  tft.drawRect(x, y, CELL_SIZE, CELL_SIZE, C_GRID);
}

static void drawGrid() {
  for (int r = 0; r < GRID_ROWS; ++r) {
    for (int c = 0; c < GRID_COLS; ++c) {
      drawCellBorder(c, r);
    }
  }
}

static void drawTopBar() {
  tft.fillRect(0, 0, SCREEN_W, TOP_BAR_H, C_TOP_BG);
  tft.drawFastHLine(0, TOP_BAR_H - 1, SCREEN_W, 0x4208);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_GREEN, C_TOP_BG);
  tft.drawString("SNAKE", 6, 4, 2);

  tft.setTextColor(C_WHITE, C_TOP_BG);
  tft.setCursor(100, 8);
  tft.setTextSize(1);
  tft.print("Score:");
  tft.print(score);
}

static void updateScore() {
  tft.fillRect(140, 6, 32, 16, C_TOP_BG);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_WHITE, C_TOP_BG);
  tft.setTextSize(1);
  tft.setCursor(140, 8);
  tft.print(score);
}

static void drawFood() {
  fillCell(food.x, food.y, currentFoodColor);
}

// ========================= D-pad (touch joystick) =========================

static constexpr uint16_t DPAD_BG      = 0x1082;   // dark blue-grey background
static constexpr uint16_t DPAD_BTN     = 0x52AA;   // light grey — clearly visible
static constexpr uint16_t DPAD_BORDER  = 0x8410;   // medium grey border
static constexpr uint16_t DPAD_ACTIVE  = 0x07E0;   // bright green when pressed
static constexpr uint16_t DPAD_CENTER  = 0x8410;   // centre dot

static constexpr int DPAD_CX   = SCREEN_W / 2;           // 86
static constexpr int DPAD_CY   = DPAD_Y + DPAD_H / 2;    // centre of d-pad area
static constexpr int DPAD_BW   = 15;   // half width  (vert buttons: UP/DOWN)
static constexpr int DPAD_BH   = 12;   // half height (vert buttons: UP/DOWN)
static constexpr int DPAD_GAP  = 16;   // gap from centre (big enough to avoid overlap)

// Button bounding boxes (no overlap)
static const BtnRect DPAD_RECTS[4] = {
  // UP:    above centre
  { DPAD_CX - DPAD_BW, DPAD_CY - DPAD_GAP - DPAD_BH * 2,
    DPAD_CX + DPAD_BW, DPAD_CY - DPAD_GAP },
  // DOWN:  below centre
  { DPAD_CX - DPAD_BW, DPAD_CY + DPAD_GAP,
    DPAD_CX + DPAD_BW, DPAD_CY + DPAD_GAP + DPAD_BH * 2 },
  // LEFT:  left of centre (swapped bw/bh)
  { DPAD_CX - DPAD_GAP - DPAD_BH * 2, DPAD_CY - DPAD_BW,
    DPAD_CX - DPAD_GAP,                DPAD_CY + DPAD_BW },
  // RIGHT: right of centre (swapped bw/bh)
  { DPAD_CX + DPAD_GAP,                DPAD_CY - DPAD_BW,
    DPAD_CX + DPAD_GAP + DPAD_BH * 2, DPAD_CY + DPAD_BW },
};

static void drawDpadBtn(int idx, bool active) {
  const BtnRect &r = DPAD_RECTS[idx];
  uint16_t border = active ? DPAD_ACTIVE : DPAD_BORDER;
  uint16_t fill   = active ? (uint16_t)0x0440 : DPAD_BTN;

  tft.fillRoundRect(r.x1, r.y1, r.x2 - r.x1, r.y2 - r.y1, 3, fill);
  tft.drawRoundRect(r.x1, r.y1, r.x2 - r.x1, r.y2 - r.y1, 3, border);

  // Triangle arrow inside button
  int cx = (r.x1 + r.x2) / 2;
  int cy = (r.y1 + r.y2) / 2;
  int s = 4;  // arrow size
  switch (idx) {
    case 0: tft.fillTriangle(cx, cy - s, cx - s, cy + s, cx + s, cy + s, border); break;  // ▲
    case 1: tft.fillTriangle(cx, cy + s, cx - s, cy - s, cx + s, cy - s, border); break;  // ▼
    case 2: tft.fillTriangle(cx - s, cy, cx + s, cy - s, cx + s, cy + s, border); break;  // ◀
    case 3: tft.fillTriangle(cx + s, cy, cx - s, cy - s, cx - s, cy + s, border); break;  // ▶
  }
}

static void drawDpadBackground() {
  // Unified dark area — no harsh divider line
  tft.fillRect(0, DPAD_Y, SCREEN_W, DPAD_H, DPAD_BG);

  // Draw the 4 buttons
  for (int i = 0; i < 4; ++i) drawDpadBtn(i, false);

  // Centre dot
  tft.fillCircle(DPAD_CX, DPAD_CY, 3, DPAD_CENTER);
}

static int  dpadPrev = -1;
static void updateDpad(int active) {
  if (active == dpadPrev) return;
  for (int i = 0; i < 4; ++i) drawDpadBtn(i, i == active);
  dpadPrev = active;
}

static int dirToDpadIdx(Direction d) {
  switch (d) {
    case DIR_UP:    return 0;
    case DIR_DOWN:  return 1;
    case DIR_LEFT:  return 2;
    case DIR_RIGHT: return 3;
  }
  return -1;
}

static void drawDpadLabel() {
  tft.fillRect(0, DPAD_Y + DPAD_H - 12, SCREEN_W, 12, DPAD_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_GRAY, DPAD_BG);
  tft.drawString("Touch or Tilt to steer", DPAD_CX, DPAD_Y + DPAD_H - 6, 1);
}

// ========================= Game logic =========================

static void markOccupied(int x, int y, bool val) {
  occupied[x][y] = val;
}

static bool isOpposite(Direction a, Direction b) {
  return (a == DIR_UP    && b == DIR_DOWN)  ||
         (a == DIR_DOWN  && b == DIR_UP)    ||
         (a == DIR_LEFT  && b == DIR_RIGHT) ||
         (a == DIR_RIGHT && b == DIR_LEFT);
}

static void spawnFood() {
  int freeCount = GRID_COLS * GRID_ROWS - (int)snakeLen;
  if (freeCount <= 0) return;

  int target = random(freeCount);
  int idx = 0;
  for (int8_t x = 0; x < GRID_COLS; ++x) {
    for (int8_t y = 0; y < GRID_ROWS; ++y) {
      if (!occupied[x][y]) {
        if (idx == target) {
          food.x = x;
          food.y = y;
          return;
        }
        ++idx;
      }
    }
  }
}

static void initSnake() {
  snakeHead = 2;
  snakeTail = 0;
  snakeLen  = 3;
  dir       = DIR_RIGHT;
  nextDir   = DIR_RIGHT;

  int startX = GRID_COLS / 2 - 1;
  int startY = GRID_ROWS / 2;
  for (int i = 0; i < snakeLen; ++i) {
    body[i].x = startX - i;
    body[i].y = startY;
  }
}

static void clearOccupied() {
  memset(occupied, 0, sizeof(occupied));
}

static void buildOccupied() {
  clearOccupied();
  for (uint16_t i = snakeTail; i != snakeHead; i = (i + 1) % MAX_LENGTH) {
    occupied[body[i].x][body[i].y] = true;
  }
  occupied[body[snakeHead].x][body[snakeHead].y] = true;
}

static void startNewGame() {
  // Clear play area
  tft.fillRect(PLAY_X, PLAY_Y, GRID_COLS * CELL_SIZE, GRID_ROWS * CELL_SIZE, C_BG);
  drawGrid();

  initSnake();
  clearOccupied();

  // Draw initial snake
  for (uint16_t i = snakeTail; i != snakeHead; i = (i + 1) % MAX_LENGTH) {
    uint16_t dist = (snakeHead - i + MAX_LENGTH) % MAX_LENGTH;
    uint16_t c = (dist % 2 == 0) ? C_MID_GREEN : C_DARK_GREEN;
    fillCell(body[i].x, body[i].y, c);
    occupied[body[i].x][body[i].y] = true;
  }
  fillCell(body[snakeHead].x, body[snakeHead].y, C_HEAD_GREEN);
  occupied[body[snakeHead].x][body[snakeHead].y] = true;

  spawnFood();
  currentFoodColor = C_RED;
  drawFood();

  score = 0;
  tickInterval = TICK_START_MS;
  foodEaten = false;
  updateScore();
  drawDpadBackground();
  drawDpadLabel();
  dpadPrev = -1;
  updateDpad(-1);
}

static bool snakeStep() {
  // Consume queued direction
  if (!isOpposite(nextDir, dir)) {
    dir = nextDir;
  }

  // Compute new head
  Point &h = body[snakeHead];
  Point newH = h;
  switch (dir) {
    case DIR_UP:    newH.y--; break;
    case DIR_DOWN:  newH.y++; break;
    case DIR_LEFT:  newH.x--; break;
    case DIR_RIGHT: newH.x++; break;
  }

  // Wall wrap (no death)
  if (newH.x < 0) newH.x = GRID_COLS - 1;
  else if (newH.x >= GRID_COLS) newH.x = 0;
  if (newH.y < 0) newH.y = GRID_ROWS - 1;
  else if (newH.y >= GRID_ROWS) newH.y = 0;

  // Self collision (exclude old tail since it will be removed unless growing)
  Point &t = body[snakeTail];
  if (occupied[newH.x][newH.y] && !(newH.x == t.x && newH.y == t.y)) {
    return false;
  }

  // Save old head (becomes body) and old tail (to be erased)
  Point oldHead = h;
  Point oldTail = t;

  // Advance head index
  uint16_t newHeadIdx = (snakeHead + 1) % MAX_LENGTH;
  body[newHeadIdx] = newH;

  // Check food
  foodEaten = (newH.x == food.x && newH.y == food.y);

  if (foodEaten) {
    // Grow: don't advance tail
    ++snakeLen;
    ++score;
    tickInterval = (tickInterval > TICK_MIN_MS + TICK_STEP_MS)
                       ? tickInterval - TICK_STEP_MS
                       : TICK_MIN_MS;
  } else {
    // Remove old tail
    occupied[t.x][t.y] = false;
    fillCell(t.x, t.y, C_BG);
    drawCellBorder(t.x, t.y);
    // Advance tail
    snakeTail = (snakeTail + 1) % MAX_LENGTH;
  }

  // Mark new head
  occupied[newH.x][newH.y] = true;
  snakeHead = newHeadIdx;

  // Draw new head
  fillCell(newH.x, newH.y, C_HEAD_GREEN);

  // Redraw old head as body
  uint16_t bodyColor = (snakeLen % 2 == 0) ? C_MID_GREEN : C_DARK_GREEN;
  fillCell(oldHead.x, oldHead.y, bodyColor);

  // Repaint food (ensure it stays on top)
  drawFood();

  if (foodEaten) {
    spawnFood();
    currentFoodColor = C_RED;
    drawFood();
  }

  return true;
}

// ========================= IMU =========================

static bool initQmi(uint8_t addr) {
  uint8_t who = 0;
  if (!read8(addr, 0x00, &who)) return false;
  if (who == 0x00 || who == 0xFF) return false;

  write8(addr, 0x02, 0x60);  // accel 1000Hz
  write8(addr, 0x03, 0x00);  // gyro off
  write8(addr, 0x08, 0x03);  // enable accel + gyro
  delay(10);

  imuAddr  = addr;
  imuIsQmi = true;
  return true;
}

static bool initLsm(uint8_t addr) {
  uint8_t who = 0;
  if (!read8(addr, 0x0F, &who)) return false;
  if (who != 0x69 && who != 0x6A && who != 0x6C) return false;

  write8(addr, 0x10, 0x60);  // accel 104Hz, 2g
  write8(addr, 0x11, 0x00);  // gyro off
  delay(10);

  imuAddr  = addr;
  imuIsQmi = false;
  return true;
}

static bool initImu() {
  bool ok = initQmi(0x6B) || initQmi(0x6A) ||
            initLsm(0x6A) || initLsm(0x6B);
  imuOk = ok;
  return ok;
}

static bool readAccel(float &x, float &y) {
  uint8_t d[6] = {};
  uint8_t reg = imuIsQmi ? 0x35 : 0x28;
  if (!readBytes(imuAddr, reg, d, sizeof(d))) return false;

  x = le16(&d[0]) / 16384.0f;
  y = le16(&d[2]) / 16384.0f;
  return true;
}

static void updateTilt() {
  if (!imuOk) return;

  float ax = 0, ay = 0;
  if (!readAccel(ax, ay)) return;

  smoothX = IMU_ALPHA * ax + (1.0f - IMU_ALPHA) * smoothX;
  smoothY = IMU_ALPHA * ay + (1.0f - IMU_ALPHA) * smoothY;
}

// Absolute direction from tilt — bigger axis wins, like rolling a marble.
// Returns true if tilt is strong enough to set a new direction.
static bool getTiltDir(Direction &out) {
  float ax = fabsf(smoothX);
  float ay = fabsf(smoothY);
  if (ax < TILT_THRESHOLD && ay < TILT_THRESHOLD) return false;

  if (ax > ay) {
    out = (smoothX > 0) ? DIR_RIGHT : DIR_LEFT;
  } else {
    out = (smoothY > 0) ? DIR_DOWN : DIR_UP;
  }
  return true;
}

// ========================= Touch D-pad =========================

static void initTouch() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  touch_init(&Wire, TOUCH_RST, TOUCH_INT);
  pinMode(TOUCH_INT, INPUT_PULLUP);
}

// Read touch and return which D-pad arrow was pressed.
// Returns true + direction on first touch-down; false on hold/release.
static bool readDpadTouch(Direction &out) {
  bool gotData = get_touch_data(&touchData);
  bool touching = gotData && touchData.touch_num > 0;

  if (!touching) {
    wasTouching = false;
    return false;
  }
  if (wasTouching) return false;
  if (millis() - lastTouchMs < TOUCH_COOLDOWN) return false;

  wasTouching = true;
  lastTouchMs = millis();

  uint16_t tx = touchData.coords[0].x;
  uint16_t ty = touchData.coords[0].y;

  int mx = SCREEN_W - 1 - tx;   // mirrored X → display coords
  int my = ty;                  // Y is direct

  // Angle-based direction from D-pad centre (much more forgiving)
  int dx = mx - DPAD_CX;
  int dy = my - DPAD_CY;
  int adx = abs(dx);
  int ady = abs(dy);

  // Dead zone near centre
  if (adx < 14 && ady < 14) return false;
  // Only respond within reasonable range of d-pad
  if (adx > 60 || ady > 60) return false;

  // Dominant axis determines direction
  if (adx > ady) {
    out = (dx > 0) ? DIR_RIGHT : DIR_LEFT;
  } else {
    out = (dy > 0) ? DIR_DOWN : DIR_UP;
  }
  return true;
}

// ========================= Buttons =========================

static void initButtons() {
  pinMode(USR1_PIN, INPUT_PULLUP);
  pinMode(USR2_PIN, INPUT_PULLUP);
}

static ButtonState readButtons() {
  static bool   last1 = true, last2 = true;
  static uint32_t t1 = 0, t2 = 0;

  bool raw1 = digitalRead(USR1_PIN);
  bool raw2 = digitalRead(USR2_PIN);
  uint32_t now = millis();

  ButtonState s = {false, false};

  if (raw1 == LOW && last1 == HIGH && now - t1 > DEBOUNCE_MS) {
    s.usr1 = true;
    t1 = now;
  }
  if (raw2 == LOW && last2 == HIGH && now - t2 > DEBOUNCE_MS) {
    s.usr2 = true;
    t2 = now;
  }

  last1 = raw1;
  last2 = raw2;
  return s;
}

// ========================= Direction helpers =========================

// ========================= Screens =========================

static void drawTitleScreen() {
  tft.fillScreen(C_BLACK);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_GREEN, C_BLACK);
  tft.drawString("SNAKE", SCREEN_W / 2, 80, 4);

  // Draw a decorative snake
  int sx = SCREEN_W / 2 - 25;
  int sy = 140;
  for (int i = 0; i < 5; ++i) {
    uint16_t c = (i == 0) ? C_HEAD_GREEN : (i % 2 ? C_MID_GREEN : C_DARK_GREEN);
    tft.fillRect(sx + i * CELL_SIZE, sy, CELL_SIZE, CELL_SIZE, c);
  }

  tft.setTextColor(C_WHITE, C_BLACK);
  tft.drawString("Press USR2 to Start", SCREEN_W / 2, 200, 2);

  tft.setTextColor(C_GRAY, C_BLACK);
  tft.drawString("Touch D-pad or tilt to steer", SCREEN_W / 2, 240, 1);

  tft.setTextColor(C_CYAN, C_BLACK);
  tft.drawString("USR1=Pause   USR2=Start", SCREEN_W / 2, 270, 1);
}

static void drawGameOverScreen() {
  tft.fillScreen(C_BLACK);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_RED, C_BLACK);
  tft.drawString("GAME OVER", SCREEN_W / 2, 110, 4);

  tft.setTextColor(C_WHITE, C_BLACK);
  String s = "Score: " + String(score);
  tft.drawString(s, SCREEN_W / 2, 170, 2);

  tft.setTextColor(C_GRAY, C_BLACK);
  tft.drawString("USR2 to restart", SCREEN_W / 2, 230, 1);
}

// ========================= Setup / loop =========================

void setup() {
  Serial.begin(115200);
  delay(500);

  randomSeed(esp_random());

  initTouch();      // Wire.begin + D17 reset for touch
  initLcd();        // D17 reset + configure LCD registers
  initButtons();
  initImu();        // I2C IMU (Wire already running)

  drawTitleScreen();
  state     = STATE_TITLE;
  lastImu   = millis();
}

void loop() {
  uint32_t now = millis();

  // --- Always poll IMU at fixed interval ---
  if (now - lastImu >= IMU_INTERVAL) {
    lastImu = now;
    updateTilt();
  }

  // --- Always poll buttons ---
  ButtonState btns = readButtons();

  // --- Food pulse animation (only during gameplay) ---
  if (state == STATE_PLAYING && now - lastPulse >= FOOD_PULSE_MS) {
    lastPulse = now;
    currentFoodColor = (currentFoodColor == C_RED) ? C_FOOD_GLOW : C_RED;
    drawFood();
  }

  // --- State machine ---
  switch (state) {

    case STATE_TITLE: {
      if (btns.usr2) {  // USR2 = Start
        startNewGame();
        drawTopBar();
        state    = STATE_PLAYING;
        lastTick = now;
        lastPulse = now;
      }
      break;
    }

    case STATE_PLAYING: {
      // USR1 toggles pause
      if (btns.usr1) {
        state = STATE_PAUSED;
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(C_YELLOW, C_BG);
        tft.drawString("PAUSED", SCREEN_W / 2, SCREEN_H / 2 + 20, 2);
        break;
      }

      // Touch D-pad: absolute direction (edge-triggered)
      Direction td;
      if (readDpadTouch(td)) {
        if (!isOpposite(td, dir)) {
          nextDir = td;
        }
        updateDpad(dirToDpadIdx(td));
      }

      // IMU tilt: absolute direction (bigger axis wins)
      if (imuOk) {
        Direction imuDir;
        if (getTiltDir(imuDir)) {
          if (!isOpposite(imuDir, dir)) {
            nextDir = imuDir;
          }
          updateDpad(dirToDpadIdx(imuDir));
        }
      }

      // Fallback: no IMU → USR2 rotates clockwise
      if (!imuOk && btns.usr2) {
        switch (dir) {
          case DIR_UP:    nextDir = DIR_RIGHT; break;
          case DIR_RIGHT: nextDir = DIR_DOWN;  break;
          case DIR_DOWN:  nextDir = DIR_LEFT;  break;
          case DIR_LEFT:  nextDir = DIR_UP;    break;
        }
      }

      // Game tick
      if (now - lastTick >= tickInterval) {
        bool alive = snakeStep();
        if (!alive) {
          state     = STATE_GAME_OVER;
          deathTime = now;
          drawGameOverScreen();
          break;
        }
        if (foodEaten) {
          updateScore();
        }
        lastTick = now;
      }
      break;
    }

    case STATE_PAUSED: {
      if (btns.usr1) {  // USR1 = unpause
        // Resume — restore display
        tft.fillRect(0, PLAY_Y, SCREEN_W, DPAD_Y - PLAY_Y, C_BG);
        drawGrid();
        for (uint16_t i = snakeTail; i != snakeHead; i = (i + 1) % MAX_LENGTH) {
          uint16_t dist = (snakeHead - i + MAX_LENGTH) % MAX_LENGTH;
          uint16_t c = (dist % 2 == 0) ? C_MID_GREEN : C_DARK_GREEN;
          fillCell(body[i].x, body[i].y, c);
        }
        fillCell(body[snakeHead].x, body[snakeHead].y, C_HEAD_GREEN);
        drawFood();
        drawDpadBackground();
        drawDpadLabel();
        dpadPrev = -1;
        updateDpad(-1);
        drawTopBar();
        state    = STATE_PLAYING;
        lastTick = now;
      }
      break;
    }

    case STATE_GAME_OVER: {
      if (now - deathTime > GAME_OVER_COOLDOWN) {
        if (btns.usr2) {  // USR2 = restart
          tft.fillRect(PLAY_X, PLAY_Y, GRID_COLS * CELL_SIZE, GRID_ROWS * CELL_SIZE, C_BG);
          drawGrid();
          startNewGame();
          drawTopBar();
          state    = STATE_PLAYING;
          lastTick = now;
          lastPulse = now;
        }
      }
      break;
    }
  }

  delay(5);
}
