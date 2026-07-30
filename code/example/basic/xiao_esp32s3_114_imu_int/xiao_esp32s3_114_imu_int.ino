/*
  XIAO ESP32-S3 Plus + 1.14 Inch Display IMU INT basic.

  SDA=D4, SCL=D5, IMU_INT=D14.
  Configures LSM6-compatible wake/motion interrupt on INT1 and prints triggers.
*/

#include <Arduino.h>
#include <Wire.h>

static constexpr uint8_t I2C_SDA_PIN = D4;
static constexpr uint8_t I2C_SCL_PIN = D5;
static constexpr uint8_t IMU_INT_PIN = D14;

static constexpr uint8_t REG_WAKE_UP_SRC = 0x1B;
static constexpr uint8_t REG_CTRL1_XL = 0x10;
static constexpr uint8_t REG_CTRL3_C = 0x12;
static constexpr uint8_t REG_TAP_CFG = 0x58;
static constexpr uint8_t REG_WAKE_UP_THS = 0x5B;
static constexpr uint8_t REG_WAKE_UP_DUR = 0x5C;
static constexpr uint8_t REG_MD1_CFG = 0x5E;

static uint8_t imuAddr = 0;
volatile bool imuIntFlag = false;
volatile uint32_t imuIntCount = 0;

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

  size_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;

  for (size_t i = 0; i < len; ++i) {
    buf[i] = Wire.read();
  }
  return true;
}

static bool read8(uint8_t addr, uint8_t reg, uint8_t *val) {
  return readBytes(addr, reg, val, 1);
}

static bool findLsm(uint8_t addr) {
  uint8_t who = 0;
  if (!read8(addr, 0x0F, &who)) return false;
  if (who != 0x69 && who != 0x6A && who != 0x6C) return false;

  imuAddr = addr;
  Serial.printf("[IMU] LSM6-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

static bool initMotionInterrupt() {
  if (!findLsm(0x6A) && !findLsm(0x6B)) {
    Serial.println("[IMU] LSM6-compatible IMU not found");
    return false;
  }

  bool ok = true;
  ok &= write8(imuAddr, REG_CTRL3_C, 0x44);     // BDU + auto-increment
  ok &= write8(imuAddr, REG_CTRL1_XL, 0x40);    // 104 Hz accel, +/-2g
  ok &= write8(imuAddr, REG_TAP_CFG, 0x80);     // embedded function interrupts
  ok &= write8(imuAddr, REG_WAKE_UP_THS, 0x05); // sensitive wake threshold
  ok &= write8(imuAddr, REG_WAKE_UP_DUR, 0x00); // no extra duration
  ok &= write8(imuAddr, REG_MD1_CFG, 0x20);     // route wake-up to INT1

  uint8_t dummy = 0;
  (void)read8(imuAddr, REG_WAKE_UP_SRC, &dummy); // clear stale latched event
  return ok;
}

void IRAM_ATTR onImuInt() {
  imuIntFlag = true;
  imuIntCount++;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);

  Serial.println();
  Serial.println("=== 1.14 IMU INT basic ===");
  Serial.println("SDA=D4, SCL=D5, IMU_INT=D14");

  if (!initMotionInterrupt()) {
    Serial.println("[IMU] motion INT init failed");
    return;
  }

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), onImuInt, RISING);
  Serial.println("[IMU] motion INT ready, move or tap the board");
}

void loop() {
  if (imuIntFlag || digitalRead(IMU_INT_PIN) == HIGH) {
    noInterrupts();
    imuIntFlag = false;
    uint32_t count = imuIntCount;
    interrupts();

    uint8_t src = 0;
    (void)read8(imuAddr, REG_WAKE_UP_SRC, &src);

    Serial.print("[IMU_INT] count=");
    Serial.print(count);
    Serial.print(" wake_src=0x");
    if (src < 16) Serial.print('0');
    Serial.println(src, HEX);
  }

  delay(10);
}
