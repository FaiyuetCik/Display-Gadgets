/*
  XIAO ESP32-S3 Plus + 1.14 Inch Display IMU basic.

  SDA=D4, SCL=D5, IMU_INT=D14.
  Supports QMI8658-compatible and LSM6DS3/LSM6DSO-compatible probing.
*/

#include <Arduino.h>
#include <Wire.h>

static constexpr uint8_t I2C_SDA_PIN = D4;
static constexpr uint8_t I2C_SCL_PIN = D5;
static constexpr uint8_t IMU_INT_PIN = D14;

enum ImuType {
  IMU_NONE = 0,
  IMU_QMI8658,
  IMU_LSM6
};

static ImuType imuType = IMU_NONE;
static uint8_t imuAddr = 0;
static float ax = 0, ay = 0, az = 0;
static float gx = 0, gy = 0, gz = 0;

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

static bool initQmi(uint8_t addr) {
  uint8_t who = 0;
  if (!read8(addr, 0x00, &who)) return false;
  if (who == 0x00 || who == 0xFF) return false;

  write8(addr, 0x02, 0x60);
  write8(addr, 0x03, 0x03);
  write8(addr, 0x04, 0x53);
  write8(addr, 0x08, 0x03);
  delay(20);

  uint8_t data[12] = {};
  if (!readBytes(addr, 0x35, data, sizeof(data))) return false;

  imuType = IMU_QMI8658;
  imuAddr = addr;
  Serial.printf("[IMU] QMI8658-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

static bool initLsm(uint8_t addr) {
  uint8_t who = 0;
  if (!read8(addr, 0x0F, &who)) return false;
  if (who != 0x69 && who != 0x6A && who != 0x6C) return false;

  write8(addr, 0x10, 0x60);
  write8(addr, 0x11, 0x60);
  delay(20);

  imuType = IMU_LSM6;
  imuAddr = addr;
  Serial.printf("[IMU] LSM6-compatible at 0x%02X, WHO=0x%02X\n", addr, who);
  return true;
}

static bool initImu() {
  imuType = IMU_NONE;
  imuAddr = 0;

  if (initQmi(0x6B)) return true;
  if (initQmi(0x6A)) return true;
  if (initLsm(0x6A)) return true;
  if (initLsm(0x6B)) return true;

  Serial.println("[IMU] not found");
  return false;
}

static bool readImu() {
  if (imuType == IMU_QMI8658) {
    uint8_t d[12] = {};
    if (!readBytes(imuAddr, 0x35, d, sizeof(d))) return false;

    ax = le16(&d[0]) / 16384.0f;
    ay = le16(&d[2]) / 16384.0f;
    az = le16(&d[4]) / 16384.0f;
    gx = le16(&d[6]) / 64.0f;
    gy = le16(&d[8]) / 64.0f;
    gz = le16(&d[10]) / 64.0f;
    return true;
  }

  if (imuType == IMU_LSM6) {
    uint8_t g[6] = {};
    uint8_t a[6] = {};
    if (!readBytes(imuAddr, 0x22, g, sizeof(g))) return false;
    if (!readBytes(imuAddr, 0x28, a, sizeof(a))) return false;

    gx = le16(&g[0]) * 0.00875f;
    gy = le16(&g[2]) * 0.00875f;
    gz = le16(&g[4]) * 0.00875f;
    ax = le16(&a[0]) * 0.000061f;
    ay = le16(&a[2]) * 0.000061f;
    az = le16(&a[4]) * 0.000061f;
    return true;
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(IMU_INT_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("=== 1.14 IMU basic ===");
  Serial.println("SDA=D4, SCL=D5, IMU_INT=D14");
  initImu();
}

void loop() {
  if (readImu()) {
    Serial.printf("[IMU] acc=%.3f,%.3f,%.3f g  gyro=%.2f,%.2f,%.2f dps\n",
                  ax, ay, az, gx, gy, gz);
  } else {
    Serial.println("[IMU] read failed");
  }

  delay(300);
}
