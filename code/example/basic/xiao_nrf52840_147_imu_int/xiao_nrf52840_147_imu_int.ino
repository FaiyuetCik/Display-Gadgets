/*
  XIAO nRF52840 Plus LSM6DS3 INT1 wake/motion basic.

  Extracted from 0521_WakeUp_147_nRF52840.
  IMU INT1 is connected to D14.
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include "LSM6DS3.h"

static constexpr uint8_t IMU_INT_PIN = D14;
static constexpr uint8_t LSM6DS3_ADDR = 0x6A;

// LSM6DS3 register addresses used to enable wake-up interrupt routing.
static constexpr uint8_t REG_WAKE_UP_SRC = 0x1B;
static constexpr uint8_t REG_CTRL1_XL = 0x10;
static constexpr uint8_t REG_CTRL3_C = 0x12;
static constexpr uint8_t REG_TAP_CFG = 0x58;
static constexpr uint8_t REG_WAKE_UP_THS = 0x5B;
static constexpr uint8_t REG_WAKE_UP_DUR = 0x5C;
static constexpr uint8_t REG_MD1_CFG = 0x5E;

LSM6DS3 imu(I2C_MODE, 0x6A);

// These variables are written inside the interrupt, so they must be volatile.
volatile bool imuIntFlag = false;
volatile uint32_t imuIntCount = 0;

static bool imuWriteReg(uint8_t reg, uint8_t val) {
  // Small raw-register helper for configuration not wrapped by the library.
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool imuReadReg(uint8_t reg, uint8_t &val) {
  // Repeated-start read: write register address, then read one byte.
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)LSM6DS3_ADDR, 1) != 1) return false;
  val = Wire.read();
  return true;
}

void imuIsr() {
  // Keep the ISR tiny: just latch that an event happened.
  imuIntFlag = true;
  imuIntCount++;
}

static bool initMotionInterrupt() {
  bool ok = true;

  // BDU + auto-increment.
  ok &= imuWriteReg(REG_CTRL3_C, 0x44);

  // Accelerometer 104 Hz, +/-2g. Wake detection uses accel data.
  ok &= imuWriteReg(REG_CTRL1_XL, 0x40);

  // Enable embedded function interrupts and configure wake-up detection.
  ok &= imuWriteReg(REG_TAP_CFG, 0x80);
  ok &= imuWriteReg(REG_WAKE_UP_THS, 0x05);
  ok &= imuWriteReg(REG_WAKE_UP_DUR, 0x00);
  ok &= imuWriteReg(REG_MD1_CFG, 0x20);

  uint8_t dummy = 0;
  // Reading WAKE_UP_SRC clears any stale latched wake event.
  imuReadReg(REG_WAKE_UP_SRC, dummy);

  // D14 receives INT1 from the IMU. Wake events are rising edges.
  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIsr, RISING);
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Wire.begin();
  Serial.println("=== IMU interrupt basic ===");
  Serial.print("imu.begin=");
  Serial.println(imu.begin());
  Serial.print("motion interrupt=");
  Serial.println(initMotionInterrupt() ? "OK" : "FAILED");
}

void loop() {
  if (imuIntFlag || digitalRead(IMU_INT_PIN) == HIGH) {
    // Copy volatile values with interrupts disabled so the print is consistent.
    noInterrupts();
    imuIntFlag = false;
    uint32_t count = imuIntCount;
    interrupts();

    uint8_t src = 0;
    // WAKE_UP_SRC tells which wake condition the IMU latched.
    imuReadReg(REG_WAKE_UP_SRC, src);

    Serial.print("IMU INT count=");
    Serial.print(count);
    Serial.print(" src=0x");
    Serial.println(src, HEX);
  }

  delay(10);
}
