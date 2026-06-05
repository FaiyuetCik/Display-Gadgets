/*
  XIAO nRF52840 Plus LSM6DS3 IMU basic.

  Required library:
    - SparkFun LSM6DS3
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include "LSM6DS3.h"

LSM6DS3 imu(I2C_MODE, 0x6A);

void setup() {
  Serial.begin(115200);
  delay(800);

  Wire.begin();

  Serial.println("=== IMU basic ===");
  int ret = imu.begin();
  Serial.print("imu.begin=");
  Serial.println(ret);
}

void loop() {
  Serial.print("acc=");
  Serial.print(imu.readFloatAccelX(), 2);
  Serial.print(",");
  Serial.print(imu.readFloatAccelY(), 2);
  Serial.print(",");
  Serial.print(imu.readFloatAccelZ(), 2);

  Serial.print(" gyr=");
  Serial.print(imu.readFloatGyroX(), 2);
  Serial.print(",");
  Serial.print(imu.readFloatGyroY(), 2);
  Serial.print(",");
  Serial.println(imu.readFloatGyroZ(), 2);

  delay(300);
}
