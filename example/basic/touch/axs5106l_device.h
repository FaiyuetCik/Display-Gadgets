#pragma once

#include <Arduino.h>
#include <Wire.h>

#define MAX_TOUCH_MAX_POINTS 2
#define AXS5106L_ADDR 0x63
#define AXS5106L_ID_REG 0x08
#define AXS5106L_TOUCH_DATA_REG 0x01

// One touch coordinate reported by the controller.
typedef struct {
  uint16_t x;
  uint16_t y;
} coords_t;

// The controller can report up to two points in this simple driver.
typedef struct {
  coords_t coords[MAX_TOUCH_MAX_POINTS];
  uint8_t touch_num;
} touch_data_t;

// Read the latest touch report. Returns false when no finger is detected.
bool get_touch_data(touch_data_t *touch_data);

// Reset and bind the driver to the I2C bus used by the sketch.
bool touch_init(TwoWire *touch_i2c, int tp_rst, int tp_int);
