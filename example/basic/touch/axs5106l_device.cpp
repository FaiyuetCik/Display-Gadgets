#include "axs5106l_device.h"

static TwoWire *g_touch_i2c = nullptr;

static bool touch_i2c_read(uint8_t driver_addr, uint8_t reg_addr, uint8_t *data, uint32_t length) {
  g_touch_i2c->beginTransmission(driver_addr);
  g_touch_i2c->write(reg_addr);
  if (g_touch_i2c->endTransmission() != 0) {
    Serial.println("Touch I2C write failed");
    return false;
  }

  g_touch_i2c->requestFrom(driver_addr, length);
  if (g_touch_i2c->available() != (int)length) {
    Serial.println("Touch I2C read failed");
    return false;
  }

  g_touch_i2c->readBytes(data, length);
  return true;
}

bool touch_init(TwoWire *touch_i2c, int tp_rst, int tp_int) {
  (void)tp_int;
  g_touch_i2c = touch_i2c;

  pinMode(tp_rst, OUTPUT);
  digitalWrite(tp_rst, LOW);
  delay(200);
  digitalWrite(tp_rst, HIGH);
  delay(300);

  uint8_t data[3] = {0};
  touch_i2c_read(AXS5106L_ADDR, AXS5106L_ID_REG, data, 3);
  Serial.print("Touch ID: ");
  Serial.print(data[0], HEX);
  Serial.print(" ");
  Serial.print(data[1], HEX);
  Serial.print(" ");
  Serial.println(data[2], HEX);
  return true;
}

bool get_touch_data(touch_data_t *touch_data) {
  uint8_t data[14] = {0};
  if (!touch_i2c_read(AXS5106L_ADDR, AXS5106L_TOUCH_DATA_REG, data, 14)) {
    return false;
  }

  touch_data->touch_num = data[1];
  if (touch_data->touch_num == 0) return false;
  if (touch_data->touch_num > MAX_TOUCH_MAX_POINTS) {
    touch_data->touch_num = MAX_TOUCH_MAX_POINTS;
  }

  for (uint8_t i = 0; i < touch_data->touch_num; ++i) {
    touch_data->coords[i].x = ((uint16_t)(data[2 + i * 6] & 0x0f) << 8) | data[3 + i * 6];
    touch_data->coords[i].y = ((uint16_t)(data[4 + i * 6] & 0x0f) << 8) | data[5 + i * 6];
  }

  return true;
}
