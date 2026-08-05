#include "axs5106l_device.h"
TwoWire *g_touch_i2c;         

static bool touch_i2c_write(uint8_t driver_addr, uint8_t reg_addr, const uint8_t *data, int length)
{
    g_touch_i2c->beginTransmission(driver_addr);
    g_touch_i2c->write(reg_addr);
    g_touch_i2c->write(data, length);


    if (g_touch_i2c->endTransmission() != 0) {
        Serial.println("The I2C transmission fails. - I2C Read\r\n");
        return false;
    }
    return true;
}

static bool touch_i2c_read(uint8_t driver_addr, uint8_t reg_addr, uint8_t *data, uint32_t length)
{
    g_touch_i2c->beginTransmission(driver_addr);
    g_touch_i2c->write(reg_addr);
    if (g_touch_i2c->endTransmission() != 0) {
        Serial.println("The I2C write fails. - I2C Read\r\n");
        return false;
    }

    g_touch_i2c->requestFrom(driver_addr, length);
    if (g_touch_i2c->available() != length) {
        Serial.println("The I2C read fails. - I2C Read\r\n");
        return false;
    }
    g_touch_i2c->readBytes(data, length);
    return true;
}

bool touch_init(TwoWire *touch_i2c, int tp_rst, int tp_int)
{
    g_touch_i2c = touch_i2c;
    pinMode(tp_rst, OUTPUT);

    digitalWrite(tp_rst, LOW);
    delay(200);
    digitalWrite(tp_rst, HIGH);
    delay(300);

    uint8_t data[5] = {0};
    touch_i2c_read(AXS5106L_ADDR, AXS5106L_ID_REG, data, 3);
    if (data[0] != 0){
        Serial.print("read: ");
        Serial.print(data[0]);Serial.print(data[1]);Serial.print(data[2]);
        Serial.println();
    }
    return true;
}


bool get_touch_data(touch_data_t *touch_data)
{
    uint8_t data[14] = {0};
    touch_i2c_read(AXS5106L_ADDR, AXS5106L_TOUCH_DATA_REG, data, 14);

    touch_data->touch_num = data[1];
    if (touch_data->touch_num == 0)
        return false;
    for (uint8_t i = 0; i < touch_data->touch_num; i++){
        touch_data->coords[i].x = ((uint16_t)(data[2+i*6] & 0x0f)) << 8;
        touch_data->coords[i].x |= data[3+i*6];
        touch_data->coords[i].y = (((uint16_t)(data[4+i*6] & 0x0f)) << 8);
        touch_data->coords[i].y |= data[5+i*6];
        // touch_data->coords[i].x  = 319 - touch_data->coords[i].x;
    }
    return true;
}

