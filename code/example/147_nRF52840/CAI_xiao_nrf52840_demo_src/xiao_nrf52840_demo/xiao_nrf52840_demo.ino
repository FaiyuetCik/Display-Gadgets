/*
  1.47 Inch Touch Display - XIAO nRF52840 Plus
*/

#include "SparkFunLSM6DS3.h"
#include <Arduino_GFX_Library.h>
#include "axs5106l_device.h"
#include <Wire.h>

#define CTP_RST_PIN    D17
#define CTP_INT_PIN    D7
#define LCD_BL         D18

LSM6DS3 myIMU(I2C_MODE, 0x6A);

/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = new Arduino_HWSPI(D3 /* DC */, D2 /* CS */);

/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
Arduino_GFX *gfx = new Arduino_ST7796(
  bus, DF_GFX_RST /* RST */, 0 /* rotation */, false /* IPS */,
  172 /* width */, 320 /* height */,
  34 /*col_offset1*/, 0 /*uint8_t row_offset1*/,
  34 /*col_offset2*/, 0 /*row_offset2*/);

void setup(void) {

  // Set LCD Backlight to Max
  pinMode(LCD_BL, OUTPUT);
  analogWrite(LCD_BL, 255);

  Serial.begin(115200);
  Serial.println("1.47 Inch Touch Display example");

  Wire.begin();
  touch_init(&Wire, CTP_RST_PIN, CTP_INT_PIN);

  myIMU.begin();

  // Init Display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  bus->beginWrite();
  bus->writeC8D8(0x36, 0x48);
  bus->endWrite();
  gfx->fillScreen(RGB565_BLACK);

  gfx->setCursor(0, 30);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_LIGHTGREEN);
  gfx->println("Hello XIAO");
  
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  gfx->setCursor(0, 60);
  gfx->print("Touch: [---,---]");
}

touch_data_t touch_data;
char coords_text[48];
char accel_text[48];
char gyro_text[48];

void loop() {

  if (get_touch_data(&touch_data)) {
    touch_data.coords[0].x = gfx->width() - 1 - touch_data.coords[0].x;
    Serial.print("x:");
    Serial.print(touch_data.coords[0].x);
    Serial.print("y:");
    Serial.println(touch_data.coords[0].y);
  
    gfx->setCursor(0, 60);
    sprintf(coords_text, "Touch: [%-3d, %-3d]", touch_data.coords[0].x, touch_data.coords[0].y);
    gfx->print(coords_text);
  }

  gfx->setCursor(0, 80);
  sprintf(accel_text, "Acc: %-5.1f, %-5.1f, %-5.1f",myIMU.readFloatAccelX(), myIMU.readFloatAccelY(), myIMU.readFloatAccelZ());
  gfx->print(accel_text);

  gfx->setCursor(0, 100);
  sprintf(gyro_text, "Gyr: %-6.1f, %-6.1f, %-6.1f",myIMU.readFloatGyroX(), myIMU.readFloatGyroY(), myIMU.readFloatGyroZ());
  gfx->print(gyro_text);

  delay(50);
}