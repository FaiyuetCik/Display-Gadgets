/*
  冲突测试：LCD（SPI）+ 触摸（I2C）是否能共存？
  不跑 LVGL，只测硬件。
*/

#include "driver.h"
#include "axs5106l_device.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <Wire.h>
#include <TFT_eSPI.h>

TFT_eSPI tft;

static constexpr uint8_t LCD_CS_PIN  = D2;
static constexpr uint8_t LCD_DC_PIN  = D3;
static constexpr uint8_t LCD_SCK_PIN = D8;
static constexpr uint8_t LCD_MOS_PIN = D10;
static constexpr uint8_t LCD_RST_PIN = D17;
static constexpr uint8_t LCD_BL_PIN  = D18;

// ---- LCD helpers (copied from lvgl_demo) ----
static void applyPanelFix() {
  tft.writecommand(0x36);
  tft.writedata(0x48);
  delay(10);
}
static void setRotation(uint8_t rot) {
  tft.setRotation(rot);
  if (rot == 0) applyPanelFix();
}
static void preparePins() {
  pinMode(LCD_CS_PIN, OUTPUT); digitalWrite(LCD_CS_PIN, HIGH);
  pinMode(LCD_DC_PIN, OUTPUT); digitalWrite(LCD_DC_PIN, HIGH);
  pinMode(LCD_SCK_PIN, OUTPUT); digitalWrite(LCD_SCK_PIN, LOW);
  pinMode(LCD_MOS_PIN, OUTPUT); digitalWrite(LCD_MOS_PIN, LOW);
}
static void backlightOn() {
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
static void initLCD() {
  preparePins();
  backlightOn();
  hardResetPanel();
  tft.init();
  setRotation(0);
  tft.invertDisplay(false);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Touch vs LCD conflict test ===");

  // --- 1. init touch first ---
  Serial.println("[1] Init touch...");
  Wire.begin();
  touch_init(&Wire, LCD_RST_PIN, D7);

  // --- 2. init LCD (hardResetPanel resets touch too!) ---
  Serial.println("[2] Init LCD...");
  initLCD();
  Serial.print("    LCD size=");
  Serial.print(tft.width());
  Serial.print("x");
  Serial.println(tft.height());

  // --- 3. re-establish I2C after LCD reset the touch controller ---
  Serial.println("[3] Re-establish I2C to touch...");
  Wire.end();
  delay(50);
  Wire.begin();
  delay(300);  // wait for touch controller to boot after reset

  // --- 4. draw something on LCD to prove it's working ---
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Touch test", 10, 10, 2);
  tft.drawString("Watch Serial", 10, 140, 2);

  Serial.println("=== Ready. Touch the screen! ===");
}

void loop() {
  touch_data_t td;
  if (get_touch_data(&td) && td.touch_num > 0) {
    Serial.print("T x=");
    Serial.print(td.coords[0].x);
    Serial.print(" y=");
    Serial.print(td.coords[0].y);
    Serial.print(" num=");
    Serial.println(td.touch_num);

    // Flash backlight to confirm
    digitalWrite(LCD_BL_PIN, LOW);  delay(20);
    digitalWrite(LCD_BL_PIN, HIGH);
  }
  delay(30);
}
