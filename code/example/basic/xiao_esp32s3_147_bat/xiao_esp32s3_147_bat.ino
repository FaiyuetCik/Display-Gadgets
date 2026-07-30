/*
  XIAO ESP32S3 battery ADC + display basic.

  This version is for the ESP32S3 board path. It only reads battery voltage
  from D16 through an external resistor divider:
    VBAT -- 316k -- ADC(D16) -- 160k -- GND

  Divider ratio:
    (316k + 160k) / 160k = 2.975
*/

#include <Arduino.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>

// #ifndef D16
// // Some ESP32 board packages expose GPIO numbers but not Dx aliases.
// #define D16 16
// #endif

static constexpr uint8_t BAT_ADC_PIN = D16;
static constexpr float BAT_DIVIDER_RATIO = (316.0f + 160.0f) / 160.0f;
static constexpr uint32_t SAMPLE_INTERVAL_MS = 1000;

// Display pins follow example/147_ESP32/0519_DashBoard_147_ESP32.
static constexpr uint8_t LCD_CS_PIN = D2;
static constexpr uint8_t LCD_DC_PIN = D3;
static constexpr uint8_t LCD_SCK_PIN = D8;
static constexpr uint8_t LCD_MOSI_PIN = D10;
static constexpr uint8_t LCD_RST_PIN = D17;
static constexpr uint8_t LCD_BL_PIN = D18;

Arduino_DataBus *lcdBus = new Arduino_SWSPI(
  LCD_DC_PIN,
  LCD_CS_PIN,
  LCD_SCK_PIN,
  LCD_MOSI_PIN,
  GFX_NOT_DEFINED
);

Arduino_GFX *gfx = new Arduino_ST7789(
  lcdBus,
  LCD_RST_PIN,
  0,
  false,
  172,
  320,
  34,
  0,
  34,
  0
);

static uint32_t lastSampleMs = 0;
static uint32_t lastBatteryMv = 0;

static void applyXIAO147PanelFix() {
  // The 172x320 JD9853A panel needs this MADCTL value for correct orientation.
  lcdBus->beginWrite();
  lcdBus->writeCommand(0x36);
  lcdBus->write(0x48);
  lcdBus->endWrite();
}

static void prepareDisplayPins() {
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_SCK_PIN, OUTPUT);
  pinMode(LCD_MOSI_PIN, OUTPUT);
  pinMode(LCD_BL_PIN, OUTPUT);

  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_SCK_PIN, LOW);
  digitalWrite(LCD_MOSI_PIN, LOW);
  digitalWrite(LCD_BL_PIN, HIGH);
}

static void initDisplay() {
  prepareDisplayPins();

  if (!gfx->begin()) {
    Serial.println("[LCD] gfx->begin() failed");
    return;
  }

  applyXIAO147PanelFix();

  gfx->fillScreen(RGB565_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_CYAN, RGB565_BLACK);
  gfx->setCursor(18, 24);
  gfx->print("ESP32S3 BAT");

  gfx->drawFastHLine(16, 62, 140, 0x8410);

  gfx->setTextSize(1);
  gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  gfx->setCursor(58, 250);
  gfx->print("D16 ADC");

  gfx->setTextColor(RGB565_YELLOW, RGB565_BLACK);
  gfx->setCursor(28, 276);
  gfx->print("316k / 160k divider");
}

static uint32_t readBatteryMilliVolts() {
  uint32_t sumMv = 0;
  static constexpr uint8_t SAMPLE_COUNT = 16;

  // Average several readings so the serial output is easier to compare.
  for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
#if defined(ARDUINO_ARCH_ESP32)
    sumMv += analogReadMilliVolts(BAT_ADC_PIN);
#else
    // Fallback for non-ESP32 compilation tests. ESP32S3 should use the branch above.
    sumMv += (uint32_t)analogRead(BAT_ADC_PIN) * 3300UL / 4095UL;
#endif
    delay(2);
  }

  uint32_t adcMv = sumMv / SAMPLE_COUNT;
  return (uint32_t)((float)adcMv * BAT_DIVIDER_RATIO);
}

static uint16_t colorForVoltage(uint32_t batteryMv) {
  if (batteryMv < 3400) return RGB565_RED;
  if (batteryMv < 3700) return RGB565_YELLOW;
  return RGB565_GREEN;
}

static void drawBatteryVoltage(uint32_t batteryMv) {
  char voltageText[16];
  snprintf(voltageText, sizeof(voltageText), "%.3fV", (float)batteryMv / 1000.0f);

  // Clear only the dynamic area so the static labels do not flicker.
  gfx->fillRect(0, 90, 172, 118, RGB565_BLACK);

  gfx->setTextSize(3);
  gfx->setTextColor(colorForVoltage(batteryMv), RGB565_BLACK);
  gfx->setCursor(26, 124);
  gfx->print(voltageText);

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  gfx->setCursor(38, 184);
  gfx->print(String(batteryMv) + " mV");
}

void setup() {
  Serial.begin(115200);
  delay(800);

#if defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(12);
  // 11 dB attenuation gives the ESP32 ADC its widest input range.
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
#endif

  Serial.println("=== ESP32S3 Battery basic ===");
  Serial.println("ADC pin: D16");
  Serial.println("Divider: 316k / 160k");

  initDisplay();
  drawBatteryVoltage(0);
}

void loop() {
  uint32_t now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  uint32_t batteryMv = readBatteryMilliVolts();
  lastBatteryMv = batteryMv;

  Serial.print("battery=");
  Serial.print(batteryMv);
  Serial.print("mV ");
  Serial.print((float)batteryMv / 1000.0f, 3);
  Serial.println("V");

  drawBatteryVoltage(lastBatteryMv);
}
