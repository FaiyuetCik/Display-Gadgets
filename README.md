**Language: [English](./README.md) | [简体中文](./README_CN.md)**

# XIAO Display Gadgets

Hardware documentation, drivers, factory firmware, and demo code for the **XIAO Display Series** — six display boards in three sizes (1.47", 1.14", 0.96"), each powered by XIAO nRF52840 Plus or XIAO ESP32-S3 Plus.

## Repository Structure

```
Display-Gadgets/
├── code/                          Legacy demos (TFT_eSPI / Arduino_GFX)
│   ├── example/
│   │   ├── basic/          Basic hardware verification sketches
│   │   ├── 147_nRF52840/   1.47" nRF52840 factory dashboard & schematics
│   │   ├── 147_ESP32/      1.47" ESP32-S3 factory dashboard & schematics
│   │   ├── 114_nRF52840/   1.14" nRF52840 factory dashboard & schematics
│   │   ├── 114_ESP32/      1.14" ESP32-S3 factory dashboard & schematics
│   │   ├── 096_nRF52840/   0.96" nRF52840 factory dashboard & schematics
│   │   ├── 096_ESP32/      0.96" ESP32-S3 factory dashboard & schematics
│   │   ├── images/         Demo screenshots, GIFs and videos
│   │   └── squareline/     SquareLine Studio UI project files
│   └── Function/           Legacy standalone single-feature demos
│       ├── 147_nRF52840/
│       ├── 147_ESP32/
│       ├── 114_nRF52840/
│       ├── 114_ESP32/
│       ├── 096_nRF52840/
│       └── 096_ESP32/
├── code_GFX2/                     Seeed_GFX2-migrated Function demos (current)
│   └── Function/
│       ├── 147_nRF52840/
│       ├── 147_ESP32/
│       ├── 114_nRF52840/
│       ├── 114_ESP32/
│       ├── 096_nRF52840/
│       └── 096_ESP32/
└── schematics/             PDF schematics for all 6 products
```

## Basic Examples

Each subdirectory under `code/example/basic/` is a standalone Arduino sketch for verifying a single hardware peripheral.

| Directory | Description |
|-----------|-------------|
| `code/example/basic/xiao_nrf52840_147_display` | 1.47" screen test with Seeed_GFX / TFT_eSPI — color bars and text |
| `code/example/basic/xiao_nrf52840_147_touch` | AXS5106L touch controller — read X/Y coordinates via I2C |
| `code/example/basic/xiao_nrf52840_147_touch_int` | Touch test with D7 interrupt signal verification |
| `code/example/basic/xiao_nrf52840_147_imu` | LSM6DS3 accelerometer + gyroscope readout via Seeed_Arduino_LSM6DS3 |
| `code/example/basic/xiao_nrf52840_147_imu_int` | LSM6DS3 D14 motion wake-up interrupt test |
| `code/example/basic/xiao_nrf52840_147_mic` | PDM microphone peak readout — serial output |
| `code/example/basic/xiao_nrf52840_147_sd` | MicroSD card mount and directory listing via nRF52 SDK SdFat |
| `code/example/basic/xiao_nrf52840_147_sd_text_reader` | Read TXT/LOG/CSV files from SD and display on screen |
| `code/example/basic/xiao_nrf52840_147_bat` | Battery ADC readout: VBAT divider enable, voltage reading, charge status |
| `code/example/basic/xiao_nrf52840_147_button` | USR1/USR2 button readout — USR1 toggles backlight brightness |
| `code/example/basic/xiao_nrf52840_147_i2s` | I2S + MAX98357A speaker test (D11=DIN, D12=BCLK, D13=LRC/WS) |
| `code/example/basic/xiao_esp32s3_147_bat` | ESP32-S3 battery ADC via D16 — 316k/160k voltage divider |
| `code/example/basic/xiao_esp32s3_114_display` | 1.14" screen test with Seeed_GFX / TFT_eSPI (135×240) |
| `code/example/basic/xiao_esp32s3_114_mic` | ESP32 PDM I2S microphone — D0=MIC_CLK, D1=MIC_DATA |
| `code/example/basic/xiao_esp32s3_114_imu` | QMI8658 / LSM6-compatible IMU on D4=SDA, D5=SCL |
| `code/example/basic/xiao_esp32s3_114_imu_int` | IMU D14 wake-up interrupt verification |
| `code/example/basic/xiao_esp32s3_114_button` | USR1=D6, USR2=D7, USR3=D19 — active-low button readout |
| `code/example/basic/xiao_esp32s3_114_bat` | Battery ADC via D16 — 316k/160k divider, ratio ≈ 2.975 |
| `code/example/basic/xiao_esp32s3_114_i2c_scan` | Grove I2C scanner on D4=SDA, D5=SCL |
| `code/example/basic/xiao_esp32s3_114_i2s` | I2S audio output test (D11=DIN, D12=BCLK, D13=LRC/WS) |

## Function Demos

Standalone single-feature demos under `code_GFX2/Function/`, migrated to **Seeed_GFX2** (the current-generation graphics library). Each is a self-contained Arduino sketch demonstrating one onboard peripheral or application pattern. Legacy TFT_eSPI/Arduino_GFX versions remain under `code/Function/`.

### 1.47" nRF52840 Plus

| Directory | Description |
|-----------|-------------|
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_graphictest` | LCD graphics stress test — drawLine/drawPixel heavy rendering |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_touch_circle` | Touch Circle — draw white circles on touch points |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_mic_canvas` | Big Volume Bar — PDM microphone peak meter with segmented bar |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_sd_image_reader` | SD BMP image reader — read and display BMP files from SD card |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_sd_unline_record` | Offline audio record/playback — PDM mic → RAM → I2S speaker |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_electronic_quicksand` | Electronic Quicksand — IMU-driven particle fluid simulation |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_wakeup` | Raise to Wake — IMU motion wake-up with battery status display |

### 1.47" ESP32-S3 Plus

| Directory | Description |
|-----------|-------------|
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_graphictest` | LCD graphics stress test for 1.47" ESP32-S3 |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_touch_circle` | Touch Circle — draw circles on touch points |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_mic_canvas` | Big Volume Bar — PDM mic peak meter with segmented bar |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_sd_record` | SD Recorder — PDM mic → SD WAV → I2S playback |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3plus_147_sd_bmp_reader_diag_v0_8` | SD BMP reader — display BMP files from SD (diagnostic version) |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_electronic_quicksand` | Electronic Quicksand — IMU-driven particle fluid simulation |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_wakeup` | Raise to Wake — IMU motion wake-up |

### 1.14" nRF52840 Plus

| Directory | Description |
|-----------|-------------|
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_graphictest` | LCD graphics stress test for 1.14" nRF52840 |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_electronic_quicksand` | Electronic Quicksand — 22×40 grid, 150 particles, 6px cells |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_wakeup` | Raise to Wake — IMU motion wake-up |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_voice_bar` | Big Volume Bar — PDM mic real-time waveform + segmented meter |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_flash_record` | Flash audio record/playback — PDM mic → InternalFS → I2S speaker |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_sht31_temperature_humidity` | SHT31 temperature & humidity sensor readout |

### 1.14" ESP32-S3 Plus

| Directory | Description |
|-----------|-------------|
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_graphictest` | LCD graphics stress test for 1.14" ESP32-S3 |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_electronic_quicksand` | Electronic Quicksand — 22×40 grid, 150 particles, 6px cells |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_wakeup` | Raise to Wake — IMU motion wake-up with light sleep |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_flash_record` | Flash audio record/playback — PDM mic → LittleFS → I2S speaker (no SD card needed) |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_voice_bar` | Big Volume Bar — PDM mic real-time waveform + segmented meter |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_sht31_temperature_humidity` | SHT31 temperature & humidity sensor readout |

### 0.96" nRF52840 Plus

| Directory | Description |
|-----------|-------------|
| `code_GFX2/Function/096_nRF52840/xiao_nrf52840_096_graphictest` | LCD graphics stress test for 0.96" nRF52840 |
| `code_GFX2/Function/096_nRF52840/xiao_nrf52840_096_electronic_quicksand` | Electronic Quicksand adapted for 0.96" 80×160 display |
| `code_GFX2/Function/096_nRF52840/xiao_nrf52840_096_wakeup` | Raise to Wake — IMU motion wake-up |
| `code_GFX2/Function/096_nRF52840/xiao_nrf52840_096_flash_record` | Flash audio record/playback — PDM mic → InternalFS → I2S speaker |

### 0.96" ESP32-S3 Plus

| Directory | Description |
|-----------|-------------|
| `code_GFX2/Function/096_ESP32/xiao_esp32s3_096_graphictest` | LCD graphics benchmark — 10 primitives with per-op timing |
| `code_GFX2/Function/096_ESP32/xiao_esp32s3_096_electronic_quicksand` | Electronic Quicksand — IMU-driven particle fluid simulation |
| `code_GFX2/Function/096_ESP32/xiao_esp32s3_096_wakeup` | Raise to Wake — IMU motion wake-up (light sleep) |
| `code_GFX2/Function/096_ESP32/xiao_esp32s3_096_flash_record` | Flash audio record/playback — PDM mic → LittleFS → I2S speaker (needs SPIFFS partition) |

## Tips & Known Issues

1. **Board package**: Use `Seeed nRF52 Boards` (v1.1.13+). This SDK corrects the D17/D19 pin swap on earlier XIAO nRF52840 revisions.

2. **Serial port not detected**: A known low-level bug in the nRF52 SDK may prevent the USB serial port from being recognized. Install [Adafruit_TinyUSB](https://github.com/adafruit/Adafruit_TinyUSB_Arduino) (also available via Arduino Library Manager) and add to every sketch:
   ```cpp
   #include <Adafruit_TinyUSB.h>
   ```

3. **Seeed_GFX2 conflicts with TFT_eSPI**: Remove any existing `TFT_eSPI` library before installing `Seeed_GFX2`. The two conflict.

4. **Missing Seeed_Arduino_FS**: If the compiler reports `Seeed_Arduino_FS` missing, install it from [GitHub](https://github.com/Seeed-Studio/Seeed_Arduino_FS) or via Arduino Library Manager.

5. **IMU driver**: All six boards use the **LSM6DS3** IMU. The factory Dashboard uses **SparkFun LSM6DS3** (`SparkFunLSM6DS3.h`), while the `code_GFX2` Function demos use **Seeed_Arduino_LSM6DS3** (`LSM6DS3.h`). The two headers are not interchangeable — install the one your target firmware needs. Some demo sketches contain a QMI8658 fallback path in code, but the hardware is LSM6DS3 only.

6. **SdFat for nRF52**: The nRF52 SDK bundles its own SdFat library. Do not install a separate SdFat version — it will conflict.

7. **Color inversion on 1.47" display**: If colors appear inverted (black ↔ white, red ↔ cyan, green ↔ purple, yellow ↔ blue), toggle `tft.invertDisplay(false)` or `tft.invertDisplay(true)` after `tft.init()` and rotation setup. This is an LCD inversion issue, not an RGB/BGR byte-order problem.

8. **SquareLine Studio workflow**: To use SquareLine-generated UIs on these boards:
   - a. Design the UI in SquareLine Studio and export the project template
   - b. Extract the package and copy the UI library folder into your Arduino libraries
   - c. Copy the screen `driver.h` into the exported Arduino sketch folder
   - d. Install `Seeed_GFX2` library
   - e. Install Seeed LVGL library and copy its `lv_conf.h` to your Arduino library path

9. **Shared SPI bus (LCD + SD)**: The LCD and SD card share the same SPI bus. After each LCD refresh, re-initialize the SD card before accessing it again:
   ```cpp
   sdCard.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, freq, &SPI));
   ```
   Otherwise SD reads may fail or hang after LCD operations.

10. **GraphicTest with SD card inserted**: Some GraphicTest sketches may hang on cold boot with an SD card inserted (especially during `Lines` rendering). Remove the SD card before running GraphicTest, or use the SD image reader demo if you need SD + LCD simultaneously.

11. **ESP32 flash_record needs a SPIFFS partition**: The 1.14" and 0.96" ESP32-S3 flash recorder demos store the WAV in `LittleFS`, which needs a SPIFFS partition. In Arduino IDE select **Tools > Partition Scheme > "Default with spiffs (3MB APP/1.5MB SPIFFS)"** before uploading — otherwise `LittleFS.begin()` fails and the screen shows "Write failed / Check flash".

## Required Libraries

| Library | Purpose | Source |
|---------|---------|--------|
| Seeed nRF52 Boards | nRF52840 board package (v1.1.13+) | Arduino Boards Manager |
| esp32 Boards by Espressif | ESP32-S3 board package (3.3.11+) | Arduino Boards Manager |
| Seeed_GFX2 | Display graphics for `code_GFX2` Function demos (replaces TFT_eSPI) | [GitHub](https://github.com/Seeed-Studio/Seeed_GFX2) / Library Manager |
| GFX Library for Arduino | Display graphics (Arduino_GFX, used by 0.96" and several demos) | Arduino Library Manager |
| Seeed_Arduino_LSM6DS3 | LSM6DS3 IMU driver (Function demos) | [GitHub](https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3) / Library Manager |
| SparkFun LSM6DS3 | LSM6DS3 IMU driver (factory Dashboard) | Arduino Library Manager |
| Seeed_Arduino_FS | Filesystem abstraction | [GitHub](https://github.com/Seeed-Studio/Seeed_Arduino_FS) / Library Manager |
| Adafruit_TinyUSB | USB serial patch for nRF52 | [GitHub](https://github.com/adafruit/Adafruit_TinyUSB_Arduino) / Library Manager |
| SdFat | SD card (bundled with nRF52 SDK) | Included in Seeed nRF52 Boards |
| PDM | PDM microphone (bundled with nRF52 SDK) | Included in Seeed nRF52 Boards |

## Wiki

User-facing documentation is published on the [Seeed Studio Wiki](https://wiki.seeedstudio.com/). Look under **Sensor → LCD Displays → Display Gadgets**.

## Related Repositories

- [Seeed_GFX2](https://github.com/Seeed-Studio/Seeed_GFX2) — Display graphics library
- [Seeed_Arduino_LSM6DS3](https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3) — IMU driver
- [Seeed_Arduino_FS](https://github.com/Seeed-Studio/Seeed_Arduino_FS) — Filesystem library
