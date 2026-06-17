# XIAO-Display-Board
Contains hardware documentation, software drivers, factory firmware, and sample demos for the latest XIAO display development board

现各屏幕资料的情况：
- https://seeedstudio.feishu.cn/wiki/Omfewh9yAiMhqVkDmgxcPzm6nug

驱动：
- https://github.com/Seeed-Studio/Seeed_GFX
- https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3

Wiki参考：
- https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
- https://wiki.seeedstudio.com/xiao_esp32s3_sense_filesystem/
- https://wiki.seeedstudio.com/seeedstudio_round_display_usage/

坑/tip：

1. 用`Seeed nRF52 Boards`这个SDK，已纠正D17和D19反接情况

2. 但是这个库存在Serial识别不了的情况，属于底层bug，需要再用一个库打补丁
https://github.com/adafruit/Adafruit_TinyUSB_Arduino 或则直接在arduino搜索下载

```
#include <Adafruit_TinyUSB.h>
```

3. 使用SEEED_GFX要将原来的TFT_eSPI库删掉

4. 报错缺少Seeed_Arduino_FS库，需要前往https://github.com/Seeed-Studio/Seeed_Arduino_FS 或者直接在arduino搜索下载

5. 使用Seeed_Arduino_LSM6DS3库驱动IMU

6. 使用sdfat库驱动sd,但是注意！不用自己下载！nrf52 SDK里自带了一个

7. 各基础功能代码结果放在了image文件夹里

8. 如果 1.47 寸屏幕出现黑白互换、红色变青色、绿色变紫色、黄色变蓝色等现象，通常不是 RGB/BGR 或字节序问题，而是 LCD inversion 状态反了。可在 `tft.init()` 和旋转/面板修正后测试 `tft.invertDisplay(false)` 或 `tft.invertDisplay(true)`。

## Basic examples

基础示例都放在 `example/basic` 目录下，每个子目录都是一个可以单独打开和烧录的 Arduino sketch，用来快速验证某一个硬件功能。

| 目录 | 功能说明 |
| --- | --- |
| `example/basic/display` | 使用 Seeed_GFX / `TFT_eSPI` 点亮 1.47 寸屏幕，显示颜色块和文字，用于验证屏幕、背光和显示方向。 |
| `example/basic/touch` | 使用 AXS5106L 触摸控制器读取触摸坐标，通过串口打印 `x/y` 数据。 |
| `example/basic/touch_int` | 在触摸读取基础上加入 D7 触摸中断，用于验证触摸 INT 信号是否正常触发。 |
| `example/basic/imu` | 使用 `Seeed_Arduino_LSM6DS3` 读取加速度计和陀螺仪数据，并通过串口输出。 |
| `example/basic/imu_int` | 配置 LSM6DS3 的 D14 中断，用于验证运动唤醒/中断信号。 |
| `example/basic/mic` | 使用 PDM 麦克风读取音频峰值，串口输出 `peak`，用于确认麦克风采样是否工作。 |
| `example/basic/sd` | 使用 nRF52 SDK 自带的 SdFat 挂载 SD 卡，并打印根目录文件列表。 |
| `example/basic/sd_bmp` | 从 SD 卡根目录读取 `/test.bmp` 或第一张 BMP 图片，并显示到 1.47 寸屏幕上。 |
| `example/basic/bat` | 读取 VBAT ADC、电池分压使能和充电状态脚，用于检查电池电压和充电状态。 |
| `example/basic/bat_esp32s3` | ESP32S3 电池读取示例，使用 D16 读取外接 `316k/160k` 分压后的电池电压。 |
| `example/basic/button` | 读取 USR1 / USR2 按键状态，USR1 会切换背光亮度，USR2 打印按键状态。 |
| `example/basic/i2s` | 使用预留 I2S 音频输出引脚 `SD=D11`、`SCK=D12`、`WS=D13` 启动 nRF52840 I2S TX 骨架。 （未测试）|

## Application

- `example/147_nRF52840/0521_WakeUp_147_nRF52840`
  - 驱动库适配：使用 Seeed_GFX / `TFT_eSPI` 驱动 1.47 寸屏幕，使用 `Seeed_Arduino_LSM6DS3` 配置 IMU D14 运动唤醒。
  - 已加入 BAT 电量显示：过 `READ_BAT=P0.14` 使能分压，`PIN_VBAT` 读取电池电压，`CHG=P0.17` 判断充电状态。
  - 屏幕 `POWER STATE` 区域显示电压和电量百分比，充电时显示黄色小闪电图标。
  - 唤醒背光已调暗，`BACKLIGHT_AWAKE_PWM = 120`；睡眠时背光关闭，保持 `BACKLIGHT_SLEEP_PWM = 0`。

- `example/147_nRF52840/xiao_147_nrf52840plus_sd_bmp_reader_stress_v0_1`
  - SD BMP 读取/刷屏压力测试示例，已按 `example/basic/sd/sd.ino` 和 `example/basic/display/display.ino` 的库用法同步。
  - 显示驱动从 `Arduino_GFX_Library` 切换为 `driver.h` + Seeed_GFX / `TFT_eSPI`，使用 `TFT_eSPI tft`、`tft.init()`、`tft.pushImage()` 刷新 BMP 行数据。
  - SD 继续使用 nRF52 SDK 自带 `SdFat`，通过 `SdSpiConfig(SD_CS_PIN, SHARED_SPI, freq, &SPI)` 初始化共享 SPI 总线。
  - 为避免 `TFT_eSPI` 或其它库中的全局 `SD` 对象冲突，草图内 `SdFat` 实例命名为 `sdCard`，不要再使用 `SdFat SD;`。
  - `tft.setSwapBytes(true)` 已放在 `tft.init()` 之后，用于 `pushImage()` 输出 RGB565 行缓冲时的字节序处理。
  - 如果出现整屏颜色取反，参考上方 tip 8 调整 `tft.invertDisplay(...)`。
