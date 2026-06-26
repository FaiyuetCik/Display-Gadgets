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

9. 无论哪个版本的Squareline，我们的板子若要使用，必须经过一下步骤：

  - a. 在squareline vision里画好UI，输出project template文件包
  - b. 解压文件包，其中包括library文件夹和UI Arduino代码文件夹，并复制library中的UI库到Arduino库里
  - c. 复制屏幕driver.h库到UI Arduino 代码文件夹里
  - d. 下载seeed 的GFX库
  - e. 下载seeed lvgl库，并将库里的lv_conf.h文件复制到Arduino下载库路径中

## Basic examples

基础示例都放在 `example/basic` 目录下，每个子目录都是一个可以单独打开和烧录的 Arduino sketch，用来快速验证某一个硬件功能。

| 目录 | 功能说明 |
| --- | --- |
| `example/basic/display` | 使用 Seeed_GFX / `TFT_eSPI` 点亮 1.47 寸屏幕，显示颜色块和文字，用于验证屏幕、背光和显示方向。 |
| `example/basic/graphictest` | LCD 图形压力测试，会大量调用 `drawLine()` / `drawPixel()` 等小图元写屏；测试时请先拔出 SD 卡，避免插卡冷启动后在 `Lines` 等项目卡住。如需验证 SD + LCD 同时工作，请使用 `example/basic/sd_image_reader`。 |
| `example/basic/touch` | 使用 AXS5106L 触摸控制器读取触摸坐标，通过串口打印 `x/y` 数据。 |
| `example/basic/touch_int` | 在触摸读取基础上加入 D7 触摸中断，用于验证触摸 INT 信号是否正常触发。 |
| `example/basic/imu` | 使用 `Seeed_Arduino_LSM6DS3` 读取加速度计和陀螺仪数据，并通过串口输出。 |
| `example/basic/imu_int` | 配置 LSM6DS3 的 D14 中断，用于验证运动唤醒/中断信号。 |
| `example/basic/mic` | 使用 PDM 麦克风读取音频峰值，串口输出 `peak`，用于确认麦克风采样是否工作。 |
| `example/basic/sd` | 使用 nRF52 SDK 自带的 SdFat 挂载 SD 卡，并打印根目录文件列表。 |
| `example/basic/sd_image_reader` | 扫描 SD 卡根目录中的图片并通过屏幕轮播；直接显示 16/24/32-bit 未压缩 BMP|
| `example/basic/sd_unline_record` | nRF52840 离线原音录放示例：屏幕显示待机、录音、保存和播放状态；按 USR1 关闭无线活动并启动外部 HFCLK，使用 PDM EasyDMA 双缓冲将 5 秒、16 kHz、16-bit 单声道 PCM 采集到 RAM，再保存为 RAW WAV；按 USR2 通过 MAX98357A/I2S 播放刚录的原音。示例不再做 HP、notch、DSP 或 CSV 统计处理，也不使用 RGB LED 状态提示。5 秒 PCM 占约 160 KB，适配 nRF52840 的 256 KB RAM。 |
| `example/basic/bat` | 读取 VBAT ADC、电池分压使能和充电状态脚，用于检查电池电压和充电状态。 |
| `example/basic/bat_esp32s3` | ESP32S3 电池读取示例，使用 D16 读取外接 `316k/160k` 分压后的电池电压。 |
| `example/basic/button` | 读取 USR1 / USR2 按键状态，USR1 会切换背光亮度，USR2 打印按键状态。 |
| `example/basic/i2s` | nRF52840 I2S + MAX98357A 喇叭测试示例：`D11=DIN`、`D12=BCLK`、`D13=LRC/WS`，按 USR1 播放约 1 秒低音量测试音。MAX98357A 不接 MCLK，但 nRF I2S master 需启用内部 MCK 发生器。 |

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
