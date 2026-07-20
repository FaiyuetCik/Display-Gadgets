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

10. LCD 和 SD 卡共用 SPI 总线。每次 LCD 刷新后，如果后续还要访问 SD 卡，需要重新初始化/挂载 SD，例如再次调用 `sdCard.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, freq, &SPI))`。否则 SD 读写可能会在 LCD 刷新后失效或卡住。

## Basic examples

基础示例都放在 `example/basic` 目录下，每个子目录都是一个可以单独打开和烧录的 Arduino sketch，用来快速验证某一个硬件功能。

| 目录 | 功能说明 |
| --- | --- |
| `example/basic/xiao_nrf52840_147_display` | 使用 Seeed_GFX / `TFT_eSPI` 点亮 1.47 寸屏幕，显示颜色块和文字，用于验证屏幕、背光和显示方向。 |
| `example/basic/xiao_nrf52840_147_touch` | 使用 AXS5106L 触摸控制器读取触摸坐标，通过串口打印 `x/y` 数据。 |
| `example/basic/xiao_nrf52840_147_touch_int` | 在触摸读取基础上加入 D7 触摸中断，用于验证触摸 INT 信号是否正常触发。 |
| `example/basic/xiao_nrf52840_147_imu` | 使用 `Seeed_Arduino_LSM6DS3` 读取加速度计和陀螺仪数据，并通过串口输出。 |
| `example/basic/xiao_nrf52840_147_imu_int` | 配置 LSM6DS3 的 D14 中断，用于验证运动唤醒/中断信号。 |
| `example/basic/xiao_nrf52840_147_mic` | 使用 PDM 麦克风读取音频峰值，串口输出 `peak`，用于确认麦克风采样是否工作。 |
| `example/basic/xiao_nrf52840_147_sd` | 使用 nRF52 SDK 自带的 SdFat 挂载 SD 卡，并打印根目录文件列表。 |
| `example/basic/xiao_nrf52840_147_sd_text_reader` | 读取 SD 卡根目录中的 TXT / LOG / CSV 文本文件，并在 1.47 寸屏幕上分页显示。 |
| `example/basic/xiao_nrf52840_147_bat` | 读取 VBAT ADC、电池分压使能和充电状态脚，用于检查电池电压和充电状态。 |
| `example/basic/xiao_esp32s3_147_bat` | ESP32S3 电池读取示例，使用 D16 读取外接 `316k/160k` 分压后的电池电压。 |
| `example/basic/xiao_nrf52840_147_button` | 读取 USR1 / USR2 按键状态，USR1 会切换背光亮度，USR2 打印按键状态。 |
| `example/basic/xiao_nrf52840_147_i2s` | nRF52840 I2S + MAX98357A 喇叭测试示例：`D11=DIN`、`D12=BCLK`、`D13=LRC/WS`，按 USR1 播放约 1 秒低音量测试音。MAX98357A 不接 MCLK，但 nRF I2S master 需启用内部 MCK 发生器。 |
| `example/basic/xiao_esp32s3_114_display` | 使用 `driver.h` + Seeed_GFX / `TFT_eSPI` 点亮 1.14 寸屏幕，配置 `D2=CS`、`D3=DC`、`D8=SCK`、`D10=MOSI`、`D17=RST`、`D18=BL`，显示颜色块和文字。 |
| `example/basic/xiao_esp32s3_114_mic` | 使用 ESP32 PDM I2S 读取板载数字麦克风，`D0=MIC_CLK`、`D1=MIC_DATA`，串口输出 `mean/peak/rms`。 |
| `example/basic/xiao_esp32s3_114_imu` | 通过 `Wire` 在 `D4=SDA`、`D5=SCL` 上探测 QMI8658-compatible 或 LSM6-compatible IMU，并串口输出加速度和陀螺仪数据；`D14=IMU_INT`。 |
| `example/basic/xiao_esp32s3_114_imu_int` | 使用 `D14=IMU_INT` 验证 LSM6-compatible IMU 运动唤醒中断，串口打印触发次数和 `WAKE_UP_SRC`。 |
| `example/basic/xiao_esp32s3_114_button` | 读取 `USR1=D6`、`USR2=D7`、`USR3=D19`，按键为 active-low，串口输出按下/释放状态。 |
| `example/basic/xiao_esp32s3_114_bat` | 使用 `D16=BAT_ADC` 读取电池 ADC；按原理图 `316k/160k` 分压换算，`battery = ADC * 2.975`。 |
| `example/basic/xiao_esp32s3_114_i2c_scan` | Grove I2C 扫描示例，使用 `D4=SDA`、`D5=SCL`，串口打印扫描到的设备地址。 |
| `example/basic/xiao_esp32s3_114_i2s` | I2S 音频输出测试示例：`D11=DIN`、`D12=BCLK`、`D13=LRC/WS`，可接 MAX98357A 播放低音量测试音。 |

## Application

- `application/xiao_nrf52840_147_wakeup`
  - 驱动库适配：使用 Seeed_GFX / `TFT_eSPI` 驱动 1.47 寸屏幕，使用 `Seeed_Arduino_LSM6DS3` 配置 IMU D14 运动唤醒。
  - 已加入 BAT 电量显示：过 `READ_BAT=P0.14` 使能分压，`PIN_VBAT` 读取电池电压，`CHG=P0.17` 判断充电状态。
  - 屏幕 `POWER STATE` 区域显示电压和电量百分比，充电时显示黄色小闪电图标。
  - 唤醒背光已调暗，`BACKLIGHT_AWAKE_PWM = 120`；睡眠时背光关闭，保持 `BACKLIGHT_SLEEP_PWM = 0`。

- `application/xiao_esp32s3_114_wakeup`
  - 1.14 寸 XIAO ESP32-S3 Plus IMU 唤醒示例，基于 `example/basic` 中的 1.14 LCD、IMU、按键和电池读取代码迁移；
  - 使用 `driver.h` + Seeed_GFX / `TFT_eSPI tft(135, 240)` 驱动屏幕，并使用 `tft.invertDisplay(true)`；
  - 按 USR1 关闭背光并进入 ESP32-S3 light sleep，LSM6-compatible IMU D14 运动唤醒或 USR2 按键可唤醒屏幕；
  - 使用 `D16=BAT_ADC` 按 `316k/160k` 分压换算电池电压。

- `application/xiao_nrf52840_147_sd_unline_record`
  - nRF52840 离线原音录放示例：屏幕显示待机、录音、保存和播放状态；
  - 按 USR1 关闭无线活动并启动外部 HFCLK，使用 PDM EasyDMA 双缓冲将 5 秒、16 kHz、16-bit 单声道 PCM 采集到 RAM，再保存为 RAW WAV；
  - 按 USR2 通过 MAX98357A/I2S 播放刚录的原音。5 秒 PCM 占约 160 KB，适配 nRF52840 的 256 KB RAM。

- `application/xiao_esp32s3_114_flash_record`
  - 1.14 寸 XIAO ESP32-S3 Plus 原音录放示例，基于 `example/basic` 中的 1.14 LCD、PDM Mic 和 I2S 输出代码迁移；
  - 按 USR1 使用板载 PDM 麦克风录制 5 秒、16 kHz、16-bit 单声道 PCM，并保存为板载 Flash / LittleFS 中的 `/REC_RAW.WAV`，不需要 SD 卡；
  - 按 USR2 读取 Flash 中的 WAV，并通过 `D11=DIN`、`D12=BCLK`、`D13=LRC/WS` 连接 MAX98357A 播放原音；
  - Flash 中只保留最新一段录音，重新录音会覆盖旧文件。

- `application/xiao_nrf52840_147_sd_image_reader`
  - SD BMP 读取/刷屏压力测试示例，已按 `example/basic/xiao_nrf52840_147_sd/xiao_nrf52840_147_sd.ino` 和 `example/basic/xiao_nrf52840_147_display/xiao_nrf52840_147_display.ino` 的库用法同步。
  - 显示驱动从 `Arduino_GFX_Library` 切换为 `driver.h` + Seeed_GFX / `TFT_eSPI`，使用 `TFT_eSPI tft`、`tft.init()`、`tft.pushImage()` 刷新 BMP 行数据。
  - SD 继续使用 nRF52 SDK 自带 `SdFat`，通过 `SdSpiConfig(SD_CS_PIN, SHARED_SPI, freq, &SPI)` 初始化共享 SPI 总线。
  - 为避免 `TFT_eSPI` 或其它库中的全局 `SD` 对象冲突，草图内 `SdFat` 实例命名为 `sdCard`，不要再使用 `SdFat SD;`。
  - `tft.setSwapBytes(true)` 已放在 `tft.init()` 之后，用于 `pushImage()` 输出 RGB565 行缓冲时的字节序处理。
  - 如果出现整屏颜色取反，参考上方 tip 8 调整 `tft.invertDisplay(...)`。
  - 扫描 SD 卡根目录中的图片并通过屏幕轮播；直接显示 16/24/32-bit 未压缩 BMP

- `application/xiao_nrf52840_147_sd_bmp_reader_stress_v0_1`
  - 旧版 SD BMP 读取/刷屏压力测试工程，保留示例图片资源和原始压力测试逻辑。

- `application/xiao_nrf52840_147_graphictest`
  - LCD 图形压力测试，会大量调用 `drawLine()` / `drawPixel()` 等小图元写屏；
  - 测试时请先拔出 SD 卡，避免插卡冷启动后在 `Lines` 等项目卡住。
  - 如需验证 SD + LCD 同时工作，请使用 `application/xiao_nrf52840_147_sd_image_reader`。

- `application/xiao_esp32s3_114_graphictest`
  - 1.14 寸 XIAO ESP32-S3 Plus LCD 图形测试示例；
  - 使用 `driver.h` + Seeed_GFX / `TFT_eSPI tft(135, 240)` 驱动屏幕，并沿用 1.14 display 示例的 `tft.invertDisplay(true)`；
  - 测试颜色条、线条、矩形、圆形、三角形、圆角矩形、文字和像素渐变。

- `application/xiao_nrf52840_147_electronic_quicksand`
  - 电子流沙动态显示示例：使用 LSM6DS3 加速度计读取重力方向，让黄色颗粒随倾斜流动和堆积；
  - 借鉴 LED matrix fluid demo 的低分辨率网格思路，在覆盖全屏的 24x45 occupancy grid 上更新颗粒位置，只擦除/重画变化格子；
  - 根据颗粒沿重力方向的深度调节流动性：顶部自由颗粒更容易滑动，底部受压颗粒阻尼更大、速度更低；
  - 当前使用流畅优先参数：180 个颗粒、8 ms 帧间隔、1 格邻域寻址；
  - 使用 Seeed_GFX / `TFT_eSPI` 驱动 1.47 寸屏幕，画面为黑底全屏黄色颗粒；
  - 适合验证 IMU + LCD 的实时交互效果，不依赖 SD 卡。

- `application/xiao_esp32s3_147_electronic_quicksand`
  - 1.47 寸 XIAO ESP32-S3 Plus 电子流沙示例，基于 ESP32-S3 1.14 版本的 IMU 读取逻辑和 nRF52840 1.47 版本的流沙参数迁移；
  - 使用 `driver.h` + Seeed_GFX / `TFT_eSPI` 驱动 172x320 屏幕，并应用 1.47 面板方向修正；
  - 使用 `D4=SDA`、`D5=SCL` 读取 QMI8658-compatible 或 LSM6-compatible IMU，把重力方向转换为颗粒流动；
  - 使用 24x45、7 px cell、180 个黄色颗粒，保持和 1.47 nRF52840 版本一致的全屏电子流沙效果。

- `application/xiao_esp32s3_114_electronic_quicksand`
  - 1.14 寸 XIAO ESP32-S3 Plus 电子流沙示例，基于 `example/basic` 中的 1.14 LCD 与 IMU 代码复刻；
  - 使用 `driver.h` + Seeed_GFX / `TFT_eSPI tft(135, 240)` 驱动 1.14 寸屏幕；
  - 使用 `D4=SDA`、`D5=SCL` 读取 QMI8658-compatible 或 LSM6-compatible IMU，把重力方向转换为颗粒流动；
  - 为适配 1.14 屏幕，流沙网格调整为 22x40、6 px cell、150 个颗粒，只刷新变化格子。
