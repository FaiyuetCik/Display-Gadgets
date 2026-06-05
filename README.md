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
| `example/basic/bat` | 读取 VBAT ADC、电池分压使能和充电状态脚，用于检查电池电压和充电状态。 |
| `example/basic/button` | 读取 USR1 / USR2 按键状态，USR1 会切换背光亮度，USR2 打印按键状态。 |
| `example/basic/i2s` | 使用预留 I2S 音频输出引脚 `SD=D11`、`SCK=D12`、`WS=D13` 启动 nRF52840 I2S TX 骨架。 （未测试）|
