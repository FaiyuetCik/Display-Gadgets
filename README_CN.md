**Language: [English](./README.md) | [简体中文](./README_CN.md)**

# XIAO Display Gadgets

**XIAO Display 系列**的硬件资料、驱动、出厂固件和示例代码 — 涵盖三种尺寸（1.47"、1.14"、0.96"），每种尺寸提供 XIAO nRF52840 Plus 和 XIAO ESP32-S3 Plus 两个版本。

## 仓库结构

```
Display-Gadgets/
├── code/                          旧版示例（TFT_eSPI / Arduino_GFX）
│   ├── example/
│   │   ├── basic/          基础硬件验证示例
│   │   ├── 147_nRF52840/   1.47" nRF52840 出厂 Dashboard 及原理图
│   │   ├── 147_ESP32/      1.47" ESP32-S3 出厂 Dashboard 及原理图
│   │   ├── 114_nRF52840/   1.14" nRF52840 出厂 Dashboard 及原理图
│   │   ├── 114_ESP32/      1.14" ESP32-S3 出厂 Dashboard 及原理图
│   │   ├── 096_nRF52840/   0.96" nRF52840 出厂 Dashboard 及原理图
│   │   ├── 096_ESP32/      0.96" ESP32-S3 出厂 Dashboard 及原理图
│   │   ├── images/         演示截图、GIF 和视频
│   │   └── squareline/     SquareLine Studio UI 工程文件
│   └── Function/           旧版独立单功能演示
│       ├── 147_nRF52840/
│       ├── 147_ESP32/
│       ├── 114_nRF52840/
│       ├── 114_ESP32/
│       ├── 096_nRF52840/
│       └── 096_ESP32/
├── code_GFX2/                     Seeed_GFX2 迁移版 Function demo（当前）
│   └── Function/
│       ├── 147_nRF52840/
│       ├── 147_ESP32/
│       ├── 114_nRF52840/
│       ├── 114_ESP32/
│       ├── 096_nRF52840/
│       └── 096_ESP32/
└── schematics/             全部 6 款产品的 PDF 原理图
```

## 基础示例

`code/example/basic/` 下的每个子目录都是一个独立的 Arduino 示例，用于验证单个硬件外设。

| 目录 | 说明 |
|-----------|-------------|
| `code/example/basic/xiao_nrf52840_147_display` | 1.47" 屏幕测试 — Seeed_GFX / TFT_eSPI 色块与文字显示 |
| `code/example/basic/xiao_nrf52840_147_touch` | AXS5106L 触摸控制器 — 通过 I2C 读取 X/Y 坐标 |
| `code/example/basic/xiao_nrf52840_147_touch_int` | 触摸测试 + D7 中断信号验证 |
| `code/example/basic/xiao_nrf52840_147_imu` | LSM6DS3 加速度计 + 陀螺仪 — Seeed_Arduino_LSM6DS3 读取 |
| `code/example/basic/xiao_nrf52840_147_imu_int` | LSM6DS3 D14 运动唤醒中断测试 |
| `code/example/basic/xiao_nrf52840_147_mic` | PDM 麦克风峰值读取 — 串口输出 |
| `code/example/basic/xiao_nrf52840_147_sd` | MicroSD 卡挂载与目录列表 — nRF52 SDK SdFat |
| `code/example/basic/xiao_nrf52840_147_sd_text_reader` | 从 SD 卡读取 TXT/LOG/CSV 文本文件并分页显示 |
| `code/example/basic/xiao_nrf52840_147_bat` | 电池 ADC 读取：分压使能、电压读取、充电状态 |
| `code/example/basic/xiao_nrf52840_147_button` | USR1/USR2 按键读取 — USR1 切换背光亮度 |
| `code/example/basic/xiao_nrf52840_147_i2s` | I2S + MAX98357A 扬声器测试 (D11=DIN, D12=BCLK, D13=LRC/WS) |
| `code/example/basic/xiao_esp32s3_147_bat` | ESP32-S3 电池 ADC — D16 读取，316k/160k 分压 |
| `code/example/basic/xiao_esp32s3_114_display` | 1.14" 屏幕测试 — Seeed_GFX / TFT_eSPI (135×240) |
| `code/example/basic/xiao_esp32s3_114_mic` | ESP32 PDM I2S 麦克风 — D0=MIC_CLK, D1=MIC_DATA |
| `code/example/basic/xiao_esp32s3_114_imu` | QMI8658 / LSM6 兼容 IMU — D4=SDA, D5=SCL |
| `code/example/basic/xiao_esp32s3_114_imu_int` | IMU D14 唤醒中断验证 |
| `code/example/basic/xiao_esp32s3_114_button` | USR1=D6, USR2=D7, USR3=D19 — 低电平有效按键 |
| `code/example/basic/xiao_esp32s3_114_bat` | 电池 ADC — D16，316k/160k 分压，ratio ≈ 2.975 |
| `code/example/basic/xiao_esp32s3_114_i2c_scan` | Grove I2C 扫描器 — D4=SDA, D5=SCL |
| `code/example/basic/xiao_esp32s3_114_i2s` | I2S 音频输出测试 (D11=DIN, D12=BCLK, D13=LRC/WS) |

## Function 演示

`code_GFX2/Function/` 下的独立单功能演示，已迁移到 **Seeed_GFX2**（新一代图形库）。每个子目录是一个完整的 Arduino sketch，展示一个板载外设或应用模式。旧版 TFT_eSPI/Arduino_GFX 版本仍保留在 `code/Function/`。

### 1.47" nRF52840 Plus

| 目录 | 说明 |
|-----------|-------------|
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_graphictest` | LCD 图形压力测试 — drawLine/drawPixel 高负载渲染 |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_touch_circle` | 触摸画圆 — 在触摸点绘制白色圆形 |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_mic_canvas` | 大音量条 — PDM 麦克风峰值分段柱状图 |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_sd_image_reader` | SD BMP 图片浏览器 — 从 SD 卡读取并显示 BMP 图片 |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_sd_unline_record` | 离线录音回放 — PDM 麦克风 → RAM → I2S 扬声器 |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_electronic_quicksand` | 电子流沙 — IMU 驱动的粒子流体模拟 |
| `code_GFX2/Function/147_nRF52840/xiao_nrf52840_147_wakeup` | 抬手亮屏 — IMU 运动唤醒 + 电池状态显示 |

### 1.47" ESP32-S3 Plus

| 目录 | 说明 |
|-----------|-------------|
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_graphictest` | 1.47" ESP32-S3 LCD 图形压力测试 |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_touch_circle` | 触摸画圆 — 在触摸点绘制圆形 |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_mic_canvas` | 大音量条 — PDM 麦克风峰值分段柱状图 |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_sd_record` | SD 录音 — PDM 麦克风 → SD WAV → I2S 扬声器 |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3plus_147_sd_bmp_reader_diag_v0_8` | SD BMP 图片浏览器 — 从 SD 卡显示 BMP（诊断版） |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_electronic_quicksand` | 电子流沙 — IMU 驱动的粒子流体模拟 |
| `code_GFX2/Function/147_ESP32/xiao_esp32s3_147_wakeup` | 抬手亮屏 — IMU 运动唤醒 |

### 1.14" nRF52840 Plus

| 目录 | 说明 |
|-----------|-------------|
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_graphictest` | 1.14" nRF52840 LCD 图形压力测试 |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_electronic_quicksand` | 电子流沙 — 22×40 网格, 150 颗粒, 6px 格 |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_wakeup` | 抬手亮屏 — IMU 运动唤醒 |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_voice_bar` | 大音量条 — PDM 麦克风实时波形 + 分段柱状图 |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_flash_record` | Flash 录音回放 — PDM 麦克风 → InternalFS → I2S 扬声器 |
| `code_GFX2/Function/114_nRF52840/xiao_nrf52840_114_sht31_temperature_humidity` | SHT31 温湿度传感器读取 |

### 1.14" ESP32-S3 Plus

| 目录 | 说明 |
|-----------|-------------|
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_graphictest` | 1.14" ESP32-S3 LCD 图形压力测试 |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_electronic_quicksand` | 电子流沙 — 22×40 网格, 150 颗粒, 6px 格 |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_wakeup` | 抬手亮屏 — IMU 运动唤醒 + light sleep |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_flash_record` | Flash 录音回放 — PDM 麦克风 → LittleFS → I2S 扬声器（无需 SD 卡） |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_voice_bar` | 大音量条 — PDM 麦克风实时波形 + 分段柱状图 |
| `code_GFX2/Function/114_ESP32/xiao_esp32s3_114_sht31_temperature_humidity` | SHT31 温湿度传感器读取 |

### 0.96" nRF52840 Plus

| 目录 | 说明 |
|-----------|-------------|
| `code_GFX2/Function/096_nRF52840/xiao_nrf52840_096_graphictest` | 0.96" nRF52840 LCD 图形压力测试 |
| `code_GFX2/Function/096_nRF52840/xiao_nrf52840_096_electronic_quicksand` | 电子流沙 — 适配 0.96" 80×160 屏幕 |
| `code_GFX2/Function/096_nRF52840/xiao_nrf52840_096_wakeup` | 抬手亮屏 — IMU 运动唤醒 |
| `code_GFX2/Function/096_nRF52840/xiao_nrf52840_096_flash_record` | Flash 录音回放 — PDM 麦克风 → InternalFS → I2S 扬声器 |

### 0.96" ESP32-S3 Plus

| 目录 | 说明 |
|-----------|-------------|
| `code_GFX2/Function/096_ESP32/xiao_esp32s3_096_graphictest` | LCD 图形基准测试 — 10 种图元逐项计时 |
| `code_GFX2/Function/096_ESP32/xiao_esp32s3_096_electronic_quicksand` | 电子流沙 — IMU 驱动的粒子流体模拟 |
| `code_GFX2/Function/096_ESP32/xiao_esp32s3_096_wakeup` | 抬手亮屏 — IMU 运动唤醒（light sleep） |
| `code_GFX2/Function/096_ESP32/xiao_esp32s3_096_flash_record` | Flash 录音回放 — PDM 麦克风 → LittleFS → I2S 扬声器（需 SPIFFS 分区） |

## 提示和已知问题

1. **开发板包**: 使用 `Seeed nRF52 Boards`（v1.1.13+）。该 SDK 修正了早期 XIAO nRF52840 版本的 D17/D19 引脚交换问题。

2. **串口无法识别**: nRF52 SDK 的一个已知底层 bug 可能导致 USB 串口无法被电脑识别。安装 [Adafruit_TinyUSB](https://github.com/adafruit/Adafruit_TinyUSB_Arduino)（也可通过 Arduino Library Manager 下载），并在每个 sketch 中添加：
   ```cpp
   #include <Adafruit_TinyUSB.h>
   ```

3. **Seeed_GFX2 与 TFT_eSPI 冲突**: 安装 `Seeed_GFX2` 前请删除已有的 `TFT_eSPI` 库，两者不兼容。

4. **缺少 Seeed_Arduino_FS**: 如编译提示缺少 `Seeed_Arduino_FS`，请从 [GitHub](https://github.com/Seeed-Studio/Seeed_Arduino_FS) 或 Arduino Library Manager 安装。

5. **IMU 驱动**: 六款板子均使用 **LSM6DS3** IMU。出厂 Dashboard 使用 **SparkFun LSM6DS3**（`SparkFunLSM6DS3.h`），而 `code_GFX2` Function demo 使用 **Seeed_Arduino_LSM6DS3**（`LSM6DS3.h`）。两个头文件互不通用——按目标固件安装对应库。部分 demo 代码里有 QMI8658 fallback 路径，但硬件只有 LSM6DS3。

6. **nRF52 的 SdFat**: nRF52 SDK 自带 SdFat 库，请勿单独安装其他版本 — 会冲突。

7. **1.47" 屏幕颜色反转**: 如果屏幕颜色出现反转（黑↔白、红↔青、绿↔紫、黄↔蓝），在 `tft.init()` 和旋转设置后调用 `tft.invertDisplay(false)` 或 `tft.invertDisplay(true)` 切换。这是 LCD inversion 问题，不是 RGB/BGR 字节序问题。

8. **SquareLine Studio 工作流程**: 即将推出 —— 这些板子的 SquareLine 生成 UI 支持尚未提供。

9. **共享 SPI 总线（LCD + SD）**: LCD 和 SD 卡共用 SPI 总线。每次 LCD 刷新后，如需访问 SD 卡请重新初始化：
   ```cpp
   sdCard.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, freq, &SPI));
   ```
   否则 SD 读写可能在 LCD 刷新后失败或卡住。

10. **GraphicTest 与 SD 卡**: 部分 GraphicTest sketch 在插入 SD 卡时冷启动可能会卡住（尤其是在 `Lines` 渲染阶段）。运行 GraphicTest 前请先拔出 SD 卡；如需同时使用 SD + LCD，请使用 SD image reader 示例。

11. **ESP32 flash_record 需要 SPIFFS 分区**: 1.14" 和 0.96" ESP32-S3 的录音 demo 把 WAV 存在 `LittleFS`，需要 SPIFFS 分区。烧录前在 Arduino IDE 选择 **Tools > Partition Scheme > "Default with spiffs (3MB APP/1.5MB SPIFFS)"**，否则 `LittleFS.begin()` 失败、屏幕显示 "Write failed / Check flash"。

## 所需库

| 库 | 用途 | 来源 |
|---------|---------|--------|
| Seeed nRF52 Boards | nRF52840 开发板包 (v1.1.13+) | Arduino Boards Manager |
| esp32 Boards by Espressif | ESP32-S3 开发板包 (3.3.11+) | Arduino Boards Manager |
| Seeed_GFX2 | `code_GFX2` Function demo 的屏幕图形库（替代 TFT_eSPI） | [GitHub](https://github.com/Seeed-Studio/Seeed_GFX2) / Library Manager |
| GFX Library for Arduino | 屏幕图形库（Arduino_GFX，0.96" 及多个 demo 使用） | Arduino Library Manager |
| Seeed_Arduino_LSM6DS3 | LSM6DS3 IMU 驱动（Function demo） | [GitHub](https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3) / Library Manager |
| SparkFun LSM6DS3 | LSM6DS3 IMU 驱动（出厂 Dashboard） | Arduino Library Manager |
| Seeed_Arduino_FS | 文件系统抽象层 | [GitHub](https://github.com/Seeed-Studio/Seeed_Arduino_FS) / Library Manager |
| Adafruit_TinyUSB | nRF52 USB 串口补丁 | [GitHub](https://github.com/adafruit/Adafruit_TinyUSB_Arduino) / Library Manager |
| SdFat | SD 卡（nRF52 SDK 自带） | 包含在 Seeed nRF52 Boards 中 |
| PDM | PDM 麦克风（nRF52 SDK 自带） | 包含在 Seeed nRF52 Boards 中 |

## Wiki

面向用户的文档发布在 [Seeed Studio Wiki](https://wiki.seeedstudio.com/)。请在 **Sensor → LCD Displays → Display Gadgets** 下查看。

## 相关仓库

- [Seeed_GFX2](https://github.com/Seeed-Studio/Seeed_GFX2) — 屏幕图形库
- [Seeed_Arduino_LSM6DS3](https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3) — IMU 驱动
- [Seeed_Arduino_FS](https://github.com/Seeed-Studio/Seeed_Arduino_FS) — 文件系统库
