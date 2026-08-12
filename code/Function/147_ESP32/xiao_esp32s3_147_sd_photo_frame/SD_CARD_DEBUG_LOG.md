# SD Card Debug Log — XIAO ESP32-S3 1.47" Photo Frame

**项目**：`xiao_esp32s3_147_sd_photo_frame`
**原始症状**：开机后 LCD 显示 "SD Photo Frame" + "Mounting SD card..." 后卡住，不继续。SD 卡已插入。
**目标**：让 SD 卡成功挂载并读取 BMP 图片进行轮播。

---

## 已确认的根因（截至 2026-08-11）

经过 15 轮迭代排查，发现了 **3 个独立的问题**：

### 问题 1：`readLE16()/readLE32()` 通过引用传 `File32&` 时 `f.read()` hang
- **现象**：`File32::read()` 在 `drawBmp()` 中通过包装函数 `readLE16(f)` 调用时 hang，但同样的 `f.read()` 直接内联调用就正常。
- **解决**：把 BMP 头解析全部改为内联 `f.read(buf, N)`，不调用 `readLE16/readLE32` 包装函数。
- **推测原因**：ESP32 编译器对 `File32&` 引用参数产生了错误的代码路径。

### 问题 2：位移运算比较 `!= 0x4D42` 在 ESP32 编译器上行为异常
- **现象**：内联 `f.read(hdr_b, 2)` 读到了 `0x42 0x4D`（"BM"签名），但 `(uint16_t)hdr_b[0] | ((uint16_t)hdr_b[1] << 8) != 0x4D42` 判断为 true，错误进入了 "not a BM file" 分支。
- **解决**：改用直接字节比较 `hdr_b[0] != 0x42 || hdr_b[1] != 0x4D`，不使用位移运算。
- **推测原因**：ESP32 GCC 的整数提升规则在特定优化下产生不正确的比较结果。

### 问题 3：LCD 操作时 SPI 频率未恢复
- **现象**：BMP 头解析成功后，`tft.fillScreen(C_BLACK)` hang。原因是 SdFat SHARED_SPI 最后一次 `SPI.beginTransaction(400kHz)` 后，SPI 停留在 400kHz，LCD 无法在此频率通信。
- **当前尝试（#15-16）**：LCD 操作前显式调用 `SPI.beginTransaction(SPISettings(40MHz, ...))` 恢复频率。**待测试**。

---

## 尝试记录

| # | 日期 | 尝试内容 | 预期 | 结果 |
|---|------|----------|------|------|
| 1 | 2026-08-03 | 原始代码直接运行 | SD 卡挂载成功 | 卡在 "Mounting SD card..." |
| 2 | 2026-08-11 | **加超时保护 + 详细串口调试**：<br>`beginSd()` 每步日志，`setup()` 分 6 步标记，打印芯片型号/Flash/PSRAM 大小 | 定位 hang 点 | ✅ **定位**：SD 挂载 OK (400kHz)，扫描到 3 个 BMP，`readLE16()` 时 hang |
| 3 | 2026-08-11 | **给所有 SD 操作加 `acquireForSd()`**：`scanImages()` 前、`drawBmp()` 前、行循环 `f.seekSet()` 前；`acquireForSd/Lcd` 加 `SPI.setFrequency` | f.read() 正常 | ❌ **恶化**：`SPI.setFrequency()` 破坏了 SdFat 内部状态，连 `scanImages()` 都 hang |
| 4 | 2026-08-11 | **回退 SPI 频率设置**：`acquireForSd/Lcd` 恢复为纯 CS 管理（不碰频率），移除 `scanImages()` 的 `acquireForSd()` | scanImages() 恢复 | ❌ 回到原点：`f.open/size/seekSet` 成功，`f.read()` 仍 hang |
| 5 | 2026-08-11 | **改用 SHARED_SPI**：`SdSpiConfig(..., SHARED_SPI, ...)` | f.read() 成功 | ❌ DEDICATED 和 SHARED 都 hang，问题不在 SPI 模式 |
| 6 | 2026-08-11 | **绕过 FAT 层**：`sdCard.card()->readSector(0/1/100/1000)` 直接读原始扇区 | 区分层级 | ✅ **全部成功**：底层 SPI 通信正常！问题在 SdFat FAT 文件层 |
| 7 | 2026-08-11 | **对比路径打开 vs 目录枚举打开 + read()** | 定位 hang 点 | ✅ **关键发现**：`setup()` 里两种方式 `read()` 都成功！`available()`=165174，`read(&b,1)` → 1。但 `drawBmp()` 仍 hang |
| 8 | 2026-08-11 | **在 setup() 末尾调 drawBmp()** | 区分 setup vs loop | ✅ drawBmp() 在 setup 里也 hang。排除 loop 上下文问题 |
| 9 | 2026-08-11 | **去掉 acquire 函数的 `pinMode()`**：只保留 `digitalWrite(HIGH)` | pinMode 是根因 | ❌ 仍 hang。pinMode 不是根因 |
| 10 | 2026-08-11 | **精确复现 drawBmp 的调用**：同一文件 `/Atest.bmp`，同样 `acquireForSd()` + `seekSet(0)` + `read(b,2)` | 文件本身是否可读 | ✅ **成功！** 读到 `0x42 0x4D`(BM)。差异定位在测试内联 vs `readLE16(File32&)` |
| 11 | 2026-08-11 | **内联 readLE16/readLE32**：drawBmp 里直接 `f.read(buf, N)` + 手动字节拼接，不调包装函数 | f.read 不 hang | ✅ **f.read 不 hang 了！** 但返回错误数据 → 发现根因 #2 |
| 12 | 2026-08-11 | **打印实际字节值** | 看读到什么 | ✅ 读到 `0x42 0x4D`(BM)！发现判断逻辑 bug → 根因 #1 确认 |
| 13 | 2026-08-11 | **字节直接比较代替位运算**：`hdr_b[0]!=0x42 \|\| hdr_b[1]!=0x4D` | BM 签名通过 | ✅ **BM 签名 + 172x320 bpp=24 header OK！** BMP 头解析成功 |
| 14 | 2026-08-11 | **调试行循环 hang 点**：打印 dataOffset/rowSize，标记 seekSet/read/pushImage | 定位下一步哪 hang | ✅ 卡在 `fillScreen BLACK` → 根因 #3：SPI 频率 400kHz 太慢 |
| 15 | 2026-08-11 | **LCD 前 `SPI.setFrequency(40M)+setDataMode(MODE0)`** | fillScreen 成功 | ❌ 仍 hang。`SPI.setFrequency` 不够 |
| 16 | 2026-08-11 | **改用 `SPI.beginTransaction(SPISettings(40M,...))/endTransaction()`**<br>包围 LCD 操作，用标准 ESP32 SPI 事务 API | fillScreen 成功，图片正常显示 | **待用户测试** |

---

## 当前状态

```
SD 卡挂载    ✅  400kHz SHARED_SPI
文件扫描     ✅  扫描到 3 个 BMP
BMP 头解析  ✅  BM签名 + 172x320 bpp=24
fillScreen  ❓  尝试 #16 待测试（SPI.beginTransaction 方式）
行循环渲染  ❓  待 fillScreen 通过后继续
图片轮播    ❓  待行循环通过后继续
```

---

## 已完成的诊断 ✓

- [x] **串口监控**：所有操作均有详细 Serial 输出，已精确定位每个 hang 点
- [x] **SPI 模式**：DEDICATED_SPI 和 SHARED_SPI 都测试过，最终使用 SHARED_SPI
- [x] **原始扇区读取**：`sdCard.card()->readSector()` 全部成功，证实 SPI 硬件层正常
- [x] **文件读取测试**：`File32::read()` 在简化测试中工作，证明 SdFat 有能力读文件
- [x] **BMP 头解析**：已成功解析出 172x320 24-bit 参数

## 待验证项

- [ ] **尝试 #16**：`SPI.beginTransaction/endTransaction` 方式能否让 LCD 恢复工作
- [ ] **SD 卡格式确认**：建议确认为 FAT32（不是 exFAT/NTFS），虽然当前卡能挂载
- [ ] **SdFat 库版本**：确认 SdFat 库是否为最新版
- [ ] **图片颜色**：BMP 是 BGR 格式，确认 RGB 转换正确

---

## 参考资料

- 代码文件：`xiao_esp32s3_147_sd_photo_frame.ino`
- 引脚配置：`driver.h`
- SD_CS=D6(GPIO6), SCK=D8(GPIO8), MISO=D9(GPIO9), MOSI=D10(GPIO10)
- LCD_CS=D2(GPIO3), LCD_DC=D3(GPIO4), LCD_RST=D17, LCD_BL=D18
- LCD 驱动 IC：JD9853A (ST7789 兼容)，172x320，MADCTL=0x48
- SdFat 库：https://github.com/greiman/SdFat
