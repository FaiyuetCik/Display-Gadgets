# LVGL Demo 进度文档

**硬件**: XIAO nRF52840 Plus + 1.47 寸触摸屏（172×320 JD9853A）
**LVGL 版本**: SeeedStudio_lvgl v8.3.4（Seeed 魔改版）
**日期**: 2026-07-24（更新）

---

## 最终状态

| 功能 | 状态 | 备注 |
|------|------|------|
| 屏幕显示（172×320） | ✅ 正常 | |
| LVGL 渲染（全屏） | ✅ 正常 | 全屏 buffer + full_refresh=1 |
| 标题/按钮/计数器/底部文字 | ✅ 可见 | 默认字体 montserrat 14 |
| 触摸（点击按钮） | ❌ 未修复 | 见下方分析 |
| 计数器递增 | ❌ | 依赖触摸 |

---

## 已修复的 Bug

### Bug 1：字体缺失 → 控件不可见
**根因**: `SeeedStudio_lvgl/lv_conf.h` 中 `LV_FONT_MONTSERRAT_20` 和 `_24` 设为 `0`
**修复**: 所有文字改用 `LV_FONT_DEFAULT`（montserrat 14）

### Bug 2：部分渲染只刷了第一个 strip
**根因**: Seeed 版 LVGL v8.3 部分渲染模式（buffer < 屏幕尺寸）有 bug——`lv_disp_flush_ready()` 后不继续下一个 strip
**尝试过**:
- `disp->disp` → 编译失败（struct 无此成员）
- `lv_disp_get_default()` → 编译失败（函数签名不匹配，Seeed 版接受 `lv_disp_drv_t*`）
- 双 buffer / 1/2 屏 buffer → 一样只刷第一段
**修复**: 全屏 buffer（107KB）+ `full_refresh=1`

### Bug 3：`lv_disp_flush_ready` 参数类型
**确认**: Seeed 版是 `lv_disp_flush_ready(lv_disp_drv_t *)`，与官方 `lv_disp_flush_ready(lv_disp_t *)` 不同。原始代码 `lv_disp_flush_ready(disp)` 正确。

---

## 未修复：触摸在 LVGL 运行时失效

### 已排除的原因

| 假设 | 验证方法 | 结论 |
|------|----------|------|
| 触摸硬件坏了 | 跑 basic touch 示例 | ✅ 硬件正常，`T x=97 y=78` |
| LCD SPI 干扰 I2C | `touch_lcd_test.ino`（init LCD + poll touch，不跑 LVGL） | ✅ 无冲突，触摸正常 |
| D17 共享 RST 导致冲突 | 同上测试 | ✅ Wire.end/begin + delay 后触摸恢复 |
| full_refresh 吃满 CPU | 切到 full_refresh=0 | ❌ 依旧不工作 |

### 核心现象
- **LVGL 不运行时**（touch_lcd_test）：loop() 里 `get_touch_data()` + `Serial.print()` 完美工作
- **LVGL 运行时**（lvgl_demo）：
  - setup() 中 Serial 正常
  - loop() 中所有 Serial 输出消失（`[TOUCH DIAG]`、`[RAW]` 一行都没）
  - LVGL indev 触摸回调从未被调用
  - 即使把触摸轮询放在 loop() 最前面、`lv_timer_handler()` 之前，也无输出

### 剩余推测
1. **LVGL 的 tick 定时器（LV_TICK_CUSTOM = millis()）可能与 nRF52 的 RTC/USB 有冲突**，导致 `millis()` 不前进或 Serial 被阻塞
2. **全屏 buffer（107KB）踩到了 USB CDC 的 DMA 缓冲区**——nRF52840 的 256KB RAM 中，107KB buffer + 48KB LVGL heap ≈ 155KB，加上 Adafruit TinyUSB 的端点 buffer 和栈，可能内存碎片严重
3. **`lv_timer_handler()` 内部禁用了中断**——在刷新期间可能长时间关中断，导致 USB SOF 丢失，CDC 连接断开

### 新发现（2026-07-24）
- `example/squareline/ui/ui.ino` 使用的是 **LVGL v9.5.0 官方版**（`lvgl@9.5.0`），不是 SeeedStudio_lvgl
- 但它依赖 SquareLine Studio 导出的 `GUI.h`，无法直接编译
- v9 的 `lv_disp_flush_ready(lv_display_t *)` 签名与官方一致，没有 Seeed v8 的魔改问题
- v9 的 buffer 可以小到 1/10 屏（~11KB），大幅降低内存，不踩 USB DMA 缓冲区

### 下一步计划（2026-07-25）
1. **写 LVGL v9 最小 demo**（`lvgl_demo_v9/`）：用官方 LVGL v9.5.0 + 1/10 小 buffer + 触摸
   - 如果 v9 的部分渲染没 bug → 显示 + 触摸都能工作
   - 如果部分渲染也有 bug → 用 full_refresh + 全屏 buffer（但 v9 全屏只需 1 个 buffer）
2. 如果 v9 也不行 → 换 ESP32S3 版本的板子试试（可能 nRF52840 的 TinyUSB 与 LVGL 有冲突）

---

## 代码改动总览（lvgl_demo.ino）

```diff
- lv_font_montserrat_20 / _24       →  LV_FONT_DEFAULT（默认字体）
- 1/10 屏 buffer + 部分渲染         →  全屏 buffer + full_refresh=1
+ memset 清零 buffer
+ 按钮显式蓝色底色 + 白色文字  
+ setup() 中强制渲染 30 次 + 切 full_refresh=0
+ 触摸回调保留 [TOUCH DIAG] 诊断打印
```

## 测试文件

| 文件 | 状态 |
|------|------|
| `lvgl_demo.ino` | 显示正常，触摸不工作 |
| `touch_lcd_test/touch_lcd_test.ino` | ✅ 触摸 + LCD 共存验证通过 |
