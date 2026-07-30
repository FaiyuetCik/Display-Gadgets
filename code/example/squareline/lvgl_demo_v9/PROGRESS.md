# LVGL v9 Demo 进度文档

**硬件**: XIAO nRF52840 Plus + 1.47 寸触摸屏（172×320 JD9853A）
**LVGL 版本**: LVGL v9.5.0（官方版，非 Seeed 魔改版）
**日期**: 2026-07-24（创建）

---

## 设计目标

跳过 Seeed LVGL v8.3 的以下已知问题：

| 问题 | v8 (Seeed 魔改) | v9 (官方) |
|------|-----------------|-----------|
| 部分渲染 bug | 只刷第一个 strip | ✅ 原生支持 |
| buffer 大小 | 被迫全屏 107KB | 1/10 屏 ≈ 11KB |
| `lv_disp_flush_ready` 签名 | 非标准 `(lv_disp_drv_t*)` | 标准 `(lv_display_t*)` |
| tick 设置 | 依赖 lv_conf.h `LV_TICK_CUSTOM` | `lv_tick_set_cb()` 运行时设置 |

## 核心改动（vs v8 demo）

| 项目 | v8 demo | v9 demo |
|------|---------|---------|
| buffer | 全屏 172×320 = 107KB | 1/10 屏 = ~11KB |
| render mode | `full_refresh=1`（被迫） | `LV_DISPLAY_RENDER_MODE_PARTIAL` |
| display 创建 | `lv_disp_drv_init()` + `lv_disp_drv_register()` | `lv_display_create()` + `lv_display_set_*()` |
| indev 创建 | `lv_indev_drv_init()` + `lv_indev_drv_register()` | `lv_indev_create()` + `lv_indev_set_*()` |
| flush 回调签名 | `(lv_disp_drv_t*, const lv_area_t*, lv_color_t*)` | `(lv_display_t*, const lv_area_t*, uint8_t*)` |
| indev 回调签名 | `(lv_indev_drv_t*, lv_indev_data_t*)` | `(lv_indev_t*, lv_indev_data_t*)` |
| touch state 常量 | `LV_INDEV_STATE_PR` / `LV_INDEV_STATE_REL` | `LV_INDEV_STATE_PRESSED` / `LV_INDEV_STATE_RELEASED` |
| 活动屏幕 | `lv_scr_act()` | `lv_screen_active()` |
| loop | `lv_timer_handler()` + 固定 delay(2) | `lv_timer_handler()` 返回下次到期时间 |

## 内存估算

- Buffer: 11,008 bytes (~11KB)
- LVGL heap: ~48KB (default `LV_MEM_SIZE`)
- Stack + globals: ~10KB
- 合计: ~70KB / 256KB — 远未触及 USB CDC DMA 区域

## 测试状态

| 功能 | 状态 | 备注 |
|------|------|------|
| 编译 | ⬜ 待测试 | |
| 屏幕显示 | ⬜ 待测试 | |
| 部分刷新（无 tearing） | ⬜ 待测试 | |
| 触摸（Serial 有 [TOUCH DIAG]） | ⬜ 待测试 | |
| 按钮响应 + 计数器 | ⬜ 待测试 | |
| Serial 始终在线（loop 中也有输出） | ⬜ 待测试 | |

## 文件

| 文件 | 说明 |
|------|------|
| `lvgl_demo_v9.ino` | 主 sketch |
| `driver.h` | TFT_eSPI 配置（与 v8 相同） |
| `axs5106l_device.h` | AXS5106L 触摸驱动头文件（与 v8 相同） |
| `axs5106l_device.cpp` | AXS5106L 触摸驱动实现（与 v8 相同） |
