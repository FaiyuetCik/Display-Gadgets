// Seeed_GFX setup for ST7789 172x320 TFT.
// User_Setup_Select.h loads this file from the sketch folder.

#define BOARD_SCREEN_COMBO 75
#define USE_XIAO_TFT_DISPLAY_BOARD

// On ESP32-S3, disabling SPI transactions in TFT_eSPI prevents mutex
// interference with SdFat (DEDICATED_SPI).  CS-pin management in
// acquireForLcd / acquireForSd keeps the two devices isolated.
#define SUPPORT_TRANSACTIONS 0
