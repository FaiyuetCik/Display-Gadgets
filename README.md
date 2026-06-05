# XIAO-Display-Board
Contains hardware documentation, software drivers, factory firmware, and sample demos for the latest XIAO display development board
现各屏幕资料的情况：https://seeedstudio.feishu.cn/wiki/Omfewh9yAiMhqVkDmgxcPzm6nug

驱动：
- https://github.com/Seeed-Studio/Seeed_GFX
- https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3

Wiki参考：
- https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
- https://wiki.seeedstudio.com/xiao_esp32s3_sense_filesystem/
- https://wiki.seeedstudio.com/seeedstudio_round_display_usage/



用Seeed nRF52 Boards这个SDK，已纠正D17和D19反接情况
但是这个库存在Serial识别不了的情况，属于底层bug，需要再用一个库打补丁
https://github.com/adafruit/Adafruit_TinyUSB_Arduino 或则直接在arduino搜索下载

```
#include <Adafruit_TinyUSB.h>
```

使用SEEED_GFX要将原来的TFT_eSPI库删掉

报错缺少Seeed_Arduino_FS库，需要前往https://github.com/Seeed-Studio/Seeed_Arduino_FS 或者直接在arduino搜索下载

使用Seeed_Arduino_LSM6DS3库驱动IMU

使用sdfat库驱动sd,但是注意！不用自己下载！nrf52 SDK里自带了一个

各基础功能代码结果放在了image文件夹里