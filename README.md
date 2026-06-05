# XIAO-Display-Board
Contains hardware documentation, software drivers, factory firmware, and sample demos for the latest XIAO display development board
现各屏幕资料的情况：https://seeedstudio.feishu.cn/wiki/Omfewh9yAiMhqVkDmgxcPzm6nug

驱动：
- https://github.com/Seeed-Studio/Seeed_GFX
- https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3
- https://github.com/greiman/SdFat

Wiki参考：
- https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
- https://wiki.seeedstudio.com/xiao_esp32s3_sense_filesystem/
- https://wiki.seeedstudio.com/seeedstudio_round_display_usage/



用Seeed nRF52 Boards这个SDK，已纠正D17和D19反接情况
但是这个库存在Serial识别不了的情况，属于底层bug，需要再用一个库打补丁
```
#include <Adafruit_TinyUSB.h>
```
