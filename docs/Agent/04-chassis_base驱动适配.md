# chassis_base 驱动适配为单测

## 背景

chassis_base 是完整机器人底盘项目, 其 dm4310_motor.c 包含 NVS Flash 零点存储、CAN1 Home 校准等功能。
用于单测时需要裁剪。

## 编译开关修改

```c
// 原始值 (chassis_base)
#define DM4310_OUTPUT_ENABLED       1   // 保持
#define DM4310_CAN1_HOME_ENABLED    1   // 改为 0
#define DM4310_NORMAL_OUTPUT_ENABLED 0   // 改为 1
#define DM4310_CAN1_HOME_USE_FLASH  1   // 改为 0
```

| 开关 | 原值 | 新值 | 说明 |
|------|------|------|------|
| OUTPUT_ENABLED | 1 | 1 | 使能 CAN 输出 |
| CAN1_HOME_ENABLED | 1 | 0 | 关闭 Home 校准 |
| NORMAL_OUTPUT_ENABLED | 0 | 1 | 使能 bringup + MIT 控制 |
| CAN1_HOME_USE_FLASH | 1 | 0 | 关闭 Flash 存储 |

## 移除内容

1. **NVS Flash 代码**: `dm4310_nvs_init()`, `dm4310_load_can1_home()` — 依赖 Flash 分区, 单测不需要
2. **寄存器写入**: `dm4310_write_u32_register()` — 通过 StdId 0x7FF 写 MIT 模式寄存器
3. **Home 校准辅助函数**: `dm4310_home_kp()`, `dm4310_home_kd()`, `dm4310_home_in_deadband()`
4. **Flash 头文件**: `<zephyr/drivers/flash.h>`, `<zephyr/fs/nvs.h>`, `<zephyr/storage/flash_map.h>`

## prj.conf 最小化

```ini
CONFIG_CAN=y
CONFIG_PRINTK=y
```

不需要 chassis_base 的 SENSOR/BMI08X/SPI/DMA/FLASH/NVS 等配置。
