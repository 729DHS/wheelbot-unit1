# DM4310 单测项目 调试文档索引

## 目录

| 序号 | 文档 | 简述 |
|------|------|------|
| 1 | [01-ENABLE-DISABLE-交换bug.md](01-ENABLE-DISABLE-交换bug.md) | ENABLE/DISABLE 尾字节定义错误导致电机不工作 |
| 2 | [02-反馈帧字节布局.md](02-反馈帧字节布局.md) | 说明书 vs 实际协议的反馈帧格式差异 |
| 3 | [03-Bringup序列.md](03-Bringup序列.md) | bringup 流程: DISABLE -> ZERO -> DONE |
| 4 | [04-chassis_base驱动适配.md](04-chassis_base驱动适配.md) | 底盘版驱动裁剪为单测用途 |
| 5 | [05-构建与烧录.md](05-构建与烧录.md) | Zephyr 构建配置和 CMSIS-DAP 烧录 |
| 6 | [06-首次烧录验证.md](06-首次烧录验证.md) | 2026-05-04 首次烧录验证记录 |
| 7 | [07-TODO.md](07-TODO.md) | 待办清单 |

## 关键经验总结

1. **ENABLE/DISABLE 尾字节**: joint_test 版定义反了, 用 chassis_base 版 (ENABLE=0xFC, DISABLE=0xFD, ZERO=0xFE)
2. **反馈帧解码**: 实际协议与说明书存在字节偏移差异, 以 C++ 参考实现和实际通信抓包为准
3. **Bringup**: 需要完整三步 DISABLE->ZERO->DONE, 跳过 ZERO 会导致位置零点异常
4. **CAN 滤波器**: 接受全部帧 (id=0x000 mask=0x000) 比窄滤波器更可靠
5. **编译开关**: chassis_base 有多层 compile-time 开关, 单测用需要关闭 CAN1_HOME 和 NVS Flash
