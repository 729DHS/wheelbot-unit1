# ENABLE/DISABLE 尾字节交换 Bug

## 问题

joint_test 版本的 dm4310_motor.h 中 ENABLE/DISABLE 定义与 C++ 参考实现相反。

## 错误定义 (joint_test)

```c
#define DM4310_CMD_DISABLE_TAIL 0xFC  // 错误: 实际是 ENABLE
#define DM4310_CMD_ENABLE_TAIL  0xFD  // 错误: 实际是 DISABLE
#define DM4310_CMD_ZERO_TAIL    0xFE
```

## 正确定义 (chassis_base / C++ 参考)

```c
#define DM4310_CMD_ENABLE_TAIL  0xFC  // 正确
#define DM4310_CMD_DISABLE_TAIL 0xFD  // 正确
#define DM4310_CMD_ZERO_TAIL    0xFE  // 正确
```

### 权威来源

C++ 参考实现 `dm4310_mit_protocol.hpp` 注释明确写明:
```
// enable:  [FF FF FF FF FF FF FF FC]
// disable: [FF FF FF FF FF FF FF FD]
// zero:    [FF FF FF FF FF FF FF FE]
```

## 影响

joint_test 的 bringup 中 `step==0` 发送 `DM4310_CMD_DISABLE_TAIL` (0xFC), 但这实际是 **ENABLE** 命令。
电机收到 ENABLE 后立即进入使能状态, 但后续 MIT 控制帧还未发送, 导致电机行为异常。

## 修复

使用 chassis_base 版本的正确定义, 或用以下值覆盖 joint_test:
```c
#define DM4310_CMD_ENABLE_TAIL  0xFC
#define DM4310_CMD_DISABLE_TAIL 0xFD
#define DM4310_CMD_ZERO_TAIL    0xFE
```
