# Bringup 序列

## 正确流程

DM4310 电机上电后需要逐电机执行 bringup 序列, 确保电机进入正常控制模式。

### Bringup 状态机

```
Step 0 (DISABLE): 失能电机, 电机自由转动
  发送 [FF FF FF FF FF FF FF FD] × 10 ticks
  ↓
Step 1 (ZERO):   将当前位置设置为电机内部零点
  发送 [FF FF FF FF FF FF FF FE] × 10 ticks
  ↓
Step 2 (DONE):   进入正常 MIT 控制模式
```

### joint_test 版问题

```c
// joint_test: 跳过 ZERO 步骤
if (step >= 0U) {
    dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data); // 实际发 ENABLE
}
tick++;
if (tick >= DM4310_BRINGUP_TICKS) {
    g_dm4310.bringup_step[i] = (step == 0U) ? 2U : step + 1U; // 跳过 step 1
```

### chassis_base 版 (正确)

```c
if (step == 0U) {
    dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
} else {
    dm4310_pack_special(DM4310_CMD_ZERO_TAIL, data);
}
tick++;
if (tick >= DM4310_BRINGUP_TICKS) {
    g_dm4310.bringup_step[i] = step + 1U; // 线性递进 0→1→2
```

### C++ 参考实现 (`Dm4310MotorWithBringup::tick()`)

```cpp
if (bringup_step_ < 2U) {
    const uint8_t tail =
        (bringup_step_ == 0U) ? dm4310::kCmdDisableTail : dm4310::kCmdZeroTail;
    const auto data = dm4310::pack_special(tail);
    (void)tx_with_stats(...);
    if (++bringup_tick_ >= 10U) {
        bringup_step_++;
        bringup_tick_ = 0U;
    }
    return;
}
```

## 时序

每 motor 每步骤 10 ticks, 1 tick = 1ms:
- 4 motors × 2 steps × 10ms = **~80ms** 完成全部 bringup
