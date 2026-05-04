# DM4310 关节电机 MIT 模式 位置锁定单测

## 项目简介

基于 Zephyr RTOS 的 DM-J4310-2EC 关节电机单元测试固件。

### 硬件平台
- **MCU**: STM32F407IG (DJI RoboMaster Type-C 板)
- **电机**: DM-J4310-2EC 减速电机 × 4
- **CAN 总线**: CAN1 (电机 1,2) + CAN2 (电机 3,4), 均为 1Mbps
- **调试器**: Horco CMSIS-DAP v2 (OpenOCD)

### 功能说明
上电后自动执行:
1. **CAN 初始化** — 初始化 CAN1/CAN2 总线
2. **Bringup 序列** — 逐电机执行 DISABLE → ZERO → 正常模式
3. **零点捕获** — bringup 完成后记录各电机当前位置作为零点
4. **MIT 阻抗位置锁定** — 以高刚度保持零点位置

当有外力推动电机偏离目标位置时, 位置误差增大, 电机内部控制律自动产生回复力矩抵抗外力。

### 电机映射
| 电机编号 | CAN 总线 | 关节 | 说明 |
|---------|---------|------|------|
| 1 | CAN1 | 左髋 | 电机 1 |
| 2 | CAN1 | 左膝 | 电机 2 |
| 3 | CAN2 | 右髋 | 电机 3 (左右对称) |
| 4 | CAN2 | 右膝 | 电机 4 (左右对称) |

### 控制参数
| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| KP | 90 | 0-500 | 位置刚度, 越大抵抗外力越强 |
| KD | 1.8 | 0-5 | 速度阻尼, 抑制震荡 |

> 可通过 OpenOCD 实时修改 `cmd_kp` / `cmd_kd` / `cmd_target_offset[0..3]`

### 构建 & 烧录
```bash
# 构建
export ZEPHYR_BASE=/home/huiming/zephyrproject/zephyr
cmake -B build -DBOARD=robomaster_c -DBOARD_ROOT=.
cmake --build build

# 烧录 (CMSIS-DAP)
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg     -c "adapter speed 1000"     -c "program build/zephyr/zephyr.elf verify reset exit"
```

### 调试 (OpenOCD)
```bash
# 连接 OpenOCD
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg

# GDB 中修改变量
(gdb) set var cmd_kp = 200
(gdb) set var cmd_kd = 3
(gdb) set var cmd_target_offset[0] = 0.1
```

### 项目结构
```
Unit1/
  boards/dji/robomaster_c/   # 板级定义 (DTS, Kconfig)
  src/
    dm4310_motor.h           # DM4310 MIT 协议驱动接口
    dm4310_motor.c           # DM4310 MIT 协议驱动实现
    main.c                   # 单测入口: MIT 位置锁定
  CMakeLists.txt             # Zephyr 构建配置
  prj.conf                   # Kconfig (CAN + PRINTK)
  docs/Agent/                # 调试经验文档
```
