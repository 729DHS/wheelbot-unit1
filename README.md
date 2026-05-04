# DM4310 弹簧阻尼手感 + 左右腿同步开发

> 注意上下文维护,以及注释维护,每次改动内容请写入changelog到./docs/change目录下,并且注意git维护以及github上传!!!

## 分支: feature/spring-damper-sync

基于弹簧阻尼手感控制 + 左右腿侧边检测 + 未来两腿同步。

## 项目简介

基于 Zephyr RTOS 的 DM-J4310-2EC 关节电机弹簧阻尼控制固件。

### 硬件平台
- **MCU**: STM32F407IG (DJI RoboMaster Type-C 板)
- **电机**: DM-J4310-2EC 减速电机 × 4
- **CAN 总线**: CAN1 (电机 1,2) + CAN2 (电机 3,4), 均为 1Mbps
- **传动**: 同步带结构 (需低力控制,避免损坏)
- **调试器**: Horco CMSIS-DAP v2 (OpenOCD)

### 功能说明
上电后自动执行:
1. **CAN 初始化** — 初始化 CAN1/CAN2 总线
2. **Bringup 序列** — 逐电机执行 DISABLE → ZERO → 正常模式
3. **Home 捕获** — bringup 完成后记录各电机当前位置作为弹簧原点
4. **弹簧阻尼保持** — 低刚度 MIT 阻抗控制, 可手动转动, 松手回弹
5. **侧边检测** — 实时识别被推动的是左腿还是右腿

**控制律 (MIT 模式):**
```
torque = KP * (home_pos - pos_actual) + KD * (0 - vel_actual)
```
- KP 提供回弹力 (弹簧), KD 提供运动阻尼 (手感顺滑)
- 低 KP 值确保同步带结构安全, 可手动推动

### 电机映射
| 电机编号 | CAN 总线 | 关节 | 所属腿 |
|---------|---------|------|--------|
| 1 | CAN1 | 左髋 | 左腿 |
| 2 | CAN1 | 左膝 | 左腿 |
| 3 | CAN2 | 右髋 | 右腿 |
| 4 | CAN2 | 右膝 | 右腿 |

### 控制参数
| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| KP | 10 | 0-500 | 弹簧刚度, 越大回弹越有力 |
| KD | 0.5 | 0-5 | 速度阻尼, 越大手感越"粘稠" |

> 可通过 OpenOCD 实时修改 `cmd_kp` / `cmd_kd` / `cmd_target_offset[0..3]`
> 侧边检测阈值: `SIDE_DETECT_THRESHOLD_RAD` = 0.03 rad (~1.7°)

### 手感调节指南

| 手感 | KP | KD | 效果 |
|------|-----|-----|------|
| 轻弹簧 | 5 | 0.2 | 很轻的回弹, 容易转动 |
| 中等弹簧 (默认) | 10 | 0.5 | 明显回弹, 需一定力转动 |
| 强弹簧 | 20 | 1.0 | 有力回弹, 较难转动 |
| 阻尼模式 | 0 | 1.5 | 纯阻尼, 无回弹 (像调音量旋钮) |
| 棘轮模拟 | 动态切换 | — | 需额外逻辑 (TODO) |

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
(gdb) set var cmd_kp = 15
(gdb) set var cmd_kd = 0.8
(gdb) set var cmd_target_offset[0] = 0.1

# 查看侧边检测状态
(gdb) p/d cmd_side_active[0]
(gdb) p/d cmd_side_active[1]
```

### 项目结构
```
Unit1/
  boards/dji/robomaster_c/   # 板级定义 (DTS, Kconfig)
  src/
    dm4310_motor.h           # DM4310 MIT 协议驱动接口
    dm4310_motor.c           # DM4310 MIT 协议驱动实现
    main.c                   # 弹簧阻尼控制 + 侧边检测
  CMakeLists.txt             # Zephyr 构建配置
  prj.conf                   # Kconfig (CAN + PRINTK)
  docs/Agent/                # 调试经验文档
```

### 未来规划
- [ ] 左右腿同步控制 (左腿带动右腿或反之)
- [ ] 多种触觉模式切换 (弹簧/阻尼/棘轮/惯性滚动)
- [ ] 动态回弹 (松手后以可控速度回到原点)
- [ ] 双腿耦合运动 (行走步态)
