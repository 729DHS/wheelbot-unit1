# 待办清单 — Unit1

## 当前分支: feature/spring-damper-sync

## 已完成

- [x] ENABLE/DISABLE 尾字节定义修正 (chassis_base 版本)
- [x] 反馈帧字节布局修正
- [x] Bringup 序列: MIT寄存器写入 → DISABLE → ZERO → ENABLE → DONE
- [x] `hold_kp`/`hold_kd` 改为 per-motor 数组 (支持独立增益)
- [x] 弹簧阻尼 MIT 控制 (KP=10, KD=0.5, 低力可手转)
- [x] 侧边检测: 滞回(ON=0.08/OFF=0.03rad) + 80ms去抖动
- [x] CONFIG_CBPRINTF_FP_SUPPORT 浮点打印
- [x] 电机映射确认: CAN1=左腿(motor1,2), CAN2=右腿(motor3,4)

## 当前问题

- [ ] **极坐标 r 方向回弹差**: 电机角度弹簧阻尼好, 但五连杆末端(r,φ)中 r 回不到原位
  - 原因: 五连杆耦合非线性映射, 电机角度回弹 ≠ 末端极坐标回弹
  - 需五连杆正逆解才能做极坐标空间弹簧阻尼
- [ ] **手感调优**: KP/KD 最佳值待定

## 未来规划

- [ ] 左右腿同步控制 (推动一条腿 → 另一条跟随)
- [ ] 多种触觉模式 (弹簧/阻尼/棘轮/惯性)
- [ ] 五连杆运动学正逆解 (电机角度 ↔ 末端 r,φ)
- [ ] 轮毂电机 PID 平衡 (与 Unit2 协同)

## 参考

- Unit2 (master): 关节锁定 + 轮毂PID平衡小车
- docs/Agent/08-弹簧阻尼极坐标发现.md
