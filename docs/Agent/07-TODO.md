# 待办清单

## 已完成

- [x] ENABLE/DISABLE 尾字节定义修正 (chassis_base 版本)
- [x] 反馈帧字节布局修正
- [x] Bringup 序列: MIT寄存器写入 → DISABLE → ZERO → ENABLE → DONE
- [x] `put_le_u32` / `dm4310_write_u32_register` 移出 #if 块, 修正 CAN 总线路由
- [x] RST 引脚连接 (上电复位问题)
- [x] bringup 可靠性: 四电机全部进入 MIT 模式

## 待处理

- [ ] **验证 MIT 位置闭环**: 外力推开后是否自动回复到目标位置
- [ ] **调优 KP/KD 参数**: 确保回复力足够且不振荡
- [ ] **整机断电重启验证**: 全流程稳定
