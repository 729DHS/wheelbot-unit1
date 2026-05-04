/**
 * @file serial_debug.h
 * @brief 串口调试数据回传接口 (usart6, 115200 8N1)
 */

#ifndef SERIAL_DEBUG_H_
#define SERIAL_DEBUG_H_

#include <stdint.h>

/**
 * @brief 初始化调试串口 (usart6)
 * @return 0 成功, 负值错误码
 */
int serial_debug_init(void);

/**
 * @brief 格式化输出调试字符串 (非阻塞)
 * @param fmt 格式化字符串
 * @param ... 可变参数
 */
void serial_debug_printf(const char *fmt, ...);

/**
 * @brief 输出电机状态调试帧
 *
 * 格式: [tick_ms] M0:p=pos,v=vel,t=tor,f=fault,T=mos/coil M1:... M2:... M3:...\n
 *
 * @param tick_ms    系统运行时间 [ms]
 * @param motor_id   电机编号数组 (1-based, 长度 4)
 * @param pos_rad    位置数组 [rad]
 * @param vel_radps  速度数组 [rad/s]
 * @param torque_nm  扭矩数组 [Nm]
 * @param fault      故障码数组
 * @param mos_temp   MOS 温度数组 [℃]
 * @param coil_temp  线圈温度数组 [℃]
 */
void serial_debug_motor_frame(uint32_t tick_ms,
			      const uint8_t motor_id[4],
			      const float pos_rad[4],
			      const float vel_radps[4],
			      const float torque_nm[4],
			      const uint8_t fault[4],
			      const uint8_t mos_temp[4],
			      const uint8_t coil_temp[4]);

#endif /* SERIAL_DEBUG_H_ */
