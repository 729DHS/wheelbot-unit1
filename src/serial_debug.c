/**
 * @file serial_debug.c
 * @brief 串口调试数据回传实现 (usart6, 115200 8N1, 轮询 TX)
 */

#include "serial_debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

/** @brief 调试输出缓冲区大小 */
#define SERIAL_DEBUG_BUF_SIZE 256

/** @brief 获取 usart6 设备节点 */
#define SERIAL_DEBUG_UART DT_NODELABEL(usart6)

static const struct device *serial_dev;

/**
 * @brief 初始化调试串口
 * @return 0 成功, -ENODEV 设备不存在
 */
int serial_debug_init(void)
{
	serial_dev = DEVICE_DT_GET(SERIAL_DEBUG_UART);
	if (!device_is_ready(serial_dev)) {
		return -ENODEV;
	}
	return 0;
}

/**
 * @brief 格式化输出调试字符串
 *
 * 使用轮询 TX 逐字节发送, 非阻塞但会占用 CPU 直到发送完成.
 *
 * @param fmt 格式化字符串
 * @param ... 可变参数
 */
void serial_debug_printf(const char *fmt, ...)
{
	char buf[SERIAL_DEBUG_BUF_SIZE];
	va_list args;
	int len;

	if (serial_dev == NULL || !device_is_ready(serial_dev)) {
		return;
	}

	va_start(args, fmt);
	len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len <= 0) {
		return;
	}

	if ((size_t)len >= sizeof(buf)) {
		len = (int)(sizeof(buf) - 1);
	}

	for (int i = 0; i < len; i++) {
		uart_poll_out(serial_dev, (unsigned char)buf[i]);
	}
}

/**
 * @brief 输出电机状态调试帧
 *
 * 紧凑格式, 一行包含全部 4 台电机的关键数据.
 * 示例:
 * [12345] 0:p1.234v0.012t0.500f00T35/36 1:p-1.234v-0.012t-0.500f00T35/36 ...
 *
 * @param tick_ms    系统运行时间 [ms]
 * @param motor_id   电机编号数组 [4]
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
			      const uint8_t coil_temp[4])
{
	char buf[SERIAL_DEBUG_BUF_SIZE];
	int off;

	off = snprintf(buf, sizeof(buf), "[%u]", tick_ms);

	for (int i = 0; i < 4; i++) {
		off += snprintf(buf + off, sizeof(buf) - (size_t)off,
				" %u:p%+.3fv%+.3ft%+.3ff%02xT%d/%d",
				(unsigned int)motor_id[i],
				(double)pos_rad[i],
				(double)vel_radps[i],
				(double)torque_nm[i],
				(unsigned int)fault[i],
				(int)mos_temp[i],
				(int)coil_temp[i]);
		if (off >= (int)sizeof(buf) - 2) {
			break;
		}
	}

	off += snprintf(buf + off, sizeof(buf) - (size_t)off, "\r\n");

	if (off > (int)sizeof(buf)) {
		off = (int)(sizeof(buf) - 1);
	}

	for (int i = 0; i < off; i++) {
		uart_poll_out(serial_dev, (unsigned char)buf[i]);
	}
}
