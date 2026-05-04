/**
 * @file dm4310_motor.c
 * @brief DM-J4310-2EC 减速电机 CAN 总线驱动实现
 *
 * 实现基于《DM-J4310-2EC V1.1减速电机说明书》第6-7页的 CAN 通信协议。
 *
 * 帧格式速查:
 * - 反馈帧 (Motor→PC): D0=ID, D1=ERR, D2-D3=POS, D4-D5=VEL, D5-D6=T, D7=--
 * - MIT控制帧 (PC→Motor): D0-D1=P_des, D2-D3=V_des, D3-D4=Kp, D5-D6=Kd, D6-D7=T_ff
 * - 特殊命令帧: D0-D6=0xFF, D7=tai_byte
 */

#include "dm4310_motor.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/** @brief CAN 接收消息队列 (8 字节 CAN 帧 × 64 深度) */
K_MSGQ_DEFINE(dm4310_can_rx_msgq, sizeof(struct can_frame), 64, 4);

volatile struct dm4310_driver g_dm4310 = {
	.magic = DM4310_MAGIC,
};

static const struct device *can1_dev;
static const struct device *can2_dev;

/**
 * @brief 根据电机索引选择对应的 CAN 设备
 * @param motor_idx 电机索引 (0-based)
 * @return CAN 设备指针
 */
static inline const struct device *motor_idx_to_can(uint8_t motor_idx)
{
	return (motor_idx < DM4310_MOTORS_ON_CAN1) ? can1_dev : can2_dev;
}

/**
 * @brief 限幅函数
 * @param v   输入值
 * @param lo  下界
 * @param hi  上界
 * @return 限幅后的值
 */
static float clampf(float v, float lo, float hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

/**
 * @brief 浮点数转定点整数
 * @param x     浮点输入值
 * @param x_min 量程下限
 * @param x_max 量程上限
 * @param bits  定点位宽
 * @return 定点整数 (四舍五入)
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset;
	uint32_t levels = ((uint32_t)1u << (uint32_t)bits) - 1u;

	x = clampf(x, x_min, x_max);
	offset = x - x_min;
	return (uint16_t)((offset * (float)levels / span) + 0.5f);
}

/**
 * @brief 定点整数转浮点数
 * @param x_int 定点整数值
 * @param x_min 量程下限
 * @param x_max 量程上限
 * @param bits  定点位宽
 * @return 浮点值
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	uint32_t levels = ((uint32_t)1u << (uint32_t)bits) - 1u;

	return ((float)x_int * span / (float)levels) + x_min;
}

/* === 公共打包/解包函数 === */

void dm4310_pack_special(uint8_t tail, uint8_t data[8])
{
	memset(data, 0xFF, 7);
	data[7] = tail;
}

void dm4310_pack_control(float pos, float vel, float kp, float kd, float tor,
			 uint8_t data[8])
{
	uint16_t p_int, v_int, kp_int, kd_int, t_int;

	p_int = float_to_uint(pos, DM4310_P_MIN, DM4310_P_MAX, 16);
	v_int = float_to_uint(vel, DM4310_V_MIN, DM4310_V_MAX, 12);
	kp_int = float_to_uint(kp, DM4310_KP_MIN, DM4310_KP_MAX, 12);
	kd_int = float_to_uint(kd, DM4310_KD_MIN, DM4310_KD_MAX, 12);
	t_int = float_to_uint(tor, DM4310_T_MIN, DM4310_T_MAX, 12);

	/*
	 * MIT 控制帧 nibble 交叉打包 (datasheet section 2A):
	 * D0        = P_des[15:8]
	 * D1        = P_des[7:0]
	 * D2        = V_des[11:4]
	 * D3[7:4]   = V_des[3:0]
	 * D3[3:0]   = Kp[11:8]
	 * D4        = Kp[7:0]
	 * D5        = Kd[11:4]
	 * D6[7:4]   = Kd[3:0]
	 * D6[3:0]   = T_ff[11:8]
	 * D7        = T_ff[7:0]
	 */
	data[0] = (uint8_t)(p_int >> 8);
	data[1] = (uint8_t)(p_int & 0xFFU);
	data[2] = (uint8_t)(v_int >> 4);
	data[3] = (uint8_t)(((v_int & 0x0FU) << 4) | (uint8_t)(kp_int >> 8));
	data[4] = (uint8_t)(kp_int & 0xFFU);
	data[5] = (uint8_t)(kd_int >> 4);
	data[6] = (uint8_t)(((kd_int & 0x0FU) << 4) | (uint8_t)(t_int >> 8));
	data[7] = (uint8_t)(t_int & 0xFFU);
}

bool dm4310_decode_feedback(const uint8_t data[8],
			    struct dm4310_motor_status *out)
{
	uint8_t motor_id;
	int p_int, v_int, t_int;

	/*
	 * 反馈帧字节布局 (datasheet section 1):
	 * D0        = motor ID (full byte)
	 * D1        = ERR / fault code
	 * D2-D3     = POS (Int16, big-endian)
	 * D4        = VEL[11:4]
	 * D5[7:4]   = VEL[3:0]
	 * D5[3:0]   = T[11:8]
	 * D6        = T[7:0]
	 * D7        = reserved (温度由独立反馈帧承载: D0=T_MOS, D1=T_Rotor)
	 */
	motor_id = data[0];
	if (motor_id == 0U || motor_id > DM4310_MOTOR_COUNT) {
		return false;
	}

	p_int = ((int)data[2] << 8) | (int)data[3];

	/* VEL: 8 bits from D4 + 4 bits from D5 upper nibble */
	v_int = ((int)data[4] << 4) | ((int)data[5] >> 4);

	/* T:   4 bits from D5 lower nibble + 8 bits from D6 */
	t_int = (((int)data[5] & 0x0F) << 8) | (int)data[6];

	out->fault = data[1];
	/* T_MOS / T_Rotor not modified here -- they come from a separate
	   temperature feedback frame (datasheet 含义续 row: D0, D1). */

	out->pos_rad   = uint_to_float(p_int, DM4310_P_MIN, DM4310_P_MAX, 16);
	out->vel_radps = uint_to_float(v_int, DM4310_V_MIN, DM4310_V_MAX, 12);
	out->torque_nm = uint_to_float(t_int, DM4310_T_MIN, DM4310_T_MAX, 12);
	out->rx_count++;
	out->last_ms = k_uptime_get_32();
	out->online = 1U;

	return true;
}

/* === 内部辅助函数 === */

/**
 * @brief 发送原始 CAN 帧
 * @param std_id 标准帧 ID
 * @param data   8 字节数据
 * @return can_send() 返回值
 */
static int dm4310_send_raw(uint16_t std_id, const uint8_t data[8])
{
	struct can_frame frame;
	int ret;
	uint8_t motor_idx = (uint8_t)(std_id - DM4310_CAN_TX_ID_BASE);

	memset(&frame, 0, sizeof(frame));
	frame.id = std_id;
	frame.dlc = 8;
	memcpy(frame.data, data, 8);

	ret = can_send(motor_idx_to_can(motor_idx), &frame, K_MSEC(2),
		       NULL, NULL);
	g_dm4310.last_send_ret = ret;

	return ret;
}

/**
 * @brief 清空接收队列, 解码所有待处理反馈帧并更新电机状态
 */
static void drain_rx(void)
{
	struct can_frame frame;

	while (k_msgq_get(&dm4310_can_rx_msgq, &frame, K_NO_WAIT) == 0) {
		struct dm4310_motor_status status;
		uint8_t mid;

		if (!dm4310_decode_feedback(frame.data, &status)) {
			continue;
		}

		/* 新格式: D0 全字节 = motor ID */
		mid = frame.data[0];
		if (mid >= 1U && mid <= DM4310_MOTOR_COUNT) {
			g_dm4310.motor[mid - 1U] = status;
		}
	}
}

/**
 * @brief 刷新在线电机位掩码
 *
 * 超时阈值为 @ref DM4310_ONLINE_TIMEOUT_MS.
 */
static void refresh_online_mask(void)
{
	uint32_t mask = 0;
	uint32_t now = k_uptime_get_32();

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		if (g_dm4310.motor[i].rx_count == 0U ||
		    (now - g_dm4310.motor[i].last_ms) > DM4310_ONLINE_TIMEOUT_MS) {
			g_dm4310.motor[i].online = 0;
		}
		if (g_dm4310.motor[i].online) {
			mask |= BIT(i);
		}
	}
	g_dm4310.online_mask = mask;
}

/**
 * @brief 初始化单个 CAN 总线
 *
 * 配置 CAN 为 ONE_SHOT 模式并添加接收滤波器.
 * 反馈帧 ID 由调试助手设置 (默认 0x00), 滤波器匹配 0x10-0x17
 * (bit[12:3]==0b00010000, 容纳 8 个电机的状态/温度帧).
 *
 * @param dev CAN 设备指针
 * @return 0 成功, 负值 CAN 操作错误码
 */
static int dm4310_init_bus(const struct device *dev)
{
	const struct can_filter filter = {
		.id = 0x10U,
		.mask = 0x1FF8U,
		.flags = 0,
	};
	int ret;

	ret = can_stop(dev);
	if (ret < 0 && ret != -EALREADY && ret != -ENETDOWN) {
		return ret;
	}

	ret = can_set_mode(dev, CAN_MODE_ONE_SHOT);
	if (ret < 0) {
		return ret;
	}

	ret = can_start(dev);
	if (ret < 0) {
		return ret;
	}

	ret = can_add_rx_filter_msgq(dev, &dm4310_can_rx_msgq, &filter);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* === 公共接口函数 === */

int dm4310_init(void)
{
	memset((void *)&g_dm4310, 0, sizeof(g_dm4310));
	g_dm4310.magic = DM4310_MAGIC;

	can1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));
	if (!device_is_ready(can1_dev)) {
		g_dm4310.init_ret = -ENODEV;
		return -ENODEV;
	}

	can2_dev = DEVICE_DT_GET(DT_NODELABEL(can2));
	if (!device_is_ready(can2_dev)) {
		g_dm4310.init_ret = -ENODEV;
		return -ENODEV;
	}

	g_dm4310.init_ret = dm4310_init_bus(can1_dev);
	if (g_dm4310.init_ret < 0) {
		return g_dm4310.init_ret;
	}

	g_dm4310.init_ret = dm4310_init_bus(can2_dev);
	if (g_dm4310.init_ret < 0) {
		return g_dm4310.init_ret;
	}

	g_dm4310.ready = 1U;
	g_dm4310.hold_kp = 32.0f;
	g_dm4310.hold_kd = 1.0f;

	return 0;
}

void dm4310_poll_rx(void)
{
	drain_rx();
	refresh_online_mask();
}

int dm4310_tick(void)
{
	uint8_t idx;
	uint8_t data[8];
	int ret = 0;

	drain_rx();
	refresh_online_mask();

	if (g_dm4310.bringup_done) {
		/* 正常运行: 每次 tick 以交错方式发送 2 台电机 */
		for (int n = 0; n < 2; n++) {
			if (g_dm4310.tx_index >= DM4310_MOTOR_COUNT) {
				g_dm4310.tx_index = 0;
			}
			idx = g_dm4310.tx_index;

			if (g_dm4310.hold_updates > 0U) {
				dm4310_pack_control(
					g_dm4310.hold_pos_rad[idx],
					0.0f, g_dm4310.hold_kp,
					g_dm4310.hold_kd, 0.0f, data);
			} else {
				dm4310_pack_control(0.0f, 0.0f, 0.0f, 0.0f,
						    0.0f, data);
			}
			ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
			g_dm4310.tx_index++;
		}
		g_dm4310.loops++;
		return ret;
	}

	/* Bringup 阶段: 每次 tick 处理一台电机 (DISABLE → DONE) */
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		uint32_t step = g_dm4310.bringup_step[i];
		uint32_t tick = g_dm4310.bringup_tick[i];

		if (step >= 2U) {
			continue;
		}

		if (step == 0U) {
			dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
		} else {
			dm4310_pack_special(DM4310_CMD_ZERO_TAIL, data);
		}

		ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + (uint16_t)i,
				      data);
		tick++;
		if (tick >= DM4310_BRINGUP_TICKS) {
			/* DISABLE 完成后跳过 ZERO 步骤, 直接标记 DONE */
			g_dm4310.bringup_step[i] =
				(step == 0U) ? 2U : step + 1U;
			if (g_dm4310.bringup_step[i] >= 2U) {
				continue;
			}
			g_dm4310.bringup_tick[i] = 0U;
		} else {
			g_dm4310.bringup_tick[i] = tick;
		}
		g_dm4310.loops++;
		return ret;
	}

	/* 检查所有电机 bringup 是否完成 */
	uint8_t all_done = 1U;
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		if (g_dm4310.bringup_step[i] < 2U) {
			all_done = 0U;
			break;
		}
	}
	g_dm4310.bringup_done = all_done;
	g_dm4310.loops++;
	return ret;
}

int dm4310_hold_positions(const float target[DM4310_MOTOR_COUNT])
{
	g_dm4310.hold_updates++;
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm4310.hold_pos_rad[i] = target[i];
	}
	return 0;
}

void dm4310_hold_reset(void)
{
	g_dm4310.hold_updates = 0;
	memset(g_dm4310.hold_pos_rad, 0, sizeof(g_dm4310.hold_pos_rad));
}

void dm4310_stop_all(void)
{
	uint8_t data[8];

	dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		(void)dm4310_send_raw(DM4310_CAN_TX_ID_BASE + (uint16_t)i,
				      data);
	}
	dm4310_hold_reset();
}

bool dm4310_is_online(uint8_t motor_id)
{
	if (motor_id < 1U || motor_id > DM4310_MOTOR_COUNT) {
		return false;
	}
	return g_dm4310.motor[motor_id - 1U].online != 0U;
}

const volatile struct dm4310_motor_status *dm4310_get(uint8_t motor_id)
{
	if (motor_id < 1U || motor_id > DM4310_MOTOR_COUNT) {
		return NULL;
	}
	return &g_dm4310.motor[motor_id - 1U];
}
