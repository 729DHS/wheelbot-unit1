#include "dm4310_motor.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

K_MSGQ_DEFINE(dm4310_can_rx_msgq, sizeof(struct can_frame), 64, 4);

#define DM4310_OUTPUT_ENABLED 1
#define DM4310_CAN1_HOME_ENABLED 0
#define DM4310_HOME_START_IDX 0U
#define DM4310_NORMAL_OUTPUT_ENABLED 1
#define DM4310_CAN1_HOME_USE_FLASH 0
#define DM4310_CAN1_HOME_NVS_ID 1U
#define DM4310_CAN1_HOME_MAGIC 0x4331484dU
#define DM4310_CAN1_HOME_VERSION 4U
#define DM4310_CAN1_HOME_ENABLE_TICKS 200U
#define DM4310_CAN1_HOME_MODE_TICKS 100U
#define DM4310_CAN1_HOLD_KP 100.0f
#define DM4310_CAN1_HOLD_KD 2.2f
#define DM4310_CAN2_HOLD_KP 100.0f
#define DM4310_CAN2_HOLD_KD 2.2f
#define DM4310_HOME_DEADBAND_RAD 0.015f
#define DM4310_CTRL_MODE_MIT 1U
#define DM4310_REG_CTRL_MODE 0x0AU
#define DM4310_REG_WRITE_ID 0x7FFU

struct dm4310_can1_home_record {
	uint32_t magic;
	uint32_t version;
	float pos_rad[DM4310_HOME_MOTOR_COUNT];
};

volatile struct dm4310_driver g_dm4310 = {
	.magic = DM4310_MAGIC,
};

static const struct device *can1_dev;
static const struct device *can2_dev;

static int dm4310_send_raw(uint16_t std_id, const uint8_t data[8]);

static inline const struct device *motor_idx_to_can(uint8_t motor_idx)
{
	return (motor_idx < DM4310_MOTORS_ON_CAN1) ? can1_dev : can2_dev;
}

static void put_le_u32(uint8_t data[4], uint32_t value)
{
	data[0] = (uint8_t)value;
	data[1] = (uint8_t)(value >> 8);
	data[2] = (uint8_t)(value >> 16);
	data[3] = (uint8_t)(value >> 24);
}

static int dm4310_write_u32_register(uint8_t motor_id, uint8_t reg, uint32_t value)
{
	uint8_t data[8];
	struct can_frame frame;
	int ret;

	data[0] = motor_id;
	data[1] = 0U;
	data[2] = 0x55U;
	data[3] = reg;
	put_le_u32(&data[4], value);

	memset(&frame, 0, sizeof(frame));
	frame.id = DM4310_REG_WRITE_ID;
	frame.dlc = 8;
	memcpy(frame.data, data, 8);

	ret = can_send(motor_idx_to_can(motor_id - 1U), &frame, K_MSEC(2), NULL, NULL);
	g_dm4310.last_send_ret = ret;

	return ret;
}

static uint8_t dm4310_home_motors_online(void)
{
	for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
		if (g_dm4310.motor[motor].online == 0U) {
			return 0U;
		}
	}
	return 1U;
}

static float dm4310_home_kp(uint8_t motor_idx)
{
	return motor_idx < DM4310_MOTORS_ON_CAN1 ? DM4310_CAN1_HOLD_KP : DM4310_CAN2_HOLD_KP;
}

static float dm4310_home_kd(uint8_t motor_idx)
{
	return motor_idx < DM4310_MOTORS_ON_CAN1 ? DM4310_CAN1_HOLD_KD : DM4310_CAN2_HOLD_KD;
}

static uint8_t dm4310_home_in_deadband(uint8_t motor_idx)
{
	float target = g_dm4310.can1_home_pos_rad[motor_idx];
	float actual = g_dm4310.motor[motor_idx].pos_rad;

	return fabsf(target - actual) < DM4310_HOME_DEADBAND_RAD ? 1U : 0U;
}

static float clampf(float v, float lo, float hi)
{
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span, offset, scaled;
	uint32_t levels;

	x = clampf(x, x_min, x_max);
	span = x_max - x_min;
	offset = x - x_min;
	levels = ((uint32_t)1u << (uint32_t)bits) - 1u;
	scaled = offset * (float)levels / span;
	return (uint16_t)(scaled + 0.5f);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span;
	uint32_t levels;

	span = x_max - x_min;
	levels = ((uint32_t)1u << (uint32_t)bits) - 1u;
	return ((float)x_int * span / (float)levels) + x_min;
}

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

	motor_id = (uint8_t)(data[0] & 0x0FU);
	if (motor_id == 0U || motor_id > DM4310_MOTOR_COUNT) {
		return false;
	}

	p_int = ((int)data[1] << 8) | (int)data[2];
	v_int = ((int)data[3] << 4) | ((int)data[4] >> 4);
	t_int = (((int)data[4] & 0x0F) << 8) | (int)data[5];

	out->motor_state = (uint8_t)(data[0] >> 4);
	out->pos_rad = uint_to_float(p_int, DM4310_P_MIN, DM4310_P_MAX, 16);
	out->vel_radps = uint_to_float(v_int, DM4310_V_MIN, DM4310_V_MAX, 12);
	out->torque_nm = uint_to_float(t_int, DM4310_T_MIN, DM4310_T_MAX, 12);
	out->mos_temp = data[6];
	out->coil_temp = data[7];
	out->rx_count++;
	out->last_ms = k_uptime_get_32();
	out->online = 1U;

	return true;
}

static int dm4310_send_raw(uint16_t std_id, const uint8_t data[8])
{
	struct can_frame frame;
	int ret;
	uint8_t motor_idx = (uint8_t)(std_id - DM4310_CAN_TX_ID_BASE);

	memset(&frame, 0, sizeof(frame));
	frame.id = std_id;
	frame.dlc = 8;
	memcpy(frame.data, data, 8);

	ret = can_send(motor_idx_to_can(motor_idx), &frame, K_MSEC(2), NULL, NULL);
	g_dm4310.last_send_ret = ret;

	return ret;
}

static void drain_rx(void)
{
	struct can_frame frame;

	while (k_msgq_get(&dm4310_can_rx_msgq, &frame, K_NO_WAIT) == 0) {
		struct dm4310_motor_status status = { 0 };
		uint8_t mid;

		if (frame.data[2] == 0x55U || frame.data[2] == 0x33U || frame.data[2] == 0xAAU) {
			continue;
		}

		if (!dm4310_decode_feedback(frame.data, &status)) {
			continue;
		}

		mid = (uint8_t)(frame.data[0] & 0x0FU);
		if (mid >= 1U && mid <= DM4310_MOTOR_COUNT) {
			g_dm4310.motor[mid - 1U] = status;
		}
	}
}

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

static int dm4310_init_bus(const struct device *dev)
{
	const struct can_filter filter = {
		.id = 0x000U,
		.mask = 0x000U,
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

	g_dm4310.can1_home_load_ret = -ENOTSUP;
#if DM4310_CAN1_HOME_USE_FLASH != 0
	g_dm4310.can1_home_load_ret = dm4310_nvs_init();
	if (g_dm4310.can1_home_load_ret == 0) {
		g_dm4310.can1_home_load_ret = dm4310_load_can1_home();
	}
#endif

	g_dm4310.ready = 1U;
	g_dm4310.hold_kp = 90.0f;
	g_dm4310.hold_kd = 1.8f;

	return 0;
}

void dm4310_poll_rx(void)
{
	drain_rx();
	refresh_online_mask();

#if DM4310_CAN1_HOME_ENABLED != 0
	if (g_dm4310.can1_home_valid == 0U && g_dm4310.can1_home_enable_ticks == 0U) {
		g_dm4310.can1_home_enable_ticks = DM4310_CAN1_HOME_MODE_TICKS +
						       DM4310_CAN1_HOME_ENABLE_TICKS;
	}

#endif
}

int dm4310_tick(void)
{
	uint8_t id;
	uint8_t idx;
	uint32_t step;
	uint32_t tick;
	uint8_t data[8];
	int ret = 0;

	drain_rx();
	refresh_online_mask();

	#if DM4310_OUTPUT_ENABLED == 0
	dm4310_hold_reset();
	g_dm4310.loops++;
	return 0;
	#endif

#if DM4310_CAN1_HOME_ENABLED != 0
	if (g_dm4310.can1_home_valid != 0U) {
		g_dm4310.can1_home_active = 1U;
		for (int n = 0; n < 2; n++) {
			if (g_dm4310.tx_index >= DM4310_MOTOR_COUNT) {
				g_dm4310.tx_index = 0;
			}
			idx = g_dm4310.tx_index;
			if (idx >= DM4310_HOME_START_IDX && idx < DM4310_HOME_MOTOR_COUNT) {
				if (g_dm4310.can1_home_enable_ticks > 0U) {
					dm4310_pack_special(DM4310_CMD_ENABLE_TAIL, data);
					g_dm4310.can1_home_enable_ticks--;
				} else if (dm4310_home_in_deadband(idx) != 0U) {
					dm4310_pack_control(g_dm4310.motor[idx].pos_rad, 0.0f,
							    0.0f, dm4310_home_kd(idx), 0.0f, data);
				} else {
					dm4310_pack_control(g_dm4310.can1_home_pos_rad[idx], 0.0f,
							    dm4310_home_kp(idx), dm4310_home_kd(idx), 0.0f, data);
				}
			} else {
				dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
			}
			ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
			g_dm4310.tx_index++;
		}
		g_dm4310.loops++;
		return ret;
	}

	for (int n = 0; n < 2; n++) {
		if (g_dm4310.tx_index >= DM4310_MOTOR_COUNT) {
			g_dm4310.tx_index = 0;
		}
		idx = g_dm4310.tx_index;
		if (idx >= DM4310_HOME_START_IDX && idx < DM4310_HOME_MOTOR_COUNT) {
			if (g_dm4310.can1_home_enable_ticks > DM4310_CAN1_HOME_ENABLE_TICKS) {
				ret = dm4310_write_u32_register(idx + 1U, DM4310_REG_CTRL_MODE,
								DM4310_CTRL_MODE_MIT);
				g_dm4310.can1_home_enable_ticks--;
				g_dm4310.tx_index++;
				continue;
			}
			dm4310_pack_special(DM4310_CMD_ENABLE_TAIL, data);
			if (g_dm4310.can1_home_enable_ticks > 0U) {
				g_dm4310.can1_home_enable_ticks--;
			}
		} else {
			dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
		}
		ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
		g_dm4310.tx_index++;
	}
	g_dm4310.loops++;
	return ret;
#endif

#if DM4310_NORMAL_OUTPUT_ENABLED == 0
	dm4310_hold_reset();
	for (int n = 0; n < 2; n++) {
		if (g_dm4310.tx_index >= DM4310_MOTOR_COUNT) {
			g_dm4310.tx_index = 0;
		}
		idx = g_dm4310.tx_index;
		dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
		ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
		g_dm4310.tx_index++;
	}
	g_dm4310.loops++;
	return ret;
#endif

	if (g_dm4310.bringup_done) {
		/* Staggered normal control: 2 motors per tick */
		for (int n = 0; n < 2; n++) {
			if (g_dm4310.tx_index >= DM4310_MOTOR_COUNT) {
				g_dm4310.tx_index = 0;
			}
			idx = g_dm4310.tx_index;
			id = idx + 1U;

			if (g_dm4310.hold_updates > 0U) {
				dm4310_pack_control(g_dm4310.hold_pos_rad[idx],
						    0.0f, g_dm4310.hold_kp,
						    g_dm4310.hold_kd, 0.0f, data);
			} else {
				dm4310_pack_control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
						    data);
			}
			ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
			g_dm4310.tx_index++;
		}
		g_dm4310.loops++;
		return ret;
	}

	/* Bringup: MIT register write -> DISABLE -> ZERO -> ENABLE */
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		step = g_dm4310.bringup_step[i];
		tick = g_dm4310.bringup_tick[i];

		if (step >= 4U) {
			continue;
		}

		/* Step 0: write MIT mode register via StdId 0x7FF */
		if (step == 0U) {
			ret = dm4310_write_u32_register((uint8_t)(i + 1U),
				DM4310_REG_CTRL_MODE, DM4310_CTRL_MODE_MIT);
			g_dm4310.bringup_step[i] = 1U;
			g_dm4310.bringup_tick[i] = 0U;
			g_dm4310.loops++;
			return ret;
		}

		if (step == 1U) {
			dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
		} else if (step == 2U) {
			dm4310_pack_special(DM4310_CMD_ZERO_TAIL, data);
		} else {
			dm4310_pack_special(DM4310_CMD_ENABLE_TAIL, data);
		}

		ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + (uint16_t)i, data);
		tick++;
		if (tick >= DM4310_BRINGUP_TICKS) {
			g_dm4310.bringup_step[i] = step + 1U;
			if (g_dm4310.bringup_step[i] >= 4U) {
				continue;
			}
			g_dm4310.bringup_tick[i] = 0U;
		} else {
			g_dm4310.bringup_tick[i] = tick;
		}
		g_dm4310.loops++;
		return ret;
	}

	/* Check if all motors done */
	uint8_t all_done = 1U;
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		if (g_dm4310.bringup_step[i] < 4U) {
			all_done = 0U;
			break;
		}
	}
	g_dm4310.bringup_done = all_done;
	g_dm4310.loops++;
	return ret;
}

int dm4310_save_can1_home_current(void)
{
#if DM4310_CAN1_HOME_USE_FLASH != 0
	struct dm4310_can1_home_record record = {
		.magic = DM4310_CAN1_HOME_MAGIC,
		.version = DM4310_CAN1_HOME_VERSION,
	};
	struct dm4310_can1_home_record old_record;
	int ret;

	if (dm4310_nvs_ready == 0U) {
		ret = dm4310_nvs_init();
		if (ret < 0) {
			g_dm4310.can1_home_save_ret = ret;
			return ret;
		}
	}

	if (dm4310_home_motors_online() == 0U) {
		g_dm4310.can1_home_save_ret = -ENOTCONN;
		return g_dm4310.can1_home_save_ret;
	}

	ret = flash_read(dm4310_nvs.flash_device, dm4310_nvs.offset,
			 &old_record, sizeof(old_record));
	if (ret == 0 && old_record.magic == DM4310_CAN1_HOME_MAGIC &&
	    old_record.version == DM4310_CAN1_HOME_VERSION) {
		for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
			record.pos_rad[motor] = old_record.pos_rad[motor];
		}
	} else {
		for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
			record.pos_rad[motor] = g_dm4310.motor[motor].pos_rad;
		}
	}

	for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
		record.pos_rad[motor] = g_dm4310.motor[motor].pos_rad;
	}

	ret = flash_erase(dm4310_nvs.flash_device, dm4310_nvs.offset, dm4310_storage_sector_size);
	if (ret < 0) {
		g_dm4310.can1_home_save_ret = ret;
		return ret;
	}

	ret = flash_write(dm4310_nvs.flash_device, dm4310_nvs.offset, &record, sizeof(record));
	if (ret < 0) {
		g_dm4310.can1_home_save_ret = ret;
		return ret;
	}

	for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
		g_dm4310.can1_home_pos_rad[motor] = record.pos_rad[motor];
	}
	g_dm4310.can1_home_valid = 1U;
	g_dm4310.can1_home_enable_ticks = DM4310_CAN1_HOME_MODE_TICKS +
					       DM4310_CAN1_HOME_ENABLE_TICKS;
	g_dm4310.can1_home_active = 1U;
	g_dm4310.can1_home_auto_saved = 1U;
	g_dm4310.can1_home_save_ret = 0;

	return 0;
#else
	ARG_UNUSED(g_dm4310);
	g_dm4310.can1_home_save_ret = -ENOTSUP;
	return -ENOTSUP;
#endif
}

int dm4310_hold_positions(const float target[DM4310_MOTOR_COUNT])
{
#if DM4310_NORMAL_OUTPUT_ENABLED == 0
	ARG_UNUSED(target);
	g_dm4310.hold_updates = 0U;
	return -EACCES;
#else
	g_dm4310.hold_updates++;
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm4310.hold_pos_rad[i] = target[i];
	}
	return 0;
#endif
}

void dm4310_hold_reset(void)
{
	g_dm4310.hold_updates = 0;
	memset((void *)g_dm4310.hold_pos_rad, 0, sizeof(g_dm4310.hold_pos_rad));
}

void dm4310_stop_all(void)
{
	uint8_t data[8];

	dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		(void)dm4310_send_raw(DM4310_CAN_TX_ID_BASE + (uint16_t)i, data);
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
