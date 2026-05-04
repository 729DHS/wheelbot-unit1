#include "lk_motor.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define READ_STATUS1_DIV 20
#define LK_MOTOR_SPEED_CMD_SCALE 100
#define LK_MOTOR_HOLD_KP_MILLI_DPS_PER_COUNT 250
#define LK_MOTOR_HOLD_MAX_SPEED_DPS 1200
#define LK_MOTOR_HOLD_DEADBAND_COUNT 24

K_MSGQ_DEFINE(lk_can1_rx_msgq, sizeof(struct can_frame), 64, 4);
K_MSGQ_DEFINE(lk_can2_rx_msgq, sizeof(struct can_frame), 64, 4);

volatile struct lk_motor_driver g_lk_motor = {
	.magic = LK_MOTOR_MAGIC,
	.next_id = 1,
};

static const struct device *const can_dev[LK_MOTOR_BUS_COUNT] = {
	DEVICE_DT_GET(DT_NODELABEL(can1)),
	DEVICE_DT_GET(DT_NODELABEL(can2)),
};

static struct k_msgq *const can_rx_msgq[LK_MOTOR_BUS_COUNT] = {
	&lk_can1_rx_msgq,
	&lk_can2_rx_msgq,
};

static int16_t le16s(const uint8_t *data)
{
	return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint16_t le16u(const uint8_t *data)
{
	return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void put_le32(uint8_t *data, int32_t value)
{
	data[0] = (uint8_t)value;
	data[1] = (uint8_t)((uint32_t)value >> 8);
	data[2] = (uint8_t)((uint32_t)value >> 16);
	data[3] = (uint8_t)((uint32_t)value >> 24);
}

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
	if (value > max_value) {
		return max_value;
	}
	if (value < min_value) {
		return min_value;
	}

	return value;
}

static bool valid_motor_id(uint8_t motor_id)
{
	return motor_id >= 1U && motor_id <= LK_MOTOR_MAX_ID;
}

static uint8_t motor_bus(uint8_t motor_id)
{
	return motor_id <= 2U ? 0U : 1U;
}

static uint8_t first_motor_on_bus(uint8_t bus)
{
	return bus == 0U ? 1U : 3U;
}

static uint8_t next_poll_cmd(void)
{
	if ((g_lk_motor.loops % READ_STATUS1_DIV) == 0U) {
		return LK_MOTOR_CMD_READ_STATUS_1;
	}

	return LK_MOTOR_CMD_READ_STATUS_2;
}

static int start_can_bus(uint8_t bus)
{
	const struct device *dev = can_dev[bus];
	int ret;

	if (!device_is_ready(dev)) {
		g_lk_motor.bus[bus].start_ret = -ENODEV;
		return -ENODEV;
	}

	g_lk_motor.ready_mask |= BIT(bus);

	g_lk_motor.bus[bus].set_mode_ret = can_set_mode(dev, CAN_MODE_ONE_SHOT);
	if (g_lk_motor.bus[bus].set_mode_ret < 0) {
		g_lk_motor.bus[bus].set_mode_ret = can_set_mode(dev, CAN_MODE_NORMAL);
	}

	ret = can_start(dev);
	g_lk_motor.bus[bus].start_ret = ret;
	if (ret < 0 && ret != -EALREADY) {
		return ret;
	}

	return 0;
}

static int add_feedback_filters(uint8_t bus)
{
	const uint8_t first_id = first_motor_on_bus(bus);

	for (int i = 0; i < 2; i++) {
		const struct can_filter filter = {
			.id = LK_MOTOR_CAN_BASE_ID + first_id + i,
			.mask = CAN_STD_ID_MASK,
			.flags = 0,
		};
		int ret = can_add_rx_filter_msgq(can_dev[bus], can_rx_msgq[bus], &filter);

		g_lk_motor.bus[bus].filter_ret[i] = ret;
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static void drain_rx(uint8_t bus)
{
	struct can_frame frame;

	while (k_msgq_get(can_rx_msgq[bus], &frame, K_NO_WAIT) == 0) {
		g_lk_motor.bus[bus].rx_count++;
		g_lk_motor.bus[bus].last_rx_ms = k_uptime_get_32();

		if (lk_motor_parse_frame(&frame, (struct lk_motor_status *)g_lk_motor.motor)) {
			g_lk_motor.bus[bus].parsed_count++;
		}
	}
}

static void refresh_online_mask(void)
{
	uint32_t mask = 0;
	uint32_t encoder_mask = 0;

	lk_motor_refresh_online((struct lk_motor_status *)g_lk_motor.motor, k_uptime_get_32());

	for (int i = 0; i < LK_MOTOR_MAX_ID; i++) {
		if (g_lk_motor.motor[i].online != 0U) {
			mask |= BIT(i);
		}
		if (g_lk_motor.motor[i].online != 0U &&
		    g_lk_motor.motor[i].encoder_valid != 0U) {
			encoder_mask |= BIT(i);
		}
	}

	g_lk_motor.online_mask = mask;
	g_lk_motor.encoder_valid_mask = encoder_mask;
}

int lk_motor_init(void)
{
	int ret;

	memset((void *)&g_lk_motor, 0, sizeof(g_lk_motor));
	g_lk_motor.magic = LK_MOTOR_MAGIC;
	g_lk_motor.next_id = 1U;
	g_lk_motor.hold_next_id = 1U;

	for (int bus = 0; bus < LK_MOTOR_BUS_COUNT; bus++) {
		ret = start_can_bus(bus);
		if (ret < 0) {
			return ret;
		}

		ret = add_feedback_filters(bus);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

void lk_motor_poll_rx(void)
{
	drain_rx(0);
	drain_rx(1);
	refresh_online_mask();
}

int lk_motor_poll_one(void)
{
	uint8_t motor_id = g_lk_motor.next_id;
	uint8_t cmd = next_poll_cmd();
	int ret;

	ret = lk_motor_read(motor_id, cmd);
	lk_motor_poll_rx();

	g_lk_motor.next_id++;
	if (g_lk_motor.next_id > LK_MOTOR_MAX_ID) {
		g_lk_motor.next_id = 1U;
	}
	g_lk_motor.loops++;

	return ret;
}

int lk_motor_read(uint8_t motor_id, enum lk_motor_cmd cmd)
{
	struct can_frame frame;
	uint8_t bus;
	struct lk_motor_status *status;
	int ret;

	if (!valid_motor_id(motor_id)) {
		return -EINVAL;
	}

	bus = motor_bus(motor_id);
	status = (struct lk_motor_status *)&g_lk_motor.motor[motor_id - 1U];

	lk_motor_make_read_frame(motor_id, cmd, &frame);
	ret = can_send(can_dev[bus], &frame, K_MSEC(2), NULL, NULL);

	g_lk_motor.last_send_ret = ret;
	g_lk_motor.last_tx_ms = k_uptime_get_32();
	g_lk_motor.last_tx_bus = bus + 1U;
	g_lk_motor.last_tx_id = motor_id;
	g_lk_motor.last_tx_cmd = cmd;
	status->tx_attempts++;
	status->last_tx_ms = g_lk_motor.last_tx_ms;
	status->last_tx_ret = ret;

	if (ret == 0) {
		g_lk_motor.bus[bus].tx_count++;
	}

	return ret;
}

int lk_motor_shutdown(uint8_t motor_id)
{
	return lk_motor_read(motor_id, LK_MOTOR_CMD_SHUTDOWN);
}

int lk_motor_run(uint8_t motor_id)
{
	return lk_motor_read(motor_id, LK_MOTOR_CMD_RUN);
}

int lk_motor_run_all(void)
{
	int first_error = 0;

	g_lk_motor.run_count++;
	for (uint8_t id = 1U; id <= LK_MOTOR_MAX_ID; id++) {
		int ret = lk_motor_run(id);

		if (ret < 0 && first_error == 0) {
			first_error = ret;
		}
	}

	g_lk_motor.last_run_ret = first_error;
	return first_error;
}

int lk_motor_stop(uint8_t motor_id)
{
	return lk_motor_read(motor_id, LK_MOTOR_CMD_STOP);
}

void lk_motor_stop_all(void)
{
	for (uint8_t id = 1U; id <= LK_MOTOR_MAX_ID; id++) {
		(void)lk_motor_stop(id);
	}
	lk_motor_hold_reset();
}

int lk_motor_set_speed_dps(uint8_t motor_id, int32_t speed_dps)
{
	struct can_frame frame;
	uint8_t bus;
	struct lk_motor_status *status;
	int ret;

	if (!valid_motor_id(motor_id)) {
		return -EINVAL;
	}

	bus = motor_bus(motor_id);
	status = (struct lk_motor_status *)&g_lk_motor.motor[motor_id - 1U];

	lk_motor_make_speed_frame(motor_id, speed_dps, &frame);
	ret = can_send(can_dev[bus], &frame, K_MSEC(2), NULL, NULL);

	g_lk_motor.last_send_ret = ret;
	g_lk_motor.last_tx_ms = k_uptime_get_32();
	g_lk_motor.last_tx_bus = bus + 1U;
	g_lk_motor.last_tx_id = motor_id;
	g_lk_motor.last_tx_cmd = LK_MOTOR_CMD_SPEED;
	status->tx_attempts++;
	status->last_tx_ms = g_lk_motor.last_tx_ms;
	status->last_tx_ret = ret;

	if (ret == 0) {
		g_lk_motor.bus[bus].tx_count++;
	}

	return ret;
}

int lk_motor_hold_positions(const uint16_t target[LK_MOTOR_MAX_ID], uint32_t valid_mask)
{
	uint8_t send_id;
	uint8_t send_index;
	uint32_t active_mask = valid_mask & LK_MOTOR_HOLD_REQUIRED_MASK;
	uint32_t pending_run_mask;
	int ret;

	g_lk_motor.hold_updates++;
	g_lk_motor.hold_valid_mask = active_mask;
	g_lk_motor.hold_run_mask &= active_mask;

	for (uint8_t id = 1U; id <= LK_MOTOR_MAX_ID; id++) {
		const uint8_t index = id - 1U;
		int16_t error;
		int32_t speed_dps;

		g_lk_motor.hold_target[index] = target[index];

		if ((valid_mask & BIT(index)) == 0U || g_lk_motor.motor[index].online == 0U) {
			g_lk_motor.hold_error[index] = 0;
			g_lk_motor.hold_speed_dps[index] = 0;
			continue;
		}

		error = (int16_t)(target[index] - g_lk_motor.motor[index].encoder);
		speed_dps = ((int32_t)error * LK_MOTOR_HOLD_KP_MILLI_DPS_PER_COUNT) / 1000;
		if (error > -LK_MOTOR_HOLD_DEADBAND_COUNT &&
		    error < LK_MOTOR_HOLD_DEADBAND_COUNT) {
			speed_dps = 0;
		}
		speed_dps = clamp_i32(speed_dps, -LK_MOTOR_HOLD_MAX_SPEED_DPS,
				      LK_MOTOR_HOLD_MAX_SPEED_DPS);

		g_lk_motor.hold_error[index] = error;
		g_lk_motor.hold_speed_dps[index] = (int16_t)speed_dps;
	}

	pending_run_mask = active_mask & ~g_lk_motor.hold_run_mask;
	if (pending_run_mask != 0U) {
		for (uint8_t offset = 0U; offset < LK_MOTOR_MAX_ID; offset++) {
			uint8_t candidate = g_lk_motor.hold_next_id + offset;

			if (candidate > LK_MOTOR_MAX_ID) {
				candidate -= LK_MOTOR_MAX_ID;
			}
			if ((pending_run_mask & BIT(candidate - 1U)) == 0U) {
				continue;
			}

			ret = lk_motor_run(candidate);
			g_lk_motor.run_count++;
			g_lk_motor.last_run_ret = ret;
			if (ret == 0) {
				g_lk_motor.hold_run_mask |= BIT(candidate - 1U);
			}
			g_lk_motor.hold_next_id = candidate + 1U;
			if (g_lk_motor.hold_next_id > LK_MOTOR_MAX_ID) {
				g_lk_motor.hold_next_id = 1U;
			}
			g_lk_motor.last_hold_ret = ret;
			return ret;
		}
	}

	send_id = g_lk_motor.hold_next_id;
	if (!valid_motor_id(send_id)) {
		send_id = 1U;
	}
	send_index = send_id - 1U;
	ret = lk_motor_set_speed_dps(send_id, g_lk_motor.hold_speed_dps[send_index]);
	g_lk_motor.hold_next_id = send_id + 1U;
	if (g_lk_motor.hold_next_id > LK_MOTOR_MAX_ID) {
		g_lk_motor.hold_next_id = 1U;
	}

	g_lk_motor.last_hold_ret = ret;
	return ret;
}

void lk_motor_hold_reset(void)
{
	g_lk_motor.hold_valid_mask = 0U;
	g_lk_motor.hold_run_mask = 0U;
	g_lk_motor.last_hold_ret = 0;
	g_lk_motor.hold_next_id = 1U;
	memset((void *)g_lk_motor.hold_target, 0, sizeof(g_lk_motor.hold_target));
	memset((void *)g_lk_motor.hold_error, 0, sizeof(g_lk_motor.hold_error));
	memset((void *)g_lk_motor.hold_speed_dps, 0, sizeof(g_lk_motor.hold_speed_dps));
}

bool lk_motor_is_online(uint8_t motor_id)
{
	if (!valid_motor_id(motor_id)) {
		return false;
	}

	return g_lk_motor.motor[motor_id - 1U].online != 0U;
}

const volatile struct lk_motor_status *lk_motor_get(uint8_t motor_id)
{
	if (!valid_motor_id(motor_id)) {
		return NULL;
	}

	return &g_lk_motor.motor[motor_id - 1U];
}

void lk_motor_make_read_frame(uint8_t motor_id, uint8_t cmd, struct can_frame *frame)
{
	memset(frame, 0, sizeof(*frame));

	frame->id = LK_MOTOR_CAN_BASE_ID + motor_id;
	frame->dlc = 8;
	frame->flags = 0;
	frame->data[0] = cmd;
}

void lk_motor_make_speed_frame(uint8_t motor_id, int32_t speed_dps, struct can_frame *frame)
{
	memset(frame, 0, sizeof(*frame));

	frame->id = LK_MOTOR_CAN_BASE_ID + motor_id;
	frame->dlc = 8;
	frame->flags = 0;
	frame->data[0] = LK_MOTOR_CMD_SPEED;
	put_le32(&frame->data[4], speed_dps * LK_MOTOR_SPEED_CMD_SCALE);
}

bool lk_motor_parse_frame(const struct can_frame *frame, struct lk_motor_status motor[])
{
	struct lk_motor_status *status;
	uint8_t motor_id;

	if ((frame->flags & CAN_FRAME_IDE) != 0U || frame->dlc != 8U ||
	    frame->id <= LK_MOTOR_CAN_BASE_ID ||
	    frame->id > (LK_MOTOR_CAN_BASE_ID + LK_MOTOR_MAX_ID)) {
		return false;
	}

	motor_id = frame->id - LK_MOTOR_CAN_BASE_ID;
	status = &motor[motor_id - 1];

	status->rx_count++;
	status->last_ms = k_uptime_get_32();
	status->last_can_id = frame->id;
	memcpy(status->last_data, frame->data, sizeof(status->last_data));
	status->online = 1;
	status->last_cmd = frame->data[0];

	switch (frame->data[0]) {
	case LK_MOTOR_CMD_READ_STATUS_1:
	case LK_MOTOR_CMD_CLEAR_ERROR:
		status->temperature = frame->data[1];
		status->voltage_cV = le16s(&frame->data[2]);
		status->bus_current_cA = le16s(&frame->data[4]);
		status->motor_state = frame->data[6];
		status->error_state = frame->data[7];
		break;
	case LK_MOTOR_CMD_READ_STATUS_2:
	case LK_MOTOR_CMD_TORQUE:
	case LK_MOTOR_CMD_SPEED:
		status->temperature = frame->data[1];
		status->iq_raw = le16s(&frame->data[2]);
		status->speed_dps = le16s(&frame->data[4]);
		status->encoder = le16u(&frame->data[6]);
		status->encoder_valid = 1U;
		break;
	case LK_MOTOR_CMD_READ_STATUS_3:
		status->temperature = frame->data[1];
		status->phase_a_raw = le16s(&frame->data[2]);
		status->phase_b_raw = le16s(&frame->data[4]);
		status->phase_c_raw = le16s(&frame->data[6]);
		break;
	default:
		break;
	}

	return true;
}

void lk_motor_refresh_online(struct lk_motor_status motor[], uint32_t now_ms)
{
	for (int i = 0; i < LK_MOTOR_MAX_ID; i++) {
		if (motor[i].rx_count == 0U || now_ms - motor[i].last_ms > LK_MOTOR_ONLINE_TIMEOUT_MS) {
			motor[i].online = 0;
			motor[i].encoder_valid = 0;
		}
	}
}
