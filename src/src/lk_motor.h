#ifndef LK_MOTOR_H_
#define LK_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

#define LK_MOTOR_MAGIC 0x4c4b4d31
#define LK_MOTOR_MAX_ID 4
#define LK_MOTOR_BUS_COUNT 2
#define LK_MOTOR_CAN_BASE_ID 0x140
#define LK_MOTOR_ONLINE_TIMEOUT_MS 1000
#define LK_MOTOR_HOLD_REQUIRED_MASK 0x0fU

enum lk_motor_cmd {
	LK_MOTOR_CMD_READ_STATUS_1 = 0x9a,
	LK_MOTOR_CMD_CLEAR_ERROR = 0x9b,
	LK_MOTOR_CMD_READ_STATUS_2 = 0x9c,
	LK_MOTOR_CMD_READ_STATUS_3 = 0x9d,
	LK_MOTOR_CMD_SHUTDOWN = 0x80,
	LK_MOTOR_CMD_RUN = 0x88,
	LK_MOTOR_CMD_STOP = 0x81,
	LK_MOTOR_CMD_TORQUE = 0xa1,
	LK_MOTOR_CMD_SPEED = 0xa2,
};

struct lk_motor_status {
	uint32_t tx_attempts;
	uint32_t last_tx_ms;
	int32_t last_tx_ret;
	uint32_t rx_count;
	uint32_t last_ms;
	uint32_t last_can_id;
	uint8_t last_data[8];
	uint8_t online;
	uint8_t last_cmd;
	uint8_t temperature;
	uint8_t motor_state;
	uint8_t error_state;
	uint8_t encoder_valid;
	uint8_t reserved;
	int16_t voltage_cV;
	int16_t bus_current_cA;
	int16_t iq_raw;
	int16_t speed_dps;
	uint16_t encoder;
	int16_t phase_a_raw;
	int16_t phase_b_raw;
	int16_t phase_c_raw;
};

struct lk_motor_bus_state {
	int32_t set_mode_ret;
	int32_t start_ret;
	int32_t filter_ret[2];
	uint32_t tx_count;
	uint32_t rx_count;
	uint32_t parsed_count;
	uint32_t last_rx_ms;
};

struct lk_motor_driver {
	uint32_t magic;
	uint32_t ready_mask;
	int32_t last_send_ret;
	uint32_t loops;
	uint32_t online_mask;
	uint32_t encoder_valid_mask;
	uint32_t last_tx_ms;
	uint8_t next_id;
	uint8_t hold_next_id;
	uint8_t last_tx_bus;
	uint8_t last_tx_id;
	uint8_t last_tx_cmd;
	uint32_t hold_updates;
	uint32_t hold_valid_mask;
	uint32_t hold_run_mask;
	int32_t last_hold_ret;
	uint32_t run_count;
	int32_t last_run_ret;
	uint16_t hold_target[LK_MOTOR_MAX_ID];
	int16_t hold_error[LK_MOTOR_MAX_ID];
	int16_t hold_speed_dps[LK_MOTOR_MAX_ID];
	struct lk_motor_bus_state bus[LK_MOTOR_BUS_COUNT];
	struct lk_motor_status motor[LK_MOTOR_MAX_ID];
};

extern volatile struct lk_motor_driver g_lk_motor;

int lk_motor_init(void);
void lk_motor_poll_rx(void);
int lk_motor_poll_one(void);
int lk_motor_read(uint8_t motor_id, enum lk_motor_cmd cmd);
int lk_motor_shutdown(uint8_t motor_id);
int lk_motor_run(uint8_t motor_id);
int lk_motor_run_all(void);
int lk_motor_stop(uint8_t motor_id);
void lk_motor_stop_all(void);
int lk_motor_set_speed_dps(uint8_t motor_id, int32_t speed_dps);
int lk_motor_hold_positions(const uint16_t target[LK_MOTOR_MAX_ID], uint32_t valid_mask);
void lk_motor_hold_reset(void);
bool lk_motor_is_online(uint8_t motor_id);
const volatile struct lk_motor_status *lk_motor_get(uint8_t motor_id);

void lk_motor_make_read_frame(uint8_t motor_id, uint8_t cmd, struct can_frame *frame);
void lk_motor_make_speed_frame(uint8_t motor_id, int32_t speed_dps, struct can_frame *frame);
bool lk_motor_parse_frame(const struct can_frame *frame, struct lk_motor_status motor[]);
void lk_motor_refresh_online(struct lk_motor_status motor[], uint32_t now_ms);

#endif
