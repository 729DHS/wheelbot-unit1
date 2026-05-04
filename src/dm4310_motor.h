#ifndef DM4310_MOTOR_H_
#define DM4310_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

#define DM4310_MAGIC 0x444d3433
#define DM4310_MOTOR_COUNT 4
#define DM4310_HOME_MOTOR_COUNT 4
#define DM4310_CAN_TX_ID_BASE 1U
#define DM4310_MASTER_ID_BASE 0x11U
#define DM4310_MOTORS_ON_CAN1  2U
#define DM4310_ONLINE_TIMEOUT_MS 500

/* MIT protocol range limits */
#define DM4310_P_MIN  (-12.5f)
#define DM4310_P_MAX  12.5f
#define DM4310_V_MIN  (-30.0f)
#define DM4310_V_MAX  30.0f
#define DM4310_KP_MIN 0.0f
#define DM4310_KP_MAX 500.0f
#define DM4310_KD_MIN 0.0f
#define DM4310_KD_MAX 5.0f
#define DM4310_T_MIN  (-10.0f)
#define DM4310_T_MAX  10.0f

/* MIT protocol special command tails */
#define DM4310_CMD_ENABLE_TAIL  0xFC
#define DM4310_CMD_DISABLE_TAIL 0xFD
#define DM4310_CMD_ZERO_TAIL    0xFE

/* Bringup ticks per step */
#define DM4310_BRINGUP_TICKS 10

struct dm4310_motor_status {
	uint32_t rx_count;
	uint32_t last_ms;
	uint8_t online;
	uint8_t motor_state;
	uint8_t mos_temp;
	uint8_t coil_temp;
	float pos_rad;
	float vel_radps;
	float torque_nm;
};

struct dm4310_driver {
	uint32_t magic;
	uint32_t ready;
	int32_t init_ret;
	int32_t last_send_ret;
	uint32_t loops;
	uint32_t online_mask;
	uint32_t bringup_step[DM4310_MOTOR_COUNT];
	uint32_t bringup_tick[DM4310_MOTOR_COUNT];
	uint8_t bringup_done;
	uint8_t tx_index;
	uint8_t can1_home_valid;
	uint8_t can1_home_auto_saved;
	uint8_t can1_home_active;
	uint16_t can1_home_enable_ticks;
	int32_t can1_home_load_ret;
	int32_t can1_home_save_ret;
	float can1_home_pos_rad[DM4310_HOME_MOTOR_COUNT];
	uint32_t hold_updates;
	float hold_kp[DM4310_MOTOR_COUNT];
	float hold_kd[DM4310_MOTOR_COUNT];
	float hold_pos_rad[DM4310_MOTOR_COUNT];
	struct dm4310_motor_status motor[DM4310_MOTOR_COUNT];
};

extern volatile struct dm4310_driver g_dm4310;

int dm4310_init(void);
void dm4310_poll_rx(void);
int dm4310_tick(void);
int dm4310_hold_positions(const float target[DM4310_MOTOR_COUNT]);
int dm4310_save_can1_home_current(void);
void dm4310_hold_reset(void);
void dm4310_stop_all(void);
bool dm4310_is_online(uint8_t motor_id);
const volatile struct dm4310_motor_status *dm4310_get(uint8_t motor_id);

/* Packing helpers (exposed for testing) */
void dm4310_pack_special(uint8_t tail, uint8_t data[8]);
void dm4310_pack_control(float pos, float vel, float kp, float kd, float tor, uint8_t data[8]);
bool dm4310_decode_feedback(const uint8_t data[8], struct dm4310_motor_status *out);

#endif
