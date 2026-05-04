#ifndef CHASSIS_BASE_H_
#define CHASSIS_BASE_H_

#include "rm_m3508.h"

#include <stdint.h>

#define CHASSIS_BASE_MAGIC 0x43484253

enum chassis_m3508_mode {
	CHASSIS_M3508_MODE_DISABLED = 0,
	CHASSIS_M3508_MODE_CURRENT = 1,
};

struct chassis_m3508_command {
	uint32_t seq;
	uint8_t mode;
	uint8_t reserved[3];
	int16_t current[RM_M3508_MOTOR_COUNT];
};

struct chassis_base_status {
	uint32_t magic;
	uint32_t init_done;
	uint32_t last_ms;
	uint32_t imu_loops;
	uint32_t control_loops;
	uint32_t imu_last_ms;
	uint32_t control_last_ms;
	int32_t sbus_init_ret;
	int32_t lk_init_ret;
	int32_t m3508_init_ret;
	int32_t imu_init_ret;
	int32_t imu_update_ret;
	int32_t lk_poll_ret;
	int32_t joint_hold_ret;
	int32_t joint_run_ret;
	int32_t m3508_apply_ret;
	uint32_t sbus_seq;
	uint32_t lk_online_mask;
	uint32_t lk_encoder_valid_mask;
	uint32_t m3508_online_mask;
	uint32_t sm_events;
	uint32_t sm_joint_hold_valid_mask;
	uint32_t balance_updates;
	uint8_t imu_ready;
	uint8_t sbus_connected;
	uint8_t sm_system_state;
	uint8_t sm_joint_state;
	uint8_t sm_chassis_state;
	uint8_t sm_fault;
	uint8_t balance_active;
	uint8_t sbus_ch5;
	uint8_t sbus_ch6;
	uint8_t sbus_ch8;
	int16_t balance_left_current;
	int16_t balance_right_current;
};

struct chassis_debug_override {
	uint32_t magic;
	uint8_t force_ch5;
	uint8_t force_ch6;
	uint8_t force_ch8;
	uint8_t force_sbus_connected;
	uint8_t force_rc;
	uint8_t reserved[3];
	int32_t rc_forward;
	int32_t rc_turn;
	uint8_t force_m3508_current;
	uint8_t reserved2[3];
	int16_t m3508_can2_current;
	int16_t m3508_can1_current;
	uint32_t x_axis_calib_seq;
	float x_axis_calib_x;
	uint32_t x_axis_calib_count;
	int32_t x_axis_calib_ret;
	uint32_t can1_home_save_seq;
	int32_t can1_home_save_ret;
};

extern volatile struct chassis_base_status g_chassis_base;
extern volatile struct chassis_debug_override g_chassis_debug_override;
extern volatile struct chassis_m3508_command g_chassis_m3508_cmd;

void chassis_base_init(void);

#endif
