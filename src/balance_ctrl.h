#ifndef BALANCE_CTRL_H_
#define BALANCE_CTRL_H_

#include <stdint.h>

#define BALANCE_CTRL_MAGIC 0x42414c31
#define BALANCE_CTRL_WHEEL_COUNT 2

struct balance_ctrl_input {
	uint32_t now_ms;
	float dt_s;
	uint8_t enabled;
	uint8_t position_hold_enabled;
	int32_t rc_forward;
	int32_t rc_turn;
	float pitch_deg;
	float pitch_rate_dps;
	uint8_t wheel_online[BALANCE_CTRL_WHEEL_COUNT];
	uint16_t wheel_angle_raw[BALANCE_CTRL_WHEEL_COUNT];
	int16_t wheel_speed_rpm[BALANCE_CTRL_WHEEL_COUNT];
};

struct balance_ctrl_output {
	uint8_t active;
	int16_t wheel_current[BALANCE_CTRL_WHEEL_COUNT];
};

struct balance_ctrl_pd {
	float kp;
	float kd;
	float output_limit;
};

struct balance_ctrl_pi {
	float kp;
	float ki;
	float integral_limit;
	float output_limit;
};

struct balance_ctrl_state {
	uint32_t magic;
	uint32_t update_count;
	uint32_t last_ms;
	uint8_t enabled;
	uint8_t captured;
	uint8_t reserved[2];
	float pitch_zero_deg;
	float pitch_deg;
	float pitch_rate_dps;
	float pitch_target_deg;
	float pitch_error_deg;
	float pitch_rate_error_dps;
	float speed_target_rpm;
	float rc_speed_target_rpm;
	float balance_speed_target_rpm;
	float left_speed_target_rpm;
	float right_speed_target_rpm;
	float turn_speed_target_rpm;
	float vehicle_speed_rpm;
	float left_speed_error_rpm;
	float right_speed_error_rpm;
	float left_speed_integral;
	float right_speed_integral;
	float pitch_p_out_rpm;
	float pitch_d_out_rpm;
	float left_speed_p_out;
	float left_speed_i_out;
	float right_speed_p_out;
	float right_speed_i_out;
	float left_current;
	float right_current;
	struct balance_ctrl_pi speed_pi;
	struct balance_ctrl_pd pitch_pd;
};

extern volatile struct balance_ctrl_state g_balance_ctrl;

void balance_ctrl_init(volatile struct balance_ctrl_state *ctrl);
void balance_ctrl_update(volatile struct balance_ctrl_state *ctrl,
			 const struct balance_ctrl_input *input,
			 struct balance_ctrl_output *output);

#endif
