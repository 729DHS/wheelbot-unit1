#include "balance_ctrl.h"

#include <math.h>
#include <string.h>

/* Balance control tunables. Keep this block together for OpenOCD tuning. */
#define BALANCE_WHEEL_SPEED_LIMIT_RPM 3000.0f
#define BALANCE_WHEEL_CURRENT_LIMIT 16384.0f
#define BALANCE_RC_INPUT_MAX 1684.0f
#define BALANCE_RC_SPEED_LIMIT_RPM BALANCE_WHEEL_SPEED_LIMIT_RPM

#define BALANCE_SPEED_KP_DEFAULT 20.0f
#define BALANCE_SPEED_KI_DEFAULT 0. 05f
#define BALANCE_SPEED_INTEGRAL_LIMIT 80000.0f

#define BALANCE_PITCH_KP_DEFAULT 90.0f
#define BALANCE_PITCH_KD_DEFAULT 4.5f

volatile struct balance_ctrl_state g_balance_ctrl = {
	.magic = BALANCE_CTRL_MAGIC,
};

static float clampf_local(float value, float min_value, float max_value)
{
	if (value > max_value) {
		return max_value;
	}
	if (value < min_value) {
		return min_value;
	}

	return value;
}

static void clear_output(struct balance_ctrl_output *output)
{
	output->active = 0U;
	output->wheel_current[0] = 0;
	output->wheel_current[1] = 0;
}

static void clear_runtime(volatile struct balance_ctrl_state *ctrl)
{
	ctrl->pitch_zero_deg = 0.0f;
	ctrl->pitch_deg = 0.0f;
	ctrl->pitch_rate_dps = 0.0f;
	ctrl->pitch_target_deg = 0.0f;
	ctrl->pitch_error_deg = 0.0f;
	ctrl->pitch_rate_error_dps = 0.0f;
	ctrl->speed_target_rpm = 0.0f;
	ctrl->rc_speed_target_rpm = 0.0f;
	ctrl->balance_speed_target_rpm = 0.0f;
	ctrl->left_speed_target_rpm = 0.0f;
	ctrl->right_speed_target_rpm = 0.0f;
	ctrl->vehicle_speed_rpm = 0.0f;
	ctrl->left_speed_error_rpm = 0.0f;
	ctrl->right_speed_error_rpm = 0.0f;
	ctrl->left_speed_integral = 0.0f;
	ctrl->right_speed_integral = 0.0f;
	ctrl->pitch_p_out_rpm = 0.0f;
	ctrl->pitch_d_out_rpm = 0.0f;
	ctrl->left_speed_p_out = 0.0f;
	ctrl->left_speed_i_out = 0.0f;
	ctrl->right_speed_p_out = 0.0f;
	ctrl->right_speed_i_out = 0.0f;
	ctrl->left_current = 0.0f;
	ctrl->right_current = 0.0f;
}

static void controller_reset(volatile struct balance_ctrl_state *ctrl)
{
	ctrl->enabled = 0U;
	ctrl->captured = 0U;
	clear_runtime(ctrl);
}

static float rc_to_speed_target_rpm(int32_t rc_forward)
{
	float target = (float)rc_forward * BALANCE_RC_SPEED_LIMIT_RPM / BALANCE_RC_INPUT_MAX;

	return clampf_local(target, -BALANCE_RC_SPEED_LIMIT_RPM,
			    BALANCE_RC_SPEED_LIMIT_RPM);
}

static float update_pitch_loop(volatile struct balance_ctrl_state *ctrl,
			       const struct balance_ctrl_input *input)
{
	ctrl->pitch_deg = input->pitch_deg;
	ctrl->pitch_rate_dps = input->pitch_rate_dps;
	ctrl->pitch_target_deg = ctrl->pitch_zero_deg;
	ctrl->pitch_error_deg = ctrl->pitch_target_deg - input->pitch_deg;
	ctrl->pitch_rate_error_dps = -input->pitch_rate_dps;
	ctrl->pitch_p_out_rpm = ctrl->pitch_pd.kp * ctrl->pitch_error_deg;
	ctrl->pitch_d_out_rpm = ctrl->pitch_pd.kd * ctrl->pitch_rate_error_dps;
	ctrl->balance_speed_target_rpm = ctrl->pitch_p_out_rpm + ctrl->pitch_d_out_rpm;

	return clampf_local(ctrl->balance_speed_target_rpm,
			    -BALANCE_WHEEL_SPEED_LIMIT_RPM,
			    BALANCE_WHEEL_SPEED_LIMIT_RPM);
}

static float update_wheel_speed_loop(volatile struct balance_ctrl_state *ctrl,
				     float target_rpm, float actual_rpm, uint8_t left)
{
	float error = target_rpm - actual_rpm;
	float integral;
	float p_out;
	float i_out;
	float current;

	if (left != 0U) {
		ctrl->left_speed_error_rpm = error;
		ctrl->left_speed_integral += error;
		ctrl->left_speed_integral =
			clampf_local(ctrl->left_speed_integral,
				     -ctrl->speed_pi.integral_limit,
				     ctrl->speed_pi.integral_limit);
		integral = ctrl->left_speed_integral;
	} else {
		ctrl->right_speed_error_rpm = error;
		ctrl->right_speed_integral += error;
		ctrl->right_speed_integral =
			clampf_local(ctrl->right_speed_integral,
				     -ctrl->speed_pi.integral_limit,
				     ctrl->speed_pi.integral_limit);
		integral = ctrl->right_speed_integral;
	}

	p_out = ctrl->speed_pi.kp * error;
	i_out = ctrl->speed_pi.ki * integral;
	current = clampf_local(p_out + i_out, -ctrl->speed_pi.output_limit,
			       ctrl->speed_pi.output_limit);

	if (left != 0U) {
		ctrl->left_speed_p_out = p_out;
		ctrl->left_speed_i_out = i_out;
		ctrl->left_current = current;
	} else {
		ctrl->right_speed_p_out = p_out;
		ctrl->right_speed_i_out = i_out;
		ctrl->right_current = current;
	}

	return current;
}

void balance_ctrl_init(volatile struct balance_ctrl_state *ctrl)
{
	memset((void *)ctrl, 0, sizeof(*ctrl));
	ctrl->magic = BALANCE_CTRL_MAGIC;

	ctrl->speed_pi.kp = BALANCE_SPEED_KP_DEFAULT;
	ctrl->speed_pi.ki = BALANCE_SPEED_KI_DEFAULT;
	ctrl->speed_pi.integral_limit = BALANCE_SPEED_INTEGRAL_LIMIT;
	ctrl->speed_pi.output_limit = BALANCE_WHEEL_CURRENT_LIMIT;

	ctrl->pitch_pd.kp = BALANCE_PITCH_KP_DEFAULT;
	ctrl->pitch_pd.kd = BALANCE_PITCH_KD_DEFAULT;
	ctrl->pitch_pd.output_limit = BALANCE_WHEEL_SPEED_LIMIT_RPM;
}

void balance_ctrl_update(volatile struct balance_ctrl_state *ctrl,
			 const struct balance_ctrl_input *input,
			 struct balance_ctrl_output *output)
{
	float rc_speed_target;
	float balance_speed_target;
	float left_current;
	float right_current;

	ctrl->update_count++;
	ctrl->last_ms = input->now_ms;
	ctrl->enabled = input->enabled;

	clear_output(output);

	if (input->enabled == 0U) {
		controller_reset(ctrl);
		return;
	}

	if (ctrl->captured == 0U) {
		ctrl->pitch_zero_deg = input->pitch_deg;
		ctrl->captured = 1U;
	}

	rc_speed_target = rc_to_speed_target_rpm(input->rc_forward);
	balance_speed_target = update_pitch_loop(ctrl, input);

	ctrl->rc_speed_target_rpm = rc_speed_target;
	ctrl->speed_target_rpm = rc_speed_target;
	ctrl->vehicle_speed_rpm =
		((float)input->wheel_speed_rpm[0] + (float)input->wheel_speed_rpm[1]) *
		0.5f;
	ctrl->left_speed_target_rpm =
		clampf_local(balance_speed_target + rc_speed_target,
			     -BALANCE_WHEEL_SPEED_LIMIT_RPM,
			     BALANCE_WHEEL_SPEED_LIMIT_RPM);
	ctrl->right_speed_target_rpm = ctrl->left_speed_target_rpm;

	left_current = update_wheel_speed_loop(ctrl, ctrl->left_speed_target_rpm,
					       (float)input->wheel_speed_rpm[0], 1U);
	right_current = update_wheel_speed_loop(ctrl, ctrl->right_speed_target_rpm,
						(float)input->wheel_speed_rpm[1], 0U);

	output->active = 1U;
	output->wheel_current[0] = (int16_t)lrintf(left_current);
	output->wheel_current[1] = (int16_t)lrintf(right_current);
}
