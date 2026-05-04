#ifndef X_AXIS_MOTOR_MAP_H_
#define X_AXIS_MOTOR_MAP_H_

#include <stddef.h>

#include "chassis_sm.h"

struct x_axis_motor_map_point {
	float x_pos;
	float motor_pos_rad[CHASSIS_SM_JOINT_COUNT];
};

int x_axis_motor_positions(float x_pos, float motor_pos_rad[CHASSIS_SM_JOINT_COUNT]);
int x_axis_motor_map_set_point(float x_pos,
				       const float motor_pos_rad[CHASSIS_SM_JOINT_COUNT]);
const struct x_axis_motor_map_point *x_axis_motor_map_points(size_t *count);

#endif
