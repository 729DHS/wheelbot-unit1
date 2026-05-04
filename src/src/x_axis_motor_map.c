#include "x_axis_motor_map.h"

#define X_AXIS_MOTOR_MAP_MAX_POINTS 16U
#define X_AXIS_MOTOR_MAP_DEFAULT_POINT_COUNT 5U
#define X_AXIS_MOTOR_MAP_X_EPSILON 0.000001f

static const struct x_axis_motor_map_point x_axis_motor_map_default[X_AXIS_MOTOR_MAP_DEFAULT_POINT_COUNT] = {
	{ .x_pos = -0.20f, .motor_pos_rad = { 0.00f, 0.00f, 0.00f, 0.00f } },
	{ .x_pos = -0.10f, .motor_pos_rad = { 0.00f, 0.00f, 0.00f, 0.00f } },
	{ .x_pos = 0.00f, .motor_pos_rad = { 0.00f, 0.00f, 0.00f, 0.00f } },
	{ .x_pos = 0.10f, .motor_pos_rad = { 0.00f, 0.00f, 0.00f, 0.00f } },
	{ .x_pos = 0.20f, .motor_pos_rad = { 0.00f, 0.00f, 0.00f, 0.00f } },
};

static struct x_axis_motor_map_point x_axis_motor_map[X_AXIS_MOTOR_MAP_MAX_POINTS];
static size_t x_axis_motor_map_count;

static float absf_local(float value)
{
	return value < 0.0f ? -value : value;
}

static const struct x_axis_motor_map_point *active_map(size_t *count)
{
	if (x_axis_motor_map_count > 0U) {
		*count = x_axis_motor_map_count;
		return x_axis_motor_map;
	}

	*count = X_AXIS_MOTOR_MAP_DEFAULT_POINT_COUNT;
	return x_axis_motor_map_default;
}

static void copy_motor_positions(const struct x_axis_motor_map_point *point,
					 float motor_pos_rad[CHASSIS_SM_JOINT_COUNT])
{
	for (int motor = 0; motor < CHASSIS_SM_JOINT_COUNT; motor++) {
		motor_pos_rad[motor] = point->motor_pos_rad[motor];
	}
}

int x_axis_motor_positions(float x_pos, float motor_pos_rad[CHASSIS_SM_JOINT_COUNT])
{
	const struct x_axis_motor_map_point *map;
	const struct x_axis_motor_map_point *left;
	const struct x_axis_motor_map_point *right;
	size_t count;
	float span;
	float ratio;

	if (motor_pos_rad == NULL) {
		return -1;
	}

	map = active_map(&count);

	if (x_pos <= map[0].x_pos) {
		copy_motor_positions(&map[0], motor_pos_rad);
		return 0;
	}

	if (x_pos >= map[count - 1U].x_pos) {
		copy_motor_positions(&map[count - 1U], motor_pos_rad);
		return 0;
	}

	for (size_t index = 0U; index < count - 1U; index++) {
		left = &map[index];
		right = &map[index + 1U];
		if (x_pos > right->x_pos) {
			continue;
		}

		span = right->x_pos - left->x_pos;
		if (span <= 0.0f) {
			return -2;
		}

		ratio = (x_pos - left->x_pos) / span;
		for (int motor = 0; motor < CHASSIS_SM_JOINT_COUNT; motor++) {
			motor_pos_rad[motor] = left->motor_pos_rad[motor] +
					      ratio * (right->motor_pos_rad[motor] -
						       left->motor_pos_rad[motor]);
		}
		return 0;
	}

	return -3;
}

int x_axis_motor_map_set_point(float x_pos,
				       const float motor_pos_rad[CHASSIS_SM_JOINT_COUNT])
{
	size_t insert_index;

	if (motor_pos_rad == NULL) {
		return -1;
	}

	for (size_t index = 0U; index < x_axis_motor_map_count; index++) {
		if (absf_local(x_axis_motor_map[index].x_pos - x_pos) <= X_AXIS_MOTOR_MAP_X_EPSILON) {
			x_axis_motor_map[index].x_pos = x_pos;
			for (int motor = 0; motor < CHASSIS_SM_JOINT_COUNT; motor++) {
				x_axis_motor_map[index].motor_pos_rad[motor] = motor_pos_rad[motor];
			}
			return 0;
		}
	}

	if (x_axis_motor_map_count >= X_AXIS_MOTOR_MAP_MAX_POINTS) {
		return -2;
	}

	insert_index = x_axis_motor_map_count;
	while (insert_index > 0U && x_axis_motor_map[insert_index - 1U].x_pos > x_pos) {
		x_axis_motor_map[insert_index] = x_axis_motor_map[insert_index - 1U];
		insert_index--;
	}

	x_axis_motor_map[insert_index].x_pos = x_pos;
	for (int motor = 0; motor < CHASSIS_SM_JOINT_COUNT; motor++) {
		x_axis_motor_map[insert_index].motor_pos_rad[motor] = motor_pos_rad[motor];
	}
	x_axis_motor_map_count++;

	return 0;
}

const struct x_axis_motor_map_point *x_axis_motor_map_points(size_t *count)
{
	size_t active_count;
	const struct x_axis_motor_map_point *map = active_map(&active_count);

	if (count != NULL) {
		*count = active_count;
	}
	return map;
}
