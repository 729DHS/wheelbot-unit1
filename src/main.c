/**
 * @file main.c
 * @brief DM4310 Spring-Damper + Side Detection
 *
 * Power-on flow:
 *   1. dm4310_init()  - init CAN1/CAN2, both buses at 1Mbps
 *   2. dm4310_tick()  - per-motor bringup (DISABLE 10 ticks -> ZERO 10 ticks -> DONE)
 *   3. After bringup, capture current motor positions as spring home
 *   4. Spring-damper hold: low-stiffness MIT impedance control
 *        torque = KP * (home_pos - pos_actual) + KD * (0 - vel_actual)
 *      Low KP allows manual turning; spring-back when released.
 *   5. Side detection: monitors which leg pair is being moved.
 *
 * Architecture for future leg sync:
 *   Left leg:  motors 1 (hip) + 2 (knee) on CAN1
 *   Right leg: motors 3 (hip) + 4 (knee) on CAN2
 *   Each side tracks its own spring home; sync couples them.
 *
 * OpenOCD debug variables (volatile, accessible via gdb/monitor mdw):
 *   cmd_kp              - spring stiffness (0-500, default 10)
 *   cmd_kd              - damping (0-5, default 0.5)
 *   cmd_target_offset[] - target position offset from home [rad]
 *   cmd_side_active[]   - [0]=left active flag, [1]=right active flag
 *   g_dm4310            - full driver state
 *
 * Motor mapping:
 *   CAN1 -> motor 1 (left hip), motor 2 (left knee)
 *   CAN2 -> motor 3 (right hip), motor 4 (right knee, mirrored)
 */

#include "dm4310_motor.h"

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define LOOP_PERIOD_MS 1U

/* ── Spring-damper gains (low force, hand-turnable) ── */
volatile float cmd_kp = 10.0f;
volatile float cmd_kd = 0.5f;
volatile float cmd_target_offset[DM4310_MOTOR_COUNT];

/* ── Side detection ── */
#define SIDE_DETECT_THRESHOLD_RAD 0.03f
volatile uint8_t cmd_side_active[2];  /* [0]=left, [1]=right */

/* ── Spring home positions ── */
static float home_pos_rad[DM4310_MOTOR_COUNT];
static bool home_captured;

/** @brief Capture current motor positions as spring home reference */
static void capture_home(void)
{
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		home_pos_rad[i] = g_dm4310.motor[i].online
			? g_dm4310.motor[i].pos_rad : 0.0f;
	}
	home_captured = true;
}

static void compute_spring_targets(float target[DM4310_MOTOR_COUNT])
{
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		target[i] = home_pos_rad[i] + cmd_target_offset[i];
		g_dm4310.hold_kp[i] = cmd_kp;
		g_dm4310.hold_kd[i] = cmd_kd;
	}
}

static void detect_active_side(const float target[DM4310_MOTOR_COUNT])
{
	float error_left  = 0.0f;
	float error_right = 0.0f;

	/* Left leg: motors 1,2 (indices 0,1) */
	for (int i = 0; i < 2; i++) {
		if (g_dm4310.motor[i].online) {
			float e = fabsf(g_dm4310.motor[i].pos_rad - target[i]);
			if (e > error_left) error_left = e;
		}
	}
	cmd_side_active[0] = (error_left > SIDE_DETECT_THRESHOLD_RAD) ? 1U : 0U;

	/* Right leg: motors 3,4 (indices 2,3) */
	for (int i = 2; i < 4; i++) {
		if (g_dm4310.motor[i].online) {
			float e = fabsf(g_dm4310.motor[i].pos_rad - target[i]);
			if (e > error_right) error_right = e;
		}
	}
	cmd_side_active[1] = (error_right > SIDE_DETECT_THRESHOLD_RAD) ? 1U : 0U;
}

/** @brief Main control thread */
static void joint_test_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	int ret = dm4310_init();
	printk("dm4310_init: %d\n", ret);

	while (1) {
		dm4310_poll_rx();

		if (g_dm4310.bringup_done) {
			if (!home_captured) {
				capture_home();
				printk("Spring home captured, KP=%.1f KD=%.1f\n",
				       (double)cmd_kp, (double)cmd_kd);
			}

			if (home_captured) {
				float target[DM4310_MOTOR_COUNT];
				compute_spring_targets(target);
				detect_active_side(target);
				dm4310_hold_positions(target);

				/* Print side activity state changes */
				static uint8_t last_side[2] = {0xFF, 0xFF};
				if (cmd_side_active[0] != last_side[0] ||
				    cmd_side_active[1] != last_side[1]) {
					printk("Side: L=%d R=%d\n",
					       cmd_side_active[0], cmd_side_active[1]);
					last_side[0] = cmd_side_active[0];
					last_side[1] = cmd_side_active[1];
				}

				/* Throttled status print: once per second */
				static uint32_t status_tick;
				if (++status_tick >= 1000) {
					status_tick = 0;
					printk("pos:[%.3f %.3f %.3f %.3f] err:[%.3f %.3f %.3f %.3f] L=%d R=%d\n",
					       (double)g_dm4310.motor[0].pos_rad,
					       (double)g_dm4310.motor[1].pos_rad,
					       (double)g_dm4310.motor[2].pos_rad,
					       (double)g_dm4310.motor[3].pos_rad,
					       (double)(g_dm4310.motor[0].pos_rad - target[0]),
					       (double)(g_dm4310.motor[1].pos_rad - target[1]),
					       (double)(g_dm4310.motor[2].pos_rad - target[2]),
					       (double)(g_dm4310.motor[3].pos_rad - target[3]),
					       cmd_side_active[0], cmd_side_active[1]);
				}
			}
		}

		dm4310_tick();
		k_sleep(K_MSEC(LOOP_PERIOD_MS));
	}
}

K_THREAD_DEFINE(joint_test_tid, 4096,
		joint_test_thread, NULL, NULL, NULL, 3, 0, 0);

int main(void)
{
	printk("=== DM4310 Spring-Damper + Side Detect ===\n");
	printk("KP=%.1f KD=%.1f  (tune via OpenOCD: set var cmd_kp=...)\n",
	       (double)cmd_kp, (double)cmd_kd);
	printk("Left:  motors 1,2 on CAN1\n");
	printk("Right: motors 3,4 on CAN2\n");
	return 0;
}
