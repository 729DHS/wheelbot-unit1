/**
 * @file main.c
 * @brief DM4310 MIT Position Hold Unit Test
 *
 * Power-on flow:
 *   1. dm4310_init()  - init CAN1/CAN2, both buses at 1Mbps
 *   2. dm4310_tick()  - per-motor bringup (DISABLE 10 ticks -> ZERO 10 ticks -> DONE)
 *   3. After bringup, capture current motor positions as zero reference
 *   4. MIT impedance hold: motor internal control law
 *        torque = KP * (pos_ref - pos_actual) + KD * (0 - vel_actual)
 *      When external force pushes motor away from target, position error
 *      increases, KP * error automatically produces restoring torque.
 *
 * OpenOCD debug variables (volatile, accessible via gdb/monitor mdw):
 *   cmd_kp              - MIT proportional gain (0-500, default 90)
 *   cmd_kd              - MIT derivative gain (0-5, default 1.8)
 *   cmd_target_offset[] - target position offset from zero [rad] (0 by default)
 *   g_dm4310            - full driver state (online_mask, motor positions, etc.)
 *
 * Motor mapping:
 *   CAN1 -> motor 1 (left hip), motor 2 (left knee)
 *   CAN2 -> motor 3 (right hip), motor 4 (right knee, mirrored)
 */

#include "dm4310_motor.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define LOOP_PERIOD_MS 1U

volatile float cmd_kp = 90.0f;
volatile float cmd_kd = 1.8f;
volatile float cmd_target_offset[DM4310_MOTOR_COUNT];

static float zero_pos_rad[DM4310_MOTOR_COUNT];
static bool zero_captured;

/** @brief Capture current motor positions as zero reference */
static void capture_zero(void)
{
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		zero_pos_rad[i] = g_dm4310.motor[i].online
			? g_dm4310.motor[i].pos_rad : 0.0f;
	}
	zero_captured = true;
}

/** @brief Main control thread: bringup -> zero capture -> MIT position hold */
static void joint_test_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	int ret = dm4310_init();
	printk("dm4310_init: %d\n", ret);

	while (1) {
		dm4310_poll_rx();
		dm4310_tick();

		if (g_dm4310.bringup_done) {
			if (!zero_captured) {
				capture_zero();
				printk("Zero captured, hold KP=%.1f KD=%.1f\n",
				       (double)cmd_kp, (double)cmd_kd);
			}

			if (zero_captured) {
				g_dm4310.hold_kp = cmd_kp;
				g_dm4310.hold_kd = cmd_kd;
				float target[DM4310_MOTOR_COUNT];
				for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
					target[i] = zero_pos_rad[i]
						+ cmd_target_offset[i];
				}
				dm4310_hold_positions(target);
			}
		}

		k_sleep(K_MSEC(LOOP_PERIOD_MS));
	}
}

K_THREAD_DEFINE(joint_test_tid, 4096,
		joint_test_thread, NULL, NULL, NULL, 3, 0, 0);

int main(void)
{
	printk("=== DM4310 Joint MIT Hold Unit Test ===\n");
	return 0;
}
