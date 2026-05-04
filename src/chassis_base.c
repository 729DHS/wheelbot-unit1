#include "chassis_base.h"

#include "balance_ctrl.h"
#include "chassis_sm.h"
#include "dm4310_motor.h"
#include "rm_imu.h"
#include "rm_m3508.h"
#include "rm_sbus.h"

volatile struct chassis_base_status g_chassis_base = {
	.magic = CHASSIS_BASE_MAGIC,
};

volatile struct chassis_debug_override g_chassis_debug_override;
volatile struct chassis_m3508_command g_chassis_m3508_cmd;

void chassis_base_init(void)
{
	g_chassis_base.magic = CHASSIS_BASE_MAGIC;
	g_chassis_base.sbus_init_ret = rm_sbus_init();
	g_chassis_base.lk_init_ret = dm4310_init();
	g_chassis_base.m3508_init_ret = rm_m3508_init();
	g_chassis_base.imu_init_ret = rm_imu_init();
	balance_ctrl_init(&g_balance_ctrl);
	chassis_sm_init(&g_chassis_sm);
	g_chassis_base.init_done = 1U;
}
