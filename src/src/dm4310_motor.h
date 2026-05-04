/**
 * @file dm4310_motor.h
 * @brief DM-J4310-2EC 减速电机 CAN 总线驱动接口
 *
 * 基于《DM-J4310-2EC V1.1减速电机说明书》实现。
 *
 * 通信参数:
 *   - CAN 标准帧 (2.0A), 波特率 1 Mbps
 *   - 控制帧 ID 规则: MIT模式=用户设定CAN_ID, 位置模式=0x100+CAN_ID, 速度模式=0x200+CAN_ID
 *   - 反馈帧 ID: 由调试助手设置 (默认 0x00)
 *
 * 支持的控制模式:
 *   - MIT 模式: 同时发送位置/速度/Kp/Kd/前馈力矩, 最灵活
 *   - 衍生模式: Kp=0,Kd≠0 → 匀速转动; Kp=0,Kd=0 → 纯力矩输出
 *
 * @note 位置控制时 Kd 不能为 0, 否则电机可能震荡或失控
 */

#ifndef DM4310_MOTOR_H_
#define DM4310_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

/** @brief Magic number for driver integrity check */
#define DM4310_MAGIC 0x444d3433

/** @brief Total number of motors on the bus */
#define DM4310_MOTOR_COUNT 4

/** @brief Base CAN TX ID for control frames (equals first motor's CAN_ID) */
#define DM4310_CAN_TX_ID_BASE 1U

/** @brief Master node CAN ID base */
#define DM4310_MASTER_ID_BASE 0x11U

/** @brief Number of motors connected to CAN1 (remainder on CAN2) */
#define DM4310_MOTORS_ON_CAN1  2U

/** @brief Milliseconds before a motor is considered offline */
#define DM4310_ONLINE_TIMEOUT_MS 500

/* === MIT protocol range limits (per datasheet DM-J4310-2EC V1.1) === */

/** @brief Minimum position setpoint [rad] */
#define DM4310_P_MIN  (-12.5f)
/** @brief Maximum position setpoint [rad] */
#define DM4310_P_MAX  12.5f

/** @brief Minimum velocity setpoint [rad/s] */
#define DM4310_V_MIN  (-30.0f)
/** @brief Maximum velocity setpoint [rad/s] */
#define DM4310_V_MAX  30.0f

/** @brief Minimum position proportional gain (range 0-500) */
#define DM4310_KP_MIN 0.0f
/** @brief Maximum position proportional gain (range 0-500) */
#define DM4310_KP_MAX 500.0f

/** @brief Minimum position derivative gain (range 0-5) */
#define DM4310_KD_MIN 0.0f
/** @brief Maximum position derivative gain (range 0-5) */
#define DM4310_KD_MAX 5.0f

/** @brief Minimum feedforward torque [Nm] */
#define DM4310_T_MIN  (-10.0f)
/** @brief Maximum feedforward torque [Nm] */
#define DM4310_T_MAX  10.0f

/* === Fault codes (ERR byte, datasheet section 1) === */

#define DM4310_FAULT_OVERVOLTAGE   0x08U /**< 超压故障 */
#define DM4310_FAULT_UNDERVOLTAGE  0x09U /**< 欠压故障 */
#define DM4310_FAULT_OVERCURRENT   0x0AU /**< 过流故障 */
#define DM4310_FAULT_MOS_OVERTEMP  0x0BU /**< MOS 过温故障 */
#define DM4310_FAULT_COIL_OVERTEMP 0x0CU /**< 线圈过温故障 */
#define DM4310_FAULT_COMM_LOSS     0x0DU /**< 通讯丢失故障 */
#define DM4310_FAULT_OVERLOAD      0x0EU /**< 过载故障 */

/* === MIT protocol special command tails === */

/** @brief 失能命令尾字节 (D0-D6=0xFF, D7=0xFC) */
#define DM4310_CMD_DISABLE_TAIL 0xFC
/** @brief 使能命令尾字节 (D0-D6=0xFF, D7=0xFD) */
#define DM4310_CMD_ENABLE_TAIL  0xFD
/** @brief 零点校准命令尾字节 (D0-D6=0xFF, D7=0xFE) */
#define DM4310_CMD_ZERO_TAIL    0xFE

/** @brief Bringup 阶段每个子步骤持续 tick 数 */
#define DM4310_BRINGUP_TICKS 10

/**
 * @brief 反馈帧字节布局 (Motor → PC, datasheet section 1)
 *
 * 无论电机处于何种模式, 反馈帧格式统一:
 * | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 |
 * |----|----|----|----|----|----|----|----|
 * | ID | ERR | POS[15:8] | POS[7:0] | VEL[11:4] | VEL[3:0] | T[11:8] | T[7:0] |
 *
 * 温度数据由独立温度反馈帧承载 (D0=T_MOS, D1=T_Rotor, 含义续行).
 *
 * 数据类型: D0=Uint8, D1=Uint8, D2-D3=Int16, VEL=12bit, T=12bit
 */
struct dm4310_motor_status {
	uint32_t rx_count;   /**< 累计收到的反馈帧数 */
	uint32_t last_ms;    /**< 最后一次收到反馈的系统时间 [ms] */
	uint8_t online;      /**< 电机是否在线 (收到过反馈且未超时) */
	uint8_t fault;       /**< 故障码 (反馈帧 D1 ERR 字节), 0=正常 */
	uint8_t mos_temp;    /**< 驱动 MOS 温度 [℃] (温度反馈帧 D0) */
	uint8_t coil_temp;   /**< 电机线圈温度 [℃] (温度反馈帧 D1) */
	float pos_rad;       /**< 当前位置 [rad] */
	float vel_radps;     /**< 当前速度 [rad/s] */
	float torque_nm;     /**< 当前扭矩 [Nm] */
};

/**
 * @brief 全局驱动状态
 *
 * 初始化前通过 magic 字段校验结构体完整性.
 */
struct dm4310_driver {
	uint32_t magic;      /**< 魔数 @see DM4310_MAGIC */
	uint32_t ready;      /**< 初始化完成标志 */
	int32_t init_ret;    /**< dm4310_init() 返回值 */
	int32_t last_send_ret; /**< 最近一次 can_send() 返回值 */
	uint32_t loops;      /**< dm4310_tick() 累计调用次数 */
	uint32_t online_mask; /**< 在线电机位掩码 (BIT(i) = motor[i] 在线) */

	/** Bringup 阶段每电机的当前步骤 (0=DISABLE, 1=ZERO, 2=DONE) */
	uint32_t bringup_step[DM4310_MOTOR_COUNT];
	/** Bringup 阶段当前步骤已执行的 tick 数 */
	uint32_t bringup_tick[DM4310_MOTOR_COUNT];
	uint8_t bringup_done; /**< 所有电机 bringup 完成标志 */

	uint8_t tx_index;    /**< 轮询发送索引 (交错发送) */

	uint32_t hold_updates; /**< dm4310_hold_positions() 调用计数 */
	float hold_kp;       /**< 锁定模式 Kp */
	float hold_kd;       /**< 锁定模式 Kd */
	float hold_pos_rad[DM4310_MOTOR_COUNT]; /**< 锁定目标位置 [rad] */

	struct dm4310_motor_status motor[DM4310_MOTOR_COUNT]; /**< 各电机状态 */
};

/** @brief 全局驱动实例 */
extern volatile struct dm4310_driver g_dm4310;

/**
 * @brief 初始化 CAN 总线和驱动状态
 * @return 0 成功, -ENODEV CAN 设备未就绪, 其他负值 CAN 操作错误码
 */
int dm4310_init(void);

/**
 * @brief 从消息队列取出反馈帧并更新电机状态
 *
 * 非阻塞, 无新帧时直接返回.
 */
void dm4310_poll_rx(void);

/**
 * @brief 驱动主循环 tick
 *
 * Bringup 阶段: 每次 tick 对一台电机发送特殊命令 (DISABLE → DONE)
 * 正常运行:   每次 tick 以交错方式发送 2 台电机的 MIT 控制帧
 *
 * @return 最后一次 can_send() 的返回值
 */
int dm4310_tick(void);

/**
 * @brief 设置所有电机的锁定目标位置
 * @param target 目标位置数组 [rad], 长度 DM4310_MOTOR_COUNT
 * @return 0
 *
 * 锁定模式通过 hold_kp/hold_kd 控制刚度和阻尼.
 * @see dm4310_hold_reset()
 */
int dm4310_hold_positions(const float target[DM4310_MOTOR_COUNT]);

/**
 * @brief 清除锁定状态和位置缓存, 恢复发送零位控制帧
 */
void dm4310_hold_reset(void);

/**
 * @brief 向所有电机发送失能命令
 */
void dm4310_stop_all(void);

/**
 * @brief 查询指定电机是否在线
 * @param motor_id 电机 ID (1-based, 1..DM4310_MOTOR_COUNT)
 * @return true 在线, false 离线或 ID 无效
 */
bool dm4310_is_online(uint8_t motor_id);

/**
 * @brief 获取指定电机的状态指针
 * @param motor_id 电机 ID (1-based, 1..DM4310_MOTOR_COUNT)
 * @return 指向电机状态的指针, ID 无效时为 NULL
 */
const volatile struct dm4310_motor_status *dm4310_get(uint8_t motor_id);

/* === 打包/解包辅助函数 (供测试使用) === */

/**
 * @brief 打包特殊命令帧 (失能/使能/校零)
 *
 * 格式: D0-D6 填充 0xFF, D7=tai_byte
 *
 * @param tail 命令尾字节
 * @param[out] data 输出 8 字节 CAN 数据
 */
void dm4310_pack_special(uint8_t tail, uint8_t data[8]);

/**
 * @brief 打包 MIT 模式控制帧 (datasheet section 2A)
 *
 * 5 个参数压缩为 8 字节 (64 bits = 16+12+12+12+12, nibble 交叉打包):
 * | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 |
 * |----|----|----|----|----|----|----|----|
 * | P[15:8] | P[7:0] | V[11:4] | V[3:0] \| Kp[11:8] | Kp[7:0] | Kd[11:4] | Kd[3:0] \| T[11:8] | T[7:0] |
 *
 * @param pos  目标位置 [rad]
 * @param vel  目标速度 [rad/s]
 * @param kp   位置比例系数 (0-500)
 * @param kd   位置微分系数 (0-5), 位置控制时不可为 0
 * @param tor  前馈力矩 [Nm]
 * @param[out] data 输出 8 字节 CAN 数据
 */
void dm4310_pack_control(float pos, float vel, float kp, float kd, float tor,
			 uint8_t data[8]);

/**
 * @brief 解码电机反馈帧 (datasheet section 1)
 *
 * 字节布局:
 * | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 |
 * |----|----|----|----|----|----|----|----|
 * | ID | ERR | POS[15:8] | POS[7:0] | VEL[11:4] | VEL[3:0] \| T[11:8] | T[7:0] | -- |
 *
 * @note 温度数据 (T_MOS/T_Rotor) 来自独立温度反馈帧, 需单独解码
 *
 * @param data 8 字节 CAN 数据
 * @param[out] out 解码后的电机状态 (mos_temp/coil_temp 不由本函数填充)
 * @return true 解码成功, false ID 为 0 或超出范围
 */
bool dm4310_decode_feedback(const uint8_t data[8],
			    struct dm4310_motor_status *out);

#endif /* DM4310_MOTOR_H_ */
