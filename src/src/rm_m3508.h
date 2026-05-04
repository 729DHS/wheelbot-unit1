}

static bool valid_id(enum rm_m3508_id id)
{
	return id >= RM_M3508_CAN1_ID201 && id < RM_M3508_MOTOR_COUNT;
}

static uint8_t motor_bus(enum rm_m3508_id id)
{
	return id == RM_M3508_CAN1_ID201 ? 0U : 1U;
}

static uint8_t motor_feedback_id(enum rm_m3508_id id)
{
	return id == RM_M3508_CAN1_ID201 ? 0x03U : 0x01U;
}

static uint8_t motor_current_slot(enum rm_m3508_id id)
{
	return motor_feedback_id(id) - 1U;
}

static int16_t motor_direction(enum rm_m3508_id id)
{
	return id == RM_M3508_CAN2_ID202 ? -1 : 1;
static bool parse_frame(uint8_t bus, const struct can_frame *frame)
{
	enum rm_m3508_id id;
	struct rm_m3508_motor *status;

}

int rm_m3508_set_current(enum rm_m3508_id id, int16_t current)
{
	if (!valid_id(id)) {
}

bool rm_m3508_is_online(enum rm_m3508_id id)
{
	if (!valid_id(id)) {
}

const volatile struct rm_m3508_motor *rm_m3508_get(enum rm_m3508_id id)
{
	if (!valid_id(id)) {
#define RM_M3508_ONLINE_TIMEOUT_MS 500

enum rm_m3508_id {
	RM_M3508_CAN1_ID201 = 0,
	RM_M3508_CAN2_ID202 = 1,
void rm_m3508_poll(void);
int rm_m3508_send_currents(void);
int rm_m3508_set_current(enum rm_m3508_id id, int16_t current);
int rm_m3508_set_all_current(int16_t can1_id201_current, int16_t can2_id202_current);
void rm_m3508_stop(void);
bool rm_m3508_is_online(enum rm_m3508_id id);
const volatile struct rm_m3508_motor *rm_m3508_get(enum rm_m3508_id id);

void rm_m3508_make_zero_current_frame(struct can_frame *frame);
