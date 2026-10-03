#ifndef DOWN_CHASSIS_CONFIG_H
#define DOWN_CHASSIS_CONFIG_H
#include <stdint.h>

/* Array indices follow the Hero project; CAN order is explicitly mapped. */
typedef enum { CHAS_LF, CHAS_LB, CHAS_RF, CHAS_RB, CHAS_MOTOR_COUNT } chassis_motor_index_t;
enum { RC_SW_UP = 1, RC_SW_DOWN = 2, RC_SW_MID = 3 };
#define CHASSIS_MAX_SPEED 500.0f
#define CHASSIS_MAX_CURRENT 2000.0f
#define CHASSIS_OFFLINE_MS 100U
#define CHASSIS_CONTROL_MS 2U
#define CHASSIS_MAX_PERIOD_MS 20U
#define RC_VALID_FRAME_COUNT 5U
#define RC_DEADBAND 20
#define RC_ARM_NEUTRAL 30
#endif
