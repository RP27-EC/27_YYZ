#ifndef DOWN_RM_MOTOR_H
#define DOWN_RM_MOTOR_H
#include "motor.h"
void RM_Motor_Stop(rm_motor_t *motor);
void RM_Motor_SpeedControl(rm_motor_t *motor, float target, uint32_t dt_ms);
#endif
