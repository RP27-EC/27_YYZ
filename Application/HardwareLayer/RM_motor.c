#include "RM_motor.h"
#include "chassis_param.h"
#include <math.h>
static float limit(float x, float max) { return x > max ? max : (x < -max ? -max : x); }
void RM_Motor_Stop(rm_motor_t *motor) { motor->base_info = (rm_motor_base_info_t){0}; }
void RM_Motor_SpeedControl(rm_motor_t *motor, float target, uint32_t dt_ms)
{
    rm_motor_base_info_t *b = &motor->base_info;
    float dt = dt_ms * 0.001f;
    b->target_speed += limit(target - b->target_speed, chassis_speed_param.acceleration_rpm_per_second * dt);
    float error = b->target_speed - motor->info->speed;
    float next_i = limit(b->integral + chassis_speed_param.ki_per_second * error * dt,
                         chassis_speed_param.integral_limit);
    float candidate = chassis_speed_param.kp * error + next_i;
    if (fabsf(candidate) <= CHASSIS_MAX_CURRENT || candidate * error < 0) { b->integral = next_i; }
    b->motor_out = (int16_t)limit(chassis_speed_param.kp * error + b->integral, CHASSIS_MAX_CURRENT);
}
