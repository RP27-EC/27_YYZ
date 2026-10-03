#include "chassis.h"
#include "carctrl.h"
#include "rc_sensor.h"
#include "RM_motor.h"
#include <math.h>

chassis_t chassis = {
    .chassisLF = &rm_motor[CHAS_LF], .chassisLB = &rm_motor[CHAS_LB],
    .chassisRF = &rm_motor[CHAS_RF], .chassisRB = &rm_motor[CHAS_RB],
    .work = Chassis_Work
};

void Chassis_Init(void)
{
    chassis.base_info = (chassis_base_info_t){0};
}

void Chassis_Mix(float front, float right, float rotate, float out[CHAS_MOTOR_COUNT])
{
    out[CHAS_LF] =  front + right + rotate;
    out[CHAS_LB] =  front - right + rotate;
    out[CHAS_RF] = -front + right + rotate;
    out[CHAS_RB] = -front - right + rotate;
    float peak = CHASSIS_MAX_SPEED;
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) {
        if (fabsf(out[i]) > peak) { peak = fabsf(out[i]); }
    }
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) { out[i] *= CHASSIS_MAX_SPEED / peak; }
}

static float stick(int16_t value)
{
    if (value > RC_DEADBAND) { return (value - RC_DEADBAND) * (CHASSIS_MAX_SPEED / (660.0f - RC_DEADBAND)); }
    if (value < -RC_DEADBAND) { return (value + RC_DEADBAND) * (CHASSIS_MAX_SPEED / (660.0f - RC_DEADBAND)); }
    return 0;
}

void Chassis_Work(chassis_t *c)
{
    rm_motor_t *wheels[CHAS_MOTOR_COUNT] = {c->chassisLF, c->chassisLB, c->chassisRF, c->chassisRB};
    if (car.car_mode != mec_car) {
        c->base_info = (chassis_base_info_t){0};
        for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) { RM_Motor_Stop(wheels[i]); }
        return;
    }
    c->base_info.target_front_speed = stick(rc_sensor.info->ch3);
    c->base_info.target_right_speed = stick(rc_sensor.info->ch2);
    c->base_info.target_cycle_speed = stick(rc_sensor.info->ch0);
    float targets[CHAS_MOTOR_COUNT];
    Chassis_Mix(c->base_info.target_front_speed, c->base_info.target_right_speed,
                c->base_info.target_cycle_speed, targets);
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) {
        RM_Motor_SpeedControl(wheels[i], targets[i], car.dt_ms);
    }
    c->base_info.target_chassisLF_speed = wheels[CHAS_LF]->base_info.target_speed;
    c->base_info.target_chassisLB_speed = wheels[CHAS_LB]->base_info.target_speed;
    c->base_info.target_chassisRF_speed = wheels[CHAS_RF]->base_info.target_speed;
    c->base_info.target_chassisRB_speed = wheels[CHAS_RB]->base_info.target_speed;
    c->base_info.output_chassisLF = wheels[CHAS_LF]->base_info.motor_out;
    c->base_info.output_chassisLB = wheels[CHAS_LB]->base_info.motor_out;
    c->base_info.output_chassisRF = wheels[CHAS_RF]->base_info.motor_out;
    c->base_info.output_chassisRB = wheels[CHAS_RB]->base_info.motor_out;
}
