/**
 * @file    chassis.c
 * @brief   麦轮平移、可选机械遥控/键鼠转向、跟随/小陀螺解算及四轮速度控制。
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis.h"
#include "carctrl.h"
#include "rc_sensor.h"
#include "RM_motor.h"
#include "gyro_control.h"
#include "gimbal_config.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
static uint32_t key_ms[4]; /**< W/S/A/D独立按住时长，毫秒。 */

/* Exported variables --------------------------------------------------------*/
volatile uint32_t chassis_manual_rotation_enable = GIMBAL_YAW_CHASSIS_STEER_ENABLE; /**< 机械遥控右杆/鼠标X转底盘共用开关，只有1开启。 */
chassis_t chassis = { /**< 四轮指针和控制函数构成的底盘对象。 */
    .chassisLF = &rm_motor[CHAS_LF], .chassisLB = &rm_motor[CHAS_LB],
    .chassisRF = &rm_motor[CHAS_RF], .chassisRB = &rm_motor[CHAS_RB],
    .work = Chassis_Work
};

/* Exported functions --------------------------------------------------------*/
/** @brief 重置底盘目标和输出快照。 */
void Chassis_Init(void)
{
    chassis_manual_rotation_enable = GIMBAL_YAW_CHASSIS_STEER_ENABLE;
    chassis.base_info = (chassis_base_info_t){0};
    for (unsigned i = 0; i < 4U; ++i) key_ms[i] = 0U;
}

/** @brief 机械遥控右杆与键鼠X共用转底盘开关，世界角模式保留原输入。 */
int Chassis_ManualRotation(void)
{
    return chassis_manual_rotation_enable == 1U && car.car_mode == mec_car &&
        (car.car_ctrl == RC_CTRL || car.car_ctrl == KEY_CTRL);
}

/** @brief 按麦轮安装方向合成四轮目标转速并等比例限幅，单位转子RPM。 */
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

/* Private functions ---------------------------------------------------------*/
/** @brief 连续死区映射遥控通道到目标轮速，单位转子RPM。 */
static float stick(int16_t value)
{
    if (value > RC_DEADBAND) { return (value - RC_DEADBAND) * (CHASSIS_MAX_SPEED / (660.0f - RC_DEADBAND)); }
    if (value < -RC_DEADBAND) { return (value + RC_DEADBAND) * (CHASSIS_MAX_SPEED / (660.0f - RC_DEADBAND)); }
    return 0;
}

/* Exported functions --------------------------------------------------------*/
/** @brief 更新左杆/WASD底盘目标和四轮PI，停止模式立即清零。 */
void Chassis_Work(chassis_t *c)
{
    rm_motor_t *wheels[CHAS_MOTOR_COUNT] = {c->chassisLF, c->chassisLB, c->chassisRF, c->chassisRB};
    if (car.car_mode != mec_car && !Car_IsGyroMode(car.car_mode)) {
        c->base_info = (chassis_base_info_t){0};
        for (unsigned i = 0; i < 4U; ++i) key_ms[i] = 0U;
        for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) { RM_Motor_Stop(wheels[i]); }
        return;
    }
    if (car.car_ctrl == KEY_CTRL && Car_IsGyroMode(car.car_mode)) {
        const uint16_t bits[4] = {RC_KEY_W, RC_KEY_S, RC_KEY_A, RC_KEY_D};
        for (unsigned i = 0; i < 4U; ++i) {
            key_ms[i] = rc_sensor.info->key_v & bits[i] ? key_ms[i] + car.dt_ms : 0U;
            if (key_ms[i] > GYRO_KEY_RAMP_MS) key_ms[i] = GYRO_KEY_RAMP_MS;
        }
        float scale = rc_sensor.info->key_v & RC_KEY_CTRL ? 0.0f : CHASSIS_MAX_SPEED / GYRO_KEY_RAMP_MS;
        c->base_info.target_front_speed = ((float)key_ms[0] - key_ms[1]) * scale;
        c->base_info.target_right_speed = ((float)key_ms[3] - key_ms[2]) * scale;
        c->base_info.target_cycle_speed = 0.0f;
    } else if (car.car_ctrl == KEY_CTRL) {
        float speed = fminf(CHASSIS_KEY_SPEED, CHASSIS_MAX_SPEED);
        uint16_t keys = rc_sensor.info->key_v;
        if (keys & RC_KEY_CTRL) { speed = 0; }
        c->base_info.target_front_speed = ((keys & RC_KEY_W ? 1 : 0) - (keys & RC_KEY_S ? 1 : 0)) * speed;
        c->base_info.target_right_speed = ((keys & RC_KEY_D ? 1 : 0) - (keys & RC_KEY_A ? 1 : 0)) * speed;
        c->base_info.target_cycle_speed = 0;
        if (Chassis_ManualRotation() && !(keys & RC_KEY_CTRL)) {
            float limit = fminf(GIMBAL_YAW_CHASSIS_STEER_MAX_RPM, CHASSIS_MAX_SPEED);
            float rotation = GIMBAL_YAW_CHASSIS_STEER_DIRECTION * rc_sensor.info->mouse_vx * GIMBAL_YAW_CHASSIS_STEER_MOUSE_GAIN;
            c->base_info.target_cycle_speed = fmaxf(-limit, fminf(rotation, limit));
        }
    } else {
        c->base_info.target_front_speed = stick(rc_sensor.info->ch3);
        c->base_info.target_right_speed = stick(rc_sensor.info->ch2);
        c->base_info.target_cycle_speed = CHASSIS_RC_ROTATION_ENABLE ? stick(rc_sensor.info->ch0) : 0;
        if (Chassis_ManualRotation()) {
            c->base_info.target_cycle_speed = GIMBAL_YAW_CHASSIS_STEER_DIRECTION * stick(rc_sensor.info->ch0) *
                (fminf(GIMBAL_YAW_CHASSIS_STEER_MAX_RPM, CHASSIS_MAX_SPEED) / CHASSIS_MAX_SPEED);
        }
    }
    if (Car_IsGyroMode(car.car_mode)) {
        Gyro_Transform(c->base_info.target_front_speed, c->base_info.target_right_speed, gyro_control.mechanical_deg,
            &c->base_info.target_front_speed, &c->base_info.target_right_speed);
        c->base_info.target_cycle_speed = car.car_mode == cycle_car ? GYRO_CYCLE_SPEED_RPM :
            (gyro_control.turning || (car.car_ctrl == RC_CTRL && gyro_chassis_follow_disable == 1U) ?
             0.0f : Gyro_Follow(gyro_control.mechanical_deg));
    }
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
