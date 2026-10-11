/**
 * @file    chassis.h
 * @brief   麦轮目标解算、机械遥控/键鼠转向分配及四轮控制接口。
 */
#ifndef DOWN_CHASSIS_H
#define DOWN_CHASSIS_H
#include "motor.h"
typedef struct {
    float target_front_speed, target_right_speed, target_cycle_speed;
    float target_chassisLF_speed, target_chassisLB_speed;
    float target_chassisRF_speed, target_chassisRB_speed;
    int16_t output_chassisLF, output_chassisLB, output_chassisRF, output_chassisRB;
} chassis_base_info_t;
typedef struct chassis_t {
    rm_motor_t *chassisLF, *chassisLB, *chassisRF, *chassisRB;
    chassis_base_info_t base_info;
    void (*work)(struct chassis_t *);
} chassis_t;
extern chassis_t chassis;
extern volatile uint32_t chassis_manual_rotation_enable; /**< 机械右杆/鼠标X分配开关：1转底盘且Yaw保持车头/车尾，0控制Yaw。 */
/** @brief 当前为机械遥控或键鼠且输入分配到底盘旋转时返回1。 */
int Chassis_ManualRotation(void);
void Chassis_Init(void);
void Chassis_Work(chassis_t *c);
/* out[] uses CHAS_LF/LB/RF/RB indices, NOT ascending CAN IDs. */
void Chassis_Mix(float front, float right, float rotate, float out[CHAS_MOTOR_COUNT]);
#endif
