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
void Chassis_Init(void);
void Chassis_Work(chassis_t *c);
/* out[] uses CHAS_LF/LB/RF/RB indices, NOT ascending CAN IDs. */
void Chassis_Mix(float front, float right, float rotate, float out[CHAS_MOTOR_COUNT]);
#endif
