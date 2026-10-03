#ifndef DOWN_MOTOR_H
#define DOWN_MOTOR_H
#include "chassis_config.h"
#define ID_CHAS_LF 0x201U
#define ID_CHAS_RF 0x202U
#define ID_CHAS_LB 0x203U
#define ID_CHAS_RB 0x204U
typedef struct {
    uint16_t angle;
    int16_t speed, current;
    uint8_t temperature;
    uint32_t frames, last_ms;
} rm_motor_info_t;
typedef struct {
    float target_speed, integral;
    int16_t motor_out;
} rm_motor_base_info_t;
typedef struct {
    rm_motor_info_t *info;
    uint16_t rx_id;
    rm_motor_base_info_t base_info;
} rm_motor_t;
extern rm_motor_info_t rm_motor_info[CHAS_MOTOR_COUNT];
extern rm_motor_t rm_motor[CHAS_MOTOR_COUNT];
void Motor_Init(void);
void Motor_Update(void);
int Motor_Receive(uint32_t can_id, const uint8_t *data, uint32_t size, uint32_t now_ms);
/* Online mask bits follow IDs 0x201..0x204, independently of enum indices. */
uint8_t Motor_OnlineMask(uint32_t now_ms);
int Motor_HasFeedback(void);
void Motor_PackCurrent(uint8_t out[8]);
#endif
