#include "motor.h"
rm_motor_info_t rm_motor_info[CHAS_MOTOR_COUNT];
rm_motor_t rm_motor[CHAS_MOTOR_COUNT] = {
    [CHAS_LF] = {.info = &rm_motor_info[CHAS_LF], .rx_id = ID_CHAS_LF},
    [CHAS_LB] = {.info = &rm_motor_info[CHAS_LB], .rx_id = ID_CHAS_LB},
    [CHAS_RF] = {.info = &rm_motor_info[CHAS_RF], .rx_id = ID_CHAS_RF},
    [CHAS_RB] = {.info = &rm_motor_info[CHAS_RB], .rx_id = ID_CHAS_RB}
};
static volatile rm_motor_info_t received_motor[CHAS_MOTOR_COUNT];
void Motor_Init(void)
{
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) {
        rm_motor_info[i] = (rm_motor_info_t){0};
        received_motor[i] = (rm_motor_info_t){0};
        rm_motor[i].base_info = (rm_motor_base_info_t){0};
    }
}
void Motor_Update(void)
{
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) { rm_motor_info[i] = received_motor[i]; }
}
int Motor_Receive(uint32_t can_id, const uint8_t *d, uint32_t size, uint32_t now_ms)
{
    if (size != 8) { return 0; }
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) {
        if (can_id != rm_motor[i].rx_id) { continue; }
        volatile rm_motor_info_t *m = &received_motor[i];
        m->angle = (uint16_t)((d[0] << 8) | d[1]);
        m->speed = (int16_t)((d[2] << 8) | d[3]);
        m->current = (int16_t)((d[4] << 8) | d[5]);
        m->temperature = d[6];
        m->last_ms = now_ms;
        ++m->frames;
        return 1;
    }
    return 0;
}
uint8_t Motor_OnlineMask(uint32_t now_ms)
{
    uint8_t mask = 0;
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) {
        if (rm_motor[i].info->frames &&
            (uint32_t)(now_ms - rm_motor[i].info->last_ms) <= CHASSIS_OFFLINE_MS) {
            mask |= 1U << (rm_motor[i].rx_id - 0x201U);
        }
    }
    return mask;
}
int Motor_HasFeedback(void)
{
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) { if (rm_motor[i].info->frames) { return 1; } }
    return 0;
}
void Motor_PackCurrent(uint8_t out[8])
{
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) {
        unsigned slot = rm_motor[i].rx_id - 0x201U;
        uint16_t value = (uint16_t)rm_motor[i].base_info.motor_out;
        out[2 * slot] = (uint8_t)(value >> 8);
        out[2 * slot + 1] = (uint8_t)value;
    }
}
