#ifndef DOWN_CARCTRL_H
#define DOWN_CARCTRL_H
#include <stdint.h>
typedef enum { RC_CTRL } car_ctrl_e;
typedef enum { sleep_car = 0, mec_car = 1 } car_mode_e;
enum {
    CAR_BLOCK_OUTPUT = 1, CAR_BLOCK_REMOTE = 2, CAR_BLOCK_MOTOR = 4,
    CAR_BLOCK_CAN = 8, CAR_BLOCK_TIMING = 16
};
typedef struct car_t {
    car_ctrl_e car_ctrl;
    car_mode_e car_mode;
    uint32_t block_reason, loops, now_ms, dt_ms;
    uint8_t online_mask, off_seen, previous_switch;
    void (*work)(struct car_t *, uint32_t now_ms, uint32_t dt_ms, int can_ok);
} car_t;
extern car_t car;
/* Retain the existing debug command: set variable mec_output_enable = 1. */
extern volatile uint32_t mec_output_enable;
void Car_Init(void);
void Car_Work(car_t *c, uint32_t now_ms, uint32_t dt_ms, int can_ok);
#endif
