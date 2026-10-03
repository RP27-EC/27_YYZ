#include "carctrl.h"
#include "rc_sensor.h"
#include "motor.h"
#include "chassis_config.h"

car_t car = {.work = Car_Work};
volatile uint32_t mec_output_enable = 0;

void Car_Init(void)
{
    car = (car_t){.car_ctrl = RC_CTRL, .car_mode = sleep_car, .work = Car_Work};
    mec_output_enable = 0;
}

void Car_Work(car_t *c, uint32_t now_ms, uint32_t dt_ms, int can_ok)
{
    ++c->loops;
    c->now_ms = now_ms;
    c->dt_ms = dt_ms;
    c->block_reason = 0;
    c->online_mask = Motor_OnlineMask(now_ms);
    if (mec_output_enable != 1) { c->block_reason |= CAR_BLOCK_OUTPUT; }
    if (!RC_Sensor_Online(now_ms)) { c->block_reason |= CAR_BLOCK_REMOTE; }
    if (c->online_mask != 15) { c->block_reason |= CAR_BLOCK_MOTOR; }
    if (!can_ok) { c->block_reason |= CAR_BLOCK_CAN; }
    if (!dt_ms || dt_ms > CHASSIS_MAX_PERIOD_MS) { c->block_reason |= CAR_BLOCK_TIMING; }
    if (c->block_reason) {
        c->car_mode = sleep_car;
        c->off_seen = 0;
        c->previous_switch = 0;
        return;
    }
    const rc_sensor_info_t *r = rc_sensor.info;
    uint8_t sw = r->s2.value;
    if (sw != RC_SW_MID) {
        c->car_mode = sleep_car;
        c->off_seen = 1;
    } else if (c->car_mode != mec_car) {
        int neutral = r->ch0 >= -RC_ARM_NEUTRAL && r->ch0 <= RC_ARM_NEUTRAL &&
                      r->ch2 >= -RC_ARM_NEUTRAL && r->ch2 <= RC_ARM_NEUTRAL &&
                      r->ch3 >= -RC_ARM_NEUTRAL && r->ch3 <= RC_ARM_NEUTRAL;
        if (c->off_seen && c->previous_switch != RC_SW_MID && neutral) {
            c->car_mode = mec_car;
        }
        c->off_seen = 0;
    }
    c->previous_switch = sw;
}
