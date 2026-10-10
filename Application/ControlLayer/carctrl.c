/**
 * @file carctrl.c
 * @brief 整车主动许可和遥控离线停机，通信与采样异常仅发布诊断。
 */
/* Includes ------------------------------------------------------------------*/
#include "carctrl.h"
#include "rc_sensor.h"
#include "motor.h"
#include "chassis_config.h"
#include "gyro_control.h"
#include <stdlib.h>

/* Exported variables --------------------------------------------------------*/
volatile uint32_t car_mode_select = CAR_BOOT_MODE; /**< 上电模式选择：1机械，2陀螺仪。 */
car_t car = {.work = Car_Work};
volatile uint32_t mec_output_enable = CHASSIS_BOOT_OUTPUT_ENABLE;

/* Private functions ---------------------------------------------------------*/
/** @brief 双下档正端回中或键鼠V按下沿切换基础模式，跨档拨轮手势丢弃。 */
static void mode_input(car_t *c, const rc_sensor_info_t *r)
{
    int toggle = c->car_ctrl == KEY_CTRL && (r->key_v & RC_KEY_V) && !(c->previous_keys & RC_KEY_V);
    if (r->s1.value != RC_SW_DOWN || r->s2.value != RC_SW_DOWN) c->mode_wheel_armed = 0U;
    else if (r->thumbwheel >= CAR_MODE_WHEEL_TRIGGER) c->mode_wheel_armed = 1U;
    else if (c->mode_wheel_armed && abs(r->thumbwheel) <= CAR_MODE_WHEEL_CENTER) {
        c->mode_wheel_armed = 0U;
        toggle = 1;
    }
    if (toggle) car_mode_select = c->selected_mode == mec_car ? gyro_car : mec_car;
}

/** @brief F保持/Ctrl按下退出，遥控上中拨杆加拨轮负端回中切换。 */
static void cycle_input(car_t *c, const rc_sensor_info_t *r)
{
    if (c->car_ctrl == KEY_CTRL) {
        c->cycle_requested = (r->key_v & RC_KEY_F) != 0U;
        if ((r->key_v & RC_KEY_CTRL) && !(c->previous_keys & RC_KEY_CTRL)) c->cycle_requested = 0U;
        c->cycle_wheel_armed = 0U;
    } else {
        if (r->s1.value != RC_SW_UP || r->s2.value != RC_SW_MID) c->cycle_wheel_armed = 0U;
        else if (r->thumbwheel <= -600) c->cycle_wheel_armed = 1U;
        else if (c->cycle_wheel_armed && abs(r->thumbwheel) <= 10) {
            c->cycle_wheel_armed = 0U;
            c->cycle_requested ^= 1U;
        }
    }
    c->previous_keys = r->key_v;
}

/* Exported functions --------------------------------------------------------*/
/** @brief 重置总使能、默认陀螺仪模式及输入手势。 */
void Car_Init(void)
{
    car = (car_t){.car_ctrl = RC_CTRL, .car_mode = sleep_car, .standby_pending = 1U, .selected_mode = CAR_BOOT_MODE, .work = Car_Work};
    mec_output_enable = CHASSIS_BOOT_OUTPUT_ENABLE;
    car_mode_select = CAR_BOOT_MODE;
}

/** @brief 清除运动会话及启动手势，保留配置和累计诊断，不等待上板ACK。 */
void Car_StandbyReset(car_t *c, uint32_t now_ms)
{
    uint32_t selected = car_mode_select == mec_car || car_mode_select == gyro_car ? car_mode_select : c->selected_mode;
    *c = (car_t){.car_ctrl = RC_CTRL, .car_mode = sleep_car, .selected_mode = selected,
        .standby_pending = 1U, .loops = c->loops, .now_ms = now_ms, .work = Car_Work,
        .timing_errors = c->timing_errors, .invalid_modes = c->invalid_modes, .cycle_entries = c->cycle_entries};
}

/** @brief 两模式统一双下档键鼠，无拨杆停机或回中启动门槛，保留遥控离线停机。 */
void Car_Work(car_t *c, uint32_t now_ms, uint32_t dt_ms, int can_ok)
{
    ++c->loops;
    c->now_ms = now_ms;
    c->raw_dt_ms = dt_ms;
    c->diag_reason = 0U;
    c->dt_ms = dt_ms && dt_ms <= CHASSIS_MAX_PERIOD_MS ? dt_ms : CHASSIS_CONTROL_MS;
    if (!dt_ms || dt_ms > CHASSIS_MAX_PERIOD_MS) {
        ++c->timing_errors;
        c->diag_reason |= CAR_BLOCK_TIMING;
    }
    uint32_t mode = car_mode_select;
    if (mode != mec_car && mode != gyro_car) {
        ++c->invalid_modes;
        mode = c->selected_mode == mec_car ? mec_car : gyro_car;
    }
    c->remote_online = (uint8_t)RC_Sensor_Online(now_ms);
    const rc_sensor_info_t *r = rc_sensor.info;
    car_ctrl_e requested = (c->remote_online && r->s1.value == RC_SW_DOWN && r->s2.value == RC_SW_DOWN) ? KEY_CTRL : RC_CTRL;
    if (!c->input_seen || c->car_ctrl != requested) {
        c->previous_keys = r->key_v;
        c->mode_wheel_armed = c->cycle_wheel_armed = c->cycle_requested = 0U;
        c->input_seen = 1U;
    }
    c->car_ctrl = requested;
    c->block_reason = 0;
    c->online_mask = Motor_OnlineMask(now_ms);
    if (mec_output_enable != 1) { c->block_reason |= CAR_BLOCK_OUTPUT; }
    if (!RC_Sensor_Online(now_ms)) { c->block_reason |= CAR_BLOCK_REMOTE; }
    if (c->online_mask != 15) { c->block_reason |= CAR_BLOCK_MOTOR; }
    if (!can_ok) { c->diag_reason |= CAR_BLOCK_CAN; }
    c->imu_age_ms = now_ms - gyro_control.last_ms;
    if (!Gyro_Ready(now_ms)) { c->diag_reason |= CAR_BLOCK_IMU; }
    if (!gyro_control.mechanical_ready || gyro_control.fault) { c->diag_reason |= CAR_BLOCK_YAW; }
    c->fault_latched = 0U;
    c->arm_waiting = 0U;
    if (c->block_reason) {
        c->car_mode = sleep_car;
        c->cycle_requested = c->cycle_wheel_armed = 0U;
        c->mode_wheel_armed = c->input_seen = 0U;
        c->previous_keys = r->key_v;
        if (!c->remote_online || mec_output_enable != 1U) {
            c->off_seen = c->previous_switch = 0U;
        }
        return;
    }
    c->selected_mode = mode;
    mode_input(c, r);
    if (car_mode_select == mec_car || car_mode_select == gyro_car) c->selected_mode = car_mode_select;
    cycle_input(c, r);
    car_mode_e next = c->cycle_requested ? cycle_car : (car_mode_e)c->selected_mode;
    if (c->car_mode == sleep_car) ++c->sessions;
    if (next == cycle_car && c->car_mode != cycle_car) ++c->cycle_entries;
    c->car_mode = next;
}
