/** @file shoot.c
 * @brief 国赛手动发射、持续反馈回读、拨盘找限位及堵转退弹。
 */
/* Includes ------------------------------------------------------------------*/
#include "shoot.h"
#include "shoot_config.h"
#include "rc_sensor.h"
#include "carctrl.h"
#include <stdlib.h>
#include <string.h>
/* Exported variables --------------------------------------------------------*/
shoot_t shoot; /**< 下板发射状态及累计诊断。 */
/* Private functions ---------------------------------------------------------*/
/** @brief 对称限幅，不改变输入标度。 */
static float limit(float x, float bound) { return x > bound ? bound : (x < -bound ? -bound : x); }
/** @brief 进入拨盘阶段并清除上阶段的堵转计时。 */
static void enter(shoot_state_t state, uint32_t now)
{
    shoot.state = state; shoot.state_ms = now; shoot.stuck_active = 0U;
}
/** @brief 发射关闭只清除动作和输入记忆，保留反馈与诊断。 */
static void stop(uint32_t now)
{
    shoot.enabled = shoot.fric_ready = 0U;
    shoot.motor_out = 0; shoot.target_speed = 0.0f;
    shoot.target_position = shoot.position;
    enter(SHOOT_OFF, now);
}
/** @brief 只接收当前可立即执行的请求，未就绪或忙碌时丢弃，不延期补发。 */
static void request(uint32_t now)
{
    ++shoot.requests;
    if (shoot.state != SHOOT_READY || !shoot.fric_ready || !shoot.feedback_online) { ++shoot.dropped_requests; return; }
    shoot.target_position += SHOOT_STEP_COUNTS;
    ++shoot.shots; enter(SHOOT_RELOAD, now);
}
/** @brief 新会话及输入来源切换时捕获电平，避免把保持的按键作为新动作。 */
static void capture_input(const rc_sensor_info_t *r)
{
    shoot.input_seen = 1U; shoot.input_source = (uint8_t)car.car_ctrl;
    shoot.last_keys = r->key_v; shoot.last_mouse = r->mouse_btn_l.value;
    shoot.wheel_armed = 0U;
}
/** @brief 双中档拨轮负端回中开关；右拨杆上档600ms一发；左键先开后单发。 */
static void inputs(uint32_t now)
{
    const rc_sensor_info_t *r = rc_sensor.info;
    if (!shoot.input_seen || shoot.input_source != (uint8_t)car.car_ctrl) { capture_input(r); return; }
    uint16_t pressed = r->key_v & (uint16_t)~shoot.last_keys;
    int click = r->mouse_btn_l.value && !shoot.last_mouse;
    if (car.car_ctrl == KEY_CTRL) {
        shoot.wheel_armed = 0U;
        if (pressed & RC_KEY_B) stop(now);
        else if (click && !shoot.enabled) { shoot.enabled = 1U; shoot.last_fire_ms = now; }
        else if (click && now - shoot.last_fire_ms >= SHOOT_FIRE_INTERVAL_MS) { shoot.last_fire_ms = now; request(now); }
        if ((pressed & RC_KEY_Q) && shoot.enabled) { shoot.target_position = shoot.position; enter(SHOOT_INIT, now); }
    } else {
        int middle = r->s1.value == 3U && r->s2.value == 3U;
        if (!middle) shoot.wheel_armed = 0U;
        else if (r->thumbwheel <= -600) shoot.wheel_armed = 1U;
        else if (shoot.wheel_armed && abs(r->thumbwheel) <= 10) {
            shoot.wheel_armed = 0U;
            if (shoot.enabled) stop(now);
            else { shoot.enabled = 1U; shoot.last_fire_ms = now; }
        }
        if (shoot.enabled && r->s2.value == 1U && now - shoot.last_fire_ms >= SHOOT_FIRE_INTERVAL_MS) {
            shoot.last_fire_ms = now; request(now);
        }
    }
    shoot.last_keys = r->key_v; shoot.last_mouse = r->mouse_btn_l.value;
}
/** @brief 使用真实时间确认堵转；过期反馈不参与堵转计时。 */
static int stuck(uint32_t now)
{
    if (abs(shoot.speed) <= SHOOT_STUCK_SPEED_RPM && abs(shoot.current) > SHOOT_STUCK_CURRENT) {
        if (!shoot.stuck_active) { shoot.stuck_active = 1U; shoot.stuck_since_ms = now; }
        return now - shoot.stuck_since_ms >= SHOOT_STUCK_MS;
    }
    shoot.stuck_active = 0U; return 0;
}
/* Exported functions --------------------------------------------------------*/
/** @brief 首次初始化清零；关控复位使用独立接口保留诊断。 */
void Shoot_Init(void) { memset(&shoot, 0, sizeof(shoot)); }
/** @brief 遥控关闭后取消动作，清除输入边沿及旧摩擦轮许可。 */
void Shoot_StandbyReset(uint32_t now)
{
    stop(now); shoot.input_seen = 0U; shoot.wheel_armed = 0U;
    shoot.last_keys = shoot.last_mouse = 0U; ++shoot.standby_resets;
}
/** @brief 持续接受拨弹反馈及摩擦轮摘要，坏数据丢弃且不续期。 */
int Shoot_Receive(uint32_t id, const uint8_t *d, unsigned n, uint32_t now)
{
    if (id == BOARD_FRIC_STATUS_ID) {
        board_fric_status_t s;
        if (!Board_Fric_DecodeStatus(d, n, &s)) { ++shoot.bad_status; return 1; }
        shoot.fric_status = s; shoot.fric_ms = now; ++shoot.fric_frames; return 1;
    }
    if (id != SHOOT_DIAL_FEEDBACK_ID) return 0;
    if (!d || n != 8U || ((unsigned)d[0] << 8 | d[1]) >= 8192U) { ++shoot.bad_feedback; return 1; }
    uint16_t angle = (uint16_t)((unsigned)d[0] << 8 | d[1]);
    if (shoot.feedback_frames) {
        int delta = (int)angle - shoot.encoder;
        if (delta > 4096) delta -= 8192;
        if (delta < -4096) delta += 8192;
        shoot.position += delta;
    } else shoot.position = angle;
    shoot.encoder = angle; shoot.speed = (int16_t)((unsigned)d[2] << 8 | d[3]);
    shoot.current = (int16_t)((unsigned)d[4] << 8 | d[5]);
    shoot.feedback_ms = now; ++shoot.feedback_frames; return 1;
}
/** @brief 执行发射局部状态机，不改变整车使能、Yaw或Pitch状态。 */
void Shoot_Update(uint32_t now)
{
    int was_online = shoot.feedback_online;
    shoot.feedback_online = shoot.feedback_frames && now - shoot.feedback_ms <= SHOOT_FEEDBACK_MS;
    shoot.fric_ready = shoot.fric_frames && now - shoot.fric_ms <= SHOOT_FEEDBACK_MS && shoot.fric_status.ready;
    if (!RC_Sensor_Online(now) || mec_output_enable != 1U || car.car_mode == sleep_car) {
        stop(now); shoot.input_seen = 0U; return;
    }
    inputs(now);
    if (!shoot.enabled) { shoot.motor_out = 0; return; }
    if (!shoot.feedback_online) {
        if (was_online) ++shoot.feedback_timeouts;
        shoot.motor_out = 0; shoot.target_speed = 0.0f; shoot.target_position = shoot.position;
        enter(SHOOT_OFF, now); return;
    }
    if (shoot.state == SHOOT_OFF) enter(SHOOT_INIT, now);
    if (shoot.state == SHOOT_RELOAD && !shoot.fric_ready) {
        shoot.target_position = shoot.position; ++shoot.dropped_requests; enter(SHOOT_READY, now);
    }
    if (shoot.state == SHOOT_INIT) {
        if (stuck(now) || now - shoot.state_ms >= SHOOT_INIT_TIMEOUT_MS) {
            if (now - shoot.state_ms >= SHOOT_INIT_TIMEOUT_MS) ++shoot.init_timeouts;
            shoot.target_position = shoot.position + SHOOT_INIT_OFFSET; enter(SHOOT_REVERT, now);
        }
    } else if (shoot.state == SHOOT_RELOAD && stuck(now)) {
        shoot.target_position = shoot.position - SHOOT_STEP_COUNTS;
        ++shoot.reversals; enter(SHOOT_REVERT, now);
    }
    if (shoot.state == SHOOT_RELOAD || shoot.state == SHOOT_REVERT) {
        int64_t error = shoot.target_position - shoot.position;
        if (llabs(error) <= SHOOT_SETTLE_COUNTS) enter(SHOOT_READY, now);
        else if (now - shoot.state_ms >= SHOOT_RELOAD_TIMEOUT_MS) {
            ++shoot.reload_timeouts; shoot.target_position = shoot.position; enter(SHOOT_READY, now);
        }
    }
    shoot.target_speed = shoot.state == SHOOT_INIT ? SHOOT_INIT_SPEED_RPM :
        limit((float)(shoot.target_position - shoot.position) * SHOOT_POSITION_KP, SHOOT_MAX_SPEED_RPM);
    shoot.motor_out = (int16_t)limit(SHOOT_SPEED_KP * (shoot.target_speed - shoot.speed),
        shoot.state == SHOOT_INIT ? SHOOT_INIT_MAX_CURRENT : SHOOT_MAX_CURRENT);
}
/** @brief 0x205电机使用0x1FF第0槽，其他槽明确保持零。 */
void Shoot_PackCurrent(uint8_t data[8])
{
    memset(data, 0, 8U); uint16_t value = (uint16_t)shoot.motor_out;
    data[0] = (uint8_t)(value >> 8); data[1] = (uint8_t)value;
}
/** @brief 发射总开关不越过遥控在线和整车输出许可。 */
board_fric_command_t Shoot_FricCommand(uint32_t now, uint8_t sequence)
{
    uint8_t online = RC_Sensor_Online(now) && mec_output_enable == 1U && car.car_mode != sleep_car;
    return (board_fric_command_t){online, online && shoot.enabled, sequence};
}
