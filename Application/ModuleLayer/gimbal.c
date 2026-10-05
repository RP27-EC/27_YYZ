/**
 * @file    gimbal.c
 * @brief   遥控/键鼠Yaw角度P与速度PI串级、独立测试、停止确认及诊断。
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal.h"
#include "yaw_probe.h"
#include "kt_yaw_protocol.h"
#include "chassis_config.h"
#include "carctrl.h"
#include "rc_sensor.h"
#include <math.h>
#include <stdlib.h>

_Static_assert(GIMBAL_YAW_COMMAND_LIMIT > 0 && GIMBAL_YAW_COMMAND_LIMIT <= INT16_MAX &&
               GIMBAL_YAW_CURRENT_LIMIT > 0 && GIMBAL_YAW_CURRENT_LIMIT <= GIMBAL_YAW_COMMAND_LIMIT,
               "Yaw current limit must fit int16_t");
_Static_assert(GIMBAL_YAW_PULSE_RAW > 0 && GIMBAL_YAW_PULSE_RAW <= GIMBAL_YAW_COMMAND_LIMIT &&
               GIMBAL_YAW_PULSE_MS >= GIMBAL_YAW_PULSE_MIN_MS &&
               GIMBAL_YAW_PULSE_MS <= GIMBAL_YAW_PULSE_MAX_MS,
               "Yaw pulse must fit commissioning limit");

/* Private variables ---------------------------------------------------------*/
static uint32_t phase_ms, previous_ms, last_tx_ms; /**< 阶段、更新、发送时刻，毫秒。 */
static uint32_t sample_frames, sample_ms; /**< 最近差分速度采样计数和时刻。 */
static uint32_t zero_baseline, zero_encoder; /**< 清零前A1回复计数和本次标定零点。 */
static int32_t angle_direction, output_direction; /**< 本次角度和电流方向快照。 */
static float sample_deg; /**< 最近速度采样连续角，度。 */
static uint8_t tx_started, zero_sent; /**< 限频起点和本阶段首帧零输出是否入队。 */
static uint32_t waiting_request, wait_start_ms, ready_start_ms; /**< 待启动请求及恢复计时，毫秒。 */
static float waiting_offset; /**< 等待期间锁存的闭环改变量，度。 */
static int32_t waiting_direction; /**< 等待期间锁存的闭环输出方向。 */
static uint8_t remote_stable; /**< 全部启动条件是否已开始连续正常计时。 */
static uint32_t drive_reply_baseline; /**< 运动阶段最近已统计的A1回复计数。 */
static uint8_t drive_started; /**< 本次已经入队至少一帧非零电流命令。 */
static int32_t waiting_pulse_raw; /**< 等待遥控期间锁存的点动幅度，原始值。 */
static uint32_t waiting_pulse_ms; /**< 等待遥控期间锁存的点动时长，毫秒。 */
static uint32_t settle_streak_ms; /**< 当前连续到位时间，毫秒。 */
static float waiting_speed, waiting_kp, waiting_ki; /**< 遥控恢复期间锁存的请求4速度及PI增益。 */
static uint32_t waiting_speed_ms; /**< 遥控恢复期间锁存的请求4时长，毫秒；0持续运行。 */
static uint8_t remote_off_seen; /**< 空闲时曾收到右拨杆停止档，下一次中档可申请启动。 */
static car_ctrl_e remote_source; /**< 本轮遥控或键鼠输入来源，途中切换会退出。 */

/* Exported variables --------------------------------------------------------*/
gimbal_yaw_t gimbal_yaw; /**< Yaw控制观察对象。 */
volatile uint32_t gimbal_yaw_enable = GIMBAL_YAW_BOOT_OUTPUT_ENABLE; /**< 配置默认Yaw许可。 */
volatile uint32_t gimbal_yaw_remote_enable = GIMBAL_YAW_REMOTE_BOOT_ENABLE; /**< 配置默认遥控/键鼠入口。 */
volatile uint32_t gimbal_yaw_angle_enable = GIMBAL_YAW_CONTROL_ANGLE_ENABLE; /**< 配置默认遥控角度外环。 */
volatile uint32_t gimbal_yaw_request = 0; /**< 一次触发请求。 */
volatile float gimbal_yaw_offset_deg = 0; /**< 闭环目标改变量，度。 */
volatile int32_t gimbal_yaw_output_direction = GIMBAL_YAW_OUTPUT_DIRECTION; /**< 已实测确认的电流方向。 */
volatile int32_t gimbal_yaw_pulse_raw = GIMBAL_YAW_PULSE_RAW; /**< 下一次点动的正幅度，原始值。 */
volatile uint32_t gimbal_yaw_pulse_ms = GIMBAL_YAW_PULSE_MS; /**< 下一次点动时长，毫秒。 */
volatile float gimbal_yaw_speed_dps = 0; /**< 下一次请求4目标速度，度/秒。 */
volatile float gimbal_yaw_speed_kp = GIMBAL_YAW_TEST_SPEED_KP; /**< 下一次请求4的Kp，原始值/(度/秒)。 */
volatile float gimbal_yaw_speed_ki = GIMBAL_YAW_TEST_SPEED_KI; /**< 下一次请求4的Ki，原始值/度。 */
volatile uint32_t gimbal_yaw_speed_ms = GIMBAL_YAW_TEST_SPEED_MS; /**< 下一次请求4时长，毫秒；0持续运行。 */
volatile float yaw_scope_target_dps, yaw_scope_speed_dps; /**< 实时目标与滤波速度镜像，度/秒。 */
volatile float yaw_scope_current_raw, yaw_scope_integral_raw; /**< 实时电流指令和积分输出镜像，原始值。 */
volatile float yaw_scope_angle_target_deg, yaw_scope_angle_actual_deg, yaw_scope_angle_error_deg; /**< 连续目标/实际/误差镜像，度。 */
volatile uint32_t yaw_scope_state; /**< 实时状态枚举镜像，数值0..9。 */

/* Private functions ---------------------------------------------------------*/
/** @brief 将浮点数限制到给定区间。 */
static float clamp(float value, float lower, float upper)
{
    if (value > upper) { return upper; }
    if (value < lower) { return lower; }
    return value;
}

/** @brief 判断控制反馈是否新鲜、连续有效并且无已知错误位。 */
static int feedback_ready(uint32_t now, int starting)
{
    return yaw_probe.angle_valid && yaw_probe.continuous_valid &&
        (uint32_t)(now - yaw_probe.feedback.last_state_ms) <=
            (starting ? YAW_PROBE_OFFLINE_MS : GIMBAL_YAW_FEEDBACK_MS) &&
        yaw_probe.feedback.error_frames &&
        (uint32_t)(now - yaw_probe.feedback.last_error_ms) <= GIMBAL_YAW_ERROR_MS &&
        !yaw_probe.feedback.status.error_state;
}

/** @brief 检查本阶段首次零输出之后收到的新鲜且近静止A1回复。 */
static int zero_reply(uint32_t now)
{
    return zero_sent && yaw_probe.feedback.torque_frames != zero_baseline &&
        (uint32_t)(now - yaw_probe.feedback.last_torque_ms) <= GIMBAL_YAW_FEEDBACK_MS &&
        abs(yaw_probe.feedback.torque_current_raw) <= GIMBAL_YAW_ZERO_CURRENT &&
        abs(yaw_probe.feedback.torque_speed_dps) <= GIMBAL_YAW_ZERO_SPEED;
}

/** @brief 转入零输出确认，记录退出原因和测试位移。 */
static void stop(gimbal_yaw_reason_t reason, uint32_t now)
{
    if (gimbal_yaw.state == GIMBAL_STOPPING) { return; }
    gimbal_yaw.reason = reason;
    gimbal_yaw.exit_block_reason = reason == GIMBAL_REASON_INTERLOCK ? gimbal_yaw.interlock_block_reason : 0;
    if (gimbal_yaw.state == GIMBAL_PULSE || gimbal_yaw.state == GIMBAL_HOLD ||
        gimbal_yaw.state == GIMBAL_SPEED || gimbal_yaw.state == GIMBAL_REMOTE) {
        gimbal_yaw.active_ms = now - phase_ms;
    }
    gimbal_yaw.last_delta_deg = gimbal_yaw.actual_deg - gimbal_yaw.start_deg;
    gimbal_yaw.control_end_deg = gimbal_yaw.actual_deg;
    gimbal_yaw.control_end_error_deg = gimbal_yaw.goal_deg - gimbal_yaw.actual_deg;
    gimbal_yaw.state = GIMBAL_STOPPING;
    gimbal_yaw.current_raw = 0;
    gimbal_yaw.speed_integral_raw = 0;
    gimbal_yaw.zero_confirmed = 0;
    gimbal_yaw.stop_unconfirmed = 0;
    zero_sent = 0;
    tx_started = 0;
    phase_ms = now;
}

/** @brief 新状态帧上更新连续角差分速度，重复快照不重复积分。 */
static void update_speed(void)
{
    if (sample_frames == yaw_probe.feedback.state_frames) { return; }
    uint32_t elapsed = yaw_probe.feedback.last_state_ms - sample_ms;
    if (elapsed && elapsed <= GIMBAL_YAW_FEEDBACK_MS) {
        float measured = (gimbal_yaw.actual_deg - sample_deg) * (1000.0f / elapsed);
        gimbal_yaw.speed_estimate_dps += GIMBAL_YAW_SPEED_FILTER *
            (measured - gimbal_yaw.speed_estimate_dps);
    }
    sample_frames = yaw_probe.feedback.state_frames;
    sample_ms = yaw_probe.feedback.last_state_ms;
    sample_deg = gimbal_yaw.actual_deg;
}

/** @brief 在非零指令入队后的运动阶段记录新鲜A1回复峰值，结束后保留。 */
static void record_drive_feedback(uint32_t now)
{
    if (!drive_started || (gimbal_yaw.state != GIMBAL_PULSE && gimbal_yaw.state != GIMBAL_HOLD &&
                          gimbal_yaw.state != GIMBAL_SPEED && gimbal_yaw.state != GIMBAL_REMOTE) ||
        yaw_probe.feedback.torque_frames == drive_reply_baseline) { return; }
    uint32_t replies = yaw_probe.feedback.torque_frames - drive_reply_baseline;
    drive_reply_baseline = yaw_probe.feedback.torque_frames;
    if ((uint32_t)(now - yaw_probe.feedback.last_torque_ms) > GIMBAL_YAW_FEEDBACK_MS) { return; }
    gimbal_yaw.drive_reply_frames += replies;
    int16_t current = yaw_probe.feedback.torque_current_raw;
    int16_t speed = yaw_probe.feedback.torque_speed_dps;
    if (abs(current) > abs(gimbal_yaw.peak_current_raw)) { gimbal_yaw.peak_current_raw = current; }
    if (abs(speed) > abs(gimbal_yaw.peak_speed_dps)) { gimbal_yaw.peak_speed_dps = speed; }
}

/** @brief 用本轮锁存增益更新速度PI，输出饱和时禁止同向积分。 */
static void speed_control(uint32_t elapsed)
{
    float seconds = elapsed * 0.001f;
    float speed_error = gimbal_yaw.speed_target_dps - gimbal_yaw.speed_estimate_dps;
    float integral = clamp(gimbal_yaw.speed_integral_raw + gimbal_yaw.speed_ki * speed_error * seconds,
                           -GIMBAL_YAW_INTEGRAL_LIMIT, GIMBAL_YAW_INTEGRAL_LIMIT);
    float proposed = gimbal_yaw.speed_kp * speed_error + integral;
    if (!((proposed > GIMBAL_YAW_CURRENT_LIMIT && speed_error > 0) ||
          (proposed < -GIMBAL_YAW_CURRENT_LIMIT && speed_error < 0))) {
        gimbal_yaw.speed_integral_raw = integral;
    }
    float current = clamp(gimbal_yaw.speed_kp * speed_error + gimbal_yaw.speed_integral_raw,
                          -GIMBAL_YAW_CURRENT_LIMIT, GIMBAL_YAW_CURRENT_LIMIT);
    gimbal_yaw.current_raw = (int16_t)(output_direction * current);
}

/** @brief 更新角度斜坡、外环和连续到位记录，再调用速度内环。 */
static void hold_control(uint32_t elapsed)
{
    float seconds = elapsed * 0.001f;
    float step = GIMBAL_YAW_TARGET_RATE_DPS * seconds;
    gimbal_yaw.target_deg += clamp(gimbal_yaw.goal_deg - gimbal_yaw.target_deg, -step, step);
    gimbal_yaw.error_deg = gimbal_yaw.target_deg - gimbal_yaw.actual_deg;
    gimbal_yaw.speed_target_dps = clamp(GIMBAL_YAW_ANGLE_KP * gimbal_yaw.error_deg,
                                        -GIMBAL_YAW_MAX_SPEED_DPS, GIMBAL_YAW_MAX_SPEED_DPS);
    int settled = fabsf(gimbal_yaw.goal_deg - gimbal_yaw.target_deg) <= 0.001f &&
        fabsf(gimbal_yaw.goal_deg - gimbal_yaw.actual_deg) <= GIMBAL_YAW_SETTLE_DEG &&
        fabsf(gimbal_yaw.speed_estimate_dps) <= GIMBAL_YAW_SETTLE_SPEED_DPS &&
        abs(yaw_probe.feedback.status.speed_dps) <= GIMBAL_YAW_SETTLE_SPEED_DPS;
    if (settled) {
        settle_streak_ms += elapsed;
        if (settle_streak_ms > gimbal_yaw.best_settled_ms) { gimbal_yaw.best_settled_ms = settle_streak_ms; }
        gimbal_yaw.speed_integral_raw = 0;
        gimbal_yaw.speed_target_dps = 0;
        gimbal_yaw.current_raw = 0;
        return;
    }
    settle_streak_ms = 0;
    speed_control(elapsed);
}

/** @brief 检查请求参数，闭环必须已经确认输出方向。 */
static gimbal_yaw_reason_t request_reason(uint32_t request, float offset, int32_t direction)
{
    if (request < 1U || request > 5U || (request == 5U && gimbal_yaw_remote_enable != 1U) ||
        (request == 3U && (!isfinite(offset) || fabsf(offset) > GIMBAL_YAW_STEP_DEG))) {
        return GIMBAL_REASON_REQUEST;
    }
    if (request >= 3U && direction != 1 && direction != -1) { return GIMBAL_REASON_DIRECTION; }
    if (request == 5U && (!isfinite(GIMBAL_YAW_CONTROL_SPEED_KP) || GIMBAL_YAW_CONTROL_SPEED_KP < 0 ||
        !isfinite(GIMBAL_YAW_CONTROL_SPEED_KI) || GIMBAL_YAW_CONTROL_SPEED_KI < 0 ||
        !isfinite(GIMBAL_YAW_CONTROL_MAX_SPEED_DPS) || GIMBAL_YAW_CONTROL_MAX_SPEED_DPS <= 0 ||
        !isfinite(GIMBAL_YAW_MOUSE_SPEED_GAIN) || GIMBAL_YAW_MOUSE_SPEED_GAIN < 0)) {
        return GIMBAL_REASON_REQUEST;
    }
    if (request == 5U && (gimbal_yaw_angle_enable > 1U ||
        (gimbal_yaw_angle_enable && (!isfinite(GIMBAL_YAW_CONTROL_ANGLE_KP) || GIMBAL_YAW_CONTROL_ANGLE_KP < 0 ||
         !isfinite(GIMBAL_YAW_CONTROL_ANGLE_RATE_DPS) || GIMBAL_YAW_CONTROL_ANGLE_RATE_DPS <= 0)))) {
        return GIMBAL_REASON_REQUEST;
    }
    if (request == 4U && (!isfinite(gimbal_yaw_speed_dps) ||
        gimbal_yaw_speed_ms > GIMBAL_YAW_SPEED_MAX_MS ||
        !isfinite(gimbal_yaw_speed_kp) || gimbal_yaw_speed_kp < 0 || gimbal_yaw_speed_kp > GIMBAL_YAW_TEST_KP_MAX ||
        !isfinite(gimbal_yaw_speed_ki) || gimbal_yaw_speed_ki < 0 || gimbal_yaw_speed_ki > GIMBAL_YAW_TEST_KI_MAX)) {
        return GIMBAL_REASON_REQUEST;
    }
    if (request <= 2U && (gimbal_yaw_pulse_raw < 1 || gimbal_yaw_pulse_raw > GIMBAL_YAW_COMMAND_LIMIT ||
        gimbal_yaw_pulse_ms < GIMBAL_YAW_PULSE_MIN_MS || gimbal_yaw_pulse_ms > GIMBAL_YAW_PULSE_MAX_MS)) {
        return GIMBAL_REASON_REQUEST;
    }
    return GIMBAL_REASON_NONE;
}

/** @brief 检查请求；仅遥控失效时先限时等待，期间不发送电流帧。 */
static void begin(uint32_t request, uint32_t now, int interlock)
{
    gimbal_yaw.start_block_reason = gimbal_yaw.interlock_block_reason;
    gimbal_yaw.wait_remote_ms = 0;
    gimbal_yaw_reason_t reason = GIMBAL_REASON_NONE;
    float offset = gimbal_yaw_offset_deg;
    int32_t direction = gimbal_yaw_output_direction;
    if (gimbal_yaw_enable != 1U) { reason = GIMBAL_REASON_DISABLED; }
    else if (request != 5U && !interlock && gimbal_yaw.interlock_block_reason == GIMBAL_BLOCK_REMOTE &&
             request_reason(request, offset, direction) == GIMBAL_REASON_NONE) {
        gimbal_yaw.state = GIMBAL_WAIT_REMOTE;
        gimbal_yaw.reason = GIMBAL_REASON_INTERLOCK;
        gimbal_yaw.current_raw = 0;
        waiting_request = request;
        waiting_offset = offset;
        waiting_direction = direction;
        waiting_pulse_raw = gimbal_yaw_pulse_raw;
        waiting_pulse_ms = gimbal_yaw_pulse_ms;
        waiting_speed = gimbal_yaw_speed_dps;
        waiting_kp = gimbal_yaw_speed_kp;
        waiting_ki = gimbal_yaw_speed_ki;
        waiting_speed_ms = gimbal_yaw_speed_ms;
        zero_encoder = yaw_probe_zero_encoder;
        angle_direction = yaw_probe_direction;
        wait_start_ms = now;
        remote_stable = 0;
        return;
    }
    else if (!interlock) { reason = GIMBAL_REASON_INTERLOCK; }
    else if (!feedback_ready(now, 1)) { reason = GIMBAL_REASON_FEEDBACK; }
    else { reason = request_reason(request, offset, direction); }
    if (reason != GIMBAL_REASON_NONE) {
        gimbal_yaw.state = GIMBAL_FAULT;
        gimbal_yaw.reason = reason;
        return;
    }
    gimbal_yaw.state = GIMBAL_ZEROING;
    gimbal_yaw.reason = GIMBAL_REASON_NONE;
    gimbal_yaw.request = request;
    gimbal_yaw.pulse_raw = request >= 3U ? 0 : (int16_t)gimbal_yaw_pulse_raw;
    gimbal_yaw.pulse_ms = request >= 3U ? 0 : gimbal_yaw_pulse_ms;
    gimbal_yaw.duration_ms = request == 5U ? 0U : request == 4U ? gimbal_yaw_speed_ms :
        (request == 3U ? GIMBAL_YAW_SESSION_MS : gimbal_yaw.pulse_ms);
    gimbal_yaw.active_ms = gimbal_yaw.exit_block_reason = 0;
    ++gimbal_yaw.sessions;
    gimbal_yaw.start_deg = gimbal_yaw.target_deg = gimbal_yaw.actual_deg;
    gimbal_yaw.goal_deg = gimbal_yaw.start_deg + (request == 3U ? offset : 0);
    gimbal_yaw.current_raw = 0;
    gimbal_yaw.zero_confirmed = 0;
    gimbal_yaw.stop_unconfirmed = 0;
    gimbal_yaw.speed_estimate_dps = 0;
    gimbal_yaw.speed_integral_raw = 0;
    gimbal_yaw.speed_setpoint_dps = request == 4U ? gimbal_yaw_speed_dps : 0;
    gimbal_yaw.speed_kp = request == 5U ? GIMBAL_YAW_CONTROL_SPEED_KP :
        request == 4U ? gimbal_yaw_speed_kp : GIMBAL_YAW_SPEED_KP;
    gimbal_yaw.speed_ki = request == 5U ? GIMBAL_YAW_CONTROL_SPEED_KI :
        request == 4U ? gimbal_yaw_speed_ki : GIMBAL_YAW_SPEED_KI;
    gimbal_yaw.angle_control = request == 5U ? gimbal_yaw_angle_enable : 0U;
    gimbal_yaw.angle_kp = request == 5U ? GIMBAL_YAW_CONTROL_ANGLE_KP : GIMBAL_YAW_ANGLE_KP;
    gimbal_yaw.control_end_deg = gimbal_yaw.control_end_error_deg = 0;
    gimbal_yaw.best_settled_ms = settle_streak_ms = 0;
    gimbal_yaw.nonzero_confirmed = gimbal_yaw.drive_reply_frames = 0;
    gimbal_yaw.peak_command_raw = gimbal_yaw.peak_current_raw = gimbal_yaw.peak_speed_dps = 0;
    drive_started = 0;
    drive_reply_baseline = yaw_probe.feedback.torque_frames;
    gimbal_yaw.error_deg = gimbal_yaw.speed_target_dps = 0;
    sample_frames = yaw_probe.feedback.state_frames;
    sample_ms = yaw_probe.feedback.last_state_ms;
    sample_deg = gimbal_yaw.actual_deg;
    zero_encoder = yaw_probe_zero_encoder;
    angle_direction = yaw_probe_direction;
    output_direction = direction;
    if (request == 5U) { remote_source = car.car_ctrl; }
    phase_ms = now;
    tx_started = zero_sent = 0;
}

/** @brief 检查整车启动时四通道回中，键鼠还需释放平移按键并停止鼠标移动。 */
static uint32_t remote_neutral_blocks(void)
{
    const rc_sensor_info_t *rc = rc_sensor.info;
    uint32_t blocks = 0;
    if (abs(rc->ch0) > RC_ARM_NEUTRAL || abs(rc->ch1) > RC_ARM_NEUTRAL ||
        abs(rc->ch2) > RC_ARM_NEUTRAL || abs(rc->ch3) > RC_ARM_NEUTRAL) { blocks |= GIMBAL_BLOCK_STICKS; }
    if (car.car_ctrl == KEY_CTRL && ((rc->key_v & RC_KEY_MOVEMENT) || rc->mouse_vx != 0)) {
        blocks |= GIMBAL_BLOCK_KEYS;
    }
    return blocks;
}

/** @brief 空闲停止档记录启动手势，近静止时重建失效连续角，中档只申请一次启动。 */
static void remote_start(uint32_t now, int interlock)
{
    if (gimbal_yaw_enable != 1U || !RC_Sensor_Online(now)) { remote_off_seen = 0; return; }
    uint8_t sw = rc_sensor.info->s2.value;
    if (sw == RC_SW_UP || sw == RC_SW_DOWN) {
        remote_off_seen = 1;
        if (yaw_probe.angle_valid && !yaw_probe.continuous_valid &&
            abs(yaw_probe.feedback.status.current_raw) <= GIMBAL_YAW_ZERO_CURRENT &&
            abs(yaw_probe.feedback.status.speed_dps) <= GIMBAL_YAW_ZERO_SPEED) { yaw_probe_angle_reset = 1; }
        return;
    }
    if (sw != RC_SW_MID || !remote_off_seen) { return; }
    remote_off_seen = 0;
    uint32_t neutral_blocks = remote_neutral_blocks();
    gimbal_yaw.interlock_block_reason |= neutral_blocks;
    begin(5U, now, interlock && !neutral_blocks);
}

/** @brief 将右杆或鼠标X映射到角度目标变化速率/直接速度，正为逆时针。 */
static float remote_input_rate(void)
{
    const rc_sensor_info_t *rc = rc_sensor.info;
    float requested = 0;
    float limit = gimbal_yaw.angle_control ? GIMBAL_YAW_CONTROL_ANGLE_RATE_DPS : GIMBAL_YAW_CONTROL_MAX_SPEED_DPS;
    if (remote_source == KEY_CTRL) {
        if (!(rc->key_v & RC_KEY_CTRL)) {
            requested = GIMBAL_YAW_MOUSE_DIRECTION * rc->mouse_vx * GIMBAL_YAW_MOUSE_SPEED_GAIN;
        }
    } else if (rc->ch0 > RC_DEADBAND) {
        requested = GIMBAL_YAW_RC_DIRECTION * (rc->ch0 - RC_DEADBAND) *
            (limit / (660.0f - RC_DEADBAND));
    } else if (rc->ch0 < -RC_DEADBAND) {
        requested = GIMBAL_YAW_RC_DIRECTION * (rc->ch0 + RC_DEADBAND) *
            (limit / (660.0f - RC_DEADBAND));
    }
    return clamp(requested, -limit, limit);
}

/** @brief 累计连续目标并经角度P生成速度，或沿用直接速度；两者共用已调速度PI。 */
static void remote_control(uint32_t elapsed)
{
    float rate = remote_input_rate();
    if (gimbal_yaw.angle_control) {
        if (remote_source == KEY_CTRL && (rc_sensor.info->key_v & RC_KEY_CTRL)) {
            gimbal_yaw.target_deg = gimbal_yaw.actual_deg;
            gimbal_yaw.speed_integral_raw = 0;
        } else { gimbal_yaw.target_deg += rate * (elapsed * 0.001f); }
        gimbal_yaw.goal_deg = gimbal_yaw.target_deg;
        gimbal_yaw.error_deg = gimbal_yaw.target_deg - gimbal_yaw.actual_deg;
        gimbal_yaw.speed_target_dps = clamp(gimbal_yaw.angle_kp * gimbal_yaw.error_deg,
            -GIMBAL_YAW_CONTROL_MAX_SPEED_DPS, GIMBAL_YAW_CONTROL_MAX_SPEED_DPS);
    } else { gimbal_yaw.speed_target_dps = rate; }
    gimbal_yaw.speed_setpoint_dps = gimbal_yaw.speed_target_dps;
    speed_control(elapsed);
}

/** @brief 等待遥控及全部条件连续恢复，超时、撤销或其他故障立即取消。 */
static void wait_remote(uint32_t now, int interlock)
{
    gimbal_yaw.wait_remote_ms = now - wait_start_ms;
    gimbal_yaw_reason_t reason = GIMBAL_REASON_NONE;
    if (gimbal_yaw_enable != 1U) { reason = GIMBAL_REASON_DISABLED; }
    else if (gimbal_yaw.interlock_block_reason & ~GIMBAL_BLOCK_REMOTE) { reason = GIMBAL_REASON_INTERLOCK; }
    else if (zero_encoder != yaw_probe_zero_encoder || angle_direction != yaw_probe_direction ||
             (waiting_request >= 3U && waiting_direction != gimbal_yaw_output_direction) ||
             (waiting_request == 3U && waiting_offset != gimbal_yaw_offset_deg) ||
             (waiting_request == 4U && (waiting_speed != gimbal_yaw_speed_dps ||
                                       waiting_kp != gimbal_yaw_speed_kp || waiting_ki != gimbal_yaw_speed_ki ||
                                       waiting_speed_ms != gimbal_yaw_speed_ms)) ||
             (waiting_request <= 2U && (waiting_pulse_raw != gimbal_yaw_pulse_raw ||
                                        waiting_pulse_ms != gimbal_yaw_pulse_ms))) { reason = GIMBAL_REASON_CALIBRATION; }
    else if (gimbal_yaw.wait_remote_ms >= GIMBAL_YAW_REMOTE_WAIT_MS) { reason = GIMBAL_REASON_REMOTE_TIMEOUT; }
    if (reason != GIMBAL_REASON_NONE) {
        gimbal_yaw.state = GIMBAL_FAULT;
        gimbal_yaw.reason = reason;
        waiting_request = 0;
        return;
    }
    if (!interlock || gimbal_yaw.interlock_block_reason) { remote_stable = 0; return; }
    if (!remote_stable) { ready_start_ms = now; remote_stable = 1; }
    if ((uint32_t)(now - ready_start_ms) < GIMBAL_YAW_REMOTE_STABLE_MS) { return; }
    uint32_t original_blocks = gimbal_yaw.start_block_reason;
    uint32_t waited_ms = gimbal_yaw.wait_remote_ms;
    begin(waiting_request, now, interlock);
    gimbal_yaw.start_block_reason = original_blocks;
    gimbal_yaw.wait_remote_ms = waited_ms;
    waiting_request = 0;
}

/** @brief 更新点动、独立速度环、角度串级控制和退出清零状态。 */
static void update_control(uint32_t now, int interlock)
{
    uint32_t elapsed = now - previous_ms;
    previous_ms = now;
    gimbal_yaw.actual_deg = yaw_probe.continuous_deg;
    uint32_t request = gimbal_yaw_request;
    gimbal_yaw_request = 0;
    if (gimbal_yaw.request == 5U && Gimbal_Yaw_OwnsBus() && remote_source != car.car_ctrl) {
        gimbal_yaw.interlock_block_reason |= GIMBAL_BLOCK_INPUT_SOURCE;
        interlock = 0;
    }
    if (gimbal_yaw.state == GIMBAL_WAIT_REMOTE) {
        wait_remote(now, interlock);
        return;
    }
    if (!Gimbal_Yaw_OwnsBus()) {
        if (gimbal_yaw_remote_enable == 1U) { remote_start(now, interlock); }
        else if (request) { begin(request, now, interlock); }
        return;
    }
    record_drive_feedback(now);
    if (gimbal_yaw.state != GIMBAL_STOPPING) {
        if (gimbal_yaw_enable != 1U || (gimbal_yaw.request == 5U && gimbal_yaw_remote_enable != 1U)) {
            stop(GIMBAL_REASON_DISABLED, now);
        }
        else if (!interlock) { stop(GIMBAL_REASON_INTERLOCK, now); }
        else if (!feedback_ready(now, gimbal_yaw.state == GIMBAL_ZEROING)) { stop(GIMBAL_REASON_FEEDBACK, now); }
        else if (elapsed > CHASSIS_MAX_PERIOD_MS) { stop(GIMBAL_REASON_PERIOD, now); }
        else if (zero_encoder != yaw_probe_zero_encoder || angle_direction != yaw_probe_direction ||
                 (gimbal_yaw.request >= 3U && output_direction != gimbal_yaw_output_direction) ||
                 (gimbal_yaw.request == 5U && gimbal_yaw.angle_control != gimbal_yaw_angle_enable)) {
            stop(GIMBAL_REASON_CALIBRATION, now);
        }
    }
    if (gimbal_yaw.state == GIMBAL_STOPPING) {
        gimbal_yaw.current_raw = 0;
        if (gimbal_yaw.zero_confirmed >= GIMBAL_YAW_ZERO_CONFIRM_COUNT && zero_reply(now)) {
            gimbal_yaw.last_delta_deg = gimbal_yaw.actual_deg - gimbal_yaw.start_deg;
            gimbal_yaw.state = gimbal_yaw.reason == GIMBAL_REASON_COMPLETE ||
                gimbal_yaw.reason == GIMBAL_REASON_DISABLED ? GIMBAL_DONE : GIMBAL_FAULT;
        } else if ((uint32_t)(now - phase_ms) >= GIMBAL_YAW_STOP_WARN_MS) { gimbal_yaw.stop_unconfirmed = 1; }
        return;
    }
    if (gimbal_yaw.state == GIMBAL_ZEROING) {
        if (gimbal_yaw.zero_confirmed && zero_reply(now)) {
            gimbal_yaw.state = gimbal_yaw.request == 5U ? GIMBAL_REMOTE : gimbal_yaw.request == 4U ? GIMBAL_SPEED :
                (gimbal_yaw.request == 3U ? GIMBAL_HOLD : GIMBAL_PULSE);
            if (gimbal_yaw.request == 5U && gimbal_yaw.angle_control) {
                gimbal_yaw.start_deg = gimbal_yaw.target_deg = gimbal_yaw.goal_deg = gimbal_yaw.actual_deg;
                gimbal_yaw.error_deg = 0;
                sample_frames = yaw_probe.feedback.state_frames;
                sample_ms = yaw_probe.feedback.last_state_ms;
                sample_deg = gimbal_yaw.actual_deg;
            }
            phase_ms = now;
        } else if ((uint32_t)(now - phase_ms) >= GIMBAL_YAW_ZEROING_MS) { stop(GIMBAL_REASON_ZERO_TIMEOUT, now); }
        return;
    }
    update_speed();
    float travel = gimbal_yaw.state == GIMBAL_PULSE ? GIMBAL_YAW_PULSE_TRAVEL_DEG : GIMBAL_YAW_TRAVEL_DEG;
    if (gimbal_yaw.state != GIMBAL_SPEED && gimbal_yaw.state != GIMBAL_REMOTE &&
        fabsf(gimbal_yaw.actual_deg - gimbal_yaw.start_deg) >= travel) {
        stop(GIMBAL_REASON_TRAVEL, now); return;
    }
    float speed_guard = gimbal_yaw.state == GIMBAL_PULSE ? GIMBAL_YAW_PULSE_SPEED_DPS : GIMBAL_YAW_SPEED_GUARD_DPS;
    if (gimbal_yaw.state == GIMBAL_SPEED) {
        speed_guard = fmaxf(speed_guard, fabsf(gimbal_yaw.speed_setpoint_dps) + GIMBAL_YAW_TEST_SPEED_MARGIN_DPS);
    } else if (gimbal_yaw.state == GIMBAL_REMOTE) {
        speed_guard = fmaxf(speed_guard, GIMBAL_YAW_CONTROL_MAX_SPEED_DPS + GIMBAL_YAW_TEST_SPEED_MARGIN_DPS);
    }
    if (fabsf(gimbal_yaw.speed_estimate_dps) > speed_guard ||
        abs(yaw_probe.feedback.status.speed_dps) > speed_guard) { stop(GIMBAL_REASON_SPEED, now); return; }
    gimbal_yaw.active_ms = now - phase_ms;
    if (gimbal_yaw.duration_ms && gimbal_yaw.active_ms >= gimbal_yaw.duration_ms) {
        stop(GIMBAL_REASON_COMPLETE, now); return;
    }
    if (gimbal_yaw.state == GIMBAL_PULSE) {
        gimbal_yaw.current_raw = gimbal_yaw.request == 1U ? gimbal_yaw.pulse_raw : -gimbal_yaw.pulse_raw;
    } else if (gimbal_yaw.state == GIMBAL_REMOTE) { remote_control(elapsed); }
    else if (gimbal_yaw.state == GIMBAL_SPEED) {
        gimbal_yaw.speed_target_dps = gimbal_yaw.speed_setpoint_dps;
        speed_control(elapsed);
    } else { hold_control(elapsed); }
}

/* Exported functions --------------------------------------------------------*/
/** @brief 恢复配置默认许可/已验证方向，清除请求和启动手势，不自动运动。 */
void Gimbal_Init(void)
{
    gimbal_yaw = (gimbal_yaw_t){0};
    gimbal_yaw_enable = GIMBAL_YAW_BOOT_OUTPUT_ENABLE;
    gimbal_yaw_remote_enable = GIMBAL_YAW_REMOTE_BOOT_ENABLE;
    gimbal_yaw_angle_enable = GIMBAL_YAW_CONTROL_ANGLE_ENABLE;
    gimbal_yaw_request = 0;
    gimbal_yaw_offset_deg = 0;
    gimbal_yaw_output_direction = GIMBAL_YAW_OUTPUT_DIRECTION;
    gimbal_yaw_pulse_raw = GIMBAL_YAW_PULSE_RAW;
    gimbal_yaw_pulse_ms = GIMBAL_YAW_PULSE_MS;
    gimbal_yaw_speed_dps = 0;
    gimbal_yaw_speed_kp = GIMBAL_YAW_TEST_SPEED_KP;
    gimbal_yaw_speed_ki = GIMBAL_YAW_TEST_SPEED_KI;
    gimbal_yaw_speed_ms = GIMBAL_YAW_TEST_SPEED_MS;
    yaw_scope_target_dps = yaw_scope_speed_dps = 0;
    yaw_scope_current_raw = yaw_scope_integral_raw = 0;
    yaw_scope_angle_target_deg = yaw_scope_angle_actual_deg = yaw_scope_angle_error_deg = 0;
    yaw_scope_state = GIMBAL_IDLE;
    phase_ms = previous_ms = last_tx_ms = sample_frames = sample_ms = 0;
    zero_baseline = zero_encoder = 0;
    angle_direction = output_direction = 0;
    sample_deg = 0;
    tx_started = zero_sent = 0;
    waiting_request = wait_start_ms = ready_start_ms = 0;
    waiting_offset = 0;
    waiting_direction = 0;
    remote_stable = 0;
    drive_reply_baseline = 0;
    drive_started = 0;
    waiting_pulse_raw = 0;
    waiting_pulse_ms = 0;
    settle_streak_ms = 0;
    waiting_speed = waiting_kp = waiting_ki = 0;
    waiting_speed_ms = 0;
    remote_off_seen = 0;
    remote_source = RC_CTRL;
}

/** @brief 更新控制并发布角度、速度和电流标量镜像，便于J-Scope采集。 */
void Gimbal_Yaw_Update(uint32_t now, int interlock)
{
    update_control(now, interlock);
    yaw_scope_target_dps = gimbal_yaw.state == GIMBAL_HOLD || gimbal_yaw.state == GIMBAL_SPEED ||
        gimbal_yaw.state == GIMBAL_REMOTE ?
        gimbal_yaw.speed_target_dps : 0;
    yaw_scope_speed_dps = gimbal_yaw.speed_estimate_dps;
    yaw_scope_current_raw = gimbal_yaw.current_raw;
    yaw_scope_integral_raw = gimbal_yaw.speed_integral_raw;
    yaw_scope_angle_target_deg = gimbal_yaw.target_deg;
    yaw_scope_angle_actual_deg = gimbal_yaw.actual_deg;
    yaw_scope_angle_error_deg = gimbal_yaw.target_deg - gimbal_yaw.actual_deg;
    yaw_scope_state = gimbal_yaw.state;
}

/** @brief 遥控入口启用或其会话尚在清零时使用整车共享CAN模式。 */
int Gimbal_Yaw_RemoteMode(void)
{
    return gimbal_yaw_remote_enable == 1U || (gimbal_yaw.request == 5U && Gimbal_Yaw_OwnsBus());
}

/** @brief 清零、点动、角度/独立速度/遥控速度或停止确认时生成Yaw帧。 */
int Gimbal_Yaw_OwnsBus(void)
{
    return (gimbal_yaw.state >= GIMBAL_ZEROING && gimbal_yaw.state <= GIMBAL_STOPPING) ||
        gimbal_yaw.state == GIMBAL_SPEED || gimbal_yaw.state == GIMBAL_REMOTE;
}

/** @brief 清零阶段及无运动输出时要求撤销旧的非零排队帧。 */
int Gimbal_Yaw_NeedsZero(void)
{
    return gimbal_yaw_enable != 1U || !Gimbal_Yaw_OwnsBus() ||
        gimbal_yaw.state == GIMBAL_ZEROING || gimbal_yaw.state == GIMBAL_STOPPING;
}

/** @brief 限频生成电流帧；点动/闭环插入错误状态查询，清零阶段仅发零电流。 */
int Gimbal_Yaw_MakeCommand(uint32_t now, uint8_t out[8])
{
    if (!Gimbal_Yaw_OwnsBus()) { return 0; }
    if (tx_started && (uint32_t)(now - last_tx_ms) < GIMBAL_YAW_TX_MS) { return 0; }
    if (!Gimbal_Yaw_NeedsZero() &&
        gimbal_yaw.tx_queued % GIMBAL_YAW_STATUS_DIVIDER == GIMBAL_YAW_STATUS_DIVIDER - 1U) {
        return KT_Yaw_BuildRead(YAW_PROBE_STATE1, out);
    }
    int16_t current = Gimbal_Yaw_NeedsZero() ? 0 : gimbal_yaw.current_raw;
    return KT_Yaw_BuildCurrent(current, out);
}

/** @brief 成功入队后推进节拍，并记录清零回复的起始计数。 */
void Gimbal_Yaw_Queued(uint32_t now, int16_t current, uint8_t command)
{
    ++gimbal_yaw.tx_queued;
    last_tx_ms = now;
    tx_started = 1;
    if (command == GIMBAL_YAW_TORQUE_COMMAND && current && !Gimbal_Yaw_NeedsZero()) {
        if (!drive_started) { drive_reply_baseline = yaw_probe.feedback.torque_frames; drive_started = 1; }
        if (abs(current) > abs(gimbal_yaw.peak_command_raw)) { gimbal_yaw.peak_command_raw = current; }
    }
    if (Gimbal_Yaw_NeedsZero() && command == GIMBAL_YAW_TORQUE_COMMAND && !current && !zero_sent) {
        zero_baseline = yaw_probe.feedback.torque_frames;
        zero_sent = 1;
    }
}

/** @brief 控制器发送成功与目标电机回复分别统计，不以入队作为成功。 */
void Gimbal_Yaw_TxComplete(int16_t current, uint8_t command, int success)
{
    if (!success) { Gimbal_Yaw_TxError(); return; }
    ++gimbal_yaw.tx_confirmed;
    if (command == GIMBAL_YAW_TORQUE_COMMAND && current) { ++gimbal_yaw.nonzero_confirmed; }
    if (Gimbal_Yaw_NeedsZero() && command == GIMBAL_YAW_TORQUE_COMMAND && !current && zero_sent) {
        ++gimbal_yaw.zero_confirmed;
    }
}

/** @brief 发送异常时锁定退出原因，持续尝试零输出。 */
void Gimbal_Yaw_TxError(void)
{
    ++gimbal_yaw.tx_errors;
    if (Gimbal_Yaw_OwnsBus()) { stop(GIMBAL_REASON_CAN, previous_ms); }
}
