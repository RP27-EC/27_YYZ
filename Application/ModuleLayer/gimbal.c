/**
 * @file    gimbal.c
 * @brief   机械编码器/陀螺仪IMU Yaw串级、输入、独立测试与停止确认。
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal.h"
#include "yaw_probe.h"
#include "kt_yaw_protocol.h"
#include "chassis_config.h"
#include "carctrl.h"
#include "gyro_control.h"
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
static uint32_t overspeed_started_ms; /**< 新鲜反馈连续超速起点，毫秒。 */
static uint32_t overspeed_check_ms; /**< 上次超速检查时刻，毫秒；长间隔不累加。 */
static uint8_t overspeed_active; /**< 连续超速计时是否有效。 */
static uint32_t phase_ms, previous_ms, last_tx_ms; /**< 阶段、更新、发送时刻，毫秒。 */
static uint32_t sample_frames, sample_ms; /**< 最近差分速度采样计数和时刻。 */
static uint32_t zero_encoder; /**< 本次标定零点，编码器计数。 */
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
static uint32_t turn_ms; /**< 转向开始时刻，毫秒。 */
static uint16_t last_keys; /**< 键盘上周期按键，检测配置换头键的按下沿。 */
static uint8_t wheel_up_seen; /**< 拨轮负端后回中事件记忆。 */
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
volatile float yaw_scope_angle_kp; /**< 本周期角度外环实际Kp镜像，1/秒。 */
volatile uint32_t yaw_scope_state; /**< 实时状态枚举镜像，数值0..9。 */

/* Private functions ---------------------------------------------------------*/
/** @brief 将浮点数限制到给定区间。 */
static float clamp(float value, float lower, float upper)
{
    if (value > upper) { return upper; }
    if (value < lower) { return lower; }
    return value;
}

/** @brief 采样年龄只记录；已上报的电机内部错误保留停机判据。 */
static int motor_fault(void)
{
    return yaw_probe.feedback.error_frames && yaw_probe.feedback.status.error_state;
}

/** @brief 发布反馈诊断，保留上次合法数据并持续查询。 */
static void record_feedback(uint32_t now)
{
    gimbal_yaw.feedback_age_ms = now - yaw_probe.feedback.last_state_ms;
    gimbal_yaw.error_age_ms = now - yaw_probe.feedback.last_error_ms;
    gimbal_yaw.imu_age_ms = now - gyro_control.last_ms;
    if (!yaw_probe.angle_valid || !yaw_probe.continuous_valid ||
        gimbal_yaw.feedback_age_ms > GIMBAL_YAW_FEEDBACK_MS ||
        !yaw_probe.feedback.error_frames || gimbal_yaw.error_age_ms > GIMBAL_YAW_ERROR_MS ||
        (gimbal_yaw.gyro_mode && !Gyro_Ready(now))) ++gimbal_yaw.feedback_warnings;
}

/** @brief 转入零输出确认，记录退出原因和测试位移。 */
static void stop(gimbal_yaw_reason_t reason, uint32_t now)
{
    if (gimbal_yaw.state == GIMBAL_STOPPING) { return; }
    gyro_control.turning = 0U;
    gimbal_yaw.angle_integral_sum = 0.0f;
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
    if (gimbal_yaw.gyro_mode && gyro_control.valid) { gimbal_yaw.speed_estimate_dps = gyro_control.speed_dps; return; }
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

/** @brief 检查主动许可与请求；非法参数丢弃，遥控未恢复时保持等待。 */
static void begin(uint32_t request, uint32_t now, int interlock)
{
    gimbal_yaw.start_block_reason = gimbal_yaw.interlock_block_reason;
    gimbal_yaw.wait_remote_ms = 0;
    gimbal_yaw_reason_t reason = GIMBAL_REASON_NONE;
    float offset = gimbal_yaw_offset_deg;
    int32_t direction = gimbal_yaw_output_direction;
    if (gimbal_yaw_enable != 1U) { reason = GIMBAL_REASON_DISABLED; }
    else if (request != 5U && !interlock && !RC_Sensor_Online(now) &&
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
    else if (motor_fault()) { reason = GIMBAL_REASON_FEEDBACK; }
    else { reason = request_reason(request, offset, direction); }
    if (reason != GIMBAL_REASON_NONE) {
        if (reason == GIMBAL_REASON_REQUEST || reason == GIMBAL_REASON_DIRECTION) ++gimbal_yaw.invalid_requests;
        gyro_control.turning = 0U;
        gimbal_yaw.angle_integral_sum = 0.0f;
        gimbal_yaw.reason = reason;
        return;
    }
    gimbal_yaw.gyro_mode = request == 5U && Car_IsGyroMode(car.car_mode);
    gimbal_yaw.imu_generation = gyro_control.generation;
    gimbal_yaw.car_session = car.sessions;
    gimbal_yaw.angle_integral_sum = 0.0f;
    last_keys = rc_sensor.info->key_v; wheel_up_seen = 0U; gyro_control.turning = 0U;
    gimbal_yaw.state = GIMBAL_ZEROING;
    gimbal_yaw.reason = GIMBAL_REASON_NONE;
    gimbal_yaw.request = request;
    gimbal_yaw.pulse_raw = request >= 3U ? 0 : (int16_t)gimbal_yaw_pulse_raw;
    gimbal_yaw.pulse_ms = request >= 3U ? 0 : gimbal_yaw_pulse_ms;
    gimbal_yaw.duration_ms = request == 5U ? 0U : request == 4U ? gimbal_yaw_speed_ms :
        (request == 3U ? GIMBAL_YAW_SESSION_MS : gimbal_yaw.pulse_ms);
    gimbal_yaw.active_ms = gimbal_yaw.exit_block_reason = 0;
    ++gimbal_yaw.sessions;
    ++gimbal_yaw.restart_attempts;
    overspeed_active = 0U; gimbal_yaw.overspeed_ms = 0U;
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
    if (request == 5U) {
        remote_source = car.car_ctrl;
        if (gimbal_yaw.gyro_mode) {
            gimbal_yaw.angle_control = 1U; gimbal_yaw.angle_kp = GYRO_YAW_ANGLE_KP;
            gimbal_yaw.speed_kp = GYRO_YAW_SPEED_KP; gimbal_yaw.speed_ki = GYRO_YAW_SPEED_KI;
        }
    }
    gimbal_yaw.angle_effective_kp = request == 3U || (request == 5U && gimbal_yaw.angle_control) ?
        gimbal_yaw.angle_kp : 0.0f;
    phase_ms = now;
    tx_started = zero_sent = 0;
}

/** @brief 按整车许可自动启动机械或世界角会话，不限制拨杆档位。 */
static void remote_start(uint32_t now, int interlock)
{
    if (gimbal_yaw.reason == GIMBAL_REASON_SPEED &&
        (!yaw_probe.feedback.state_frames || now - yaw_probe.feedback.last_state_ms > GIMBAL_YAW_FEEDBACK_MS ||
         abs(yaw_probe.feedback.status.speed_dps) > gimbal_yaw.overspeed_limit_dps ||
         (gimbal_yaw.gyro_mode && gyro_control.valid &&
          (!Gyro_Ready(now) || fabsf(gyro_control.speed_dps) > gimbal_yaw.overspeed_limit_dps)))) return;
    if ((mec_output_enable != 1U || !RC_Sensor_Online(now)) && yaw_probe.angle_valid && !yaw_probe.continuous_valid &&
        abs(yaw_probe.feedback.status.current_raw) <= GIMBAL_YAW_ZERO_CURRENT &&
        abs(yaw_probe.feedback.status.speed_dps) <= GIMBAL_YAW_ZERO_SPEED) yaw_probe_angle_reset = 1U;
    if (gimbal_yaw_enable == 1U && RC_Sensor_Online(now) && interlock &&
        (car.car_mode == mec_car || Car_IsGyroMode(car.car_mode))) begin(5U, now, interlock);
}

/** @brief 将右杆或鼠标X映射到角度目标变化速率/直接速度，正为逆时针。 */
static float remote_input_rate(void)
{
    const rc_sensor_info_t *rc = rc_sensor.info;
    if (gimbal_yaw.gyro_mode) {
        return remote_source == KEY_CTRL ? GYRO_MOUSE_YAW_GAIN * rc->mouse_x :
            (abs(rc->ch0) > RC_DEADBAND ? GYRO_RC_YAW_GAIN * rc->ch0 : 0.0f);
    }
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

/** @brief 机械遥控小误差用低Kp，中间连续插值，大误差用会话锁存Kp。 */
static float mechanical_angle_kp(float error)
{
    if (!GIMBAL_YAW_CONTROL_ANGLE_SOFT_ENABLE) return gimbal_yaw.angle_kp;
    float magnitude = fabsf(error);
    float near_kp = fminf(GIMBAL_YAW_CONTROL_ANGLE_KP_NEAR, gimbal_yaw.angle_kp);
    if (magnitude <= GIMBAL_YAW_CONTROL_ANGLE_NEAR_DEG) return near_kp;
    if (magnitude >= GIMBAL_YAW_CONTROL_ANGLE_FAR_DEG) return gimbal_yaw.angle_kp;
    float blend = (magnitude - GIMBAL_YAW_CONTROL_ANGLE_NEAR_DEG) /
        (GIMBAL_YAW_CONTROL_ANGLE_FAR_DEG - GIMBAL_YAW_CONTROL_ANGLE_NEAR_DEG);
    return near_kp + (gimbal_yaw.angle_kp - near_kp) * blend;
}

/** @brief 两模式共用R按下沿或双上拨杆负端回中；等待期间消费重复动作。 */
static int turn_requested(const rc_sensor_info_t *rc)
{
    int key_edge = (rc->key_v & GYRO_TURN_KEY_MASK) && !(last_keys & GYRO_TURN_KEY_MASK);
    int wheel_edge = 0;
    last_keys = rc->key_v;
    if (remote_source != RC_CTRL || rc->s1.value != RC_SW_UP || rc->s2.value != RC_SW_UP ||
        gyro_control.turning) {
        wheel_up_seen = 0U;
    } else if (rc->thumbwheel <= GYRO_TURN_WHEEL_TRIGGER) {
        wheel_up_seen = 1U;
    } else if (wheel_up_seen && abs(rc->thumbwheel) <= GYRO_TURN_WHEEL_CENTER) {
        wheel_up_seen = 0U;
        wheel_edge = 1;
    }
    return GYRO_TURN_ENABLE && !gyro_control.turning && (remote_source == KEY_CTRL ? key_edge : wheel_edge);
}

/** @brief 两模式共用换头到位及超时记录，超时不清除角度目标。 */
static void turn_update(uint32_t now)
{
    if (gyro_control.turning && fabsf(gimbal_yaw.error_deg) <= GYRO_TURN_SETTLE_DEG) {
        gyro_control.turning = 0U; ++gimbal_yaw.turn_completed;
    } else if (gyro_control.turning && now - turn_ms >= GYRO_TURN_MS) {
        gyro_control.turning = 0U; ++gimbal_yaw.turn_timeouts;
    }
}

/** @brief 累计目标并生成速度；换头期间保持连续角方向，机械遥控按误差调节Kp。 */
static void remote_control(uint32_t now, uint32_t elapsed)
{
    float rate = remote_input_rate();
    if (turn_requested(rc_sensor.info)) {
        gimbal_yaw.target_deg += GYRO_TURN_ANGLE_DEG;
        gyro_control.turning = 1U; turn_ms = now; ++gimbal_yaw.turn_count;
    }
    if (gimbal_yaw.gyro_mode) {
        if (!gyro_control.turning || remote_source == KEY_CTRL) gimbal_yaw.target_deg += rate * elapsed * 0.001f;
        gimbal_yaw.error_deg = gyro_control.turning ? gimbal_yaw.target_deg - gimbal_yaw.actual_deg :
            remainderf(remainderf(gimbal_yaw.target_deg, 360.0f) - remainderf(gimbal_yaw.actual_deg, 360.0f), 360.0f);
        gimbal_yaw.target_deg = gimbal_yaw.actual_deg + gimbal_yaw.error_deg;
        gimbal_yaw.goal_deg = gimbal_yaw.target_deg;
        turn_update(now);
        gimbal_yaw.angle_integral_sum = clamp(gimbal_yaw.angle_integral_sum + gimbal_yaw.error_deg,
            -GYRO_YAW_ANGLE_SUM_MAX, GYRO_YAW_ANGLE_SUM_MAX);
        gimbal_yaw.angle_effective_kp = gimbal_yaw.angle_kp;
        gimbal_yaw.speed_target_dps = clamp(gimbal_yaw.angle_effective_kp * gimbal_yaw.error_deg +
            GYRO_YAW_ANGLE_KI * gimbal_yaw.angle_integral_sum, -GYRO_YAW_MAX_SPEED_DPS, GYRO_YAW_MAX_SPEED_DPS);
        gimbal_yaw.speed_setpoint_dps = gimbal_yaw.speed_target_dps;
        speed_control(elapsed);
        return;
    }
    if (gimbal_yaw.angle_control || gyro_control.turning) {
        if (!gyro_control.turning && remote_source == KEY_CTRL && (rc_sensor.info->key_v & RC_KEY_CTRL)) {
            gimbal_yaw.target_deg = gimbal_yaw.actual_deg;
            gimbal_yaw.speed_integral_raw = 0;
        } else if (!gyro_control.turning || remote_source == KEY_CTRL) {
            gimbal_yaw.target_deg += rate * (elapsed * 0.001f);
        }
        gimbal_yaw.goal_deg = gimbal_yaw.target_deg;
        gimbal_yaw.error_deg = gimbal_yaw.target_deg - gimbal_yaw.actual_deg;
        turn_update(now);
        gimbal_yaw.angle_effective_kp = mechanical_angle_kp(gimbal_yaw.error_deg);
        gimbal_yaw.speed_target_dps = clamp(gimbal_yaw.angle_effective_kp * gimbal_yaw.error_deg,
            -GIMBAL_YAW_CONTROL_MAX_SPEED_DPS, GIMBAL_YAW_CONTROL_MAX_SPEED_DPS);
    } else { gimbal_yaw.angle_effective_kp = 0.0f; gimbal_yaw.speed_target_dps = rate; }
    gimbal_yaw.speed_setpoint_dps = gimbal_yaw.speed_target_dps;
    speed_control(elapsed);
}

/** @brief 遥控恢复后重试请求，等待超时只记录；主动撤销或标定变化仍取消。 */
static void wait_remote(uint32_t now, int interlock)
{
    gimbal_yaw.wait_remote_ms = now - wait_start_ms;
    gimbal_yaw_reason_t reason = GIMBAL_REASON_NONE;
    if (gimbal_yaw_enable != 1U) { reason = GIMBAL_REASON_DISABLED; }
    else if (zero_encoder != yaw_probe_zero_encoder || angle_direction != yaw_probe_direction ||
             (waiting_request >= 3U && waiting_direction != gimbal_yaw_output_direction) ||
             (waiting_request == 3U && waiting_offset != gimbal_yaw_offset_deg) ||
             (waiting_request == 4U && (waiting_speed != gimbal_yaw_speed_dps ||
                                       waiting_kp != gimbal_yaw_speed_kp || waiting_ki != gimbal_yaw_speed_ki ||
                                       waiting_speed_ms != gimbal_yaw_speed_ms)) ||
             (waiting_request <= 2U && (waiting_pulse_raw != gimbal_yaw_pulse_raw ||
                                        waiting_pulse_ms != gimbal_yaw_pulse_ms))) { reason = GIMBAL_REASON_CALIBRATION; }
    else if (gimbal_yaw.wait_remote_ms >= GIMBAL_YAW_REMOTE_WAIT_MS) { ++gimbal_yaw.feedback_warnings; }
    if (reason != GIMBAL_REASON_NONE) {
        gimbal_yaw.state = GIMBAL_FAULT;
        gyro_control.turning = 0U;
        gimbal_yaw.angle_integral_sum = 0.0f;
        gimbal_yaw.reason = reason;
        waiting_request = 0;
        return;
    }
    if (!interlock) { remote_stable = 0; return; }
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
    if (!elapsed || elapsed > CHASSIS_MAX_PERIOD_MS) {
        ++gimbal_yaw.period_errors;
        elapsed = CHASSIS_CONTROL_MS;
    }
    record_feedback(now);
    int gyro_feedback = Gimbal_Yaw_OwnsBus() ? gimbal_yaw.gyro_mode : Car_IsGyroMode(car.car_mode);
    gimbal_yaw.actual_deg = gyro_feedback && gyro_control.valid ? gyro_control.continuous_deg : yaw_probe.continuous_deg;
    if (gimbal_yaw.gyro_mode && gyro_control.generation != gimbal_yaw.imu_generation) {
        gimbal_yaw.imu_generation = gyro_control.generation;
        gimbal_yaw.target_deg = gimbal_yaw.goal_deg = gimbal_yaw.actual_deg;
        gimbal_yaw.angle_integral_sum = gimbal_yaw.speed_integral_raw = 0.0f;
        gyro_control.turning = 0U;
        ++gimbal_yaw.reference_rebases;
    }
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
        int invalid_calibration = yaw_probe_zero_encoder >= YAW_PROBE_ENCODER_COUNTS ||
            (yaw_probe_direction != 1 && yaw_probe_direction != -1) ||
            (gimbal_yaw.request >= 3U && gimbal_yaw_output_direction != 1 && gimbal_yaw_output_direction != -1) ||
            (gimbal_yaw.request == 5U && !gimbal_yaw.gyro_mode && gimbal_yaw_angle_enable > 1U);
        if (invalid_calibration) ++gimbal_yaw.invalid_requests;
        if (gimbal_yaw_enable != 1U || (gimbal_yaw.request == 5U && gimbal_yaw_remote_enable != 1U)) {
            stop(GIMBAL_REASON_DISABLED, now);
        }
        else if (!interlock) { stop(GIMBAL_REASON_INTERLOCK, now); }
        else if (gimbal_yaw.request == 5U && (gimbal_yaw.gyro_mode ? !Car_IsGyroMode(car.car_mode) : car.car_mode != mec_car)) { stop(GIMBAL_REASON_CALIBRATION, now); }
        else if (motor_fault()) { stop(GIMBAL_REASON_FEEDBACK, now); }
        else if (!invalid_calibration && (zero_encoder != yaw_probe_zero_encoder || angle_direction != yaw_probe_direction ||
                 (gimbal_yaw.request >= 3U && output_direction != gimbal_yaw_output_direction) ||
                 (gimbal_yaw.request == 5U && !gimbal_yaw.gyro_mode && gimbal_yaw.angle_control != gimbal_yaw_angle_enable))) {
            stop(GIMBAL_REASON_CALIBRATION, now);
        }
    }
    if (gimbal_yaw.state == GIMBAL_STOPPING) {
        gimbal_yaw.current_raw = 0;
        gimbal_yaw.last_delta_deg = gimbal_yaw.actual_deg - gimbal_yaw.start_deg;
        gimbal_yaw.state = gimbal_yaw.reason == GIMBAL_REASON_COMPLETE ||
            gimbal_yaw.reason == GIMBAL_REASON_DISABLED ? GIMBAL_DONE : GIMBAL_FAULT;
        return;
    }
    if (gimbal_yaw.state == GIMBAL_ZEROING) {
        if (gimbal_yaw.zero_confirmed) {
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
        } else if ((uint32_t)(now - phase_ms) >= GIMBAL_YAW_ZEROING_MS) {
            ++gimbal_yaw.zero_timeouts;
            phase_ms = now;
        }
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
        speed_guard = fmaxf(speed_guard, (gimbal_yaw.gyro_mode ? GYRO_YAW_MAX_SPEED_DPS : GIMBAL_YAW_CONTROL_MAX_SPEED_DPS) + GIMBAL_YAW_TEST_SPEED_MARGIN_DPS);
    }
    gimbal_yaw.overspeed_limit_dps = speed_guard;
    int speed_fresh = gimbal_yaw.gyro_mode && gyro_control.valid ? Gyro_Ready(now) && gimbal_yaw.imu_age_ms <= GIMBAL_YAW_FEEDBACK_MS :
        yaw_probe.feedback.state_frames && gimbal_yaw.feedback_age_ms <= GIMBAL_YAW_FEEDBACK_MS;
    int overspeed = (speed_fresh && fabsf(gimbal_yaw.speed_estimate_dps) > speed_guard) ||
        (yaw_probe.feedback.state_frames && gimbal_yaw.feedback_age_ms <= GIMBAL_YAW_FEEDBACK_MS &&
         abs(yaw_probe.feedback.status.speed_dps) > speed_guard);
    if (overspeed) {
        ++gimbal_yaw.overspeed_samples;
        if (now - overspeed_check_ms > GIMBAL_YAW_FEEDBACK_MS) overspeed_active = 0U;
        if (!overspeed_active) { overspeed_active = 1U; overspeed_started_ms = now; }
        gimbal_yaw.overspeed_ms = now - overspeed_started_ms;
        if (gimbal_yaw.overspeed_ms >= GIMBAL_YAW_OVERSPEED_MS) {
            ++gimbal_yaw.overspeed_trips;
            stop(GIMBAL_REASON_SPEED, now); return;
        }
    } else { overspeed_active = 0U; gimbal_yaw.overspeed_ms = 0U; }
    overspeed_check_ms = now;
    gimbal_yaw.active_ms = now - phase_ms;
    if (gimbal_yaw.duration_ms && gimbal_yaw.active_ms >= gimbal_yaw.duration_ms) {
        stop(GIMBAL_REASON_COMPLETE, now); return;
    }
    if (gimbal_yaw.state == GIMBAL_PULSE) {
        gimbal_yaw.current_raw = gimbal_yaw.request == 1U ? gimbal_yaw.pulse_raw : -gimbal_yaw.pulse_raw;
    } else if (gimbal_yaw.state == GIMBAL_REMOTE) { remote_control(now, elapsed); }
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
    yaw_scope_angle_kp = 0.0f;
    yaw_scope_state = GIMBAL_IDLE;
    overspeed_started_ms = overspeed_check_ms = overspeed_active = 0U;
    phase_ms = previous_ms = last_tx_ms = sample_frames = sample_ms = 0;
    zero_encoder = 0;
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
    remote_source = RC_CTRL; turn_ms = last_keys = wheel_up_seen = 0U;
}

/** @brief 清除已停机的Yaw会话，保留参数与发送统计，重新建立机械角参考。 */
static void standby_reset(uint32_t now)
{
    gimbal_yaw = (gimbal_yaw_t){.tx_queued = gimbal_yaw.tx_queued, .tx_confirmed = gimbal_yaw.tx_confirmed,
        .tx_errors = gimbal_yaw.tx_errors, .nonzero_confirmed = gimbal_yaw.nonzero_confirmed,
        .feedback_warnings = gimbal_yaw.feedback_warnings, .period_errors = gimbal_yaw.period_errors,
        .invalid_requests = gimbal_yaw.invalid_requests, .zero_timeouts = gimbal_yaw.zero_timeouts,
        .restart_attempts = gimbal_yaw.restart_attempts, .reference_rebases = gimbal_yaw.reference_rebases,
        .overspeed_samples = gimbal_yaw.overspeed_samples, .overspeed_trips = gimbal_yaw.overspeed_trips};
    gimbal_yaw_request = 0U;
    gimbal_yaw_offset_deg = gimbal_yaw_speed_dps = 0.0f;
    phase_ms = previous_ms = now;
    last_tx_ms = sample_frames = sample_ms = zero_encoder = 0U;
    angle_direction = output_direction = waiting_direction = 0;
    sample_deg = waiting_offset = waiting_speed = waiting_kp = waiting_ki = 0.0f;
    tx_started = zero_sent = remote_stable = drive_started = wheel_up_seen = 0U;
    waiting_request = wait_start_ms = ready_start_ms = drive_reply_baseline = waiting_pulse_ms = 0U;
    waiting_pulse_raw = 0;
    settle_streak_ms = waiting_speed_ms = turn_ms = last_keys = 0U;
    remote_source = RC_CTRL;
    yaw_probe_angle_reset = 1U;
    gyro_control.fault = gyro_control.turning = 0U;
    yaw_scope_target_dps = yaw_scope_current_raw = yaw_scope_integral_raw = 0.0f;
    yaw_scope_angle_target_deg = yaw_scope_angle_error_deg = 0.0f;
    yaw_scope_angle_kp = 0.0f;
    yaw_scope_state = GIMBAL_IDLE;
}

/** @brief 遥控离线立即清除软件会话，零电流由驱动持续发送，不等待ACK。 */
void Gimbal_Yaw_RequestStandby(uint32_t now)
{
    standby_reset(now);
}

/** @brief 报告软件已回待命，不证明电机收到或执行零电流。 */
int Gimbal_Yaw_StandbyReady(void)
{
    return !gimbal_yaw.standby_pending && gimbal_yaw.state == GIMBAL_IDLE;
}

/** @brief 更新控制并发布角度、速度和电流标量镜像，便于J-Scope采集。 */
void Gimbal_Yaw_Update(uint32_t now, int interlock)
{
    update_control(now, interlock);
    if (gimbal_yaw.standby_pending && !Gimbal_Yaw_OwnsBus()) standby_reset(now);
    yaw_scope_target_dps = gimbal_yaw.state == GIMBAL_HOLD || gimbal_yaw.state == GIMBAL_SPEED ||
        gimbal_yaw.state == GIMBAL_REMOTE ?
        gimbal_yaw.speed_target_dps : 0;
    yaw_scope_speed_dps = gimbal_yaw.speed_estimate_dps;
    yaw_scope_current_raw = gimbal_yaw.current_raw;
    yaw_scope_integral_raw = gimbal_yaw.speed_integral_raw;
    yaw_scope_angle_target_deg = gimbal_yaw.target_deg;
    yaw_scope_angle_actual_deg = gimbal_yaw.actual_deg;
    yaw_scope_angle_error_deg = gimbal_yaw.state == GIMBAL_REMOTE && gimbal_yaw.gyro_mode ? gimbal_yaw.error_deg :
        gimbal_yaw.gyro_mode ? remainderf(gimbal_yaw.target_deg - gimbal_yaw.actual_deg, 360.0f) :
        gimbal_yaw.target_deg - gimbal_yaw.actual_deg;
    yaw_scope_angle_kp = gimbal_yaw.angle_effective_kp;
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
    if (tx_started && (uint32_t)(now - last_tx_ms) < GIMBAL_YAW_TX_MS) { return 0; }
    if (gimbal_yaw.tx_queued % GIMBAL_YAW_STATUS_DIVIDER == GIMBAL_YAW_STATUS_DIVIDER - 1U) {
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

/** @brief 发送异常只记录，不改变会话，下一调度周期继续尝试。 */
void Gimbal_Yaw_TxError(void)
{
    ++gimbal_yaw.tx_errors;

}
