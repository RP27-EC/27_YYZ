/**
 * @file    gimbal_pitch.c
 * @brief   国赛IMU速度内环、周期积分、重力补偿及现有遥控启动握手。
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_pitch.h"
#include <math.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static uint32_t update_ms, phase_ms, phase_frames; /**< 调度和阶段起点，毫秒及状态帧数。 */
static uint32_t query_ms, drive_ms; /**< 已入队帧的节拍时间，毫秒。 */
static uint8_t updated, queried, driven, off_seen, source_key; /**< 周期初始化及拨杆启动记忆。 */
static uint8_t zero_ack, enable_ack, disable_acks, query_index; /**< 发送确认和回读轮询位置。 */
static uint8_t sequence_seen, last_sequence; /**< 只接受新序号的板间租约。 */
static uint32_t encoder_ms; /**< 上一个角差分参考帧时刻，毫秒。 */
static float encoder_deg; /**< 上一个角差分参考机械角，度。 */
static uint8_t encoder_seen; /**< 是否已有角差分参考帧。 */

/* Exported variables --------------------------------------------------------*/
gimbal_pitch_t gimbal_pitch; /**< 上板反馈与控制状态。 */
volatile uint32_t gimbal_pitch_enable = PITCH_BOOT_ENABLE; /**< 用户输出许可。 */
volatile uint32_t gimbal_pitch_speed_guard_enable = PITCH_SPEED_GUARD_ENABLE; /**< 仅控制选定反馈速度阈值退出，默认按本轮试验配置。 */
volatile uint32_t gimbal_pitch_loop_mode = PITCH_LOOP_MODE_DEFAULT; /**< 用户调试模式请求，停止后切换并重新拨杆。 */
volatile float pitch_scope_target_deg, pitch_scope_actual_deg, pitch_scope_error_deg; /**< 角度镜像，度。 */
volatile float pitch_scope_target_dps, pitch_scope_speed_dps, pitch_scope_torque; /**< 速度和力矩镜像。 */
volatile float pitch_scope_goal_deg, pitch_scope_encoder_dps; /**< 最终角目标与角差分速度镜像。 */
volatile float pitch_scope_speed_error_dps; /**< 速度环误差镜像，度/秒。 */
volatile float pitch_scope_angle_integral_dps; /**< 角度积分输出镜像，度/秒。 */
volatile float pitch_scope_gravity_torque, pitch_scope_speed_loop_torque; /**< 原生方向补偿和速度PI贡献镜像，MIT标度。 */
volatile float pitch_scope_motor_dps, pitch_scope_imu_deg; /**< 电机速度对照和国赛IMU角镜像。 */
volatile uint32_t pitch_scope_imu_ready; /**< 新鲜且校准通过的IMU许可镜像。 */

/* Private functions ---------------------------------------------------------*/
/** @brief 对有限控制量做对称限幅。 */
static float clip(float value, float limit)
{
    return fmaxf(-limit, fminf(limit, value));
}

/** @brief 只有完整、有限、适用的MIT回读标度才可输出。 */
static int profile_valid(void)
{
    return gimbal_pitch.parameter_mask == 15U && gimbal_pitch.control_mode == 1U &&
        isfinite(gimbal_pitch.pmax) && gimbal_pitch.pmax >= 1.2f && gimbal_pitch.pmax <= 100.0f &&
        isfinite(gimbal_pitch.vmax) && gimbal_pitch.vmax > 0.0f && gimbal_pitch.vmax <= 1000.0f &&
        isfinite(gimbal_pitch.tmax) && gimbal_pitch.tmax >= PITCH_TORQUE_LIMIT && gimbal_pitch.tmax <= 1000.0f;
}

/** @brief 用实际回读标度解码电机反馈，机械正方向沿用国赛。 */
static void decode_feedback(void)
{
    const uint8_t *d = gimbal_pitch.raw;
    float position = ((uint32_t)d[1] << 8) | d[2];
    float velocity = ((uint32_t)d[3] << 4) | (d[4] >> 4);
    float torque = ((uint32_t)(d[4] & 15U) << 8) | d[5];
    float relative;
    gimbal_pitch.motor_rad = (position / 65535.0f * 2.0f - 1.0f) * gimbal_pitch.pmax;
    relative = PITCH_FEEDBACK_DIRECTION * (gimbal_pitch.motor_rad - PITCH_ZERO_RAD);
    relative = remainderf(relative, 6.283185307f);
    gimbal_pitch.actual_deg = relative * PITCH_RAD_TO_DEG;
    gimbal_pitch.motor_speed_dps = (velocity / 4095.0f * 2.0f - 1.0f) * gimbal_pitch.vmax *
        PITCH_FEEDBACK_DIRECTION * PITCH_RAD_TO_DEG;
    if (!PITCH_USE_IMU) gimbal_pitch.speed_dps = gimbal_pitch.motor_speed_dps;
    gimbal_pitch.feedback_torque = (torque / 4095.0f * 2.0f - 1.0f) * gimbal_pitch.tmax;
}

/** @brief 由不同时间的状态帧估算平均角速度，不替代电机速度回报或保护。 */
static void encoder_speed_update(uint32_t now)
{
    uint32_t elapsed = now - encoder_ms;
    gimbal_pitch.encoder_speed_valid = encoder_seen && elapsed > 0U && elapsed <= PITCH_FEEDBACK_TIMEOUT_MS;
    if (gimbal_pitch.encoder_speed_valid)
        gimbal_pitch.encoder_speed_dps = remainderf(gimbal_pitch.actual_deg - encoder_deg, 360.0f) * 1000.0f / (float)elapsed;
    if (!encoder_seen || elapsed > 0U) {
        encoder_deg = gimbal_pitch.actual_deg;
        encoder_ms = now;
        encoder_seen = 1U;
    }
}

/** @brief 待机与握手阶段同步实际角作为参考，不施加保持力矩。 */
static void capture_target(void)
{
    gimbal_pitch.target_deg = gimbal_pitch.actual_deg;
    gimbal_pitch.goal_deg = gimbal_pitch.actual_deg;
    gimbal_pitch.error_deg = 0.0f;
    gimbal_pitch.speed_target_dps = 0.0f;
    gimbal_pitch.speed_error_dps = 0.0f;
    gimbal_pitch.angle_integral_dps = 0.0f;
    gimbal_pitch.angle_integral_sum = 0.0f;
    gimbal_pitch.flat_return = 0U;
}

/** @brief 使用国赛IMU俯仰角，返回分段补偿的原生MIT力矩。 */
static float gravity_compensation(float pitch_deg)
{
    float angle = pitch_deg + PITCH_GRAVITY_ANGLE_OFFSET_DEG;
    float torque = 0.0f;
    if (!PITCH_GRAVITY_ENABLE) return 0.0f;
    if (angle <= -4.0f) torque = PITCH_GRAVITY_LOW_TORQUE;
    else if (angle < 10.0f) torque = PITCH_GRAVITY_LOW_TORQUE * (10.0f - angle) / 14.0f;
    else if (angle > 30.0f) torque = PITCH_GRAVITY_HIGH_TORQUE * fminf((angle - 30.0f) / 17.0f, 1.0f);
    return PITCH_GRAVITY_SCALE * torque;
}

/** @brief 国赛角度积分：每周期累加误差、积分分离、累计限幅，不乘dt。 */
static void angle_integral_update(void)
{
    float error = gimbal_pitch.error_deg;
    if (PITCH_ANGLE_INTEGRAL_SEP_DEG > 0.0f && fabsf(error) > PITCH_ANGLE_INTEGRAL_SEP_DEG)
        gimbal_pitch.angle_integral_sum = 0.0f;
    else gimbal_pitch.angle_integral_sum = clip(gimbal_pitch.angle_integral_sum + error, PITCH_ANGLE_INTEGRAL_MAX);
    gimbal_pitch.angle_integral_dps = PITCH_ANGLE_KI * gimbal_pitch.angle_integral_sum;
}

/** @brief 更新角度模式的回平射或手动参考，并计算速度内环目标。 */
static void angle_reference_update(uint8_t flags, float dt, float low, float high)
{
    if (flags & BOARD_PITCH_CTRL) { capture_target(); gimbal_pitch.integral = 0.0f; gimbal_pitch.speed_integral_sum = 0.0f; }
    else if (gimbal_pitch.command.rate_dps != 0.0f) {
        gimbal_pitch.flat_return = 0U;
        gimbal_pitch.target_deg += gimbal_pitch.command.rate_dps * dt;
    } else if (gimbal_pitch.flat_return) {
        gimbal_pitch.target_deg += clip(gimbal_pitch.goal_deg - gimbal_pitch.target_deg, PITCH_FLAT_RATE_DPS * dt);
        if (fabsf(gimbal_pitch.goal_deg - gimbal_pitch.target_deg) < 0.0001f) {
            gimbal_pitch.target_deg = gimbal_pitch.goal_deg;
            gimbal_pitch.flat_return = 0U;
        }
    }
    gimbal_pitch.target_deg = fmaxf(low, fminf(high, gimbal_pitch.target_deg));
    if (!gimbal_pitch.flat_return) gimbal_pitch.goal_deg = gimbal_pitch.target_deg;
    gimbal_pitch.error_deg = gimbal_pitch.target_deg - gimbal_pitch.actual_deg;
    angle_integral_update();
    gimbal_pitch.speed_target_dps = clip(PITCH_ANGLE_KP * gimbal_pitch.error_deg + gimbal_pitch.angle_integral_dps, PITCH_MAX_SPEED_DPS);
}

/** @brief 速度模式直接使用右杆/鼠标输入，绕过角度外环并取消回平射。 */
static void speed_reference_update(uint8_t flags, float low, float high)
{
    capture_target();
    gimbal_pitch.speed_target_dps = (flags & BOARD_PITCH_CTRL) ? 0.0f : clip(gimbal_pitch.command.rate_dps, PITCH_MAX_SPEED_DPS);
    if (flags & BOARD_PITCH_CTRL) { gimbal_pitch.integral = 0.0f; gimbal_pitch.speed_integral_sum = 0.0f; }
    if ((gimbal_pitch.actual_deg >= high && gimbal_pitch.speed_target_dps > 0.0f) ||
        (gimbal_pitch.actual_deg <= low && gimbal_pitch.speed_target_dps < 0.0f)) gimbal_pitch.speed_target_dps = 0.0f;
}

/** @brief 撤销输出并进入重复失能与新反馈确认阶段。 */
static void stop(pitch_reason_t reason, uint32_t now)
{
    gimbal_pitch.exit_sample = (pitch_exit_sample_t){
        .state = gimbal_pitch.state, .loop_mode = gimbal_pitch.loop_mode, .reason = reason, .ms = now, .feedback_ms = gimbal_pitch.last_ms,
        .target_deg = gimbal_pitch.target_deg, .actual_deg = gimbal_pitch.actual_deg,
        .speed_target_dps = gimbal_pitch.speed_target_dps, .speed_dps = gimbal_pitch.speed_dps,
        .speed_error_dps = gimbal_pitch.speed_error_dps,
        .encoder_speed_dps = gimbal_pitch.encoder_speed_dps, .torque = gimbal_pitch.torque,
        .angle_integral_dps = gimbal_pitch.angle_integral_dps,
        .gravity_torque = gimbal_pitch.gravity_torque, .speed_loop_torque = gimbal_pitch.speed_loop_torque,
        .input_rate_dps = gimbal_pitch.command.rate_dps, .encoder_speed_valid = gimbal_pitch.encoder_speed_valid
    };
    memcpy(gimbal_pitch.exit_sample.raw, gimbal_pitch.raw, 8U);
    gimbal_pitch.reason = reason;
    gimbal_pitch.exit_blocks = gimbal_pitch.blocks;
    gimbal_pitch.state = PITCH_STOPPING;
    gimbal_pitch.torque = 0.0f;
    gimbal_pitch.integral = 0.0f;
    gimbal_pitch.angle_integral_dps = 0.0f;
    gimbal_pitch.angle_integral_sum = gimbal_pitch.speed_integral_sum = 0.0f;
    gimbal_pitch.gravity_torque = gimbal_pitch.speed_loop_torque = 0.0f;
    gimbal_pitch.speed_target_dps = 0.0f;
    gimbal_pitch.speed_error_dps = 0.0f;
    gimbal_pitch.flat_return = 0U;
    off_seen = 0U;
    phase_ms = now;
    phase_frames = gimbal_pitch.frames;
    disable_acks = 0U;
    driven = 0U;
}

/** @brief 发布无需展开结构体的采样变量。 */
static void scope_update(void)
{
    pitch_scope_target_deg = gimbal_pitch.target_deg;
    pitch_scope_actual_deg = gimbal_pitch.actual_deg;
    pitch_scope_error_deg = gimbal_pitch.error_deg;
    pitch_scope_target_dps = gimbal_pitch.speed_target_dps;
    pitch_scope_speed_dps = gimbal_pitch.speed_dps;
    pitch_scope_torque = gimbal_pitch.torque;
    pitch_scope_goal_deg = gimbal_pitch.goal_deg;
    pitch_scope_encoder_dps = gimbal_pitch.encoder_speed_dps;
    pitch_scope_speed_error_dps = gimbal_pitch.speed_error_dps;
    pitch_scope_angle_integral_dps = gimbal_pitch.angle_integral_dps;
    pitch_scope_gravity_torque = gimbal_pitch.gravity_torque;
    pitch_scope_speed_loop_torque = gimbal_pitch.speed_loop_torque;
    pitch_scope_motor_dps = gimbal_pitch.motor_speed_dps;
    pitch_scope_imu_deg = gimbal_pitch.imu_pitch_deg;
    pitch_scope_imu_ready = !PITCH_USE_IMU || (gimbal_pitch.imu_ready && update_ms - gimbal_pitch.imu_last_ms <= PITCH_IMU_TIMEOUT_MS);
}

/** @brief 将有符号MIT物理标度量编码为无符号域。 */
static uint32_t encode(float value, float extent, uint32_t maximum)
{
    return (uint32_t)lroundf((clip(value, extent) / extent + 1.0f) * 0.5f * (float)maximum);
}

/** @brief 生成仅含力矩的MIT帧，位置/速度比例与微分增益均为零。 */
static void torque_frame(float torque, uint8_t d[8])
{
    uint32_t p = encode(0.0f, gimbal_pitch.pmax, 65535U);
    uint32_t v = encode(0.0f, gimbal_pitch.vmax, 4095U);
    uint32_t t = encode(torque, gimbal_pitch.tmax, 4095U);
    d[0] = (uint8_t)(p >> 8); d[1] = (uint8_t)p;
    d[2] = (uint8_t)(v >> 4); d[3] = (uint8_t)(v << 4);
    d[4] = 0U; d[5] = 0U; d[6] = (uint8_t)(t >> 8); d[7] = (uint8_t)t;
}

/* Exported functions --------------------------------------------------------*/
/** @brief 初始化只读探测，不接受开机时已有的中档作为启动。 */
void Gimbal_Pitch_Init(void)
{
    memset(&gimbal_pitch, 0, sizeof(gimbal_pitch));
    gimbal_pitch.state = PITCH_PROBE;
    gimbal_pitch_enable = PITCH_BOOT_ENABLE;
    gimbal_pitch_speed_guard_enable = PITCH_SPEED_GUARD_ENABLE;
    gimbal_pitch_loop_mode = PITCH_LOOP_MODE_DEFAULT;
    gimbal_pitch.loop_mode = (pitch_loop_mode_t)PITCH_LOOP_MODE_DEFAULT;
    updated = queried = driven = off_seen = source_key = 0U;
    zero_ack = enable_ack = disable_acks = query_index = sequence_seen = last_sequence = 0U;
    update_ms = phase_ms = phase_frames = query_ms = drive_ms = 0U;
    encoder_ms = 0U; encoder_deg = 0.0f; encoder_seen = 0U;
    scope_update();
}

/** @brief IMU校准与数据有效时更新内环反馈，无效时禁止启动并由控制调度退出。 */
void Gimbal_Pitch_ImuUpdate(float pitch_deg, float speed_dps, uint32_t now, int ready)
{
    gimbal_pitch.imu_ready = ready && isfinite(pitch_deg) && isfinite(speed_dps);
    if (!gimbal_pitch.imu_ready) return;
    gimbal_pitch.imu_pitch_deg = PITCH_IMU_DIRECTION * pitch_deg;
    gimbal_pitch.imu_speed_dps = PITCH_IMU_DIRECTION * speed_dps;
    gimbal_pitch.imu_last_ms = now;
    ++gimbal_pitch.imu_updates;
    if (PITCH_USE_IMU) gimbal_pitch.speed_dps = gimbal_pitch.imu_speed_dps;
}

/** @brief 接收参数、状态与带CRC命令；参数回读不充当电机状态心跳。 */
int Gimbal_Pitch_Receive(uint32_t id, const uint8_t *d, unsigned length, uint32_t now)
{
    board_pitch_command_t command;
    uint32_t bits, mask = 0U;
    float value;
    if (d == NULL || length != 8U) return 0;
    if (id == BOARD_PITCH_COMMAND_ID) {
        if (!Board_Pitch_DecodeCommand(d, length, &command)) return 0;
        if (sequence_seen && command.sequence == last_sequence) return 0;
        sequence_seen = 1U;
        last_sequence = command.sequence;
        gimbal_pitch.command = command;
        if (command.flags & BOARD_PITCH_ACTIVE) {
            ++gimbal_pitch.active_command_frames;
            gimbal_pitch.last_active_command = command;
            gimbal_pitch.peak_input_rate_dps = fmaxf(gimbal_pitch.peak_input_rate_dps, fabsf(command.rate_dps));
            if (!off_seen && (gimbal_pitch.state == PITCH_PROBE || gimbal_pitch.state == PITCH_WAIT ||
                gimbal_pitch.state == PITCH_FAULT)) ++gimbal_pitch.unarmed_active_frames;
        }
        gimbal_pitch.board_last_ms = now;
        ++gimbal_pitch.board_frames;
        return 1;
    }
    if (id != PITCH_FEEDBACK_ID) return 0;
    if (d[0] == PITCH_MOTOR_ID && d[1] == 0U && d[2] == 0x33U) {
        bits = (uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
        memcpy(&value, &bits, sizeof(value));
        if (d[3] == 10U) { gimbal_pitch.control_mode = bits; mask = 8U; }
        else if (d[3] == 21U) { gimbal_pitch.pmax = value; mask = 1U; }
        else if (d[3] == 22U) { gimbal_pitch.vmax = value; mask = 2U; }
        else if (d[3] == 23U) { gimbal_pitch.tmax = value; mask = 4U; }
        else return 0;
        gimbal_pitch.parameter_mask |= mask;
        ++gimbal_pitch.parameter_frames;
        if (profile_valid() && gimbal_pitch.frames != 0U) decode_feedback();
        return 1;
    }
    if ((d[0] & 15U) != PITCH_MOTOR_ID || ((d[0] >> 4) > 1U && (d[0] >> 4) < 8U)) return 0;
    memcpy(gimbal_pitch.raw, d, 8U);
    gimbal_pitch.motor_state = d[0] >> 4;
    gimbal_pitch.mos_temperature = d[6];
    gimbal_pitch.rotor_temperature = d[7];
    gimbal_pitch.last_ms = now;
    ++gimbal_pitch.frames;
    if (profile_valid()) {
        decode_feedback();
        encoder_speed_update(now);
    }
    return 1;
}

/** @brief 根据锁存模式更新速度单环或角度串级，模式改变需要停止后重新启动。 */
void Gimbal_Pitch_Update(uint32_t now, int can_ready)
{
    uint32_t elapsed = updated ? now - update_ms : PITCH_COMMAND_MS;
    uint32_t requested_mode = gimbal_pitch_loop_mode;
    uint8_t flags = gimbal_pitch.command.flags;
    float low = PITCH_MIN_RAD * PITCH_RAD_TO_DEG, high = PITCH_MAX_RAD * PITCH_RAD_TO_DEG;
    float error, output, dt = (float)elapsed * 0.001f;
    updated = 1U; update_ms = now;
    gimbal_pitch.online = profile_valid() && gimbal_pitch.frames && now - gimbal_pitch.last_ms <= PITCH_FEEDBACK_TIMEOUT_MS;
    gimbal_pitch.board_online = gimbal_pitch.board_frames && now - gimbal_pitch.board_last_ms <= PITCH_BOARD_TIMEOUT_MS;
    if (!gimbal_pitch.board_online) off_seen = 0U;
    gimbal_pitch.blocks = 0U;
    if (!profile_valid()) gimbal_pitch.blocks |= PITCH_BLOCK_PARAMS;
    if (!gimbal_pitch.online) gimbal_pitch.blocks |= PITCH_BLOCK_FEEDBACK;
    if (!gimbal_pitch.board_online || !(flags & BOARD_PITCH_RC_ONLINE)) gimbal_pitch.blocks |= PITCH_BLOCK_BOARD;
    if (!gimbal_pitch_enable) gimbal_pitch.blocks |= PITCH_BLOCK_ENABLE;
    if (!(flags & BOARD_PITCH_ACTIVE)) gimbal_pitch.blocks |= PITCH_BLOCK_SWITCH;
    if (gimbal_pitch.online && (gimbal_pitch.actual_deg < low - PITCH_LIMIT_MARGIN_DEG ||
        gimbal_pitch.actual_deg > high + PITCH_LIMIT_MARGIN_DEG)) gimbal_pitch.blocks |= PITCH_BLOCK_LIMIT;
    if (!can_ready) gimbal_pitch.blocks |= PITCH_BLOCK_CAN;
    if (PITCH_USE_IMU && (!gimbal_pitch.imu_ready || now - gimbal_pitch.imu_last_ms > PITCH_IMU_TIMEOUT_MS))
        gimbal_pitch.blocks |= PITCH_BLOCK_IMU;
    if (gimbal_pitch.state == PITCH_STOPPING) {
        gimbal_pitch.stop_unconfirmed = now - phase_ms >= PITCH_STOP_WARN_MS;
        if (disable_acks >= 2U && gimbal_pitch.online && gimbal_pitch.motor_state == 0U && gimbal_pitch.frames > phase_frames) {
            gimbal_pitch.state = PITCH_FAULT;
            gimbal_pitch.stop_unconfirmed = 0U;
        }
    } else if (gimbal_pitch.state == PITCH_ZEROING || gimbal_pitch.state == PITCH_ENABLING || gimbal_pitch.state == PITCH_ACTIVE) {
        if (gimbal_pitch.blocks) stop(gimbal_pitch.blocks == PITCH_BLOCK_IMU ? PITCH_REASON_IMU : PITCH_REASON_INTERLOCK, now);
        else if (requested_mode > PITCH_LOOP_ANGLE || requested_mode != (uint32_t)gimbal_pitch.loop_mode) stop(PITCH_REASON_MODE, now);
        else if (gimbal_pitch.motor_state >= 8U || (gimbal_pitch.state == PITCH_ACTIVE && gimbal_pitch.motor_state != 1U)) stop(PITCH_REASON_MOTOR, now);
        else if (elapsed > PITCH_MAX_PERIOD_MS) stop(PITCH_REASON_PERIOD, now);
        else if (gimbal_pitch_speed_guard_enable && fabsf(gimbal_pitch.speed_dps) > PITCH_SPEED_GUARD_DPS) stop(PITCH_REASON_SPEED, now);
        else if ((flags & BOARD_PITCH_KEY) != source_key) stop(PITCH_REASON_INTERLOCK, now);
        else if (gimbal_pitch.state != PITCH_ACTIVE && now - phase_ms > PITCH_ENABLE_TIMEOUT_MS) stop(PITCH_REASON_TIMEOUT, now);
        else if (gimbal_pitch.state == PITCH_ZEROING && zero_ack) {
            gimbal_pitch.state = PITCH_ENABLING; phase_ms = now; phase_frames = gimbal_pitch.frames; driven = 0U;
        } else if (gimbal_pitch.state == PITCH_ENABLING && enable_ack && gimbal_pitch.frames > phase_frames && gimbal_pitch.motor_state == 1U) {
            gimbal_pitch.state = PITCH_ACTIVE;
            ++gimbal_pitch.active_entries;
            gimbal_pitch.target_deg = fmaxf(low, fminf(high, gimbal_pitch.actual_deg));
            if (gimbal_pitch.loop_mode == PITCH_LOOP_ANGLE) {
                gimbal_pitch.goal_deg = fmaxf(low, fminf(high, PITCH_FLAT_ANGLE_DEG));
                gimbal_pitch.flat_return = 1U;
            } else capture_target();
        } else if (gimbal_pitch.state == PITCH_ACTIVE) {
            if (gimbal_pitch.loop_mode == PITCH_LOOP_SPEED) speed_reference_update(flags, low, high);
            else angle_reference_update(flags, dt, low, high);
            error = gimbal_pitch.speed_target_dps - gimbal_pitch.speed_dps;
            gimbal_pitch.speed_error_dps = error;
            gimbal_pitch.speed_integral_sum = clip(gimbal_pitch.speed_integral_sum + error, PITCH_SPEED_INTEGRAL_MAX);
            gimbal_pitch.integral = PITCH_SPEED_KI * gimbal_pitch.speed_integral_sum;
            gimbal_pitch.gravity_torque = gravity_compensation(PITCH_USE_IMU ? gimbal_pitch.imu_pitch_deg : gimbal_pitch.actual_deg - PITCH_FLAT_ANGLE_DEG);
            gimbal_pitch.speed_loop_torque = PITCH_OUTPUT_DIRECTION * clip(PITCH_SPEED_KP * error + gimbal_pitch.integral, PITCH_SPEED_OUTPUT_LIMIT);
            output = clip(gimbal_pitch.speed_loop_torque + gimbal_pitch.gravity_torque, PITCH_TORQUE_LIMIT);
            if ((gimbal_pitch.actual_deg >= high && output * PITCH_OUTPUT_DIRECTION > 0.0f) ||
                (gimbal_pitch.actual_deg <= low && output * PITCH_OUTPUT_DIRECTION < 0.0f)) output = 0.0f;
            gimbal_pitch.torque = output;
            gimbal_pitch.peak_torque = fmaxf(gimbal_pitch.peak_torque, fabsf(gimbal_pitch.torque));
        }
    } else {
        gimbal_pitch.torque = 0.0f;
        if (gimbal_pitch.frames && gimbal_pitch.motor_state != 0U) stop(PITCH_REASON_MOTOR, now);
        else if (!profile_valid()) { gimbal_pitch.state = PITCH_PROBE; off_seen = 0U; }
        else if (gimbal_pitch.board_online && (flags & BOARD_PITCH_RC_ONLINE) && !(flags & BOARD_PITCH_ACTIVE)) {
            off_seen = 1U; gimbal_pitch.state = PITCH_WAIT;
        } else if (gimbal_pitch.board_online && (flags & BOARD_PITCH_ACTIVE) && off_seen) {
            off_seen = 0U;
            if (!(flags & BOARD_PITCH_NEUTRAL)) gimbal_pitch.blocks |= PITCH_BLOCK_NEUTRAL;
            ++gimbal_pitch.start_attempts;
            gimbal_pitch.start_flags = flags;
            gimbal_pitch.start_blocks = gimbal_pitch.blocks;
            if (gimbal_pitch.blocks || !gimbal_pitch.online || gimbal_pitch.motor_state != 0U) {
                gimbal_pitch.state = PITCH_FAULT; gimbal_pitch.reason = PITCH_REASON_INTERLOCK; gimbal_pitch.exit_blocks = gimbal_pitch.blocks;
            } else if (requested_mode > PITCH_LOOP_ANGLE) {
                gimbal_pitch.state = PITCH_FAULT; gimbal_pitch.reason = PITCH_REASON_MODE; gimbal_pitch.exit_blocks = 0U;
            } else {
                ++gimbal_pitch.sessions;
                gimbal_pitch.state = PITCH_ZEROING; gimbal_pitch.reason = PITCH_REASON_NONE;
                phase_ms = now; zero_ack = enable_ack = 0U; driven = 0U;
                source_key = flags & BOARD_PITCH_KEY;
                gimbal_pitch.loop_mode = (pitch_loop_mode_t)requested_mode;
                gimbal_pitch.integral = 0.0f;
                gimbal_pitch.angle_integral_sum = gimbal_pitch.speed_integral_sum = 0.0f;
            }
        }
    }
    if (gimbal_pitch.state != PITCH_ACTIVE && gimbal_pitch.online) capture_target();
    scope_update();
}

/** @brief 调度MIT零输出/使能/力矩/失能及只读0x7FF参数和状态请求。 */
int Gimbal_Pitch_MakeFrame(uint32_t now, uint32_t *id, uint8_t d[8], uint8_t *kind)
{
    static const uint8_t rid[4] = {21U, 22U, 23U, 10U};
    uint32_t interval = gimbal_pitch.state == PITCH_ACTIVE ? PITCH_COMMAND_MS : PITCH_SPECIAL_MS;
    if (!id || !d || !kind) return 0;
    if ((!driven || now - drive_ms >= interval) &&
        (gimbal_pitch.state == PITCH_STOPPING || (profile_valid() &&
         (gimbal_pitch.state == PITCH_ZEROING || gimbal_pitch.state == PITCH_ENABLING || gimbal_pitch.state == PITCH_ACTIVE)))) {
        *id = PITCH_MOTOR_ID;
        if (gimbal_pitch.state == PITCH_STOPPING || gimbal_pitch.state == PITCH_ENABLING) {
            memset(d, 0xFF, 8U);
            d[7] = gimbal_pitch.state == PITCH_STOPPING ? 0xFDU : 0xFCU;
            *kind = gimbal_pitch.state == PITCH_STOPPING ? PITCH_TX_DISABLE : PITCH_TX_ENABLE;
        } else {
            torque_frame(gimbal_pitch.state == PITCH_ACTIVE ? gimbal_pitch.torque : 0.0f, d);
            *kind = gimbal_pitch.state == PITCH_ACTIVE ? PITCH_TX_TORQUE : PITCH_TX_ZERO;
        }
        return 1;
    }
    if (queried && now - query_ms < PITCH_QUERY_MS) return 0;
    memset(d, 0, 8U); *id = 0x7FFU; *kind = PITCH_TX_QUERY; d[0] = PITCH_MOTOR_ID;
    if (!profile_valid()) { d[2] = 0x33U; d[3] = rid[query_index]; }
    else d[2] = 0xCCU;
    return 1;
}

/** @brief 入队仅决定发送节拍，不改变使能握手结果。 */
void Gimbal_Pitch_Queued(uint32_t now, uint8_t kind)
{
    if (kind == PITCH_TX_QUERY) { queried = 1U; query_ms = now; query_index = (query_index + 1U) & 3U; }
    else { driven = 1U; drive_ms = now; }
}

/** @brief 根据控制器TXOK推进零输出/使能/失能确认。 */
void Gimbal_Pitch_TxComplete(uint32_t now, uint8_t kind, int success)
{
    if (!success) {
        ++gimbal_pitch.tx_errors;
        if (gimbal_pitch.state == PITCH_ACTIVE || gimbal_pitch.state == PITCH_ZEROING || gimbal_pitch.state == PITCH_ENABLING) stop(PITCH_REASON_CAN, now);
        return;
    }
    ++gimbal_pitch.tx_confirmed;
    if (kind == PITCH_TX_ZERO && gimbal_pitch.state == PITCH_ZEROING) zero_ack = 1U;
    if (kind == PITCH_TX_ENABLE && gimbal_pitch.state == PITCH_ENABLING) enable_ack = 1U;
    if (kind == PITCH_TX_DISABLE && gimbal_pitch.state == PITCH_STOPPING && disable_acks < 2U) ++disable_acks;
}

/** @brief 退出阶段必须取消待发送的使能和力矩。 */
int Gimbal_Pitch_NeedsStop(void)
{
    return gimbal_pitch.state == PITCH_STOPPING || gimbal_pitch.state == PITCH_FAULT || gimbal_pitch.state == PITCH_PROBE;
}
