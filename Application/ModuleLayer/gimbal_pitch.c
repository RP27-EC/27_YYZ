/**
 * @file    gimbal_pitch.c
 * @brief   Pitch串级、异常数据保留诊断、持续回读及危险条件下的退出。
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
static uint8_t sequence_seen, last_sequence; /**< 记录重复序号，合法重复帧仍接受。 */
static uint8_t standby_held; /**< 本次离线请求保持期间只复位一次，在线后重新等待启动。 */
static uint8_t restart_in_progress; /**< 自动重新握手正在进行。 */
static uint8_t handshake_warned; /**< 当前握手阶段已记录超时。 */
static uint32_t encoder_ms; /**< 上一个角差分参考帧时刻，毫秒。 */
static float encoder_deg; /**< 上一个角差分参考机械角，度。 */
static uint8_t encoder_seen; /**< 是否已有角差分参考帧。 */
static uint32_t enable_retry_ms; /**< ACTIVE追加使能帧最近入队时刻，毫秒。 */
static uint8_t enable_retry_seen; /**< 是否已经入队追加使能帧。 */

/* Exported variables --------------------------------------------------------*/
gimbal_pitch_t gimbal_pitch; /**< 上板反馈与控制状态。 */
pitch_board_rx_diag_t pitch_board_rx_diag; /**< CAN2命令接收诊断。 */
pitch_diagnostics_t pitch_diag; /**< 非停机异常、丢弃样本及持续时间。 */
volatile uint32_t gimbal_pitch_enable = PITCH_BOOT_ENABLE; /**< 用户输出许可。 */
volatile uint32_t gimbal_pitch_speed_guard_enable = PITCH_SPEED_GUARD_ENABLE; /**< 仅控制选定反馈速度阈值退出，默认按本轮试验配置。 */
volatile uint32_t gimbal_pitch_loop_mode = PITCH_LOOP_MODE_DEFAULT; /**< 用户调试模式请求，停止后切换并重新拨杆。 */
volatile float pitch_scope_target_deg, pitch_scope_actual_deg, pitch_scope_error_deg; /**< 角度镜像，度。 */
volatile float pitch_scope_target_dps, pitch_scope_speed_dps, pitch_scope_torque; /**< 速度和力矩镜像。 */
volatile float pitch_scope_goal_deg, pitch_scope_encoder_dps; /**< 最终角目标与角差分速度镜像。 */
volatile float pitch_scope_speed_error_dps; /**< 速度环误差镜像，度/秒。 */
volatile float pitch_scope_angle_integral_dps; /**< 角度积分输出镜像，度/秒。 */
volatile float pitch_scope_gravity_torque, pitch_scope_speed_loop_torque; /**< 原生方向补偿和速度PI贡献镜像，MIT标度。 */
volatile float pitch_scope_motor_dps, pitch_scope_imu_deg; /**< 电机速度对照和IMU角镜像。 */
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

/** @brief 用实际回读标度解码电机反馈，机械正方向沿用。 */
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
    gimbal_pitch.target_deg = gimbal_pitch.control_actual_deg;
    gimbal_pitch.goal_deg = gimbal_pitch.control_actual_deg;
    gimbal_pitch.error_deg = 0.0f;
    gimbal_pitch.speed_target_dps = 0.0f;
    gimbal_pitch.speed_error_dps = 0.0f;
    gimbal_pitch.angle_integral_dps = 0.0f;
    gimbal_pitch.angle_integral_sum = 0.0f;
    gimbal_pitch.flat_return = 0U;
}

/** @brief 使用IMU俯仰角，返回分段补偿的原生MIT力矩。 */
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

/** @brief 角度积分：每周期累加误差、积分分离、累计限幅，不乘dt。 */
static void angle_integral_update(void)
{
    float error = gimbal_pitch.error_deg;
    float sep = gimbal_pitch.gyro_mode ? PITCH_GYRO_ANGLE_SEP_DEG : PITCH_ANGLE_INTEGRAL_SEP_DEG;
    float limit = gimbal_pitch.gyro_mode ? PITCH_GYRO_ANGLE_SUM_MAX : PITCH_ANGLE_INTEGRAL_MAX;
    float ki = gimbal_pitch.gyro_mode ? PITCH_GYRO_ANGLE_KI : PITCH_ANGLE_KI;
    if (sep > 0.0f && fabsf(error) > sep)
        gimbal_pitch.angle_integral_sum = 0.0f;
    else gimbal_pitch.angle_integral_sum = clip(gimbal_pitch.angle_integral_sum + error, limit);
    gimbal_pitch.angle_integral_dps = ki * gimbal_pitch.angle_integral_sum;
}

/** @brief 更新角度模式的回平射或手动参考，并计算速度内环目标。 */
static void angle_reference_update(uint8_t flags, float dt, float low, float high)
{
    if ((flags & BOARD_PITCH_CTRL) && !gimbal_pitch.gyro_mode) { capture_target(); gimbal_pitch.integral = 0.0f; gimbal_pitch.speed_integral_sum = 0.0f; }
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
    gimbal_pitch.error_deg = gimbal_pitch.target_deg - gimbal_pitch.control_actual_deg;
    angle_integral_update();
    gimbal_pitch.speed_target_dps = clip((gimbal_pitch.gyro_mode ? PITCH_GYRO_ANGLE_KP : PITCH_ANGLE_KP) * gimbal_pitch.error_deg +
        gimbal_pitch.angle_integral_dps, gimbal_pitch.gyro_mode ? PITCH_GYRO_MAX_SPEED_DPS : PITCH_MAX_SPEED_DPS);
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

/** @brief 撤销自动重启许可，保留累计次数与最近原因供Watch观察。 */
static void cancel_recovery(void)
{
    gimbal_pitch.recovery_pending = gimbal_pitch.recovery_waiting = 0U;
    restart_in_progress = 0U;
}

/** @brief 明确撤销许可或危险事件清零输出并保留自动重启请求。 */
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
    gimbal_pitch.stop_unconfirmed = 1U;
    driven = 0U;
    cancel_recovery();
    gimbal_pitch.recovery_pending = 1U;
    gimbal_pitch.recovery_blocks = gimbal_pitch.exit_blocks;
}

/** @brief 锁存离线软件复位事件，不等待电机反馈完成复位。 */
static void request_standby(void)
{
    cancel_recovery();
    if (!standby_held) {
        standby_held = 1U;
        gimbal_pitch.standby_pending = 1U;
        gimbal_pitch.standby_ready = 0U;
    }
}

/** @brief 清除控制会话并进入软件待命，实际停止发送结果独立记录。 */
static void standby_reset(uint32_t now)
{
    gimbal_pitch.state = PITCH_STANDBY;
    gimbal_pitch.reason = PITCH_REASON_NONE;
    gimbal_pitch.sessions = 0U;
    gimbal_pitch.start_blocks = gimbal_pitch.start_flags = gimbal_pitch.exit_blocks = 0U;
    gimbal_pitch.last_active_command = (board_pitch_command_t){0};
    gimbal_pitch.command.rate_dps = 0.0f;
    gimbal_pitch.torque = gimbal_pitch.integral = gimbal_pitch.speed_integral_sum = 0.0f;
    gimbal_pitch.gravity_torque = gimbal_pitch.speed_loop_torque = 0.0f;
    gimbal_pitch.stop_unconfirmed = 1U;
    gimbal_pitch.loop_mode = (pitch_loop_mode_t)gimbal_pitch_loop_mode;
    gimbal_pitch.gyro_mode = (gimbal_pitch.command.flags & BOARD_PITCH_GYRO) != 0U;
    gimbal_pitch.control_actual_deg = gimbal_pitch.gyro_mode ? gimbal_pitch.imu_pitch_deg : gimbal_pitch.actual_deg;
    capture_target();
    off_seen = source_key = zero_ack = enable_ack = disable_acks = driven = queried = query_index = 0U;
    encoder_seen = gimbal_pitch.encoder_speed_valid = 0U;
    gimbal_pitch.encoder_speed_dps = 0.0f;
    phase_ms = query_ms = drive_ms = now;
    phase_frames = gimbal_pitch.frames;
    gimbal_pitch.standby_pending = 0U;
    gimbal_pitch.standby_ready = 1U;
}

/** @brief 发布无需展开结构体的采样变量。 */
static void scope_update(void)
{
    pitch_scope_target_deg = gimbal_pitch.target_deg;
    pitch_scope_actual_deg = gimbal_pitch.control_actual_deg;
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
/** @brief 初始化参数探测；明确在线ACTIVE许可允许自动启动。 */
void Gimbal_Pitch_Init(void)
{
    pitch_board_rx_diag = (pitch_board_rx_diag_t){0};
    pitch_diag = (pitch_diagnostics_t){0};
    memset(&gimbal_pitch, 0, sizeof(gimbal_pitch));
    gimbal_pitch.state = PITCH_PROBE;
    gimbal_pitch_enable = PITCH_BOOT_ENABLE;
    gimbal_pitch_speed_guard_enable = PITCH_SPEED_GUARD_ENABLE;
    gimbal_pitch_loop_mode = PITCH_LOOP_MODE_DEFAULT;
    gimbal_pitch.loop_mode = (pitch_loop_mode_t)PITCH_LOOP_MODE_DEFAULT;
    updated = queried = driven = off_seen = source_key = 0U;
    zero_ack = enable_ack = disable_acks = query_index = sequence_seen = last_sequence = 0U;
    standby_held = restart_in_progress = handshake_warned = 0U;
    update_ms = phase_ms = phase_frames = query_ms = drive_ms = 0U;
    encoder_ms = 0U; encoder_deg = 0.0f; encoder_seen = 0U;
    enable_retry_ms = 0U; enable_retry_seen = 0U;
    scope_update();
}

/** @brief 仅有效IMU采样更新数据及时间，异常样本不覆盖最近有效参考。 */
void Gimbal_Pitch_ImuUpdate(float pitch_deg, float speed_dps, uint32_t now, int ready)
{
    if (!ready || !isfinite(pitch_deg) || !isfinite(speed_dps)) {
        ++pitch_diag.imu_errors; pitch_diag.last_reject_ms = now; return;
    }
    gimbal_pitch.imu_ready = 1U;
    gimbal_pitch.imu_pitch_deg = PITCH_IMU_DIRECTION * pitch_deg;
    gimbal_pitch.imu_speed_dps = PITCH_IMU_DIRECTION * speed_dps;
    gimbal_pitch.imu_last_ms = now;
    ++gimbal_pitch.imu_updates;
    if (PITCH_USE_IMU) gimbal_pitch.speed_dps = gimbal_pitch.imu_speed_dps;
}

/** @brief 同步完整上板姿态，Pitch和Yaw分别保持安装坐标，不重复取反Yaw。 */
void Gimbal_Pitch_ImuAttitudeUpdate(float pitch_deg, float pitch_dps, float yaw_deg, float yaw_dps, uint32_t now, int ready)
{
    if (!ready || !isfinite(pitch_deg) || !isfinite(pitch_dps) || !isfinite(yaw_deg) || !isfinite(yaw_dps)) {
        ++pitch_diag.imu_errors; pitch_diag.last_reject_ms = now; return;
    }
    Gimbal_Pitch_ImuUpdate(pitch_deg, pitch_dps, now, ready);
    gimbal_pitch.imu_yaw_ready = 1U;
    gimbal_pitch.imu_yaw_deg = remainderf(yaw_deg, 360.0f);
    gimbal_pitch.imu_yaw_dps = yaw_dps;
}

/** @brief 接收参数、状态与带CRC命令；参数回读不充当电机状态心跳。 */
int Gimbal_Pitch_Receive(uint32_t id, const uint8_t *d, unsigned length, uint32_t now)
{
    board_pitch_command_t command;
    uint32_t bits, mask = 0U;
    float value;
    if (d == NULL || length != 8U) {
        if (id == PITCH_FEEDBACK_ID) ++pitch_diag.feedback_errors;
        if (id == BOARD_PITCH_COMMAND_ID) ++pitch_diag.command_errors;
        pitch_diag.last_reject_ms = now; return 0;
    }
    ++pitch_board_rx_diag.raw_frames;
    pitch_board_rx_diag.last_ms = now; pitch_board_rx_diag.last_id = id;
    if (id == BOARD_PITCH_COMMAND_ID) {
        ++pitch_board_rx_diag.command_frames;
        memcpy(pitch_board_rx_diag.command_raw, d, 8U);
        if (!Board_Pitch_DecodeCommand(d, length, &command)) {
            ++pitch_board_rx_diag.decode_errors; ++pitch_diag.command_errors;
            pitch_diag.last_reject_ms = now; return 0;
        }
        if (sequence_seen && command.sequence == last_sequence) ++pitch_board_rx_diag.repeated;
        sequence_seen = 1U;
        last_sequence = command.sequence;
        gimbal_pitch.command = command;
        if (!(command.flags & BOARD_PITCH_ACTIVE)) cancel_recovery();
        if ((command.flags & BOARD_PITCH_RESET) || !(command.flags & BOARD_PITCH_RC_ONLINE)) request_standby();
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
        int valid = 0;
        if (d[3] == 10U) { mask = 8U; valid = bits == 1U; }
        else if (d[3] == 21U) { mask = 1U; valid = isfinite(value) && value >= 1.2f && value <= 100.0f; }
        else if (d[3] == 22U) { mask = 2U; valid = isfinite(value) && value > 0.0f && value <= 1000.0f; }
        else if (d[3] == 23U) { mask = 4U; valid = isfinite(value) && value >= PITCH_TORQUE_LIMIT && value <= 1000.0f; }
        if (!valid) {
            ++pitch_diag.parameter_errors;
            pitch_diag.parameter_last_ms = pitch_diag.last_reject_ms = now;
            pitch_diag.parameter_rid = d[3]; pitch_diag.parameter_raw = bits;
            pitch_diag.parameter_reject_mask |= mask;
            return 0;
        }
        pitch_diag.parameter_reject_mask &= ~mask;
        if (mask == 8U) gimbal_pitch.control_mode = bits;
        if (mask == 1U) gimbal_pitch.pmax = value;
        if (mask == 2U) gimbal_pitch.vmax = value;
        if (mask == 4U) gimbal_pitch.tmax = value;
        gimbal_pitch.parameter_mask |= mask;
        ++gimbal_pitch.parameter_frames;
        if (profile_valid() && gimbal_pitch.frames != 0U) decode_feedback();
        return 1;
    }
    if ((d[0] & 15U) != PITCH_MOTOR_ID || ((d[0] >> 4) > 1U && (d[0] >> 4) < 8U)) {
        ++pitch_diag.feedback_errors; pitch_diag.last_reject_ms = now; return 0;
    }
    memcpy(gimbal_pitch.raw, d, 8U);
    if (gimbal_pitch.frames && now - gimbal_pitch.last_ms > PITCH_FEEDBACK_TIMEOUT_MS) {
        pitch_diag.limit_active = 0U; pitch_diag.limit_duration_ms = 0U;
    }
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

/** @brief 刷新诊断与持续越界计时，旧样本不计为持续越界。 */
static void diagnose(uint32_t now, uint32_t elapsed, int can_ready, float low, float high)
{
    uint32_t flags = 0U, previous = pitch_diag.flags;
    pitch_diag.parameter_missing = gimbal_pitch.parameter_mask != 15U;
    pitch_diag.control_mode_invalid = gimbal_pitch.control_mode != 1U || (pitch_diag.parameter_reject_mask & 8U) != 0U;
    pitch_diag.scale_invalid = (pitch_diag.parameter_reject_mask & 7U) != 0U || !isfinite(gimbal_pitch.pmax) || gimbal_pitch.pmax < 1.2f || gimbal_pitch.pmax > 100.0f ||
        !isfinite(gimbal_pitch.vmax) || gimbal_pitch.vmax <= 0.0f || gimbal_pitch.vmax > 1000.0f ||
        !isfinite(gimbal_pitch.tmax) || gimbal_pitch.tmax < PITCH_TORQUE_LIMIT || gimbal_pitch.tmax > 1000.0f;
    pitch_diag.feedback_age_ms = gimbal_pitch.frames ? now - gimbal_pitch.last_ms : UINT32_MAX;
    pitch_diag.command_age_ms = gimbal_pitch.board_frames ? now - gimbal_pitch.board_last_ms : UINT32_MAX;
    pitch_diag.imu_age_ms = gimbal_pitch.imu_updates ? now - gimbal_pitch.imu_last_ms : UINT32_MAX;
    pitch_diag.can_duration_ms = gimbal_pitch.tx_failure_active ? now - gimbal_pitch.tx_failure_since_ms : 0U;
    if (!profile_valid() || pitch_diag.parameter_reject_mask) flags |= PITCH_BLOCK_PARAMS;
    if (!gimbal_pitch.online) flags |= PITCH_BLOCK_FEEDBACK;
    if (!gimbal_pitch.board_online) flags |= PITCH_BLOCK_BOARD;
    if (!can_ready || gimbal_pitch.tx_failure_active) flags |= PITCH_BLOCK_CAN;
    if (PITCH_USE_IMU && (!gimbal_pitch.imu_ready || pitch_diag.imu_age_ms > PITCH_IMU_TIMEOUT_MS)) flags |= PITCH_BLOCK_IMU;
    if (elapsed > PITCH_MAX_PERIOD_MS) { flags |= PITCH_DIAG_PERIOD; ++pitch_diag.period_errors; }
    if ((gimbal_pitch.command.flags & BOARD_PITCH_KEY) != source_key) {
        flags |= PITCH_DIAG_SOURCE; ++pitch_diag.source_changes;
        source_key = gimbal_pitch.command.flags & BOARD_PITCH_KEY;
    }
    pitch_diag.handshake_duration_ms = (gimbal_pitch.state == PITCH_ZEROING || gimbal_pitch.state == PITCH_ENABLING) ? now - phase_ms : 0U;
    if (pitch_diag.handshake_duration_ms > PITCH_ENABLE_TIMEOUT_MS) {
        flags |= PITCH_DIAG_HANDSHAKE;
        if (!handshake_warned) { ++pitch_diag.handshake_timeouts; handshake_warned = 1U; }
    }
    if (gimbal_pitch.online && isfinite(gimbal_pitch.actual_deg) &&
        (gimbal_pitch.actual_deg < low - PITCH_LIMIT_MARGIN_DEG || gimbal_pitch.actual_deg > high + PITCH_LIMIT_MARGIN_DEG)) {
        flags |= PITCH_BLOCK_LIMIT;
        if (!pitch_diag.limit_active) {
            pitch_diag.limit_active = 1U; pitch_diag.limit_since_ms = now; ++pitch_diag.limit_events;
        }
        pitch_diag.limit_duration_ms = now - pitch_diag.limit_since_ms;
        if (pitch_diag.limit_duration_ms >= PITCH_LIMIT_HOLD_MS) pitch_diag.limit_tripped = 1U;
    } else {
        pitch_diag.limit_active = 0U; pitch_diag.limit_duration_ms = 0U;
        if (gimbal_pitch.online) pitch_diag.limit_tripped = 0U;
    }
    pitch_diag.flags = flags;
    if (flags & ~previous) ++pitch_diag.events;
}

/** @brief 更新控制；通信及采样异常只诊断，保留明确撤销许可和危险退出。 */
void Gimbal_Pitch_Update(uint32_t now, int can_ready)
{
    uint32_t elapsed = updated ? now - update_ms : PITCH_COMMAND_MS;
    uint32_t requested_mode = gimbal_pitch_loop_mode;
    uint8_t flags = gimbal_pitch.command.flags;
    float low = PITCH_MIN_RAD * PITCH_RAD_TO_DEG, high = PITCH_MAX_RAD * PITCH_RAD_TO_DEG;
    float error, output, dt = (float)(elapsed <= PITCH_MAX_PERIOD_MS ? elapsed : PITCH_COMMAND_MS) * 0.001f;
    updated = 1U; update_ms = now;
    if (gimbal_pitch.state == PITCH_PROBE || gimbal_pitch.state == PITCH_WAIT || gimbal_pitch.state == PITCH_FAULT || gimbal_pitch.state == PITCH_STANDBY)
        gimbal_pitch.gyro_mode = (flags & BOARD_PITCH_GYRO) != 0U;
    gimbal_pitch.control_actual_deg = gimbal_pitch.gyro_mode ? gimbal_pitch.imu_pitch_deg : gimbal_pitch.actual_deg;
    if (gimbal_pitch.gyro_mode) {
        low = fmaxf(low, PITCH_GYRO_MIN_MECH_DEG); high = fminf(high, PITCH_GYRO_MAX_MECH_DEG);
    }
    gimbal_pitch.online = profile_valid() && gimbal_pitch.frames && now - gimbal_pitch.last_ms <= PITCH_FEEDBACK_TIMEOUT_MS;
    gimbal_pitch.board_online = gimbal_pitch.board_frames && now - gimbal_pitch.board_last_ms <= PITCH_BOARD_TIMEOUT_MS;
    diagnose(now, elapsed, can_ready, low, high);
    gimbal_pitch.blocks = 0U;
    if (!(flags & BOARD_PITCH_RC_ONLINE)) gimbal_pitch.blocks |= PITCH_BLOCK_BOARD;
    if (!gimbal_pitch_enable) gimbal_pitch.blocks |= PITCH_BLOCK_ENABLE;
    if (!(flags & BOARD_PITCH_ACTIVE)) gimbal_pitch.blocks |= PITCH_BLOCK_SWITCH;
    if (pitch_diag.limit_tripped) gimbal_pitch.blocks |= PITCH_BLOCK_LIMIT;
    if (gimbal_pitch.standby_pending) {
        cancel_recovery(); standby_reset(now);
    }
    if (standby_held) {
        if (!(flags & BOARD_PITCH_RC_ONLINE) || (flags & BOARD_PITCH_RESET)) {
            scope_update(); return;
        }
        standby_held = 0U; gimbal_pitch.standby_ready = 0U; gimbal_pitch.state = PITCH_WAIT;
    }
    if (gimbal_pitch.state == PITCH_STOPPING) {
        gimbal_pitch.state = PITCH_FAULT;
        scope_update(); return;
    }
    int running = gimbal_pitch.state == PITCH_ZEROING || gimbal_pitch.state == PITCH_ENABLING || gimbal_pitch.state == PITCH_ACTIVE;
    if (!running && gimbal_pitch.online) capture_target();
    int bad_mode = requested_mode > PITCH_LOOP_ANGLE || ((flags & BOARD_PITCH_GYRO) && requested_mode != PITCH_LOOP_ANGLE);
    if (gimbal_pitch.blocks || (flags & BOARD_PITCH_RESET) || gimbal_pitch.motor_state >= 8U || bad_mode ||
        (gimbal_pitch_speed_guard_enable && fabsf(gimbal_pitch.speed_dps) > PITCH_SPEED_GUARD_DPS)) {
        pitch_reason_t reason = gimbal_pitch.motor_state >= 8U ? PITCH_REASON_MOTOR :
            bad_mode ? PITCH_REASON_MODE : gimbal_pitch.blocks ? PITCH_REASON_INTERLOCK :
            (flags & BOARD_PITCH_RESET) ? PITCH_REASON_INTERLOCK : PITCH_REASON_SPEED;
        if (running) stop(reason, now);
        else { gimbal_pitch.torque = 0.0f; gimbal_pitch.reason = reason; gimbal_pitch.state = PITCH_WAIT; }
        scope_update(); return;
    }
    if (running && (requested_mode != (uint32_t)gimbal_pitch.loop_mode ||
        ((flags & BOARD_PITCH_GYRO) != 0U) != gimbal_pitch.gyro_mode)) {
        stop(PITCH_REASON_MODE, now); scope_update(); return;
    }
    if (!running) {
        ++gimbal_pitch.start_attempts;
        gimbal_pitch.start_flags = flags; gimbal_pitch.start_blocks = gimbal_pitch.blocks;
        if (!profile_valid()) { gimbal_pitch.state = PITCH_PROBE; scope_update(); return; }
        if (gimbal_pitch.recovery_pending) {
            ++gimbal_pitch.recovery_attempts; gimbal_pitch.last_recovery_ms = now; restart_in_progress = 1U;
        }
        gimbal_pitch.recovery_pending = gimbal_pitch.recovery_waiting = 0U;
        ++gimbal_pitch.sessions;
        gimbal_pitch.state = PITCH_ZEROING; gimbal_pitch.reason = PITCH_REASON_NONE;
        gimbal_pitch.loop_mode = (pitch_loop_mode_t)requested_mode;
        phase_ms = now; phase_frames = gimbal_pitch.frames;
        zero_ack = enable_ack = driven = handshake_warned = 0U;
        capture_target(); gimbal_pitch.torque = gimbal_pitch.integral = gimbal_pitch.speed_integral_sum = 0.0f;
    } else if (gimbal_pitch.state == PITCH_ZEROING && zero_ack) {
        gimbal_pitch.state = PITCH_ENABLING; phase_ms = now; phase_frames = gimbal_pitch.frames;
        driven = handshake_warned = 0U;
    } else if (gimbal_pitch.state == PITCH_ENABLING && enable_ack && gimbal_pitch.frames != phase_frames && gimbal_pitch.motor_state == 1U) {
        gimbal_pitch.state = PITCH_ACTIVE; ++gimbal_pitch.active_entries;
        if (restart_in_progress) {
            ++gimbal_pitch.recoveries; gimbal_pitch.last_recovered_ms = now; restart_in_progress = 0U;
        }
        capture_target();
        if (gimbal_pitch.loop_mode == PITCH_LOOP_ANGLE && !gimbal_pitch.gyro_mode) {
            gimbal_pitch.goal_deg = fmaxf(low, fminf(high, PITCH_FLAT_ANGLE_DEG)); gimbal_pitch.flat_return = 1U;
        }
    } else if (gimbal_pitch.state == PITCH_ACTIVE) {
        if (gimbal_pitch.loop_mode == PITCH_LOOP_SPEED) speed_reference_update(flags, low, high);
        else {
            float offset = gimbal_pitch.gyro_mode ? gimbal_pitch.imu_pitch_deg - gimbal_pitch.actual_deg : 0.0f;
            angle_reference_update(flags, dt, low + offset, high + offset);
        }
        error = gimbal_pitch.speed_target_dps - gimbal_pitch.speed_dps;
        gimbal_pitch.speed_error_dps = error;
        gimbal_pitch.speed_integral_sum = clip(gimbal_pitch.speed_integral_sum + error, gimbal_pitch.gyro_mode ? PITCH_GYRO_SPEED_SUM_MAX : PITCH_SPEED_INTEGRAL_MAX);
        gimbal_pitch.integral = (gimbal_pitch.gyro_mode ? PITCH_GYRO_SPEED_KI : PITCH_SPEED_KI) * gimbal_pitch.speed_integral_sum;
        gimbal_pitch.gravity_torque = gravity_compensation(PITCH_USE_IMU ? gimbal_pitch.imu_pitch_deg : gimbal_pitch.actual_deg - PITCH_FLAT_ANGLE_DEG);
        gimbal_pitch.speed_loop_torque = PITCH_OUTPUT_DIRECTION * clip((gimbal_pitch.gyro_mode ? PITCH_GYRO_SPEED_KP : PITCH_SPEED_KP) * error + gimbal_pitch.integral, PITCH_SPEED_OUTPUT_LIMIT);
        output = clip(gimbal_pitch.speed_loop_torque + gimbal_pitch.gravity_torque, PITCH_TORQUE_LIMIT);
        if ((gimbal_pitch.actual_deg >= high && output * PITCH_OUTPUT_DIRECTION > 0.0f) ||
            (gimbal_pitch.actual_deg <= low && output * PITCH_OUTPUT_DIRECTION < 0.0f)) output = 0.0f;
        gimbal_pitch.torque = output;
        gimbal_pitch.peak_torque = fmaxf(gimbal_pitch.peak_torque, fabsf(output));
    }
    scope_update();
}

/** @brief 持续交替读四个参数和反馈，握手及失能优先，回读不被力矩帧饿死。 */
int Gimbal_Pitch_MakeFrame(uint32_t now, uint32_t *id, uint8_t d[8], uint8_t *kind)
{
    static const uint8_t rid[4] = {21U, 22U, 23U, 10U};
    int disabling = gimbal_pitch.state == PITCH_STOPPING || gimbal_pitch.state == PITCH_FAULT ||
        gimbal_pitch.state == PITCH_STANDBY || (gimbal_pitch.state == PITCH_WAIT && gimbal_pitch.reason != PITCH_REASON_NONE);
    int special = disabling || gimbal_pitch.state == PITCH_ZEROING || gimbal_pitch.state == PITCH_ENABLING;
    if (!id || !d || !kind) return 0;
    if (special && (!driven || now - drive_ms >= PITCH_SPECIAL_MS)) {
        *id = PITCH_MOTOR_ID;
        if (disabling || gimbal_pitch.state == PITCH_ENABLING) {
            memset(d, 0xFF, 8U); d[7] = disabling ? 0xFDU : 0xFCU;
            *kind = disabling ? PITCH_TX_DISABLE : PITCH_TX_ENABLE;
            return 1;
        }
        if (profile_valid()) { torque_frame(0.0f, d); *kind = PITCH_TX_ZERO; return 1; }
    }
    if (!queried || now - query_ms >= PITCH_QUERY_MS) {
        memset(d, 0, 8U); *id = 0x7FFU; *kind = PITCH_TX_QUERY; d[0] = PITCH_MOTOR_ID;
        if (!(query_index & 1U)) { d[2] = 0x33U; d[3] = rid[query_index >> 1]; }
        else d[2] = 0xCCU;
        return 1;
    }
    if (gimbal_pitch.state == PITCH_ACTIVE && profile_valid() && (!driven || now - drive_ms >= PITCH_COMMAND_MS)) {
        if (gimbal_pitch.frames && gimbal_pitch.motor_state == 0U &&
            (!enable_retry_seen || now - enable_retry_ms >= PITCH_SPECIAL_MS)) {
            *id = PITCH_MOTOR_ID; *kind = PITCH_TX_ENABLE; memset(d, 0xFF, 8U); d[7] = 0xFCU; return 1;
        }
        *id = PITCH_MOTOR_ID; *kind = PITCH_TX_TORQUE; torque_frame(gimbal_pitch.torque, d); return 1;
    }
    return 0;
}

/** @brief 入队仅决定发送节拍，不改变使能握手结果。 */
void Gimbal_Pitch_Queued(uint32_t now, uint8_t kind)
{
    if (kind == PITCH_TX_QUERY) { queried = 1U; query_ms = now; query_index = (query_index + 1U) & 7U; }
    else { driven = 1U; drive_ms = now; }
    if (kind == PITCH_TX_ENABLE && gimbal_pitch.state == PITCH_ACTIVE) {
        enable_retry_seen = 1U; enable_retry_ms = now; ++pitch_diag.enable_retry_queued;
    }
}

/** @brief 控制器重启只撤销旧确认，握手阶段重新从零输出或新失能反馈开始。 */
void Gimbal_Pitch_TransportRestart(uint32_t now)
{
    zero_ack = enable_ack = disable_acks = 0U;
    query_ms = drive_ms = now;
    queried = driven = 0U;
    enable_retry_seen = 0U;
    gimbal_pitch.recovery_waiting = 0U;
    if (gimbal_pitch.state == PITCH_ZEROING || gimbal_pitch.state == PITCH_ENABLING)
        gimbal_pitch.state = PITCH_ZEROING;
    if (gimbal_pitch.state == PITCH_STOPPING) gimbal_pitch.stop_unconfirmed = 0U;
}

/** @brief 发送失败仅累计事件，真实电机命令TXOK恢复发送并推进握手确认。 */
void Gimbal_Pitch_TxComplete(uint32_t now, uint8_t kind, int success)
{
    if (!success) {
        ++gimbal_pitch.tx_errors;
        if (!gimbal_pitch.tx_failure_active) {
            gimbal_pitch.tx_failure_active = 1U;
            gimbal_pitch.tx_failure_since_ms = now;
            gimbal_pitch.tx_failure_streak = 0U;
        }
        if (gimbal_pitch.tx_failure_streak != UINT32_MAX) ++gimbal_pitch.tx_failure_streak;
        return;
    }
    ++gimbal_pitch.tx_confirmed;
    if (kind <= PITCH_TX_DISABLE) {
        gimbal_pitch.tx_failure_active = 0U;
        gimbal_pitch.tx_failure_streak = gimbal_pitch.tx_failure_since_ms = 0U;
    }
    if (kind == PITCH_TX_ZERO && gimbal_pitch.state == PITCH_ZEROING) zero_ack = 1U;
    if (kind == PITCH_TX_ENABLE && gimbal_pitch.state == PITCH_ENABLING && !enable_ack) {
        phase_frames = gimbal_pitch.frames;
        enable_ack = 1U;
    }
    if (kind == PITCH_TX_DISABLE) { if (disable_acks < 2U) ++disable_acks; gimbal_pitch.stop_unconfirmed = 0U; }
}

/** @brief 退出阶段必须取消待发送的使能和力矩。 */
int Gimbal_Pitch_NeedsStop(void)
{
    return gimbal_pitch.state != PITCH_ACTIVE && gimbal_pitch.state != PITCH_ZEROING && gimbal_pitch.state != PITCH_ENABLING;
}
