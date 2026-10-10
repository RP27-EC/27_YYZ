/**
 * @file    yaw_probe.c
 * @brief   维护Yaw只读反馈、暂定零点与连续回绕角度，不提供运动控制。
 */

/* Includes ------------------------------------------------------------------*/
#include "yaw_probe.h"
#include "gimbal_config.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    int64_t accumulated_counts;     /**< 本段连续角累计计数，已应用方向符号。 */
    uint32_t last_frames;           /**< 上次处理的状态2计数，用于识别丢样。 */
    uint32_t last_ms;               /**< 上次状态2时刻，毫秒。 */
    uint32_t zero_encoder;          /**< 本段使用的暂定零点。 */
    int32_t direction;              /**< 本段使用的方向符号。 */
    uint16_t last_encoder;          /**< 上次处理的编码器计数。 */
    uint8_t started;                /**< 是否曾建立角度跟踪。 */
    uint8_t valid;                  /**< 本段累计计数是否连续有效。 */
} yaw_angle_track_t;

/* Private variables ---------------------------------------------------------*/
static volatile yaw_probe_rx_t received; /**< CAN中断更新、任务复制的反馈缓存。 */
static yaw_angle_track_t angle_track;    /**< 任务侧连续角跟踪，仅用于反馈观察。 */
static uint32_t last_query_ms;           /**< 上次实际入队时刻，毫秒。 */
static uint8_t query_started;            /**< 当前启用期间是否已成功入队。 */
static uint8_t query_allowed;            /**< 本周期是否已通过独立许可检查。 */

/* Exported variables --------------------------------------------------------*/
yaw_probe_t yaw_probe;                                                  /**< 任务侧反馈和查询诊断对象。 */
volatile uint32_t yaw_probe_enable = GIMBAL_YAW_BOOT_PROBE_ENABLE;      /**< 配置默认反馈查询开关。 */
volatile uint32_t yaw_probe_zero_encoder = YAW_PROBE_ZERO_ENCODER;      /**< 暂定正前计数。 */
volatile int32_t yaw_probe_direction = YAW_PROBE_DIRECTION;             /**< 俯视逆时针为正。 */
volatile uint32_t yaw_probe_angle_reset = 0;                            /**< 手动重建连续角请求。 */

/* Private functions ---------------------------------------------------------*/
/** @brief 将一圈内编码器差值映射到[-32768,32768)计数。 */
static int32_t wrap_counts(int32_t counts)
{
    if (counts >= YAW_PROBE_HALF_COUNTS) { counts -= YAW_PROBE_ENCODER_COUNTS; }
    else if (counts < -YAW_PROBE_HALF_COUNTS) { counts += YAW_PROBE_ENCODER_COUNTS; }
    return counts;
}

/** @brief 发布合法端点的最短角差，丢样记诊断；非法角度舍去并继续等待。 */
static void update_angles(void)
{
    uint32_t zero = yaw_probe_zero_encoder;
    int32_t direction = yaw_probe_direction;
    if (zero >= YAW_PROBE_ENCODER_COUNTS || (direction != 1 && direction != -1)) {
        ++yaw_probe.invalid_parameters;
        return;
    }
    if (!yaw_probe.online) { ++yaw_probe.stale_samples; return; }
    yaw_probe.angle_valid = 1U;
    uint16_t encoder = yaw_probe.feedback.status.encoder;
    int32_t relative = wrap_counts(direction * ((int32_t)encoder - (int32_t)zero));
    if (!angle_track.started || yaw_probe_angle_reset == 1U ||
        angle_track.zero_encoder != zero || angle_track.direction != direction) {
        angle_track.accumulated_counts = relative;
        angle_track.zero_encoder = zero;
        angle_track.direction = direction;
        angle_track.started = angle_track.valid = 1;
        yaw_probe_angle_reset = 0;
    } else if (angle_track.last_frames != yaw_probe.feedback.state_frames) {
        int32_t delta = wrap_counts((int32_t)encoder - angle_track.last_encoder);
        uint32_t samples = yaw_probe.feedback.state_frames - angle_track.last_frames;
        if (samples > 1U) yaw_probe.skipped_samples += samples - 1U;
        if (delta == -YAW_PROBE_HALF_COUNTS) {
            ++yaw_probe.ambiguous_samples;
            return;
        } else { angle_track.accumulated_counts += direction * delta; }
        angle_track.valid = 1U;
    }
    angle_track.last_encoder = encoder;
    angle_track.last_frames = yaw_probe.feedback.state_frames;
    angle_track.last_ms = yaw_probe.feedback.last_state_ms;
    yaw_probe.relative_deg = relative * (360.0f / YAW_PROBE_ENCODER_COUNTS);
    yaw_probe.continuous_deg = angle_track.accumulated_counts * (360.0f / YAW_PROBE_ENCODER_COUNTS);
    yaw_probe.continuous_valid = angle_track.valid;
}

/* Exported functions --------------------------------------------------------*/
/** @brief 重置接收缓存和诊断，使用配置的上电查询许可。 */
void Yaw_Probe_Init(void)
{
    received = (yaw_probe_rx_t){0};
    yaw_probe = (yaw_probe_t){0};
    angle_track = (yaw_angle_track_t){0};
    yaw_probe_enable = GIMBAL_YAW_BOOT_PROBE_ENABLE;
    yaw_probe_zero_encoder = YAW_PROBE_ZERO_ENCODER;
    yaw_probe_direction = YAW_PROBE_DIRECTION;
    yaw_probe_angle_reset = 0;
    last_query_ms = 0;
    query_started = 0;
    query_allowed = 0;
}

/** @brief 缓存目标CAN编号的8字节帧，按命令更新对应状态字段。 */
int Yaw_Probe_Receive(uint32_t can_id, const uint8_t *data, uint32_t size, uint32_t now_ms)
{
    if (can_id != YAW_PROBE_CAN_ID || !data || size != 8U) { return 0; }
    yaw_probe_rx_t next = received;
    uint8_t command = KT_Yaw_DecodeStatus(data, size, &next.status);
    for (unsigned i = 0; i < 8U; ++i) { next.raw[i] = data[i]; }
    next.last_command = data[0];
    next.last_ms = now_ms;
    ++next.frames;
    if (command == YAW_PROBE_STATE2 || command == GIMBAL_YAW_TORQUE_COMMAND) {
        ++next.state_frames;
        next.last_state_ms = now_ms;
        if (command == GIMBAL_YAW_TORQUE_COMMAND) {
            ++next.torque_frames;
            next.last_torque_ms = now_ms;
            next.torque_current_raw = next.status.current_raw;
            next.torque_speed_dps = next.status.speed_dps;
        }
    } else if (command == YAW_PROBE_STATE1) {
        ++next.error_frames;
        next.last_error_ms = now_ms;
    }
    else { ++next.unknown_frames; }
    received = next;
    return 1;
}

/** @brief 复制完整快照，以状态2判断在线并处理编码器回绕。 */
void Yaw_Probe_Update(uint32_t now_ms)
{
    yaw_probe.feedback = received;
    yaw_probe.online = yaw_probe.feedback.state_frames &&
        (uint32_t)(now_ms - yaw_probe.feedback.last_state_ms) <= YAW_PROBE_OFFLINE_MS;
    yaw_probe.encoder_deg = yaw_probe.feedback.status.encoder * (360.0f / 65536.0f);
    update_angles();
}

/** @brief 校验调用者提供的查询条件；整车模式由驱动放行底盘并行条件。 */
int Yaw_Probe_Allowed(int output_disabled, int chassis_stopped, int can_ready)
{
    yaw_probe.block_reason = 0;
    query_allowed = 0;
    if (yaw_probe_enable == 1U) {
        if (!output_disabled) { yaw_probe.block_reason |= YAW_PROBE_BLOCK_OUTPUT; }
        if (!chassis_stopped) { yaw_probe.block_reason |= YAW_PROBE_BLOCK_CHASSIS; }
        if (!can_ready) { yaw_probe.block_reason |= YAW_PROBE_BLOCK_CAN; }
        if (!yaw_probe.block_reason) { query_allowed = 1; return 1; }
    }
    query_started = 0;
    return 0;
}

/** @brief 以入队时间限频，并按4次状态2、1次状态1生成只读命令。 */
int Yaw_Probe_MakeQuery(uint32_t now_ms, uint8_t out[8])
{
    if (yaw_probe_enable != 1U || !query_allowed || yaw_probe.block_reason) { return 0; }
    if (query_started && (uint32_t)(now_ms - last_query_ms) < YAW_PROBE_QUERY_MS) { return 0; }
    uint8_t command = yaw_probe.tx_queued % YAW_PROBE_STATE1_DIVIDER == YAW_PROBE_STATE1_DIVIDER - 1U ?
                      YAW_PROBE_STATE1 : YAW_PROBE_STATE2;
    return KT_Yaw_BuildRead(command, out);
}

/** @brief 标记入队成功，失败/忙碌时不消耗下次查询节拍。 */
void Yaw_Probe_QueryQueued(uint32_t now_ms)
{
    ++yaw_probe.tx_queued;
    last_query_ms = now_ms;
    query_started = 1;
}
