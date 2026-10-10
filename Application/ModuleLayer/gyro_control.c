/**
 * @file    gyro_control.c
 * @brief   上板Yaw连续角、IMU失效检查与云台坐标底盘变换。
 */
/* Includes ------------------------------------------------------------------*/
#include "gyro_control.h"
#include <math.h>
/* Exported variables --------------------------------------------------------*/
volatile float gyro_scope_yaw_deg, gyro_scope_yaw_dps, gyro_scope_continuous_deg;    /**< IMU采样镜像，度、度/秒。 */
volatile float gyro_scope_mechanical_deg;                                            /**< 编码器相对角镜像，度。 */
volatile uint32_t gyro_scope_imu_ready;                                              /**< 新鲜IMU许可镜像。 */
gyro_control_t gyro_control;                                                         /**< 姿态与连续角观察对象。 */
gyro_rx_diag_t gyro_rx_diag;                                                         /**< 解码后IMU到达和拒绝原因计数。 */
volatile int32_t gyro_yaw_angle_direction = GYRO_YAW_ANGLE_DIRECTION;                /**< IMU角度符号。 */
volatile int32_t gyro_yaw_speed_direction = GYRO_YAW_SPEED_DIRECTION;                /**< IMU速度符号。 */
/* Exported functions --------------------------------------------------------*/
/** @brief 重置IMU接收参考与方向。 */
void Gyro_Init(void)
{
    gyro_control = (gyro_control_t){0};
    gyro_rx_diag = (gyro_rx_diag_t){0};
    gyro_scope_yaw_deg = gyro_scope_yaw_dps = gyro_scope_continuous_deg = gyro_scope_mechanical_deg = 0.0f;
    gyro_scope_imu_ready = 0U;
    gyro_yaw_angle_direction = GYRO_YAW_ANGLE_DIRECTION;
    gyro_yaw_speed_direction = GYRO_YAW_SPEED_DIRECTION;
}
/** @brief 清除运动故障、转向和姿态参考，保留方向配置及接收诊断。 */
void Gyro_StandbyReset(void)
{
    gyro_control = (gyro_control_t){.frames = gyro_control.frames, .generation = gyro_control.generation + 1U};
    gyro_scope_yaw_deg = gyro_scope_yaw_dps = gyro_scope_continuous_deg = gyro_scope_mechanical_deg = 0.0f;
    gyro_scope_imu_ready = 0U;
}

/** @brief 在控制决策前发布机械相对角与独立IMU镜像。 */
void Gyro_Update(uint32_t now, float mechanical_deg, int mechanical_ready)
{
    if (mechanical_ready && isfinite(mechanical_deg)) gyro_control.mechanical_deg = mechanical_deg;
    gyro_control.mechanical_ready = mechanical_ready && isfinite(mechanical_deg);
    gyro_scope_yaw_deg = gyro_control.yaw_deg;
    gyro_scope_yaw_dps = gyro_control.speed_dps;
    gyro_scope_continuous_deg = gyro_control.continuous_deg;
    gyro_scope_mechanical_deg = mechanical_deg;
    gyro_scope_imu_ready = Gyro_Ready(now);
}
/** @brief 丢弃异常和重复样本，仅有效新序号推进连续角与接收时间。 */
void Gyro_ReceiveImu(const board_imu_status_t *s, uint32_t now)
{
    if (!s) return;
    ++gyro_rx_diag.received; gyro_rx_diag.last_ms = now;
    if (!s->ready || !isfinite(s->yaw_deg) || !isfinite(s->yaw_dps)) {
        if (!s->ready) ++gyro_rx_diag.not_ready;
        else ++gyro_rx_diag.nonfinite;
        return;
    }
    if ((gyro_yaw_angle_direction != 1 && gyro_yaw_angle_direction != -1) ||
        (gyro_yaw_speed_direction != 1 && gyro_yaw_speed_direction != -1)) {
        ++gyro_rx_diag.bad_direction;
        return;
    }
    if (gyro_control.seen && s->sequence == gyro_control.sequence) { ++gyro_rx_diag.repeated; return; }
    if (gyro_control.seen && now - gyro_control.last_ms > gyro_rx_diag.max_gap_ms)
        gyro_rx_diag.max_gap_ms = now - gyro_control.last_ms;
    float yaw = remainderf(s->yaw_deg * gyro_yaw_angle_direction, 360.0f);
    if (!Gyro_Ready(now)) {
        gyro_control.continuous_deg = yaw;
        ++gyro_control.generation;
    } else gyro_control.continuous_deg += remainderf(yaw - gyro_control.yaw_deg, 360.0f);
    gyro_control.yaw_deg = yaw;
    gyro_control.speed_dps = s->yaw_dps * gyro_yaw_speed_direction;
    gyro_control.angle_direction = gyro_yaw_angle_direction;
    gyro_control.speed_direction = gyro_yaw_speed_direction;
    gyro_control.last_ms = now;
    gyro_control.sequence = s->sequence;
    gyro_control.seen = gyro_control.valid = 1U;
    ++gyro_control.frames;
}
/** @brief 报告IMU的新鲜度及方向一致性，只用于诊断，不直接禁止输出。 */
int Gyro_Ready(uint32_t now)
{
    return gyro_control.valid && gyro_control.seen && now - gyro_control.last_ms <= GYRO_IMU_TIMEOUT_MS &&
        gyro_control.angle_direction == gyro_yaw_angle_direction && gyro_control.speed_direction == gyro_yaw_speed_direction;
}
/** @brief 顺时针相对角旋转前进和右移目标。 */
void Gyro_Transform(float front, float right, float relative_deg, float *of, float *orr)
{
    float angle = -relative_deg * 0.01745329252f;
    *of = front * cosf(angle) - right * sinf(angle);
    *orr = right * cosf(angle) + front * sinf(angle);
}
/** @brief 180度对称折叠后计算一次加二次跟随量。 */
float Gyro_Follow(float relative_deg)
{
    float error = remainderf(-relative_deg, 180.0f) * (4096.0f / 180.0f);
    float quadratic = (GYRO_FOLLOW_AT_90_RPM - GYRO_FOLLOW_LINEAR * 2048.0f) / (2048.0f * 2048.0f);
    return GYRO_FOLLOW_LINEAR * error + quadratic * error * fabsf(error);
}
