/**
 * @file    drv_pitch.h
 * @brief   上板CAN2发送容错、非阻塞恢复、实际发送确认和板间状态回传。
 */
#ifndef UP_DRV_PITCH_H
#define UP_DRV_PITCH_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported typedef ----------------------------------------------------------*/
typedef enum {
    PITCH_CAN_RECOVERY_IDLE, PITCH_CAN_RECOVERY_ABORT, PITCH_CAN_RECOVERY_INIT,
    PITCH_CAN_RECOVERY_SYNC, PITCH_CAN_RECOVERY_BACKOFF
} pitch_can_recovery_state_t; /**< 正常、取消旧帧、进入初始化、同步及等待重试。 */
typedef struct {
    uint32_t queued, confirmed, errors, aborted; /**< CAN2入队、TXOK、失败与主动取消次数。 */
    uint8_t pending, ready; /**< 自有邮箱占用及软件控制许可；CAN异常仅记录，不撤销许可。 */
    uint8_t transport_ready, failure_active; /**< 硬件此刻可以收发、连续失败期尚未取得真实TXOK。 */
    uint32_t failure_streak, failure_since_ms, last_tx_ok_ms; /**< 连续失败数、失败期起点及最近实际TXOK时刻，毫秒。 */
    uint32_t unavailable_samples; /**< 硬件不可收发的20ms间隔观察累计数，不等于发送失败总数。 */
    uint32_t failure_duration_ms, last_failure_duration_ms; /**< 当前及最近完成发送故障期的持续时间，毫秒。 */
    uint32_t status_encode_errors, imu_encode_errors; /**< 摘要编码异常次数，失败后继续调度电机帧。 */
    uint32_t error_passive_events, bus_off_events; /**< 错误被动和Bus-Off的进入次数。 */
    uint32_t arbitration_lost, tx_timeouts; /**< 仲裁失败和单个邮箱20ms超时次数。 */
    pitch_can_recovery_state_t recovery_state; /**< CAN硬件非阻塞恢复阶段。 */
    uint32_t recovery_attempts, recoveries, recovery_failures, last_recovery_ms; /**< 恢复请求、恢复后真实TXOK、阶段失败及最近请求时刻，毫秒。 */
    uint8_t recovery_unconfirmed; /**< 硬件已同步，仍等待恢复后的真实TXOK确认。 */
    uint32_t imu_queued, imu_confirmed, imu_errors; /**< IMU帧入队、TXOK及失败次数。 */
    uint32_t status_queued, status_confirmed, status_errors; /**< 状态帧入队、TXOK及失败次数。 */
    uint32_t can_esr, failure_ms, failure_esr, failure_tsr; /**< 当前错误状态及最近发送失败时刻毫秒和硬件快照。 */
} pitch_io_t;
/* Exported variables --------------------------------------------------------*/
extern pitch_io_t pitch_io; /**< CAN2传输观察对象。 */
/* Exported functions --------------------------------------------------------*/
/** @brief CAN启动后初始化单邮箱调度。 */
void Drv_Pitch_Init(void);
/** @brief 轮询发送完成、运行2ms控制并发送电机帧和板间摘要。 */
void Drv_Pitch_Poll(uint32_t now);
#endif
