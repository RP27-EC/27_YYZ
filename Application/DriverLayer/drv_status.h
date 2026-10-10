/** @file drv_status.h
 * @brief 下板UART/CAN持续重试及只读故障统计。
 */
#ifndef DOWN_DRV_STATUS_H
#define DOWN_DRV_STATUS_H
#include <stdint.h>
typedef struct {
    uint32_t init_ok, fdcan_clock_hz, uart_events, uart_errors, bad_frames;
    uint32_t tx_queued, tx_errors, can_bus_off, can_error_passive;
    uint32_t can_last_error, rc_good_streak;
    uint32_t uart_error_code, uart_last_error_ms, uart_restarts, uart_init_errors; /**< UART错误位、时刻毫秒及重启/初始化失败次数。 */
    uint32_t uart_retry_pending; /**< UART等待任务重启，诊断量。 */
    uint32_t uart_last_event_ms; /**< 最近接收事件时刻，毫秒；无事件时也定期重启接收。 */
    uint32_t can1_init_ok, can1_init_errors, can1_read_errors, can1_bad_frames; /**< CAN1初始化状态及初始化/读取/格式失败次数。 */
    uint32_t can1_status_errors, can1_bus_off_events, can1_recovery_state; /**< 协议读取失败数、Bus-Off次数及恢复阶段0/1/2。 */
    uint32_t can1_recovery_attempts, can1_recoveries, can1_last_error_ms; /**< CAN1恢复尝试、完成数及最近故障时刻毫秒。 */
    uint32_t can1_failure_ms, can1_recovery_ms; /**< 最近连续发送失败时长及恢复尝试时刻，毫秒。 */
    uint32_t can1_recovery_duration_ms; /**< 本次外设恢复持续时间，毫秒。 */
    uint32_t can1_failed_transfers, can1_consecutive_tx_errors, can1_tx_confirmed; /**< CAN1发送失败、连续失败及硬件TXOK次数，不作为失能条件。 */
} mec_io_status_t;
/* Keep existing Watch expressions for communication diagnostics. */
extern volatile mec_io_status_t mec_io;
#endif
