/**
 * @file    drv_pitch_link.h
 * @brief   H723 CAN2发送Pitch遥控/键鼠输入、接收F407状态摘要。
 */
#ifndef DOWN_DRV_PITCH_LINK_H
#define DOWN_DRV_PITCH_LINK_H
/* Includes ------------------------------------------------------------------*/
#include "board_pitch_protocol.h"
/* Exported typedef ----------------------------------------------------------*/
typedef enum {
    PITCH_LINK_CAN_RUNNING, PITCH_LINK_CAN_BACKOFF, PITCH_LINK_CAN_SYNCHRONIZING
} pitch_link_can_state_t; /**< 0正常收发、1清空旧帧并退避、2等待硬件总线恢复。 */
typedef struct {
    board_pitch_command_t command; /**< 最近构造的上板命令。 */
    board_pitch_status_t status; /**< 最近校验通过的上板反馈摘要。 */
    uint32_t frames, last_ms, queued, confirmed, errors, bad_frames, aborted; /**< 接收心跳及发送统计，时刻单位毫秒。 */
    uint8_t init_ok, online, pending; /**< 初始化、摘要在线及发送槽占用。 */
    uint8_t standby_requested, standby_confirmed, standby_ack; /**< 待命请求、发送确认及上板失能复位确认。 */
} pitch_link_t;
typedef struct {
    uint32_t ms, code, cccr, psr, ecr; /**< 采样时刻毫秒、LEC及对应硬件状态；code=1..6为具体错误。 */
} pitch_can_error_sample_t;
typedef struct {
    uint32_t raw_frames, last_ms, last_id; /**< 已从CAN2 FIFO取出的帧数、时刻毫秒、标准ID。 */
    uint32_t imu_frames, status_frames, format_errors, decode_errors; /**< IMU/状态ID到达数及帧格式/协议拒绝数。 */
    board_imu_status_t last_imu; /**< 最近协议解码成功的IMU，尚未经过有效性和序号检查。 */
    uint8_t last_raw[8]; /**< 最近接收帧原始字节。 */
    uint32_t failure_ms, failure_cccr, failure_psr, failure_ecr; /**< 最近发送失败时刻毫秒及CAN2硬件寄存器快照。 */
    pitch_can_error_sample_t first_error, last_error, first_bus_off; /**< 首个/最近具体错误及首次Bus-Off快照，上电或初始化清零。 */
    uint32_t error_samples[8]; /**< 按LEC索引统计1..6错误的轮询观察次数，非总线全部错误次数。 */
    uint32_t bus_off_events; /**< 观察到Bus-Off由0变1的次数。 */
    uint8_t bus_off; /**< 实际Bus-Off状态，只限制本CAN外设收发，不触发整车待命。 */
    uint8_t recovery_state; /**< 恢复阶段，取pitch_link_can_state_t枚举值。 */
    uint32_t recovery_attempts, recoveries, recovery_timeouts; /**< 启动硬件恢复、恢复完成及等待超时次数。 */
    uint32_t last_recovery_ms, last_recovered_ms; /**< 最近硬件恢复尝试和完成时刻，毫秒。 */
    uint32_t discarded_rx, recovery_progress_samples; /**< 恢复前丢弃的旧接收帧及硬件恢复进度LEC=5观察次数。 */
    uint32_t init_errors, init_retries, read_errors, status_age_ms, status_timeouts; /**< 初始化失败/重试、接收失败、状态年龄毫秒及超时观察次数。 */
    uint32_t consecutive_tx_errors, failure_duration_ms; /**< 连续发送失败次数及持续时间毫秒，只触发外设恢复，不停控。 */
} pitch_link_diag_t;
/* Exported variables --------------------------------------------------------*/
extern pitch_link_t pitch_link; /**< 下板Watch中的上板状态入口。 */
extern pitch_link_diag_t pitch_link_diag; /**< CAN2收发诊断及Bus-Off恢复状态，不作为整车停控条件。 */
/* Exported functions --------------------------------------------------------*/
/** @brief 使用CAN2 PB5/PB6、1Mbps及独立Message RAM初始化板间CAN。 */
void Pitch_Link_Init(void);
/** @brief 非阻塞轮询摘要、映射输入并调度10ms命令。 */
void Pitch_Link_Update(uint32_t now);
/** @brief 控制决策前接收最新上板状态与IMU帧。 */
void Pitch_Link_Receive(uint32_t now);
/** @brief 取出一次Bus-Off诊断事件，不要求控制任务待命。 */
int Pitch_Link_TakeBusOff(void);
/** @brief 持续发送待命复位请求，取消旧ACTIVE许可，保留序号与通信诊断。 */
void Pitch_Link_RequestStandby(uint32_t now);
/** @brief 遥控重新在线后返回成功；不等待上板待命回复。 */
int Pitch_Link_StandbyReady(uint32_t now);
/** @brief 收到待命确认后结束请求，后续由整车回中启动。 */
void Pitch_Link_FinishStandby(void);
#endif
