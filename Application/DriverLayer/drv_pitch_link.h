/**
 * @file    drv_pitch_link.h
 * @brief   H723 CAN2发送Pitch遥控/键鼠输入、接收F407状态摘要。
 */
#ifndef DOWN_DRV_PITCH_LINK_H
#define DOWN_DRV_PITCH_LINK_H
/* Includes ------------------------------------------------------------------*/
#include "board_pitch_protocol.h"
/* Exported typedef ----------------------------------------------------------*/
typedef struct {
    board_pitch_command_t command; /**< 最近构造的上板命令。 */
    board_pitch_status_t status; /**< 最近校验通过的上板反馈摘要。 */
    uint32_t frames, last_ms, queued, confirmed, errors, bad_frames, aborted; /**< 接收心跳及发送统计，时刻单位毫秒。 */
    uint8_t init_ok, online, pending; /**< 初始化、摘要在线及发送槽占用。 */
} pitch_link_t;
/* Exported variables --------------------------------------------------------*/
extern pitch_link_t pitch_link; /**< 下板Watch中的上板状态入口。 */
/* Exported functions --------------------------------------------------------*/
/** @brief 使用CAN2 PB5/PB6、1Mbps及独立Message RAM初始化板间CAN。 */
void Pitch_Link_Init(void);
/** @brief 非阻塞轮询摘要、映射输入并调度10ms命令。 */
void Pitch_Link_Update(uint32_t now);
#endif
