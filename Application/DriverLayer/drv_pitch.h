/**
 * @file    drv_pitch.h
 * @brief   上板CAN2 Pitch单邮箱调度、实际发送确认和板间状态回传。
 */
#ifndef UP_DRV_PITCH_H
#define UP_DRV_PITCH_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported typedef ----------------------------------------------------------*/
typedef struct {
    uint32_t queued, confirmed, errors, aborted; /**< CAN2入队、TXOK、失败与主动取消次数。 */
    uint8_t pending, ready; /**< 自有邮箱占用及当前CAN可用状态。 */
} pitch_io_t;
/* Exported variables --------------------------------------------------------*/
extern pitch_io_t pitch_io; /**< CAN2传输观察对象。 */
/* Exported functions --------------------------------------------------------*/
/** @brief CAN启动后初始化单邮箱调度。 */
void Drv_Pitch_Init(void);
/** @brief 轮询发送完成、运行2ms控制并发送电机帧和板间摘要。 */
void Drv_Pitch_Poll(uint32_t now);
#endif
