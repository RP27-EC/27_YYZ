/**
 * @file    can_protocol.c
 * @brief   Pitch机械模式只路由CAN2电机参数/状态与下板命令。
 */
/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"
#include "gimbal_pitch.h"
/* Exported functions --------------------------------------------------------*/
/** @brief 此阶段不接入CAN1摩擦轮或旧模板电机。 */
void CAN1_rxDataHandler(uint32_t id, uint8_t *data)
{
    (void)id; (void)data;
}
/** @brief 接收经底层验证的标准8字节帧，电机与板间ID由核心精确匹配。 */
void CAN2_rxDataHandler(uint32_t id, uint8_t *data)
{
    (void)Gimbal_Pitch_Receive(id, data, 8U, HAL_GetTick());
}
