/**
 * @file    can_protocol.c
 * @brief   CAN1路由六摩擦轮，CAN2路由Pitch和独立发射许可。
 */
/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"
#include "gimbal_pitch.h"
#include "fric.h"
/* Exported functions --------------------------------------------------------*/
/** @brief 只向六摩擦轮模块路由CAN1标准反馈。 */
void CAN1_rxDataHandler(uint32_t id, uint8_t *data)
{
    (void)Fric_ReceiveMotor(id, data, 8U, HAL_GetTick());
}
/** @brief 接收经底层验证的标准8字节帧，电机与板间ID由核心精确匹配。 */
void CAN2_rxDataHandler(uint32_t id, uint8_t *data)
{
    if (id == BOARD_FRIC_COMMAND_ID) {
        (void)Fric_ReceiveCommand(data, 8U, HAL_GetTick());
        return;
    }
    (void)Gimbal_Pitch_Receive(id, data, 8U, HAL_GetTick());
}
