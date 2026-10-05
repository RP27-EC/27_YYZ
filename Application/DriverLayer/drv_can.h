/**
 * @file    drv_can.h
 * @brief   底盘CAN1、遥控Yaw共享发送和独立测试的驱动接口。
 */
#ifndef DOWN_DRV_CAN_H
#define DOWN_DRV_CAN_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported functions --------------------------------------------------------*/
/** @brief 初始化底盘和Yaw共享的CAN1收发。 */
void Drv_CAN_Init(void);
/** @brief 更新CAN诊断并返回通信是否可用于底盘控制。 */
int Drv_CAN_Ready(void);
/** @brief 调度Yaw控制/查询和优先清零；整车模式空档返回0，允许四轮发送。 */
int Drv_CAN_PollYaw(uint32_t now_ms);
/** @brief 按CAN ID顺序发送四轮电流，不覆盖未完成的Yaw查询。 */
void CAN_Send(void);
#endif
