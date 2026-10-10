/**
 ******************************************************************************
 * @file        driver.c
 * @author      RobotPilots@2020
 * @brief       上板Pitch驱动管理，仅启用CAN接收与受控发送。
 ******************************************************************************
 * @attention   
 * 
 * Copyright 2020 RobotPilots
 *  
 * @Version     V1.0
 * @date        9-September-2020
 ****************************************************************************
 */
 
/* Includes ------------------------------------------------------------------*/
#include "driver.h"
#include "drv_pitch.h"
#include "drv_fric.h"

/* Exported functions --------------------------------------------------------*/

/** @brief 启动CAN一次并初始化Pitch调度，不启动旧模板UART或发射PWM。 */
void DRIVER_Init(void)
{
	CAN_Filter_Init();
    Drv_Pitch_Init();
    Drv_Fric_Init(HAL_GetTick());
}
