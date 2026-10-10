/** @file drv_fric.h
 * @brief 上板CAN1六摩擦轮发送与非阻塞恢复。
 */
#ifndef UP_DRV_FRIC_H
#define UP_DRV_FRIC_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported functions --------------------------------------------------------*/
void Drv_Fric_Init(uint32_t now);
void Drv_Fric_Poll(uint32_t now);
#endif
