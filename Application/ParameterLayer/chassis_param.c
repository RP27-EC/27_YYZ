/**
 * @file    chassis_param.c
 * @brief   轮子PID参数和加速度限制。
 */

 /* Includes ------------------------------------------------------------------*/
#include "chassis_param.h"

const chassis_pid_param_t chassis_speed_param = {
    .kp = 8.0f,                                 /**< Kp */
    .ki_per_second = 20.0f,                     /**< Ki */  
    .integral_limit = 2000.0f,                  /**< 积分限幅 */
    .acceleration_rpm_per_second = 30000.0f,    /**< 加速度限制 */
};
