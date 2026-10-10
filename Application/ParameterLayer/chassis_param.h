/**
 * @file    chassis_param.h
 * @brief   轮子PID参数和加速度限制。
 */
#ifndef DOWN_CHASSIS_PARAM_H
#define DOWN_CHASSIS_PARAM_H
/* Exported typedef ----------------------------------------------------------*/
typedef struct {
    float kp, ki_per_second, integral_limit, acceleration_rpm_per_second;
} chassis_pid_param_t;
/* Exported variables --------------------------------------------------------*/
extern const chassis_pid_param_t chassis_speed_param;
#endif
