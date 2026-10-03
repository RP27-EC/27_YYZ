#ifndef DOWN_CHASSIS_PARAM_H
#define DOWN_CHASSIS_PARAM_H
typedef struct {
    float kp, ki_per_second, integral_limit, acceleration_rpm_per_second;
} chassis_pid_param_t;
extern const chassis_pid_param_t chassis_speed_param;
#endif
