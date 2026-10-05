#include "chassis_param.h"
/* Preserve the proven low-speed controller; Ki explicitly multiplies seconds. */
const chassis_pid_param_t chassis_speed_param = {
    .kp = 8.0f,
    .ki_per_second = 20.0f,
    .integral_limit = 2000.0f,
    .acceleration_rpm_per_second = 4000.0f,
};
