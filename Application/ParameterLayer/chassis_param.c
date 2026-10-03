#include "chassis_param.h"
/* Preserve the proven low-speed controller; Ki explicitly multiplies seconds. */
const chassis_pid_param_t chassis_speed_param = {8.0f, 20.0f, 500.0f, 500.0f};
