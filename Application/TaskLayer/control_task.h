#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "imu_sensor.h"

void Control_Init(void);
void StartControlTask(void const * argument);


#endif
