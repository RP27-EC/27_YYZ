#ifndef DOWN_LED_TASK_H
#define DOWN_LED_TASK_H
#include "main.h"
#include "cmsis_os.h"
void StartLedTask(void const *argument);
extern volatile uint32_t led_task_count;
extern volatile uint32_t led_task_heartbeat;
#endif
