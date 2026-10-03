#include "led_task.h"
volatile uint32_t led_task_count;
volatile uint32_t led_task_heartbeat;
void StartLedTask(void const *argument)
{
    (void)argument;
    for (;;) {
        /* The F4 PH10 LED is not a verified LED on DM-MC02. */
        led_task_heartbeat ^= 1U;
        ++led_task_count;
        osDelay(500);
    }
}
