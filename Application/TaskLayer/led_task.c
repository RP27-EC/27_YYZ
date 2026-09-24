#include "led_task.h"

/* 记录 LED 翻转次数 */
volatile uint32_t led_task_count = 0;

void StartLedTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        // 翻转蓝灯状态
        HAL_GPIO_TogglePin(LED_PORT, LED_BLUE_PIN);
        led_task_count++;
        osDelay(500);
    }
}
