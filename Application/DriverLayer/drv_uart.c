#include "main.h"
#include "drv_uart.h"
#include "drv_status.h"
#include "rc_sensor.h"
UART_HandleTypeDef huart5;
static uint8_t dbus_buffer[36];
static void receive_start(void)
{
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart5, dbus_buffer, sizeof(dbus_buffer)) != HAL_OK) {
        RC_Sensor_Invalidate();
        mec_io.rc_good_streak = 0;
        ++mec_io.uart_errors;
    }
}
void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    if (uart->Instance != UART5) { return; }
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef pin = {0};
    pin.Pin = GPIO_PIN_2;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_PULLUP;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    pin.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &pin);
    HAL_NVIC_SetPriority(UART5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}


void Drv_UART_Init(void)
{
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 100000;
    huart5.Init.WordLength = UART_WORDLENGTH_9B; /* 8 payload bits + even parity. */
    huart5.Init.StopBits = UART_STOPBITS_2; /* Same as the Hero DBUS configuration. */
    huart5.Init.Parity = UART_PARITY_EVEN;
    huart5.Init.Mode = UART_MODE_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    if (HAL_UART_Init(&huart5) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) { Error_Handler(); }

    receive_start();
}

void Drv_UART_Poll(void)
{
    if (huart5.RxState == HAL_UART_STATE_READY) { receive_start(); }
}
void UART5_IRQHandler(void) { HAL_UART_IRQHandler(&huart5); }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart != &huart5) { return; }
    ++mec_io.uart_events;
    RC_Sensor_Receive(dbus_buffer, size, HAL_GetTick());
    mec_io.bad_frames = rc_receive_status.bad_frames;
    mec_io.rc_good_streak = rc_receive_status.good_streak;
    receive_start();
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart5) { return; }
    ++mec_io.uart_errors;
    RC_Sensor_Invalidate();
    mec_io.rc_good_streak = 0;
    HAL_UART_AbortReceive(uart);
    receive_start();
}
