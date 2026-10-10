/** @file drv_uart.c
 * @brief DBUS持续接收和任务重启，错误不撤销最后合法快照。
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "drv_uart.h"
#include "drv_status.h"
#include "rc_sensor.h"
#include "chassis_config.h"
/* Exported variables --------------------------------------------------------*/
UART_HandleTypeDef huart5; /**< DBUS UART5句柄。 */
/* Private variables ---------------------------------------------------------*/
static uint8_t dbus_buffer[36]; /**< DBUS接收缓存，字节。 */
static uint8_t initialized; /**< UART配置成功。 */
static uint32_t retry_ms; /**< 最近启动时刻，毫秒。 */
/* Private functions ---------------------------------------------------------*/
/** @brief 记录故障并交由任务重试，不改有效帧时刻。 */
static void record_error(void)
{
    ++mec_io.uart_errors;
    mec_io.uart_error_code = huart5.ErrorCode;
    mec_io.uart_last_error_ms = HAL_GetTick();
    mec_io.uart_retry_pending = 1U;
}
/** @brief 启动接收，失败仍保留后续重试路径。 */
static void receive_start(void)
{
    retry_ms = HAL_GetTick();
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart5, dbus_buffer, sizeof(dbus_buffer)) != HAL_OK) {
        record_error();
    } else mec_io.uart_retry_pending = 0U;
}
/* Exported functions --------------------------------------------------------*/
/** @brief 配置DBUS接收引脚和中断。 */
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


/** @brief 初始化100kbit/s DBUS；失败由任务持续重试。 */
void Drv_UART_Init(void)
{
    initialized = 0U;
    retry_ms = HAL_GetTick();
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 100000;
    huart5.Init.WordLength = UART_WORDLENGTH_9B;
    huart5.Init.StopBits = UART_STOPBITS_2;
    huart5.Init.Parity = UART_PARITY_EVEN;
    huart5.Init.Mode = UART_MODE_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    if (HAL_UART_Init(&huart5) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) {
        ++mec_io.uart_init_errors; record_error(); return;
    }

    initialized = 1U;
    mec_io.uart_last_event_ms = HAL_GetTick();
    receive_start();
}

/** @brief 每10ms重试失败接收，不修改遥控在线期限。 */
void Drv_UART_Poll(void)
{
    if (HAL_GetTick() - retry_ms < 10U) return;
    if (!initialized) { ++mec_io.uart_restarts; Drv_UART_Init(); return; }
    if (HAL_GetTick() - mec_io.uart_last_event_ms > CHASSIS_OFFLINE_MS &&
        HAL_GetTick() - retry_ms >= CHASSIS_OFFLINE_MS) mec_io.uart_retry_pending = 1U;
    if (mec_io.uart_retry_pending) {
        ++mec_io.uart_restarts;
        if (HAL_UART_AbortReceive(&huart5) != HAL_OK) { retry_ms = HAL_GetTick(); record_error(); return; }
        receive_start();
    } else if (huart5.RxState == HAL_UART_STATE_READY) receive_start();
}
/** @brief 处理UART硬件中断。 */
void UART5_IRQHandler(void) { HAL_UART_IRQHandler(&huart5); }
/** @brief 丢弃非法DBUS帧并继续接收下一帧。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart != &huart5) { return; }
    ++mec_io.uart_events;
    mec_io.uart_last_event_ms = HAL_GetTick();
    if (mec_io.uart_retry_pending) { ++mec_io.bad_frames; return; }
    RC_Sensor_Receive(dbus_buffer, size, HAL_GetTick());
    mec_io.bad_frames = rc_receive_status.bad_frames;
    mec_io.rc_good_streak = rc_receive_status.good_streak;
    receive_start();
}
/** @brief IRQ仅记录错误，任务中重新启动接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart5) { return; }
    record_error();
}
