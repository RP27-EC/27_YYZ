#include "main.h"
#include "cmsis_os.h"
#include "imu_sensor.h"
#include "control_task.h"

volatile uint32_t debug_stage;
volatile uint32_t debug_core_clock_hz;
volatile uint32_t debug_cpuid;
volatile uint32_t debug_device_id;
volatile uint32_t debug_imu_init_code;
volatile uint32_t debug_error_code;
void MX_FREERTOS_Init(void);
void Down_Board_Init(void);

int main(void)
{
    debug_stage = 1;
    SCB->VTOR = FLASH_BASE;
    SystemCoreClockUpdate();
    debug_core_clock_hz = SystemCoreClock;
    debug_cpuid = SCB->CPUID;
    debug_device_id = DBGMCU->IDCODE;
    if (HAL_Init() != HAL_OK) { Error_Handler(); }
    debug_stage = 2;

    /* Initial H723 bring-up uses the reset HSI clock, with a separate PLL1Q for CAN. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    Down_Board_Init();
    debug_stage = 3;

    imu_sensor.init(&imu_sensor);
    debug_imu_init_code = imu_sensor.work_state.init_code;
    debug_stage = 4;
    Control_Init();
    MX_FREERTOS_Init();
    debug_stage = 5;
    osKernelStart();
    debug_error_code = 1;
    Error_Handler();
}

void Error_Handler(void)
{
    if (debug_error_code == 0U) { debug_error_code = 2U; }
    __disable_irq();
    while (1) { __NOP(); }
}
