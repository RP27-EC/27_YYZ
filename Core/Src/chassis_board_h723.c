#include "main.h"
#include "chassis_board.h"
#include "drv_status.h"

void Chassis_Board_ClockInit(void)
{
    /* CPU stays on HSI64. PLL1Q supplies CAN: 64/8*40/4 = 80 MHz. */
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 40;
    osc.PLL.PLLP = 2;
    osc.PLL.PLLQ = 4;
    osc.PLL.PLLR = 2;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { Error_Handler(); }
    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_UART5;
    clk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    clk.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK) { Error_Handler(); }
    mec_io.fdcan_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    if (mec_io.fdcan_clock_hz != 80000000U) { Error_Handler(); }

    /* Reference DM-MC02 board: PC15 enables peripheral 5 V (DBUS receiver). */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOC, PWR_5V_EN_Pin, GPIO_PIN_SET);
    GPIO_InitTypeDef pin = {0};
    pin.Pin = PWR_5V_EN_Pin;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &pin);

}
