/**
 * @file    BMI088Middleware.c
 * @brief   SPI1与BMI088片选/延时适配，记录有界SPI传输失败。
 */
/* Includes ------------------------------------------------------------------*/
#include "BMI088Middleware.h"
#include "main.h"

/* Exported variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
volatile uint32_t BMI088_io_errors; /**< SPI失败计数，上层据此拒绝异常帧。 */

/* Exported functions --------------------------------------------------------*/
void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

uint32_t btick = 0;

uint32_t bmi2_micros(void)
{
	register uint32_t bms, bus;

	bms = HAL_GetTick();

	bus = TIM2->CNT;
	
  btick = bms*1000 + bus;
    
	return btick;
} 

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us)
{
	uint32_t now = bmi2_micros();
	
	while((bmi2_micros() - now) < us);

}


void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/** @brief 单字节SPI交换，5ms超时后记录失败并返回确定值。 */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0U;
    if (HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 5U) != HAL_OK) ++BMI088_io_errors;
    return rx_data;
}

