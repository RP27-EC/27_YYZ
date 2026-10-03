#include "BMI088Middleware.h"
#include "main.h"
#include "drv_tick.h"
extern SPI_HandleTypeDef hspi2;
volatile uint32_t imu_spi_error_count;
void BMI088_GPIO_init(void) {}
void BMI088_com_init(void) {}
void BMI088_delay_us(uint16_t us) { delay_us(us); }
void BMI088_delay_ms(uint16_t ms) { delay_ms(ms); }
void BMI088_ACCEL_NS_L(void) { HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET); }
void BMI088_ACCEL_NS_H(void) { HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET); }
void BMI088_GYRO_NS_L(void) { HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET); }
void BMI088_GYRO_NS_H(void) { HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET); }
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t received = 0;
    if (HAL_SPI_TransmitReceive(&hspi2, &txdata, &received, 1, 10) != HAL_OK) {
        ++imu_spi_error_count;
    }
    return received;
}
