/**
 * @file    device.c
 * @brief   初始化坐标BMI088与Pitch探测，遥控仍由下板转发。
 */
/* Includes ------------------------------------------------------------------*/
#include "device.h"
#include "gimbal_pitch.h"
#include "bmi.h"
#include "fric.h"
/* Exported functions --------------------------------------------------------*/
/** @brief 初始化只读探测和控制状态，不初始化旧模板电机输出。 */
void DEVICE_Init(void)
{
    Gimbal_Pitch_Init();
    Fric_Init();
#if PITCH_USE_IMU
    gim_trans.arz = PITCH_IMU_ARZ_DEG;
    gim_trans.ary = PITCH_IMU_ARY_DEG;
    gim_trans.arx = PITCH_IMU_ARX_DEG;
    imu_sensor.init(&imu_sensor);
#endif
}
