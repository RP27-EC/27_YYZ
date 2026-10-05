/**
 * @file    control_task.c
 * @brief   先采集国赛BMI088姿态/速度，再按2ms调度Pitch内环。
 */
/* Includes ------------------------------------------------------------------*/
#include "control_task.h"
#include "drv_pitch.h"
#include "pitch_config.h"
#include "gimbal_pitch.h"
#include "bmi.h"
#include "BMI088driver.h"
/* Private variables ---------------------------------------------------------*/
static uint32_t imu_previous_ms; /**< Mahony实际采样间隔参考时刻，毫秒。 */
/* Exported variables --------------------------------------------------------*/
extern bmi_t bmi; /**< BMI088 Mahony解算器，由bmi.c定义。 */
volatile uint32_t imu_task_count = 0; /**< 控制任务累计循环数。 */
volatile uint32_t imu_update_count = 0; /**< IMU调用更新次数，校准阶段也累计。 */
volatile float imu_gyro_x, imu_gyro_y, imu_gyro_z; /**< 可选IMU原始角速度镜像。 */
/* Exported functions --------------------------------------------------------*/
/** @brief 非阻塞轮询Pitch任务，并让出CPU给LED和系统任务。 */
void StartControlTask(void const *argument)
{
    (void)argument;
    for (;;) {
        ++imu_task_count;
#if PITCH_USE_IMU
        if (imu_sensor.work_state.err_code == IMU_NONE_ERR || imu_sensor.work_state.err_code == IMU_DATA_CALI) {
            uint32_t now = HAL_GetTick();
            uint32_t interval = imu_previous_ms ? now - imu_previous_ms : 1U;
            imu_previous_ms = now;
            bmi.halfT = (float)(interval > 0U && interval <= 20U ? interval : 1U) * 0.0005f;
            bmi.Kp = now <= 400U ? 1000.0f : 0.125f;
            imu_sensor.update(&imu_sensor);
            imu_gyro_x = imu_sensor.info->raw_info.gyro_x;
            imu_gyro_y = imu_sensor.info->raw_info.gyro_y;
            imu_gyro_z = imu_sensor.info->raw_info.gyro_z;
            ++imu_update_count;
            Gimbal_Pitch_ImuUpdate(imu_sensor.info->base_info.pitch, imu_sensor.info->base_info.rate_pitch, HAL_GetTick(),
                BMI088_read_valid && imu_sensor.info->init_flag && imu_sensor.work_state.cali_end &&
                imu_sensor.work_state.err_code == IMU_NONE_ERR);
        }
        else Gimbal_Pitch_ImuUpdate(0.0f, 0.0f, HAL_GetTick(), 0);
#endif
        Drv_Pitch_Poll(HAL_GetTick());
        osDelay(1);
    }
}
