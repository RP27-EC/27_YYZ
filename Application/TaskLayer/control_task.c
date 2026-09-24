/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   读取imu信息
  ******************************************************************************
  */
#include "control_task.h"

/* 调试观察变量 */
volatile uint32_t imu_task_count = 0;
volatile uint32_t imu_update_count = 0;

volatile float imu_gyro_x = 0.0f;
volatile float imu_gyro_y = 0.0f;
volatile float imu_gyro_z = 0.0f;

void StartControlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        /* 每进入一次任务循环，加 1 */
        imu_task_count++;

        /* 正常状态和校准状态下，都需要持续更新 IMU */
        if (imu_sensor.work_state.err_code == IMU_NONE_ERR ||
            imu_sensor.work_state.err_code == IMU_DATA_CALI)
        {
            /* 读取传感器并执行模板里的更新处理 */
            imu_sensor.update(&imu_sensor);

            /* 保存三轴角速度，方便在调试窗口查看 */
            imu_gyro_x = imu_sensor.info->raw_info.gyro_x;
            imu_gyro_y = imu_sensor.info->raw_info.gyro_y;
            imu_gyro_z = imu_sensor.info->raw_info.gyro_z;

            imu_update_count++;
        }

        /* 当前任务等待，让其他就绪任务有机会运行 */
        osDelay(1);
    }
}



