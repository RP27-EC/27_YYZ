/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   读取imu信息
  ******************************************************************************
  */
#include "control_task.h"
#include "rc_sensor.h"
#include "motor.h"
#include "carctrl.h"
#include "chassis.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "drv_status.h"
#include "chassis_board.h"

static uint32_t previous_ms;
void Control_Init(void)
{
    rc_sensor.init(&rc_sensor);
    Motor_Init();
    Car_Init();
    Chassis_Init();
    Chassis_Board_ClockInit();
    Drv_UART_Init();
    Drv_CAN_Init();
    previous_ms = HAL_GetTick();
    mec_io.init_ok = 1;
}

static void Control_ChassisUpdate(void)
{
    uint32_t now = HAL_GetTick();
    if (now - previous_ms < CHASSIS_CONTROL_MS || !mec_io.init_ok) { return; }
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    rc_sensor.update(&rc_sensor);
    Motor_Update();
    now = HAL_GetTick();
    __set_PRIMASK(saved);
    uint32_t elapsed = now - previous_ms;
    previous_ms = now;
    car.work(&car, now, elapsed, Drv_CAN_Ready());
    chassis.work(&chassis);
    Drv_UART_Poll();
    CAN_Send();
}


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
        Control_ChassisUpdate();

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
