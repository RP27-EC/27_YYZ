/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   调度底盘、Yaw角度串级、CAN2 Pitch板间输入和IMU更新。
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "control_task.h"
#include "rc_sensor.h"
#include "motor.h"
#include "carctrl.h"
#include "chassis.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "drv_status.h"
#include "chassis_board.h"
#include "yaw_probe.h"
#include "gimbal.h"
#include "drv_pitch_link.h"

/* Private variables ---------------------------------------------------------*/
static uint32_t previous_ms; /**< 最近一次底盘控制时刻，毫秒。 */

/* Exported variables --------------------------------------------------------*/
volatile uint32_t imu_task_count = 0;   /**< 控制任务累计循环数。 */
volatile uint32_t imu_update_count = 0; /**< IMU累计成功更新次数。 */
volatile float imu_gyro_x = 0.0f;       /**< IMU原始X轴角速度，沿用传感器标度。 */
volatile float imu_gyro_y = 0.0f;       /**< IMU原始Y轴角速度，沿用传感器标度。 */
volatile float imu_gyro_z = 0.0f;       /**< IMU原始Z轴角速度，沿用传感器标度。 */

/* Private functions ---------------------------------------------------------*/
/** @brief 每约2ms复制反馈、控制四轮并在Yaw发送空档发送四轮电流。 */
static void Control_ChassisUpdate(void)
{
    uint32_t now = HAL_GetTick();
    if (now - previous_ms < CHASSIS_CONTROL_MS || !mec_io.init_ok) { return; }
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    rc_sensor.update(&rc_sensor);
    Motor_Update();
    now = HAL_GetTick();
    Yaw_Probe_Update(now);
    __set_PRIMASK(saved);
    uint32_t elapsed = now - previous_ms;
    previous_ms = now;
    car.work(&car, now, elapsed, Drv_CAN_Ready());
    chassis.work(&chassis);
    Drv_UART_Poll();
    Pitch_Link_Update(now);
    if (!Drv_CAN_PollYaw(now)) { CAN_Send(); }
}


/* Exported functions --------------------------------------------------------*/
/** @brief 初始化设备和默认控制许可，底盘/Yaw均等待拨杆启动手势。 */
void Control_Init(void)
{
    rc_sensor.init(&rc_sensor);
    Motor_Init();
    Car_Init();
    Chassis_Init();
    Yaw_Probe_Init();
    Gimbal_Init();
    Chassis_Board_ClockInit();
    Drv_UART_Init();
    Drv_CAN_Init();
    Pitch_Link_Init();
    previous_ms = HAL_GetTick();
    mec_io.init_ok = 1;
}

/** @brief 运行底盘周期控制及IMU更新，并通过延时让出CPU。 */
void StartControlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        imu_task_count++;
        Control_ChassisUpdate();

        if (imu_sensor.work_state.err_code == IMU_NONE_ERR ||
            imu_sensor.work_state.err_code == IMU_DATA_CALI)
        {
            imu_sensor.update(&imu_sensor);

            imu_gyro_x = imu_sensor.info->raw_info.gyro_x;
            imu_gyro_y = imu_sensor.info->raw_info.gyro_y;
            imu_gyro_z = imu_sensor.info->raw_info.gyro_z;

            imu_update_count++;
        }

        osDelay(1);
    }
}
