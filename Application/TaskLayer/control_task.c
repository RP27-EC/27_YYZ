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
#include "gyro_control.h"
#include "shoot.h"

/* Private variables ---------------------------------------------------------*/
static uint32_t previous_ms; /**< 最近一次底盘控制时刻，毫秒。 */
static uint8_t remote_was_online; /**< 只在遥控在线到离线边沿请求一次控制复位。 */

/* Exported variables --------------------------------------------------------*/
volatile uint32_t imu_task_count = 0;   /**< 控制任务累计循环数。 */
volatile uint32_t imu_update_count = 0; /**< IMU累计成功更新次数。 */
volatile float imu_gyro_x = 0.0f;       /**< IMU原始X轴角速度，沿用传感器标度。 */
volatile float imu_gyro_y = 0.0f;       /**< IMU原始Y轴角速度，沿用传感器标度。 */
volatile float imu_gyro_z = 0.0f;       /**< IMU原始Z轴角速度，沿用传感器标度。 */

/* Private functions ---------------------------------------------------------*/
/** @brief 撤销整车会话并通知两轴待命，外设和反馈采样继续运行。 */
static void Control_RequestStandby(uint32_t now)
{
    Car_StandbyReset(&car, now);
    Gyro_StandbyReset();
    Gimbal_Yaw_RequestStandby(now);
    Pitch_Link_RequestStandby(now);
    Shoot_StandbyReset(now);
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    RC_Sensor_StandbyReset();
    rc_sensor.update(&rc_sensor);
    __set_PRIMASK(saved);
}

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
    Pitch_Link_Receive(now);
    int remote_online = RC_Sensor_Online(now);
    (void)Pitch_Link_TakeBusOff();
    if (!remote_online && remote_was_online) {
        Control_RequestStandby(now);
        remote_online = RC_Sensor_Online(now);
    }
    remote_was_online = (uint8_t)remote_online;
    if (car.standby_pending && remote_online) {
        Pitch_Link_FinishStandby();
        car.standby_pending = 0U;
        car.arm_waiting = 0U;
        car.arm_ms = 0U;
    }
    car.yaw_age_ms = now - yaw_probe.feedback.last_state_ms;
    Gyro_Update(now, yaw_probe.relative_deg, yaw_probe.angle_valid && yaw_probe.continuous_valid &&
        now - yaw_probe.feedback.last_state_ms <= YAW_PROBE_OFFLINE_MS && !yaw_probe.feedback.status.error_state);
    car.work(&car, now, elapsed, Drv_CAN_Ready());
    chassis.work(&chassis);
    Shoot_Update(now);
    Drv_UART_Poll();
    Pitch_Link_Update(now);
    if (!Drv_CAN_PollYaw(now)) { CAN_Send(); }
}


/* Exported functions --------------------------------------------------------*/
/** @brief 初始化设备和默认陀螺仪模式，遥控合法在线后按统一许可运行。 */
void Control_Init(void)
{
    rc_sensor.init(&rc_sensor);
    Motor_Init();
    Car_Init();
    Chassis_Init();
    Yaw_Probe_Init();
    Gimbal_Init();
    Gyro_Init();
    Shoot_Init();
    Chassis_Board_ClockInit();
    Drv_UART_Init();
    Drv_CAN_Init();
    Pitch_Link_Init();
    previous_ms = HAL_GetTick();
    remote_was_online = 0U;
    Control_RequestStandby(previous_ms);
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
