/**
 * @file    chassis_config.h
 * @brief   麦轮底盘许可、输入分配、轮速/电流及遥控超时配置。
 */
#ifndef DOWN_CHASSIS_CONFIG_H
#define DOWN_CHASSIS_CONFIG_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported typedef ----------------------------------------------------------*/
typedef enum { CHAS_LF, CHAS_LB, CHAS_RF, CHAS_RB, CHAS_MOTOR_COUNT } chassis_motor_index_t;
enum { RC_SW_UP = 1, RC_SW_DOWN = 2, RC_SW_MID = 3 };
/* Exported macro ------------------------------------------------------------*/
#define CHASSIS_BOOT_OUTPUT_ENABLE 1U       /**< 上电默认许可；遥控离线仍撤销整车输出。 */
#define CAR_MODE_WHEEL_TRIGGER 600         /**< 双下档切换机械/陀螺仪的拨轮正端阈值，DBUS原始值。 */
#define CAR_MODE_WHEEL_CENTER 10           /**< 模式切换拨轮回中允许的绝对原始值。 */
#ifndef CHASSIS_RC_ROTATION_ENABLE
#define CHASSIS_RC_ROTATION_ENABLE 0U       /**< 当前ch0分配Yaw；1仅保留旧右杆底盘旋转入口。 */
#endif
#ifndef CHASSIS_MAX_SPEED
#define CHASSIS_MAX_SPEED 8000.0f           /**< 四轮目标转子转速上限，RPM。 */
#endif
#ifndef CHASSIS_MAX_CURRENT
#define CHASSIS_MAX_CURRENT 8000.0f         /**< 四轮电调电流指令上限，原始CAN值，不是安培。 */
#endif
#define CHASSIS_KEY_SPEED 1000.0f           /**< WASD目标轮速限幅，电机转子RPM。 */
#define CHASSIS_OFFLINE_MS 300U             /**< 遥控和四轮最后合法反馈超时，毫秒。 */
#define CHASSIS_CONTROL_MS 2U               /**< 控制周期，毫秒。 */
#define CHASSIS_MAX_PERIOD_MS 20U           /**< 控制周期异常保护上限，毫秒。 */
#define RC_VALID_FRAME_COUNT 5U             /**< 失效后恢复所需连续合法DBUS帧数。 */
#define RC_DEADBAND 20                      /**< 遥控摇杆连续死区，原始通道值。 */
#define RC_ARM_NEUTRAL 30                   /**< 启动时允许的摇杆偏离中位量，原始值。 */
#endif
