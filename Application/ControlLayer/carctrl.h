/**
 * @file carctrl.h
 * @brief 机械/陀螺仪/小陀螺许可、输入来源及遥控复位接口。
 */
#ifndef DOWN_CARCTRL_H
#define DOWN_CARCTRL_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported typedef ----------------------------------------------------------*/
typedef enum { RC_CTRL = 0, KEY_CTRL = 1 } car_ctrl_e;
typedef enum { sleep_car = 0, mec_car = 1, gyro_car = 2, cycle_car = 3 } car_mode_e;
enum {
    CAR_BLOCK_OUTPUT = 1, CAR_BLOCK_REMOTE = 2, CAR_BLOCK_MOTOR = 4,
    CAR_BLOCK_CAN = 8, CAR_BLOCK_TIMING = 16, CAR_BLOCK_IMU = 32, CAR_BLOCK_YAW = 64,
    CAR_BLOCK_STANDBY = 128 /**< 两板撤销输出及待命复位尚未完成。 */
};
typedef struct car_t {
    uint32_t arm_ms, sessions, selected_mode; /**< 兼容计时、启动代次及小陀螺退出后恢复的基础模式。 */
    uint8_t arm_waiting, fault_latched; /**< 兼容旧Watch的字段；当前不作为启动或停控门槛。 */
    uint8_t standby_pending; /**< 遥控离线后等待合法遥控重新在线；不等待两轴ACK。 */
    uint8_t remote_online; /**< 当前遥控合法快照在线许可，不等同于运动已使能。 */
    uint8_t cycle_requested, cycle_wheel_armed; /**< 小陀螺请求及拨轮负端回中记忆，失能时清除。 */
    uint8_t mode_wheel_armed, input_seen; /**< 双下档拨轮正端记忆及当前输入来源已捕获标志。 */
    uint16_t previous_keys; /**< 上周期键盘快照，用于V和Ctrl按下事件。 */
    uint32_t cycle_entries; /**< 小陀螺进入次数，待命复位后保留。 */
    car_ctrl_e car_ctrl; /**< 遥控或键鼠输入来源。 */
    car_mode_e car_mode; /**< 当前允许的控制模式，sleep不输出。 */
    uint32_t block_reason, loops, now_ms, dt_ms; /**< 阻塞位、循环数、时刻与周期，后两者毫秒。 */
    uint32_t diag_reason, raw_dt_ms, timing_errors, invalid_modes; /**< 仅诊断的异常位、实际周期毫秒及异常观察次数。 */
    uint32_t imu_age_ms, yaw_age_ms; /**< 最后有效IMU和Yaw反馈年龄，毫秒；不作为停控门槛。 */
    uint8_t online_mask, off_seen, previous_switch; /**< 四轮在线位及兼容旧Watch的拨杆字段。 */
    void (*work)(struct car_t *, uint32_t now_ms, uint32_t dt_ms, int can_ok);
} car_t;
/* Exported variables --------------------------------------------------------*/
extern car_t car; /**< 整车模式与许可观察对象。 */
extern volatile uint32_t car_mode_select; /**< 基础模式1机械、2陀螺仪；V或双下档拨轮正端回中切换。 */
/* 总使能：底盘及两轴；只有1有效。 */
extern volatile uint32_t mec_output_enable;
/* Exported functions --------------------------------------------------------*/
/** @brief 普通陀螺仪和小陀螺共用世界角控制及平移坐标变换。 */
static inline int Car_IsGyroMode(car_mode_e mode)
{
    return mode == gyro_car || mode == cycle_car;
}
/** @brief 初始化整车模式和许可。 */
void Car_Init(void);
/** @brief 撤销输出并清除遥控会话，诊断计数保留。 */
void Car_StandbyReset(car_t *c, uint32_t now_ms);
/** @brief 更新统一拨杆输入、基础模式、小陀螺及遥控离线许可。 */
void Car_Work(car_t *c, uint32_t now_ms, uint32_t dt_ms, int can_ok);
#endif
