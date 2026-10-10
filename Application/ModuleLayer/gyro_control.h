/**
 * @file    gyro_control.h
 * @brief   上板IMU租约、方向变换及云台坐标下的底盘解算接口。
 */
#ifndef DOWN_GYRO_CONTROL_H
#define DOWN_GYRO_CONTROL_H
/* Includes ------------------------------------------------------------------*/
#include "board_pitch_protocol.h"
#include "gyro_config.h"
/* Exported typedef ----------------------------------------------------------*/
typedef struct {
    float yaw_deg, continuous_deg, speed_dps;           /**< 方向变换后的IMU角/连续角/角速度，度、度/秒。 */
    float mechanical_deg;                               /**< 编码器相对车头角，逆时针正，度。 */
    uint32_t last_ms, frames, generation;               /**< 新IMU时刻、计数和连续参考代次。 */
    int32_t angle_direction, speed_direction;           /**< 当前连续参考锁存的方向。 */
    uint8_t sequence, seen, valid, mechanical_ready;    /**< 序号与IMU/机械角有效标志。 */
    uint8_t fault;                                      /**< Yaw控制退出锁存，撤销总使能或遥控离线后清除。 */
    uint8_t turning;                                    /**< 正在执行180度转向，暂时取消底盘跟随。 */
} gyro_control_t;
typedef struct {
    uint32_t received, last_ms, not_ready, nonfinite, bad_direction, repeated;              /**< 解码后到达数、时刻毫秒及各接收拒绝计数。 */
    uint32_t max_gap_ms;                                                                    /**< 连续有效新序号IMU样本的最大接收间隔，毫秒。 */
} gyro_rx_diag_t;
/* Exported variables --------------------------------------------------------*/
extern volatile float gyro_scope_yaw_deg, gyro_scope_yaw_dps, gyro_scope_continuous_deg;    /**< 独立IMU角/速度/连续角镜像，度、度/秒。 */
extern volatile float gyro_scope_mechanical_deg;                                            /**< 相对底盘的编码器Yaw角镜像，度。 */
extern volatile uint32_t gyro_scope_imu_ready;                                              /**< IMU新序号租约当前有效标志。 */
extern gyro_control_t gyro_control;                                                         /**< 下板Watch观察对象。 */
extern gyro_rx_diag_t gyro_rx_diag;                                                         /**< IMU接收判断诊断，不参与控制。 */
extern volatile int32_t gyro_yaw_angle_direction;                                           /**< 上板Yaw角方向，±1；停止时调整。 */
extern volatile int32_t gyro_yaw_speed_direction;                                           /**< 上板Yaw速度方向，±1；停止时调整。 */
/* Exported functions --------------------------------------------------------*/
/** @brief 清除姿态租约及连续参考，恢复配置默认方向。 */
void Gyro_Init(void);
/** @brief 撤销旧会话参考和故障，重新等待有效IMU，保留参数及诊断。 */
void Gyro_StandbyReset(void);
/** @brief 发布机械角有效状态与独立IMU J-Scope镜像，不改变控制目标。 */
void Gyro_Update(uint32_t now, float mechanical_deg, int mechanical_ready);
/** @brief 接收有效新IMU序号；异常或重复样本保留旧数据，不刷新租约。 */
void Gyro_ReceiveImu(const board_imu_status_t *sample, uint32_t now);
/** @brief 检查校准、采样新鲜度及方向快照。 */
int Gyro_Ready(uint32_t now);
/** @brief 将云台坐标平移目标旋转至底盘坐标，输入输出单位RPM。 */
void Gyro_Transform(float front, float right, float relative_deg, float *out_front, float *out_right);
/** @brief 跟随曲线，前后对称选择最小车体转角，输出轮速混合RPM。 */
float Gyro_Follow(float relative_deg);
#endif
