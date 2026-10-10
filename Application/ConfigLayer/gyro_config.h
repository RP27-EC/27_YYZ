/**
 * @file    gyro_config.h
 * @brief   陀螺仪输入、Yaw串级及底盘跟随参数；机械模式参数独立保留。
 */
#ifndef DOWN_GYRO_CONFIG_H
#define DOWN_GYRO_CONFIG_H
/* Exported macro ------------------------------------------------------------*/
#define CAR_BOOT_MODE 2U                /**< 1机械模式，2陀螺仪模式，上电默认。 */
#define GYRO_ARM_MS 1500U               /**< 四杆/键鼠回中且反馈正常的连续启动等待，毫秒。 */
#define GYRO_IMU_TIMEOUT_MS 500U        /**< 本轮排查的上板有效新IMU超时，毫秒；丢弃样本不续期。 */
#define GYRO_YAW_ANGLE_DIRECTION 1      /**< 上板Yaw角方向，±1；现场确认逆时针为正。 */
#define GYRO_YAW_SPEED_DIRECTION 1      /**< 上板Yaw角速度方向，±1；独立现场确认。 */
#define GYRO_RC_YAW_GAIN (-0.2f)        /**< 2ms周期换算ch0到目标角速率，度/秒/通道原始值。 */
#define GYRO_MOUSE_FILTER_SAMPLES 10U   /**< 鼠标X/Y均值窗口，合法DBUS帧数。 */
#define GYRO_MOUSE_YAW_GAIN (-2.5f)     /**< 2ms周期换算鼠标X到目标角速率，度/秒/原始值。 */
#define GYRO_RC_PITCH_GAIN 0.05f        /**< 2ms周期换算ch1到俯仰目标角速率，度/秒/原始值。 */
#define GYRO_MOUSE_PITCH_GAIN 1.5f      /**< 2ms周期换算鼠标Y速率倍率，方向沿用本车已验证配置。 */
#define GYRO_YAW_ANGLE_KP 16.0f         /**< Yaw角度P，1/秒。 */
#define GYRO_YAW_ANGLE_KI 0.2f          /**< Yaw角度I，乘每周期误差累计值，不乘dt。 */
#define GYRO_YAW_ANGLE_SUM_MAX 10.0f    /**< Yaw角度误差累计限幅，度乘次数。 */
#define GYRO_YAW_SPEED_KP 30.0f         /**< Yaw速度P，原始电流值/(度/秒)。 */
#define GYRO_YAW_SPEED_KI 0.0f          /**< Yaw速度I，初值0。 */
#define GYRO_YAW_MAX_SPEED_DPS 1200.0f  /**< Yaw外环输出限幅，度/秒；电流上限仍由gimbal_config限制。 */
#define GYRO_TURN_ENABLE 1U             /**< 启用机械/陀螺仪/小陀螺共用的遥控和键鼠换头。 */
#define GYRO_TURN_KEY_MASK (1U << 8)    /**< 换头按键位，默认R；位定义见rc_sensor.h。 */
#define GYRO_TURN_ANGLE_DEG 180.0f      /**< 单次目标角增量，度；正为逆时针，改为-180可反向。 */
#define GYRO_TURN_SETTLE_DEG 10.0f       /**< 结束换头等待的绝对角误差，度。 */
#define GYRO_TURN_MS 800U               /**< 换头等待上限，毫秒；到时恢复手动输入，目标角不清除。 */
#define GYRO_TURN_WHEEL_TRIGGER (-600)  /**< 两拨杆上档时拨轮负端触发阈值，DBUS原始值。 */
#define GYRO_TURN_WHEEL_CENTER 10       /**< 拨轮回中允许的绝对原始值。 */
#define GYRO_FOLLOW_LINEAR 5.0f         /**< 底盘跟随一次项，RPM/编码器刻度。 */
#define GYRO_FOLLOW_AT_90_RPM 30720.0f  /**< 90度跟随曲线幅值，最终四轮仍统一限幅。 */
#define GYRO_KEY_RAMP_MS 800U           /**< 400周期键盘平移由0到满目标的时间，毫秒。 */
#define GYRO_CYCLE_SPEED_RPM 5000.0f    /**< 定速小陀螺旋转混合量，电机转子RPM；最终四轮统一限幅。 */
#endif
