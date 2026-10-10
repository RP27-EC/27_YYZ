/**
 * @file    pitch_config.h
 * @brief   机械/陀螺仪Pitch参数、诊断年龄、持续回读及危险越界配置。
 */
#ifndef UP_PITCH_CONFIG_H
#define UP_PITCH_CONFIG_H

/* Exported macro ------------------------------------------------------------*/
#define PITCH_BOOT_ENABLE 1U                        /**< 默认控制许可；需可编码的有效标度和下板明确输出许可。 */
#define PITCH_LOOP_MODE_DEFAULT 1U                  /**< 调试默认模式：0速度单环，1角度外环加速度内环；上电赋给模式变量。 */
#define PITCH_USE_IMU 1U                            /**< 内环使用IMU角速度，重力补偿使用IMU俯仰角；无效时不回退电机速度。 */
#define PITCH_IMU_ARZ_DEG 180.0f                    /**< IMU安装坐标Z旋转，度。 */
#define PITCH_IMU_ARY_DEG 180.0f                    /**< IMU安装坐标Y旋转，度。 */
#define PITCH_IMU_ARX_DEG 0.0f                      /**< IMU安装坐标X旋转，度。 */
#define PITCH_IMU_DIRECTION 1.0f                    /**< 变换后的IMU角度/角速度方向，首次停止档手动核对抬头为正。 */
#define PITCH_IMU_TIMEOUT_MS 500U                   /**< 最近有效IMU采样期限，毫秒；异常样本丢弃，短暂缺失保留上次有效值。 */
#define PITCH_MOTOR_ID 0x001U                       /**< 上板CAN2 Pitch控制ID。 */
#define PITCH_FEEDBACK_ID 0x011U                    /**< Pitch反馈ID，不推断具体电机型号。 */
#define PITCH_ZERO_RAD 0.909712553f                 /**< 机械零点，电机原生弧度，待实物核对。 */
#define PITCH_FEEDBACK_DIRECTION (-1.0f)            /**< 抬头为正，编码器/速度取反。 */
#define PITCH_OUTPUT_DIRECTION (-1.0f)              /**< 抬头正控制量对应负MIT力矩。 */
#define PITCH_MIN_RAD (-0.21f)                      /**< 机械低端限位，抬头为正的相对角，弧度。 */
#define PITCH_MAX_RAD 0.81f                         /**< 机械高端限位，相对角，弧度。 */
#define PITCH_FLAT_ANGLE_DEG 8.27833271f            /**< 用户实测平射角，沿用当前actual_deg坐标，度。 */
#define PITCH_FLAT_RATE_DPS 5.0f                    /**< 中档回平射时目标角变化速率上限，度/秒，非实际速度保证。 */
#define PITCH_LIMIT_MARGIN_DEG 2.0f                 /**< 实际反馈越界退出裕量，度；目标仍严格限制在范围。 */
#define PITCH_LIMIT_HOLD_MS 3000U                   /**< 有效机械反馈持续越界达到此时间才停机，毫秒。 */
#define PITCH_ANGLE_KP 9.0f                         /**< 机械角度P，目标速度/角度误差，1/秒。 */
#ifndef PITCH_ANGLE_KI
#define PITCH_ANGLE_KI 0.045f                       /**< 角度I，乘每周期累加的角误差，不乘dt。 */
#endif
#define PITCH_ANGLE_INTEGRAL_MAX 800.0f             /**< 角度误差累计限幅，度乘采样次数；I输出最大36度/秒。 */
#define PITCH_ANGLE_INTEGRAL_SEP_DEG 10.0f          /**< 角度误差超过此值清积分，度；0表示不分离。 */
#define PITCH_GRAVITY_ENABLE 1U                     /**< 在实际控制阶段启用分段重力补偿，两种模式共用。 */
#define PITCH_GRAVITY_SCALE 1.0f                    /**< 经验补偿幅值倍率，0关闭，负值翻转补偿方向。 */
#define PITCH_GRAVITY_ANGLE_OFFSET_DEG 0.0f         /**< 补偿使用IMU俯仰角加此偏移，度。 */
#define PITCH_GRAVITY_LOW_TORQUE 0.5f               /**< 水平角不高于-4度时的原生MIT补偿力矩。 */
#define PITCH_GRAVITY_HIGH_TORQUE (-1.0f)           /**< 水平角不低于47度时的原生MIT补偿力矩。 */
#define PITCH_SPEED_KP 0.1f                         /**< 机械速度P，协议力矩单位/(度/秒)。 */
#define PITCH_SPEED_KI 0.0f                         /**< 速度I，乘每周期累加的速度误差，默认关闭。 */
#define PITCH_SPEED_INTEGRAL_MAX 8.0f               /**< 速度误差累计限幅，度/秒乘采样次数。 */
#define PITCH_SPEED_OUTPUT_LIMIT 5.0f               /**< 速度PI输出限幅，补偿相加前的MIT力矩幅值。 */
#define PITCH_TORQUE_LIMIT 10.0f                    /**< 驱动最终MIT力矩编码限幅；速度PI先限5，再加补偿。 */
#define PITCH_MAX_SPEED_DPS 100.0f                  /**< 机械角度外环速度输出限幅，度/秒。 */
#define PITCH_SPEED_GUARD_ENABLE 0U                 /**< 用户要求暂时关闭速度阈值退出；改为1恢复，上电赋给运行开关。 */
#define PITCH_SPEED_GUARD_DPS 60.0f                 /**< 选定反馈速度退出阈值，度/秒；仅速度保护开关非零时生效。 */
#define PITCH_BOARD_TIMEOUT_MS 500U                 /**< 最近合法下板命令年龄诊断阈值，毫秒；不失能。 */
#define PITCH_FEEDBACK_TIMEOUT_MS 500U              /**< 电机反馈年龄诊断阈值，毫秒；参数回复不续期，不失能。 */
#define PITCH_CAN_RESTART_FAILURES 3U               /**< CAN恢复请求的最少连续失败次数，仲裁失败只重试帧。 */
#define PITCH_CAN_RESTART_DELAY_MS 50U              /**< 连续发送错误后启动CAN恢复的最短持续时间，毫秒。 */
#define PITCH_CAN_RECOVERY_STAGE_MS 50U             /**< CAN取消、进入初始化和总线同步各阶段的等待上限，毫秒。 */
#define PITCH_CAN_RECOVERY_RETRY_MS 200U            /**< 两次CAN硬件恢复请求的最小间隔，毫秒。 */
#define PITCH_MAX_PERIOD_MS 50U                     /**< 周期异常诊断阈值，毫秒；异常周期用2ms计算目标步长。 */
#define PITCH_ENABLE_TIMEOUT_MS 1500U               /**< 零输出/使能握手超时诊断阈值，毫秒；继续重试。 */
#define PITCH_QUERY_MS 20U                          /**< 只读参数/状态探测最短间隔，毫秒。 */
#define PITCH_COMMAND_MS 2U                         /**< 正常MIT力矩发送最短间隔，毫秒。 */
#define PITCH_SPECIAL_MS 20U                        /**< 使能/失能特殊帧发送间隔，毫秒。 */
#define PITCH_STATUS_MS 20U                         /**< 向下板回传摘要的间隔，毫秒。 */
#define PITCH_STOP_WARN_MS 1000U                    /**< 停止尚未收到失能反馈的提示阈值，毫秒。 */
#define PITCH_GYRO_MIN_MECH_DEG (-6.42136955f)      /**< 用户实测俯角-7.421度向内留1度，机械角软下限。 */
#define PITCH_GYRO_MAX_MECH_DEG 45.9892426f         /**< 用户实测仰角46.989度向内留1度，机械角软上限。 */
#define PITCH_GYRO_ANGLE_KP 12.0f                   /**< 陀螺仪角度P，1/秒。 */
#define PITCH_GYRO_ANGLE_KI 0.08f                   /**< 陀螺仪角度I，乘每周期误差累计。 */
#define PITCH_GYRO_ANGLE_SUM_MAX 200.0f             /**< 陀螺仪角误差累计限幅，度乘次数。 */
#define PITCH_GYRO_ANGLE_SEP_DEG 5.0f               /**< 陀螺仪积分分离阈值，度。 */
#define PITCH_GYRO_MAX_SPEED_DPS 400.0f             /**< 陀螺仪位置环输出限幅，度/秒。 */
#define PITCH_GYRO_SPEED_KP 0.14f                   /**< 陀螺仪速度P，MIT力矩单位/(度/秒)。 */
#define PITCH_GYRO_SPEED_KI 0.0f                    /**< 陀螺仪速度I，初值0。 */
#define PITCH_GYRO_SPEED_SUM_MAX 0.0f               /**< 陀螺仪速度误差累计限幅，初值0。 */
#define PITCH_IMU_STATUS_MS 10U                     /**< 上板向下板发送新IMU摘要的间隔，毫秒。 */
#define PITCH_RAD_TO_DEG 57.295779513f              /**< 弧度转度系数。 */
#endif
