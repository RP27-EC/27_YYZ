/**
 * @file    gimbal_config.h
 * @brief   Yaw遥控/键鼠角度串级控制与独立调试的许可、输入及参数。
 */
#ifndef DOWN_GIMBAL_CONFIG_H
#define DOWN_GIMBAL_CONFIG_H

/* Exported macro ------------------------------------------------------------*/
#define GIMBAL_YAW_REMOTE_BOOT_ENABLE 1U            /**< 上电启用遥控/键鼠Yaw，需右拨杆停止档到中档启动。 */
#define GIMBAL_YAW_BOOT_OUTPUT_ENABLE 1U            /**< Yaw局部许可；仍须总使能和遥控在线。 */
#define GIMBAL_YAW_BOOT_PROBE_ENABLE 1U             /**< 上电自动查询Yaw反馈，不需要控制台开启。 */
#define GIMBAL_YAW_OUTPUT_DIRECTION 1               /**< 已实测正电流使编码器角增大，俯视逆时针。 */
#define GIMBAL_YAW_CONTROL_ANGLE_ENABLE 1U          /**< 遥控入口默认角度串级；改0重新烧录可恢复直接速度控制。 */
#define GIMBAL_YAW_CONTROL_ANGLE_KP 16.0f           /**< 机械遥控大误差角度Kp，1/秒，无I/D。 */
#define GIMBAL_YAW_CONTROL_ANGLE_SOFT_ENABLE 1U     /**< 机械遥控角度环启用小误差低增益，0恢复固定Kp。 */
#define GIMBAL_YAW_CONTROL_ANGLE_KP_NEAR 4.0f       /**< 小误差角度Kp，1/秒。 */
#define GIMBAL_YAW_CONTROL_ANGLE_NEAR_DEG 2.0f      /**< 小误差区上界，绝对角误差，度。 */
#define GIMBAL_YAW_CONTROL_ANGLE_FAR_DEG 6.0f       /**< 恢复大误差Kp的绝对角误差，度；中间线性过渡。 */
#define GIMBAL_YAW_CONTROL_ANGLE_RATE_DPS 100.0f    /**< 角度模式满杆及鼠标改变目标角的最大速率，度/秒。 */
#define GIMBAL_YAW_CONTROL_SPEED_KP 40.0f           /**< 遥控/键鼠速度环Kp，原始电流值/(度/秒)。 */
#define GIMBAL_YAW_CONTROL_SPEED_KI 2.0f            /**< 遥控/键鼠速度环Ki，原始电流值/度，保留已测试值。 */
#define GIMBAL_YAW_CONTROL_MAX_SPEED_DPS 1200.0f    /**< 角度外环输出或直接速度模式的目标速度限幅，度/秒。 */
#define GIMBAL_YAW_RC_DIRECTION (-1)                /**< 右杆右拨正ch0对应顺时针，角速度取负。 */
#define GIMBAL_YAW_MOUSE_DIRECTION (-1)             /**< 鼠标右移正mouse_vx对应顺时针，角速度取负。 */
#define GIMBAL_YAW_MOUSE_SPEED_GAIN 1.0f            /**< DBUS鼠标X原始值到目标角速度，度/秒/原始值。 */
#define GIMBAL_YAW_TORQUE_COMMAND 0xA1U             /**< KT电流闭环命令，电机内部电流环。 */
#define GIMBAL_YAW_TX_MS 6U                         /**< 运动帧入队最短间隔，毫秒。 */
#define GIMBAL_YAW_FEEDBACK_MS 30U                  /**< 反馈陈旧诊断和超速证据有效期，毫秒；陈旧本身不停控。 */
#define GIMBAL_YAW_OVERSPEED_MS 3000U               /**< 新鲜反馈连续超速失能时间，毫秒。 */
#define GIMBAL_YAW_ERROR_MS 500U                    /**< 状态1错误位允许的最大反馈间隔，毫秒。 */
#define GIMBAL_YAW_STATUS_DIVIDER 20U               /**< 点动/闭环每20次入队插入一次状态1查询。 */
#define GIMBAL_YAW_ZEROING_MS 300U                  /**< 清零发送未确认的重试告警周期，毫秒；不锁存失能。 */
#define GIMBAL_YAW_REMOTE_WAIT_MS 1000U             /**< 仅遥控失效时，启动请求最多等待的毫秒数。 */
#define GIMBAL_YAW_REMOTE_STABLE_MS 100U            /**< 等待后全部启动条件必须连续正常的毫秒数。 */
#define GIMBAL_YAW_STOP_WARN_MS 1000U               /**< 停止未确认警示阈值；仍继续请求零输出。 */
#define GIMBAL_YAW_PULSE_MS 500U                    /**< Watch点动时长默认值，毫秒。 */
#define GIMBAL_YAW_PULSE_MIN_MS 20U                 /**< 点动时长允许的最小值，毫秒。 */
#define GIMBAL_YAW_PULSE_MAX_MS 1000U               /**< 点动时长允许的最大值，毫秒。 */
#define GIMBAL_YAW_SESSION_MS 3000U                 /**< 请求3角度闭环测试持续时间，毫秒。 */
#define GIMBAL_YAW_PULSE_RAW 200                    /**< Watch点动幅度默认值，正原始值，不是安培。 */
#define GIMBAL_YAW_COMMAND_LIMIT 500                /**< 点动及协议帧绝对上限，原始电流指令。 */
#define GIMBAL_YAW_CURRENT_LIMIT 500                /**< 遥控/键鼠及请求3/4闭环输出上限，原始电流指令。 */
#define GIMBAL_YAW_ZERO_CURRENT 10                  /**< 零输出确认允许的反馈电流原始绝对值。 */
#define GIMBAL_YAW_ZERO_SPEED 2                     /**< 零输出确认允许的协议速度绝对值，度/秒。 */
#define GIMBAL_YAW_ZERO_CONFIRM_COUNT 3U            /**< 退出至少确认发送成功的零电流帧数。 */
#define GIMBAL_YAW_PULSE_TRAVEL_DEG 30.0f           /**< 方向点动相对起点最大允许位移，度。 */
#define GIMBAL_YAW_PULSE_SPEED_DPS 120.0f           /**< 点动速度保护阈值，度/秒。 */
#define GIMBAL_YAW_STEP_DEG 5.0f                    /**< 单次角度目标相对起点最大改变量，度。 */
#define GIMBAL_YAW_TRAVEL_DEG 8.0f                  /**< 首轮闭环相对起点允许的最大实际位移，度。 */
#define GIMBAL_YAW_MAX_SPEED_DPS 10.0f              /**< 外环目标角速度上限，度/秒。 */
#define GIMBAL_YAW_SPEED_GUARD_DPS 60.0f            /**< 角度模式速度保护阈值及请求4保护下限，度/秒。 */
#define GIMBAL_YAW_TEST_SPEED_MARGIN_DPS 30.0f      /**< 请求4允许反馈速度超出目标绝对值的裕量，度/秒。 */
#define GIMBAL_YAW_TARGET_RATE_DPS 10.0f            /**< 目标角度斜坡速度，度/秒。 */
#define GIMBAL_YAW_ANGLE_KP 2.0f                    /**< 角度误差到目标角速度的比例，1/秒。 */
#define GIMBAL_YAW_SPEED_KP 20.0f                   /**< 速度误差到电流指令的比例，原始值/(度/秒)。 */
#define GIMBAL_YAW_SPEED_KI 60.0f                   /**< 速度误差积分到电流指令的系数，原始值/度。 */
#define GIMBAL_YAW_TEST_SPEED_MS 0U                 /**< 请求4时长初值，毫秒；0持续到撤销许可或条件失效。 */
#define GIMBAL_YAW_SPEED_MAX_MS 600000U             /**< 请求4有限时长允许的最大值，毫秒。 */
#define GIMBAL_YAW_TEST_SPEED_KP 20.0f              /**< 请求4的Watch Kp初值，原始值/(度/秒)。 */
#define GIMBAL_YAW_TEST_SPEED_KI 0.0f               /**< 请求4的Watch Ki初值，先关闭积分，原始值/度。 */
#define GIMBAL_YAW_TEST_KP_MAX 50.0f                /**< 请求4允许的Kp上限，原始值/(度/秒)。 */
#define GIMBAL_YAW_TEST_KI_MAX 100.0f               /**< 请求4允许的Ki上限，原始值/度。 */
#define GIMBAL_YAW_INTEGRAL_LIMIT 120.0f            /**< 速度环积分输出绝对上限，原始值。 */
#define GIMBAL_YAW_SPEED_FILTER 0.25f               /**< 编码器差分速度一阶滤波系数，0..1。 */
#define GIMBAL_YAW_SETTLE_DEG 0.3f                  /**< 目标到位允许的角度误差，度。 */
#define GIMBAL_YAW_SETTLE_SPEED_DPS 1.0f            /**< 到位时协议及估计速度绝对上限，度/秒。 */
#define GIMBAL_YAW_SETTLE_MS 100U                   /**< 判读到位要求的最短连续稳定时间，毫秒。 */
#endif
