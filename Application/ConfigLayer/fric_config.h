/** @file fric_config.h
 * @brief 六摩擦轮实际运行参数，速度为电机转子RPM。
 */
#ifndef UP_FRIC_CONFIG_H
#define UP_FRIC_CONFIG_H
/* Exported macro ------------------------------------------------------------*/
#define FRIC_FRONT_RPM 2900.0f              /**< Shoot实例的前级基础转速幅值，RPM。 */
#define FRIC_BACK_RPM 2850.0f               /**< Shoot实例的后级基础转速幅值，RPM。 */
#define FRIC_FRONT_OFFSET_RPM 300.0f        /**< 普通模式前级补偿；上轮加、左轮加、右轮减。 */
#define FRIC_KP 27.0f                       /**< 摩擦轮速度P，原始电流/RPM。 */
#define FRIC_KI_PER_2MS 0.5f                /**< 每2ms积分系数，原始电流/RPM。 */
#define FRIC_INTEGRAL_MAX 6000.0f           /**< 积分输出限幅，原始电流指令。 */
#define FRIC_FRONT_CURRENT_MAX 10000.0f     /**< 前级电流指令限幅，原始值。 */
#define FRIC_BACK_CURRENT_MAX 12000.0f      /**< 后级电流指令限幅，原始值。 */
#define FRIC_READY_MIN_RPM 1300.0f          /**< 六轮按正确方向达到此速度后允许拨弹。 */
#define FRIC_FEEDBACK_MS 300U               /**< 单电机有效反馈期限，毫秒。 */
#define FRIC_CONTROL_MS 2U                  /**< 速度环周期，毫秒。 */
#define FRIC_TX_TIMEOUT_MS 20U              /**< CAN1单帧超时取消，毫秒。 */
#define FRIC_RECOVERY_RETRY_MS 200U         /**< CAN1恢复重试间隔，毫秒。 */
#define FRIC_RECOVERY_STAGE_MS 50U          /**< CAN1初始化阶段最长等待，毫秒。 */
#endif
