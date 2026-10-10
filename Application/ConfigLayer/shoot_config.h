/** @file shoot_config.h
 * @brief 英雄国赛拨弹参数，角度为8192刻度每转的电机转子累计编码器值。
 */
#ifndef DOWN_SHOOT_CONFIG_H
#define DOWN_SHOOT_CONFIG_H
/* Exported macro ------------------------------------------------------------*/
#define SHOOT_DIAL_FEEDBACK_ID 0x205U /**< 下板CAN2拨弹反馈。 */
#define SHOOT_DIAL_CURRENT_ID 0x1FFU /**< 下板CAN2拨弹电流组，使用第0槽。 */
#define SHOOT_DIAL_PERIOD_MS 2U /**< 拨弹电流最短发送间隔，毫秒。 */
#define SHOOT_FIRE_INTERVAL_MS 600U /**< 国赛300个2ms周期的最短发射请求间隔。 */
#define SHOOT_INIT_SPEED_RPM (-800.0f) /**< 拨盘找限位转子RPM。 */
#define SHOOT_INIT_OFFSET 15000LL /**< 找限位后目标偏移，编码器刻度。 */
#define SHOOT_STEP_COUNTS 31481LL /**< 一发拨弹目标增量，编码器刻度。 */
#define SHOOT_INIT_TIMEOUT_MS 2000U /**< 找限位最长时间，毫秒。 */
#define SHOOT_RELOAD_TIMEOUT_MS 300U /**< 单发及回退动作最长时间，毫秒。 */
#define SHOOT_SETTLE_COUNTS 3000LL /**< 拨弹及回退到位误差，编码器刻度。 */
#define SHOOT_STUCK_SPEED_RPM 30 /**< 堵转转子速度上限，RPM。 */
#define SHOOT_STUCK_CURRENT 6000 /**< 堵转反馈电流阈值，电调原始值。 */
#define SHOOT_STUCK_MS 200U /**< 新鲜反馈持续堵转确认时间，毫秒。 */
#define SHOOT_FEEDBACK_MS 300U /**< 拨弹和摩擦轮反馈时效，毫秒。 */
#define SHOOT_SPEED_KP 15.0f /**< 拨弹速度环P，原始电流/RPM。 */
#define SHOOT_POSITION_KP 0.15f /**< 拨弹位置环P，RPM/编码器刻度。 */
#define SHOOT_MAX_SPEED_RPM 3000.0f /**< 拨弹位置环输出限幅，转子RPM。 */
#define SHOOT_INIT_MAX_CURRENT 12000.0f /**< 找限位速度环电流指令限幅，原始值。 */
#define SHOOT_MAX_CURRENT 14000.0f /**< 拨弹位置控制电流指令限幅，原始值。 */
#endif
