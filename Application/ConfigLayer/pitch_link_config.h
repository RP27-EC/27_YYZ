/**
 * @file    pitch_link_config.h
 * @brief   下板CAN2 Pitch输入映射与板间发送配置。
 */
#ifndef DOWN_PITCH_LINK_CONFIG_H
#define DOWN_PITCH_LINK_CONFIG_H
/* Exported macro ------------------------------------------------------------*/
#define PITCH_LINK_PERIOD_MS 10U /**< 下板命令发送周期，毫秒。 */
#define PITCH_LINK_TIMEOUT_MS 100U /**< 上板摘要在线有效期，毫秒。 */
#define PITCH_LINK_TX_TIMEOUT_MS 20U /**< 待发帧超时取消阈值，毫秒。 */
#define PITCH_LINK_RC_DEADZONE 20 /**< 右杆上下零区，DBUS原始通道值。 */
#define PITCH_LINK_RC_MAX 660.0f /**< DBUS通道满量程。 */
#define PITCH_LINK_MOUSE_GAIN 1.0f /**< 鼠标Y原始值到目标角速率的系数，度/秒/原始单位。 */
#define PITCH_LINK_MOUSE_DIRECTION 1.0f /**< 按本车操作反馈反转鼠标Pitch映射，使上移对应抬头；不改变遥控杆方向。 */
#define PITCH_LINK_RAM_OFFSET 1280U /**< FDCAN2共享Message RAM起点，32位字；避开CAN1。 */
#endif
