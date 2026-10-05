/**
 * @file    board_pitch_protocol.h
 * @brief   两板Pitch机械控制的单帧、版本和CRC协议；与旧D1/D2报文不同。
 */
#ifndef BOARD_PITCH_PROTOCOL_H
#define BOARD_PITCH_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/
#define BOARD_PITCH_COMMAND_ID 0x0D0U /**< 下板到上板命令，CAN2标准8字节。 */
#define BOARD_PITCH_STATUS_ID 0x020U /**< 上板到下板状态摘要，CAN2标准8字节。 */
#define BOARD_PITCH_MAX_RATE_DPS 20.0f /**< 本阶段上下输入最大目标角变化速率，度/秒。 */
#define BOARD_PITCH_RC_ONLINE 1U /**< 遥控数据有效。 */
#define BOARD_PITCH_ACTIVE 2U /**< 下板已进入机械模式并完成拨杆启动。 */
#define BOARD_PITCH_KEY 4U /**< 当前来源为键鼠，否则遥控。 */
#define BOARD_PITCH_CTRL 8U /**< 键鼠Ctrl，撤销待追赶目标并捕获当前位置。 */
#define BOARD_PITCH_NEUTRAL 16U /**< 启动时四通道回中、键鼠释放。 */

/* Exported typedef ----------------------------------------------------------*/
typedef struct {
    uint8_t flags; /**< 上述许可/模式位，未定义位必须为0。 */
    uint8_t sequence; /**< 发送序号，允许255到0回绕。 */
    float rate_dps; /**< 目标俯仰角变化速率，度/秒，抬头为正。 */
} board_pitch_command_t;
typedef struct {
    uint8_t state; /**< 上板控制状态枚举。 */
    uint8_t blocks; /**< 上板当前阻塞位。 */
    float angle_deg; /**< 上板相对机械角，度。 */
    uint8_t motor_state; /**< 电机协议状态高半字节，0失能/1使能。 */
} board_pitch_status_t;

/* Exported functions --------------------------------------------------------*/
/** @brief 编码版本化控制帧，速率用有符号百分之一度/秒及CRC8。 */
int Board_Pitch_EncodeCommand(const board_pitch_command_t *command, uint8_t out[8]);
/** @brief 校验长度、版本、CRC及速率，解码控制帧。 */
int Board_Pitch_DecodeCommand(const uint8_t *data, unsigned length, board_pitch_command_t *command);
/** @brief 编码有符号百分之一度的机械角状态摘要。 */
int Board_Pitch_EncodeStatus(const board_pitch_status_t *status, uint8_t out[8]);
/** @brief 校验帧格式并解码状态摘要。 */
int Board_Pitch_DecodeStatus(const uint8_t *data, unsigned length, board_pitch_status_t *status);
#endif
