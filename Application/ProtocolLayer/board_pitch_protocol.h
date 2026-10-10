/**
 * @file    board_pitch_protocol.h
 * @brief   两板Pitch机械/陀螺仪控制的单帧、版本和CRC协议；与旧D1/D2报文不同。
 */
#ifndef BOARD_PITCH_PROTOCOL_H
#define BOARD_PITCH_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/
#define BOARD_PITCH_COMMAND_ID 0x0D0U /**< 下板到上板命令，CAN2标准8字节。 */
#define BOARD_PITCH_STATUS_ID 0x020U /**< 上板到下板状态摘要，CAN2标准8字节。 */
#define BOARD_PITCH_MAX_RATE_DPS 300.0f /**< 协议允许的最大目标角变化速率，度/秒。 */
#define BOARD_IMU_STATUS_ID 0x021U /**< 上板Yaw IMU摘要，版本3，带采样序号与CRC。 */
#define BOARD_PITCH_VERSION 3U /**< 待命复位握手版本，两板必须同步升级。 */
#define BOARD_PITCH_RESET 64U /**< 持续请求上板撤销输出、清除会话并回待命。 */
#define BOARD_PITCH_STANDBY_STATE 7U /**< 上板确认失能并完成本次待命复位。 */
#define BOARD_PITCH_GYRO 32U /**< 外环使用IMU绝对俯仰角。 */
#define BOARD_PITCH_RC_ONLINE 1U /**< 遥控数据有效。 */
#define BOARD_PITCH_ACTIVE 2U /**< 下板已完成模式启动。 */
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

typedef struct {
    uint8_t ready, sequence; /**< 校准有效标志与7位采样序号。 */
    float yaw_deg, yaw_dps; /**< 上板安装变换后Yaw，度、度/秒。 */
} board_imu_status_t;

/* Exported functions --------------------------------------------------------*/
/** @brief 编码Yaw IMU摘要，角度百分之一度、速度十分之一度/秒。 */
int Board_Imu_Encode(const board_imu_status_t *sample, uint8_t out[8]);
/** @brief 校验版本、CRC和量程，解码IMU摘要。 */
int Board_Imu_Decode(const uint8_t *data, unsigned length, board_imu_status_t *sample);
/** @brief 编码版本化控制帧，速率用有符号百分之一度/秒及CRC8。 */
int Board_Pitch_EncodeCommand(const board_pitch_command_t *command, uint8_t out[8]);
/** @brief 校验长度、版本、CRC及速率，解码控制帧。 */
int Board_Pitch_DecodeCommand(const uint8_t *data, unsigned length, board_pitch_command_t *command);
/** @brief 编码有符号百分之一度的机械角状态摘要。 */
int Board_Pitch_EncodeStatus(const board_pitch_status_t *status, uint8_t out[8]);
/** @brief 校验帧格式并解码状态摘要。 */
int Board_Pitch_DecodeStatus(const uint8_t *data, unsigned length, board_pitch_status_t *status);
#endif
