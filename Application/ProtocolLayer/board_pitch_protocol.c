/**
 * @file    board_pitch_protocol.c
 * @brief   Pitch两板帧的显式字节编码及CRC8；上下板使用同一实现。
 */

/* Includes ------------------------------------------------------------------*/
#include "board_pitch_protocol.h"
#include <math.h>
#include <stddef.h>

/* Private functions ---------------------------------------------------------*/
/** @brief CRC8初值0xFF、多项式0x31，覆盖前7个字节。 */
static uint8_t crc8(const uint8_t *data)
{
    uint8_t crc = 0xff;
    for (unsigned i = 0; i < 7; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) { crc = crc & 0x80U ? (uint8_t)((crc << 1) ^ 0x31U) : (uint8_t)(crc << 1); }
    }
    return crc;
}
/** @brief 验证单帧魔数、协议版本和CRC，拒绝不完整帧。 */
static int valid(const uint8_t *data, unsigned length)
{
    return data && length == 8U && data[0] == 0x50U && data[1] == BOARD_PITCH_VERSION && data[7] == crc8(data);
}
/** @brief 从大端两个字节读取有符号定点数，不依赖负数强制转换。 */
static int32_t signed16(const uint8_t *data)
{
    uint32_t raw = (uint32_t)data[0] << 8 | data[1];
    return raw & 0x8000U ? (int32_t)raw - 65536 : (int32_t)raw;
}

/* Exported functions --------------------------------------------------------*/
/** @brief 编码单帧Pitch速率与控制许可，检查有限值和定义位。 */
int Board_Pitch_EncodeCommand(const board_pitch_command_t *command, uint8_t out[8])
{
    if (!command || !out || !isfinite(command->rate_dps) || fabsf(command->rate_dps) > BOARD_PITCH_MAX_RATE_DPS || (command->flags & ~127U)) { return 0; }
    if ((command->flags & BOARD_PITCH_RESET) && ((command->flags & (BOARD_PITCH_RC_ONLINE | BOARD_PITCH_ACTIVE)) || command->rate_dps != 0.0f)) return 0;
    uint16_t rate = (uint16_t)(int32_t)lroundf(command->rate_dps * 100.0f);
    out[0] = 0x50; out[1] = BOARD_PITCH_VERSION; out[2] = command->flags; out[3] = command->sequence;
    out[4] = (uint8_t)(rate >> 8); out[5] = (uint8_t)rate; out[6] = 0; out[7] = crc8(out);
    return 1;
}
/** @brief 校验CRC、版本、长度与保留位后解码控制帧。 */
int Board_Pitch_DecodeCommand(const uint8_t *data, unsigned length, board_pitch_command_t *command)
{
    if (!command || !valid(data, length) || (data[2] & ~127U) || data[6]) { return 0; }
    float rate = signed16(data + 4) * 0.01f;
    if (fabsf(rate) > BOARD_PITCH_MAX_RATE_DPS) { return 0; }
    if ((data[2] & BOARD_PITCH_RESET) && ((data[2] & (BOARD_PITCH_RC_ONLINE | BOARD_PITCH_ACTIVE)) || rate != 0.0f)) return 0;
    *command = (board_pitch_command_t){data[2], data[3], rate};
    return 1;
}
/** @brief 编码上板控制状态和百分之一度的机械角。 */
int Board_Pitch_EncodeStatus(const board_pitch_status_t *status, uint8_t out[8])
{
    if (!status || !out || !isfinite(status->angle_deg) || fabsf(status->angle_deg) > 180.0f || status->state > BOARD_PITCH_STANDBY_STATE || status->motor_state > 15U) { return 0; }
    uint16_t angle = (uint16_t)(int32_t)lroundf(status->angle_deg * 100.0f);
    out[0] = 0x50; out[1] = BOARD_PITCH_VERSION; out[2] = status->state; out[3] = status->blocks;
    out[4] = (uint8_t)(angle >> 8); out[5] = (uint8_t)angle; out[6] = status->motor_state; out[7] = crc8(out);
    return 1;
}
/** @brief 校验CRC及枚举范围后解码反馈摘要。 */
int Board_Pitch_DecodeStatus(const uint8_t *data, unsigned length, board_pitch_status_t *status)
{
    if (!status || !valid(data, length)) { return 0; }
    float angle = signed16(data + 4) * 0.01f;
    if (fabsf(angle) > 180.0f || data[6] > 15U || data[2] > BOARD_PITCH_STANDBY_STATE) { return 0; }
    *status = (board_pitch_status_t){data[2], data[3], angle, data[6]};
    return 1;
}

/** @brief 生成带7位序号的IMU姿态帧，速度量程±3276.7度/秒。 */
int Board_Imu_Encode(const board_imu_status_t *s, uint8_t out[8])
{
    if (!s || !out || s->ready > 1U || s->sequence > 127U || !isfinite(s->yaw_deg) ||
        !isfinite(s->yaw_dps) || fabsf(s->yaw_deg) > 180.0f || fabsf(s->yaw_dps) > 3276.7f) return 0;
    uint16_t angle = (uint16_t)(int32_t)lroundf(s->yaw_deg * 100.0f);
    uint16_t speed = (uint16_t)(int32_t)lroundf(s->yaw_dps * 10.0f);
    out[0] = 0x50U; out[1] = BOARD_PITCH_VERSION; out[2] = (uint8_t)((s->sequence << 1) | s->ready);
    out[3] = (uint8_t)(angle >> 8); out[4] = (uint8_t)angle;
    out[5] = (uint8_t)(speed >> 8); out[6] = (uint8_t)speed; out[7] = crc8(out);
    return 1;
}
/** @brief 拒绝损坏帧和角度越界，序号去重由接收模块处理。 */
int Board_Imu_Decode(const uint8_t *d, unsigned length, board_imu_status_t *s)
{
    if (!s || !valid(d, length)) return 0;
    float angle = signed16(d + 3) * 0.01f;
    if (fabsf(angle) > 180.0f) return 0;
    *s = (board_imu_status_t){d[2] & 1U, d[2] >> 1, angle, signed16(d + 5) * 0.1f};
    return 1;
}
