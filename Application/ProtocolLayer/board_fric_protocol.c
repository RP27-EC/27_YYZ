/** @file board_fric_protocol.c
 * @brief 显式字节编码、类型隔离及CRC8校验；上下板保持相同实现。
 */
/* Includes ------------------------------------------------------------------*/
#include "board_fric_protocol.h"
#include <stddef.h>
/* Private functions ---------------------------------------------------------*/
/** @brief CRC8初值0xFF、多项式0x31，覆盖前七字节。 */
static uint8_t crc8(const uint8_t *d)
{
    uint8_t crc = 0xffU;
    for (unsigned i = 0U; i < 7U; ++i) {
        crc ^= d[i];
        for (unsigned b = 0U; b < 8U; ++b) crc = crc & 128U ? (uint8_t)((crc << 1) ^ 0x31U) : (uint8_t)(crc << 1);
    }
    return crc;
}
/** @brief 验证长度、独立魔数、类型、版本及校验和。 */
static int valid(const uint8_t *d, unsigned n, uint8_t type)
{
    return d && n == 8U && d[0] == 0x46U && d[1] == type && d[6] == 1U && d[7] == crc8(d);
}
/* Exported functions --------------------------------------------------------*/
/** @brief 编码持续状态许可，不在链路重发时产生新的拨弹动作。 */
int Board_Fric_EncodeCommand(const board_fric_command_t *s, uint8_t out[8])
{
    if (!s || !out || s->online > 1U || s->enabled > 1U || (s->enabled && !s->online)) return 0;
    uint8_t d[8] = {0x46U, 1U, s->online, s->enabled, s->sequence, 0U, 1U, 0U};
    d[7] = crc8(d);
    for (unsigned i = 0U; i < 8U; ++i) out[i] = d[i];
    return 1;
}
/** @brief 坏帧不改变调用方保存的上一有效命令。 */
int Board_Fric_DecodeCommand(const uint8_t *d, unsigned n, board_fric_command_t *s)
{
    if (!s || !valid(d, n, 1U) || d[2] > 1U || d[3] > 1U || d[5] || (d[3] && !d[2])) return 0;
    *s = (board_fric_command_t){d[2], d[3], d[4]}; return 1;
}
/** @brief 编码六轮在线及达速结果。 */
int Board_Fric_EncodeStatus(const board_fric_status_t *s, uint8_t out[8])
{
    if (!s || !out || s->enabled > 1U || s->ready > 1U || s->online_mask > 63U ||
        (s->ready && (!s->enabled || s->online_mask != 63U))) return 0;
    uint8_t d[8] = {0x46U, 2U, s->enabled, s->ready, s->online_mask, s->sequence, 1U, 0U};
    d[7] = crc8(d);
    for (unsigned i = 0U; i < 8U; ++i) out[i] = d[i];
    return 1;
}
/** @brief 拒绝不一致的达速摘要；坏帧不续期。 */
int Board_Fric_DecodeStatus(const uint8_t *d, unsigned n, board_fric_status_t *s)
{
    if (!s || !valid(d, n, 2U) || d[2] > 1U || d[3] > 1U || d[4] > 63U ||
        (d[3] && (!d[2] || d[4] != 63U))) return 0;
    *s = (board_fric_status_t){d[2], d[3], d[4], d[5]}; return 1;
}
