/** @file board_fric_protocol.h
 * @brief 六摩擦轮许可与就绪摘要，独立于现有Pitch版本。
 */
#ifndef BOARD_FRIC_PROTOCOL_H
#define BOARD_FRIC_PROTOCOL_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported macro ------------------------------------------------------------*/
#define BOARD_FRIC_COMMAND_ID 0x0D2U /**< 下板到上板摩擦轮许可，标准8字节。 */
#define BOARD_FRIC_STATUS_ID 0x022U /**< 上板到下板六轮就绪摘要，标准8字节。 */
#define BOARD_FRIC_PERIOD_MS 10U /**< 许可发送间隔，毫秒。 */
#define BOARD_FRIC_STATUS_MS 20U /**< 摘要发送间隔，毫秒。 */
#define BOARD_FRIC_LEASE_MS 300U /**< 上板遥控许可失联时撤销发射输出，毫秒。 */
/* Exported typedef ----------------------------------------------------------*/
typedef struct {
    uint8_t online, enabled, sequence; /**< 遥控在线、发射总开关、发送序号。 */
} board_fric_command_t;
typedef struct {
    uint8_t enabled, ready, online_mask, sequence; /**< 总开关、六轮达速、六位在线及摘要序号。 */
} board_fric_status_t;
/* Exported functions --------------------------------------------------------*/
int Board_Fric_EncodeCommand(const board_fric_command_t *s, uint8_t out[8]);
int Board_Fric_DecodeCommand(const uint8_t *d, unsigned length, board_fric_command_t *s);
int Board_Fric_EncodeStatus(const board_fric_status_t *s, uint8_t out[8]);
int Board_Fric_DecodeStatus(const uint8_t *d, unsigned length, board_fric_status_t *s);
#endif
