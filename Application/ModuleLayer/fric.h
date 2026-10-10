/** @file fric.h
 * @brief 六摩擦轮速度控制和有效许可租约，不改变Pitch或整车使能。
 */
#ifndef UP_FRIC_H
#define UP_FRIC_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "board_fric_protocol.h"
/* Exported typedef ----------------------------------------------------------*/
enum { FRIC_SLOT_F_UP, FRIC_SLOT_F_RIGHT, FRIC_SLOT_F_LEFT, FRIC_SLOT_B_UP, FRIC_SLOT_B_RIGHT, FRIC_SLOT_B_LEFT, FRIC_COUNT };
typedef struct {
    uint16_t encoder; /**< 单圈转子编码器，8192刻度每转。 */
    int16_t speed, current, output; /**< 转子RPM、反馈及输出电流原始值。 */
    float target, integral; /**< 转子RPM目标及积分输出原始值。 */
    uint32_t frames, last_ms, bad_frames; /**< 有效反馈计数、时刻毫秒及坏帧数。 */
} fric_motor_t;
typedef struct {
    fric_motor_t motor[FRIC_COUNT]; /**< 前上/右/左、后上/右/左六轮。 */
    uint8_t enabled, ready, online_mask; /**< 有效发射许可、六轮达速及六位反馈在线。 */
    uint8_t command_seen, sequence; /**< 已收到有效下板许可及摘要序号。 */
    board_fric_command_t command; /**< 最后有效下板许可。 */
    uint32_t command_frames, command_ms, command_errors; /**< 有效许可数、时刻毫秒及坏帧数。 */
    uint32_t lease_timeouts, loops, feedback_timeouts; /**< 许可失联、控制循环及电机反馈离线边沿次数。 */
    uint32_t tx_queued, tx_confirmed, tx_errors, tx_aborted; /**< CAN1入队、TXOK、错误及取消次数。 */
    uint32_t bus_off_events, recovery_attempts, recoveries, recovery_failures; /**< 总线关闭、恢复请求、恢复后TXOK及恢复阶段失败次数。 */
    uint32_t can_esr; /**< 上板CAN1当前错误寄存器快照。 */
    uint8_t recovery_state; /**< CAN1恢复阶段：0运行、1等待初始化、2等待同步、3退避。 */
} fric_t;
/* Exported variables --------------------------------------------------------*/
extern fric_t fric; /**< 上板六摩擦轮Watch对象。 */
/* Exported functions --------------------------------------------------------*/
void Fric_Init(void);
int Fric_ReceiveCommand(const uint8_t *data, unsigned length, uint32_t now);
int Fric_ReceiveMotor(uint32_t id, const uint8_t *data, unsigned length, uint32_t now);
void Fric_Update(uint32_t now, uint32_t dt_ms);
void Fric_PackCurrent(unsigned group, uint8_t data[8]);
board_fric_status_t Fric_Status(void);
#endif
