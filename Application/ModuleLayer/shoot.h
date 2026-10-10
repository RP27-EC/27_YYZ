/** @file shoot.h
 * @brief 英雄国赛遥控/键鼠发射输入和RM拨弹状态机。
 */
#ifndef DOWN_SHOOT_H
#define DOWN_SHOOT_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "board_fric_protocol.h"
/* Exported typedef ----------------------------------------------------------*/
typedef enum { SHOOT_OFF, SHOOT_INIT, SHOOT_READY, SHOOT_RELOAD, SHOOT_REVERT } shoot_state_t;
typedef struct {
    shoot_state_t state; /**< 关闭、找限位、就绪、拨弹、退弹。 */
    uint8_t enabled, fric_ready, feedback_online; /**< 发射总开关、六轮达速摘要有效、拨弹反馈有效。 */
    uint8_t input_seen, input_source, wheel_armed, last_mouse; /**< 输入已捕获、来源、拨轮手势及左键前态。 */
    uint16_t last_keys, encoder; /**< 上次按键及转子单圈编码器。 */
    int16_t speed, current, motor_out; /**< 转子RPM、反馈及输出电流原始值。 */
    int64_t position, target_position; /**< 转子累计编码器刻度。 */
    float target_speed; /**< 拨弹速度目标，转子RPM。 */
    uint32_t feedback_frames, feedback_ms, bad_feedback; /**< 有效反馈数、最近有效时刻毫秒、坏帧数。 */
    uint32_t state_ms, last_fire_ms; /**< 状态起点及最近发射请求时刻，毫秒。 */
    uint32_t stuck_since_ms; /**< 连续新鲜堵转反馈起点，毫秒。 */
    uint8_t stuck_active; /**< 堵转计时有效。 */
    board_fric_status_t fric_status; /**< 上板最近有效六轮摘要。 */
    uint32_t fric_frames, fric_ms, bad_status; /**< 有效摩擦轮摘要数、时刻毫秒、坏摘要数。 */
    uint32_t requests, shots, dropped_requests, reversals; /**< 请求、接受拨弹、丢弃请求/动作、堵转回退次数；shots不是实测出弹数。 */
    uint32_t init_timeouts, reload_timeouts, feedback_timeouts, standby_resets; /**< 初始化、拨弹/回退、反馈超时及关控复位次数。 */
    uint32_t dial_queued, dial_confirmed, dial_errors; /**< 拨弹电流入队、TXOK、失败或取消次数。 */
    uint32_t fric_queued, fric_confirmed, fric_errors; /**< 摩擦轮许可入队、TXOK、失败或取消次数。 */
} shoot_t;
/* Exported variables --------------------------------------------------------*/
extern shoot_t shoot; /**< 发射Watch对象，不反向撤销底盘或云台许可。 */
/* Exported functions --------------------------------------------------------*/
void Shoot_Init(void);
void Shoot_StandbyReset(uint32_t now);
void Shoot_Update(uint32_t now);
int Shoot_Receive(uint32_t id, const uint8_t *data, unsigned length, uint32_t now);
void Shoot_PackCurrent(uint8_t data[8]);
board_fric_command_t Shoot_FricCommand(uint32_t now, uint8_t sequence);
#endif
