/** @file fric.c
 * @brief 英雄六摩擦轮映射、速度PI及就绪摘要。
 */
/* Includes ------------------------------------------------------------------*/
#include "fric.h"
#include "fric_config.h"
#include <string.h>
/* Exported variables --------------------------------------------------------*/
fric_t fric; /**< 六轮反馈、目标及故障累计。 */
/* Private functions ---------------------------------------------------------*/
/** @brief 对称限幅，单位随调用方。 */
static float limit(float x, float bound) { return x > bound ? bound : (x < -bound ? -bound : x); }
/* Exported functions --------------------------------------------------------*/
/** @brief 上电默认关闭发射输出，等待下板新许可。 */
void Fric_Init(void) { memset(&fric, 0, sizeof(fric)); }
/** @brief 坏帧丢弃；合法重复序号也持续更新遥控许可。 */
int Fric_ReceiveCommand(const uint8_t *d, unsigned n, uint32_t now)
{
    board_fric_command_t command;
    if (!Board_Fric_DecodeCommand(d, n, &command)) { ++fric.command_errors; return 0; }
    fric.command = command; fric.command_ms = now; fric.command_seen = 1U; ++fric.command_frames;
    return 1;
}
/** @brief CAN1反馈0x201/202/203及0x205/206/207，坏帧不覆盖有效速度。 */
int Fric_ReceiveMotor(uint32_t id, const uint8_t *d, unsigned n, uint32_t now)
{
    unsigned index;
    if (id >= 0x201U && id <= 0x203U) index = id - 0x201U;
    else if (id >= 0x205U && id <= 0x207U) index = id - 0x205U + 3U;
    else return 0;
    fric_motor_t *m = &fric.motor[index];
    if (!d || n != 8U || ((unsigned)d[0] << 8 | d[1]) >= 8192U) { ++m->bad_frames; return 1; }
    m->encoder = (uint16_t)((unsigned)d[0] << 8 | d[1]);
    m->speed = (int16_t)((unsigned)d[2] << 8 | d[3]);
    m->current = (int16_t)((unsigned)d[4] << 8 | d[5]);
    m->last_ms = now; ++m->frames; return 1;
}
/** @brief 许可失联或关闭仅撤销发射输出；新鲜反馈恢复后自动继续速度计算。 */
void Fric_Update(uint32_t now, uint32_t dt_ms)
{
    const float targets[FRIC_COUNT] = {
        -FRIC_FRONT_RPM + FRIC_FRONT_OFFSET_RPM, -FRIC_FRONT_RPM - FRIC_FRONT_OFFSET_RPM,
         FRIC_FRONT_RPM + FRIC_FRONT_OFFSET_RPM, -FRIC_BACK_RPM, -FRIC_BACK_RPM, FRIC_BACK_RPM
    };
    int enabled = fric.command_seen && now - fric.command_ms <= BOARD_FRIC_LEASE_MS &&
        fric.command.online && fric.command.enabled;
    if (fric.enabled && !enabled && now - fric.command_ms > BOARD_FRIC_LEASE_MS) ++fric.lease_timeouts;
    uint8_t previous_mask = fric.online_mask;
    fric.enabled = (uint8_t)enabled; fric.ready = (uint8_t)enabled; fric.online_mask = 0U; ++fric.loops;
    float interval = (float)(dt_ms && dt_ms <= 20U ? dt_ms : FRIC_CONTROL_MS) / 2.0f;
    for (unsigned i = 0U; i < FRIC_COUNT; ++i) {
        fric_motor_t *m = &fric.motor[i];
        int online = m->frames && now - m->last_ms <= FRIC_FEEDBACK_MS;
        if (online) fric.online_mask |= (uint8_t)(1U << i);
        else if (previous_mask & (1U << i)) ++fric.feedback_timeouts;
        m->target = enabled ? targets[i] : 0.0f;
        if (!enabled || !online) { m->output = 0; m->integral = 0.0f; fric.ready = 0U; continue; }
        float error = m->target - m->speed;
        float max_current = i < 3U ? FRIC_FRONT_CURRENT_MAX : FRIC_BACK_CURRENT_MAX;
        float integral = limit(m->integral + FRIC_KI_PER_2MS * interval * error, FRIC_INTEGRAL_MAX);
        float candidate = FRIC_KP * error + integral;
        if ((candidate <= max_current && candidate >= -max_current) || candidate * error < 0.0f) m->integral = integral;
        m->output = (int16_t)limit(FRIC_KP * error + m->integral, max_current);
        if ((targets[i] > 0.0f ? m->speed : -m->speed) < FRIC_READY_MIN_RPM) fric.ready = 0U;
    }
}
/** @brief 前组三槽发送0x200、后三槽发送0x1FF，第四槽保持零。 */
void Fric_PackCurrent(unsigned group, uint8_t data[8])
{
    memset(data, 0, 8U);
    if (group > 1U) return;
    for (unsigned i = 0U; i < 3U; ++i) {
        uint16_t value = (uint16_t)fric.motor[group * 3U + i].output;
        data[2U * i] = (uint8_t)(value >> 8); data[2U * i + 1U] = (uint8_t)value;
    }
}
/** @brief 构造独立摘要，不占用或修改Pitch状态字段。 */
board_fric_status_t Fric_Status(void)
{
    return (board_fric_status_t){fric.enabled, fric.ready, fric.online_mask, fric.sequence++};
}
