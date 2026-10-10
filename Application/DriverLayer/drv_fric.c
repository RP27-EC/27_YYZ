/** @file drv_fric.c
 * @brief 独占上板CAN1，交替发送六轮电流并持续尝试总线恢复。
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_fric.h"
#include "fric.h"
#include "fric_config.h"
#include "can.h"
/* Private variables ---------------------------------------------------------*/
static uint32_t mailbox, pending_ms, control_ms, phase_ms, recovery_ms, recovery_ier;
static uint8_t pending, cancelling, group, sent_nonzero, bus_off_seen, recovery_seen;
/* Private functions ---------------------------------------------------------*/
/** @brief 只在运行时询问HAL；初始化期间使用硬件空邮箱位。 */
static int busy(void)
{
    if (hcan1.State == HAL_CAN_STATE_LISTENING || hcan1.State == HAL_CAN_STATE_READY)
        return HAL_CAN_IsTxMessagePending(&hcan1, mailbox) != 0U;
    return !(hcan1.Instance->TSR & (mailbox << CAN_TSR_TME0_Pos));
}
/** @brief CAN1所有邮箱仅属于摩擦轮，故障时丢弃旧电流后重新同步。 */
static void recovery(uint32_t now)
{
    fric.can_esr = hcan1.Instance->ESR;
    int bus_off = (fric.can_esr & CAN_ESR_BOFF) != 0U;
    if (bus_off && !bus_off_seen) ++fric.bus_off_events;
    bus_off_seen = (uint8_t)bus_off;
    if (fric.recovery_state == 0U) {
        if (!bus_off && hcan1.State == HAL_CAN_STATE_LISTENING && !(hcan1.Instance->MCR & CAN_MCR_INRQ)) return;
    } else if (fric.recovery_state == 1U) {
        if (hcan1.Instance->MSR & CAN_MSR_INAK) {
            hcan1.State = HAL_CAN_STATE_READY; hcan1.Instance->MCR &= ~(CAN_MCR_SLEEP | CAN_MCR_INRQ);
            fric.recovery_state = 2U; phase_ms = now;
        } else if (now - phase_ms >= FRIC_RECOVERY_STAGE_MS) { ++fric.recovery_failures; fric.recovery_state = 3U; }
        return;
    } else if (fric.recovery_state == 2U) {
        if (!(hcan1.Instance->MSR & CAN_MSR_INAK) && !bus_off) {
            hcan1.State = HAL_CAN_STATE_LISTENING; hcan1.ErrorCode = HAL_CAN_ERROR_NONE;
            hcan1.Instance->IER = recovery_ier; fric.recovery_state = 0U; recovery_seen = 1U;
        } else if (now - phase_ms >= FRIC_RECOVERY_STAGE_MS) { ++fric.recovery_failures; fric.recovery_state = 3U; }
        return;
    }
    if (fric.recovery_attempts && now - recovery_ms < FRIC_RECOVERY_RETRY_MS) return;
    if (!fric.recovery_state) recovery_ier = hcan1.Instance->IER;
    recovery_ms = phase_ms = now; ++fric.recovery_attempts;
    if (pending) { ++fric.tx_aborted; pending = cancelling = 0U; }
    hcan1.Instance->IER = 0U;
    hcan1.Instance->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
    hcan1.Instance->MCR = (hcan1.Instance->MCR & ~CAN_MCR_SLEEP) | CAN_MCR_INRQ;
    fric.recovery_state = 1U;
}
/* Exported functions --------------------------------------------------------*/
/** @brief 首次启动仅清空发送所有权，不复写电机参数或Pitch配置。 */
void Drv_Fric_Init(uint32_t now)
{
    mailbox = pending_ms = phase_ms = recovery_ms = 0U; control_ms = now;
    pending = cancelling = group = sent_nonzero = bus_off_seen = recovery_seen = 0U;
    recovery_ier = hcan1.Instance->IER;
}
/** @brief 任务每毫秒轮询，速度计算2ms一次，发送失败记录并继续发送最新目标。 */
void Drv_Fric_Poll(uint32_t now)
{
    uint32_t saved = __get_PRIMASK(); __disable_irq();
    if (now - control_ms >= FRIC_CONTROL_MS) { Fric_Update(now, now - control_ms); control_ms = now; }
    recovery(now);
    if (fric.recovery_state) { fric.ready = 0U; __set_PRIMASK(saved); return; }
    if (pending) {
        if (busy()) {
            if (!cancelling && ((sent_nonzero && !fric.enabled) || now - pending_ms >= FRIC_TX_TIMEOUT_MS)) {
                if (HAL_CAN_AbortTxRequest(&hcan1, mailbox) == HAL_OK) { cancelling = 1U; ++fric.tx_aborted; }
                if (now - pending_ms >= FRIC_TX_TIMEOUT_MS) {
                    ++fric.tx_errors; hcan1.Instance->MCR |= CAN_MCR_INRQ;
                }
            }
            __set_PRIMASK(saved); return;
        }
        unsigned shift = mailbox == CAN_TX_MAILBOX0 ? 0U : (mailbox == CAN_TX_MAILBOX1 ? 8U : 16U);
        if (!cancelling) {
            if (hcan1.Instance->TSR & (CAN_TSR_TXOK0 << shift)) {
                ++fric.tx_confirmed;
                if (recovery_seen) { ++fric.recoveries; recovery_seen = 0U; }
            } else ++fric.tx_errors;
        }
        pending = cancelling = 0U;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) {
        uint8_t data[8]; Fric_PackCurrent(group, data);
        CAN_TxHeaderTypeDef header = {0};
        header.StdId = group ? 0x1FFU : 0x200U;
        header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA; header.DLC = 8U;
        if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) == HAL_OK) {
            pending = 1U; pending_ms = now; ++fric.tx_queued; group ^= 1U;
            sent_nonzero = 0U;
            for (unsigned i = 0U; i < 8U; ++i) if (data[i]) sent_nonzero = 1U;
        } else ++fric.tx_errors;
    }
    __set_PRIMASK(saved);
}
