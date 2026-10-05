/**
 * @file    drv_pitch.c
 * @brief   CAN2单邮箱发送，退出时取消旧输出，用TXOK而非入队推进Pitch握手。
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_pitch.h"
#include "gimbal_pitch.h"
#include "can.h"

/* Private macro -------------------------------------------------------------*/
#define PITCH_TX_STATUS 255U /**< 板间摘要，不参与电机使能握手。 */
#define PITCH_TX_TIMEOUT_MS 20U /**< 单次邮箱等待上限，毫秒。 */
/* Private variables ---------------------------------------------------------*/
static uint32_t mailbox, pending_ms, poll_ms, status_ms; /**< 邮箱掩码及调度时刻，毫秒。 */
static uint8_t pending_kind, cancelling; /**< 待完成帧类型和已发起取消标记。 */
/* Exported variables --------------------------------------------------------*/
pitch_io_t pitch_io; /**< 本驱动拥有的CAN2发送状态。 */

/* Private functions ---------------------------------------------------------*/
/** @brief 只把相应邮箱的TXOK作为成功，取消帧不视为确认。 */
static void complete(uint32_t now)
{
    unsigned shift = mailbox == CAN_TX_MAILBOX0 ? 0U : (mailbox == CAN_TX_MAILBOX1 ? 8U : 16U);
    int success = !cancelling && (hcan2.Instance->TSR & (CAN_TSR_TXOK0 << shift));
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    if (success) ++pitch_io.confirmed;
    else ++pitch_io.errors;
    if (pending_kind != PITCH_TX_STATUS && !cancelling) Gimbal_Pitch_TxComplete(now, pending_kind, success);
    __set_PRIMASK(saved);
    pitch_io.pending = cancelling = 0U;
}

/** @brief 单次发送入队，并保存后续确认需要的邮箱和帧类型。 */
static void submit(uint32_t now, uint32_t id, uint8_t data[8], uint8_t kind)
{
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = id; header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    if (HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox) == HAL_OK) {
        pitch_io.pending = 1U; ++pitch_io.queued;
        pending_ms = now; pending_kind = kind;
        if (kind == PITCH_TX_STATUS) status_ms = now;
        else Gimbal_Pitch_Queued(now, kind);
    } else {
        ++pitch_io.errors;
        if (kind != PITCH_TX_STATUS) Gimbal_Pitch_TxComplete(now, kind, 0);
    }
}

/* Exported functions --------------------------------------------------------*/
/** @brief 初始化调度时间，CAN过滤器与中断由驱动管理器先配置。 */
void Drv_Pitch_Init(void)
{
    pitch_io = (pitch_io_t){0};
    mailbox = 0U; pending_kind = cancelling = 0U;
    poll_ms = status_ms = HAL_GetTick();
}

/** @brief 控制与IRQ共享状态短暂加锁，邮箱尚未释放时绝不排队新力矩。 */
void Drv_Pitch_Poll(uint32_t now)
{
    uint32_t id, saved;
    uint8_t data[8], kind;
    int available = 0;
    board_pitch_status_t status;
    if (now - poll_ms < PITCH_COMMAND_MS) return;
    poll_ms = now;
    if (pitch_io.pending && !HAL_CAN_IsTxMessagePending(&hcan2, mailbox)) complete(now);
    pitch_io.ready = HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_LISTENING &&
        !(hcan2.Instance->ESR & (CAN_ESR_BOFF | CAN_ESR_EPVF));
    saved = __get_PRIMASK(); __disable_irq();
    Gimbal_Pitch_Update(now, pitch_io.ready);
    if (pitch_io.pending) {
        int old_motion = pending_kind == PITCH_TX_ENABLE || pending_kind == PITCH_TX_TORQUE || pending_kind == PITCH_TX_ZERO;
        if (!cancelling && ((old_motion && Gimbal_Pitch_NeedsStop()) || now - pending_ms >= PITCH_TX_TIMEOUT_MS)) {
            if (HAL_CAN_AbortTxRequest(&hcan2, mailbox) == HAL_OK) { cancelling = 1U; ++pitch_io.aborted; }
        }
        __set_PRIMASK(saved);
        return;
    }
    if (HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_LISTENING && HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) {
        if (now - status_ms >= PITCH_STATUS_MS && gimbal_pitch.state != PITCH_ZEROING && gimbal_pitch.state != PITCH_STOPPING) {
            status.state = (uint8_t)gimbal_pitch.state; status.blocks = (uint8_t)gimbal_pitch.blocks;
            status.angle_deg = gimbal_pitch.actual_deg; status.motor_state = gimbal_pitch.motor_state;
            available = Board_Pitch_EncodeStatus(&status, data);
            id = BOARD_PITCH_STATUS_ID; kind = PITCH_TX_STATUS;
        } else available = Gimbal_Pitch_MakeFrame(now, &id, data, &kind);
        if (!available && now - status_ms >= PITCH_STATUS_MS) {
            status.state = (uint8_t)gimbal_pitch.state; status.blocks = (uint8_t)gimbal_pitch.blocks;
            status.angle_deg = gimbal_pitch.actual_deg; status.motor_state = gimbal_pitch.motor_state;
            available = Board_Pitch_EncodeStatus(&status, data);
            id = BOARD_PITCH_STATUS_ID; kind = PITCH_TX_STATUS;
        }
        if (available) submit(now, id, data, kind);
    }
    __set_PRIMASK(saved);
}
