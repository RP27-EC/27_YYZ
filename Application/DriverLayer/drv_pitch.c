/**
 * @file    drv_pitch.c
 * @brief   CAN2持续重试、故障诊断及非阻塞初始化恢复，TXOK推进Pitch握手。
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_pitch.h"
#include "gimbal_pitch.h"
#include "can.h"
#include "fric.h"

/* Private macro -------------------------------------------------------------*/
#define PITCH_TX_IMU 254U /**< Yaw IMU摘要，不参与电机使能握手。 */
#define PITCH_TX_FRIC 253U /**< 独立摩擦轮就绪摘要，不参与Pitch握手。 */
#define PITCH_TX_STATUS 255U /**< 板间摘要，不参与电机使能握手。 */
#define PITCH_TX_TIMEOUT_MS 20U /**< 单次邮箱等待上限，毫秒。 */
#define PITCH_ALL_MAILBOXES (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) /**< CAN2所有发送邮箱掩码。 */
#define PITCH_ALL_RQCP (CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2) /**< 写1清除三个邮箱的旧完成标记。 */
/* Private variables ---------------------------------------------------------*/
static uint32_t mailbox, pending_ms, poll_ms, status_ms, imu_ms; /**< 邮箱掩码及调度时刻，毫秒。 */
static uint32_t fric_status_ms; /**< 六摩擦轮摘要上次入队时刻，毫秒。 */
static uint8_t pending_kind, cancelling, pending_failed; /**< 待完成帧类型、已发起取消及邮箱超时已计一次标记。 */
static uint8_t passive_seen, bus_off_seen, unavailable_seen; /**< 硬件状态边沿和故障观察节拍初始化。 */
static uint8_t restart_needed, motor_restart_needed; /**< 实际发送错误需要恢复硬件，仲裁失败仅重试最新帧。 */
static uint32_t unavailable_ms, recovery_phase_ms, recovery_ier; /**< 不可用观察及恢复阶段时刻，毫秒；保存的中断许可。 */
/* Exported variables --------------------------------------------------------*/
pitch_io_t pitch_io; /**< 本驱动拥有的CAN2发送状态。 */

/* Private functions ---------------------------------------------------------*/
/** @brief HAL异常时仍用硬件空邮箱位确认取消完成，避免HAL返回0误当释放。 */
static int hardware_pending(uint32_t mailboxes)
{
    HAL_CAN_StateTypeDef state = HAL_CAN_GetState(&hcan2);
    uint32_t empty = mailboxes << CAN_TSR_TME0_Pos;
    if (state == HAL_CAN_STATE_LISTENING || state == HAL_CAN_STATE_READY)
        return HAL_CAN_IsTxMessagePending(&hcan2, mailboxes) != 0U;
    return (hcan2.Instance->TSR & empty) != empty;
}

/** @brief 故障期计数饱和递增，实际成功前保持首次失败时刻。 */
static void failure_observed(uint32_t now)
{
    if (!pitch_io.failure_active) {
        pitch_io.failure_active = 1U;
        pitch_io.failure_since_ms = now;
    }
    if (pitch_io.failure_streak != UINT32_MAX) ++pitch_io.failure_streak;
}

/** @brief 记录一次真实发送失败，纯仲裁失败不请求硬件初始化。 */
static void failed(uint32_t now, uint8_t kind, int arbitration)
{
    ++pitch_io.errors;
    pitch_io.failure_ms = now;
    pitch_io.failure_esr = hcan2.Instance->ESR;
    pitch_io.failure_tsr = hcan2.Instance->TSR;
    failure_observed(now);
    if (arbitration) ++pitch_io.arbitration_lost;
    else {
        restart_needed = 1U;
        if (kind < PITCH_TX_FRIC) motor_restart_needed = 1U;
    }
    if (kind == PITCH_TX_IMU) ++pitch_io.imu_errors;
    else if (kind == PITCH_TX_STATUS) ++pitch_io.status_errors;
    else if (kind != PITCH_TX_FRIC) Gimbal_Pitch_TxComplete(now, kind, 0);
}

/** @brief 只由真正TXOK结束驱动故障期，硬件同步本身不算发送成功。 */
static void succeeded(uint32_t now, uint8_t kind)
{
    ++pitch_io.confirmed;
    pitch_io.last_tx_ok_ms = now;
    if (pitch_io.failure_active) pitch_io.last_failure_duration_ms = now - pitch_io.failure_since_ms;
    pitch_io.failure_active = 0U;
    pitch_io.failure_streak = 0U;
    restart_needed = 0U;
    if (pitch_io.recovery_unconfirmed) {
        ++pitch_io.recoveries;
        pitch_io.recovery_unconfirmed = 0U;
    }
    if (kind == PITCH_TX_IMU) ++pitch_io.imu_confirmed;
    else if (kind == PITCH_TX_STATUS) ++pitch_io.status_confirmed;
    else if (kind != PITCH_TX_FRIC) {
        motor_restart_needed = 0U;
        Gimbal_Pitch_TxComplete(now, kind, 1);
    }
}

/** @brief 只把相应邮箱的TXOK作为成功，取消帧不视为确认。 */
static void complete(uint32_t now)
{
    unsigned shift = mailbox == CAN_TX_MAILBOX0 ? 0U : (mailbox == CAN_TX_MAILBOX1 ? 8U : 16U);
    uint32_t tsr = hcan2.Instance->TSR;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    if (!cancelling) {
        if (tsr & (CAN_TSR_TXOK0 << shift)) succeeded(now, pending_kind);
        else if (!pending_failed) failed(now, pending_kind, (tsr & (CAN_TSR_ALST0 << shift)) && !(tsr & (CAN_TSR_TERR0 << shift)));
    }
    __set_PRIMASK(saved);
    pitch_io.pending = cancelling = pending_failed = 0U;
}

/** @brief 单次发送入队，并保存后续确认需要的邮箱和帧类型。 */
static void submit(uint32_t now, uint32_t id, uint8_t data[8], uint8_t kind)
{
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = id; header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    if (HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox) == HAL_OK) {
        pitch_io.pending = 1U; ++pitch_io.queued;
        if (kind == PITCH_TX_IMU) ++pitch_io.imu_queued;
        else if (kind == PITCH_TX_STATUS) ++pitch_io.status_queued;
        pending_ms = now; pending_kind = kind; pending_failed = 0U;
        if (kind == PITCH_TX_STATUS) status_ms = now;
        else if (kind == PITCH_TX_IMU) imu_ms = now;
        else if (kind == PITCH_TX_FRIC) fric_status_ms = now;
        else Gimbal_Pitch_Queued(now, kind);
    } else {
        failed(now, kind, 0);
    }
}

/** @brief 硬件收发能力与500ms控制宽限分开，错误被动不直接禁止发送。 */
static int transport_available(void)
{
    return HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_LISTENING &&
        !(hcan2.Instance->ESR & CAN_ESR_BOFF) && !(hcan2.Instance->MSR & CAN_MSR_INAK) &&
        !(hcan2.Instance->MCR & CAN_MCR_INRQ) &&
        (pitch_io.recovery_state == PITCH_CAN_RECOVERY_IDLE || pitch_io.recovery_state == PITCH_CAN_RECOVERY_BACKOFF);
}

/** @brief 记录状态进入事件，硬件持续不可用时按20ms观察计入故障期。 */
static void observe_hardware(uint32_t now)
{
    uint32_t esr = hcan2.Instance->ESR;
    int passive = (esr & CAN_ESR_EPVF) != 0U, bus_off = (esr & CAN_ESR_BOFF) != 0U;
    if (passive && !passive_seen) ++pitch_io.error_passive_events;
    if (bus_off && !bus_off_seen) ++pitch_io.bus_off_events;
    passive_seen = (uint8_t)passive; bus_off_seen = (uint8_t)bus_off;
    pitch_io.can_esr = esr;
    pitch_io.transport_ready = (uint8_t)transport_available();
    if (!pitch_io.transport_ready && (!unavailable_seen || now - unavailable_ms >= PITCH_SPECIAL_MS)) {
        unavailable_seen = 1U; unavailable_ms = now;
        ++pitch_io.unavailable_samples;
        failure_observed(now);
    }
    if (pitch_io.transport_ready) unavailable_seen = 0U;
}

/** @brief 恢复失败仅进入退避，不伪造可用状态或清除累计故障超时。 */
static void recovery_failed(uint32_t now)
{
    ++pitch_io.recovery_failures;
    pitch_io.recovery_state = PITCH_CAN_RECOVERY_BACKOFF;
    recovery_phase_ms = now;
    hcan2.Instance->IER = recovery_ier;
}

/** @brief 取消旧发送后请求初始化；每次轮询只推进一步，不等待SysTick。 */
static void recover_can(uint32_t now)
{
    pitch_can_recovery_state_t state = pitch_io.recovery_state;
    if (state == PITCH_CAN_RECOVERY_IDLE || state == PITCH_CAN_RECOVERY_BACKOFF) {
        int motor_fault = motor_restart_needed && gimbal_pitch.tx_failure_active &&
            gimbal_pitch.tx_failure_streak >= PITCH_CAN_RESTART_FAILURES &&
            now - gimbal_pitch.tx_failure_since_ms >= PITCH_CAN_RESTART_DELAY_MS;
        int needed = !pitch_io.transport_ready || motor_fault ||
            (restart_needed && pitch_io.failure_active && pitch_io.failure_streak >= PITCH_CAN_RESTART_FAILURES &&
             now - pitch_io.failure_since_ms >= PITCH_CAN_RESTART_DELAY_MS);
        if (!needed) {
            pitch_io.recovery_state = PITCH_CAN_RECOVERY_IDLE;
            return;
        }
        if (pitch_io.recovery_attempts && now - pitch_io.last_recovery_ms < PITCH_CAN_RECOVERY_RETRY_MS) return;
        ++pitch_io.recovery_attempts;
        pitch_io.last_recovery_ms = recovery_phase_ms = now;
        recovery_ier = hcan2.Instance->IER;
        hcan2.Instance->IER = 0U;
        if (pitch_io.pending) { cancelling = 1U; ++pitch_io.aborted; }
        if (HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_LISTENING || HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_READY)
            (void)HAL_CAN_AbortTxRequest(&hcan2, PITCH_ALL_MAILBOXES);
        else hcan2.Instance->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
        Gimbal_Pitch_TransportRestart(now);
        pitch_io.recovery_state = PITCH_CAN_RECOVERY_ABORT;
        pitch_io.transport_ready = 0U;
        pitch_io.recovery_unconfirmed = 0U;
        return;
    }
    if (state == PITCH_CAN_RECOVERY_ABORT && !hardware_pending(PITCH_ALL_MAILBOXES)) {
        pitch_io.pending = cancelling = pending_failed = 0U;
        hcan2.Instance->TSR = PITCH_ALL_RQCP;
        hcan2.Instance->MCR = (hcan2.Instance->MCR & ~CAN_MCR_SLEEP) | CAN_MCR_INRQ;
        pitch_io.recovery_state = PITCH_CAN_RECOVERY_INIT;
        recovery_phase_ms = now;
    } else if (state == PITCH_CAN_RECOVERY_INIT && (hcan2.Instance->MSR & CAN_MSR_INAK)) {
        for (unsigned i = 0U; i < 3U && (hcan2.Instance->RF0R & CAN_RF0R_FMP0); ++i)
            hcan2.Instance->RF0R = CAN_RF0R_RFOM0;
        for (unsigned i = 0U; i < 3U && (hcan2.Instance->RF1R & CAN_RF1R_FMP1); ++i)
            hcan2.Instance->RF1R = CAN_RF1R_RFOM1;
        hcan2.State = HAL_CAN_STATE_READY;
        hcan2.Instance->MCR &= ~(CAN_MCR_SLEEP | CAN_MCR_INRQ);
        pitch_io.recovery_state = PITCH_CAN_RECOVERY_SYNC;
        recovery_phase_ms = now;
    } else if (state == PITCH_CAN_RECOVERY_SYNC && !(hcan2.Instance->MSR & CAN_MSR_INAK) &&
               !(hcan2.Instance->ESR & CAN_ESR_BOFF)) {
        hcan2.State = HAL_CAN_STATE_LISTENING;
        hcan2.ErrorCode = HAL_CAN_ERROR_NONE;
        hcan2.Instance->IER = recovery_ier;
        pitch_io.recovery_state = PITCH_CAN_RECOVERY_IDLE;
        pitch_io.recovery_unconfirmed = 1U;
        pitch_io.transport_ready = (uint8_t)transport_available();
    } else if (now - recovery_phase_ms >= PITCH_CAN_RECOVERY_STAGE_MS) recovery_failed(now);
}

/* Exported functions --------------------------------------------------------*/
/** @brief 初始化调度时间，CAN过滤器与中断由驱动管理器先配置。 */
void Drv_Pitch_Init(void)
{
    pitch_io = (pitch_io_t){0};
    mailbox = 0U; pending_kind = cancelling = pending_failed = 0U;
    passive_seen = bus_off_seen = unavailable_seen = restart_needed = motor_restart_needed = 0U;
    unavailable_ms = recovery_phase_ms = recovery_ier = 0U;
    poll_ms = status_ms = imu_ms = HAL_GetTick();
    fric_status_ms = poll_ms;
}

/** @brief 控制与IRQ共享状态短暂加锁，邮箱尚未释放时绝不排队新力矩。 */
void Drv_Pitch_Poll(uint32_t now)
{
    uint32_t id, saved;
    uint8_t data[8], kind;
    int available = 0;
    board_pitch_status_t status;
    board_imu_status_t imu;
    if (now - poll_ms < PITCH_COMMAND_MS) return;
    poll_ms = now;
    saved = __get_PRIMASK(); __disable_irq();
    if (pitch_io.pending && !hardware_pending(mailbox)) complete(now);
    observe_hardware(now);
    recover_can(now);
    pitch_io.ready = 1U;
    pitch_io.failure_duration_ms = pitch_io.failure_active ? now - pitch_io.failure_since_ms : 0U;
    if (pitch_io.failure_active) pitch_io.last_failure_duration_ms = pitch_io.failure_duration_ms;
    Gimbal_Pitch_Update(now, pitch_io.transport_ready);
    if (pitch_io.pending) {
        int old_motion = pending_kind == PITCH_TX_ENABLE || pending_kind == PITCH_TX_TORQUE || pending_kind == PITCH_TX_ZERO;
        int stopping = old_motion && Gimbal_Pitch_NeedsStop();
        if (!cancelling && (stopping || now - pending_ms >= PITCH_TX_TIMEOUT_MS)) {
            if (!stopping && !pending_failed) {
                pending_failed = 1U;
                ++pitch_io.tx_timeouts;
                failed(now, pending_kind, 0);
            }
            if (HAL_CAN_AbortTxRequest(&hcan2, mailbox) == HAL_OK) { cancelling = 1U; ++pitch_io.aborted; }
        }
        __set_PRIMASK(saved);
        return;
    }
    if (pitch_io.transport_ready && HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) {
        if (now - status_ms >= PITCH_STATUS_MS && gimbal_pitch.state != PITCH_ZEROING && gimbal_pitch.state != PITCH_STOPPING) {
            status.state = (uint8_t)gimbal_pitch.state; status.blocks = (uint8_t)gimbal_pitch.blocks;
            status.angle_deg = gimbal_pitch.actual_deg; status.motor_state = gimbal_pitch.motor_state;
            available = Board_Pitch_EncodeStatus(&status, data);
            id = BOARD_PITCH_STATUS_ID; kind = PITCH_TX_STATUS;
            if (!available) ++pitch_io.status_encode_errors;
        }
        if (!available && now - imu_ms >= PITCH_IMU_STATUS_MS &&
            ((gimbal_pitch.state != PITCH_ZEROING && gimbal_pitch.state != PITCH_STOPPING) || gimbal_pitch.standby_pending)) {
            imu.ready = gimbal_pitch.imu_yaw_ready && gimbal_pitch.imu_ready && now - gimbal_pitch.imu_last_ms <= PITCH_IMU_TIMEOUT_MS;
            imu.sequence = (uint8_t)(gimbal_pitch.imu_updates & 127U);
            imu.yaw_deg = gimbal_pitch.imu_yaw_deg; imu.yaw_dps = gimbal_pitch.imu_yaw_dps;
            available = Board_Imu_Encode(&imu, data);
            id = BOARD_IMU_STATUS_ID; kind = PITCH_TX_IMU;
            if (!available) ++pitch_io.imu_encode_errors;
        }
        if (!available && fric.command_seen && now - fric_status_ms >= BOARD_FRIC_STATUS_MS) {
            board_fric_status_t fric_status = Fric_Status();
            available = Board_Fric_EncodeStatus(&fric_status, data);
            id = BOARD_FRIC_STATUS_ID; kind = PITCH_TX_FRIC;
        }
        if (!available) available = Gimbal_Pitch_MakeFrame(now, &id, data, &kind);
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
