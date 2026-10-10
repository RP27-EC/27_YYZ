/**
 * @file    drv_pitch_link.c
 * @brief   CAN2板间收发与非阻塞Bus-Off恢复，不占用底盘与Yaw的CAN1。
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_pitch_link.h"
#include "pitch_link_config.h"
#include "main.h"
#include "rc_sensor.h"
#include "carctrl.h"
#include "gyro_control.h"
#include "chassis_config.h"
#include "shoot.h"
#include "shoot_config.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static FDCAN_HandleTypeDef pitch_can; /**< 独立CAN2句柄；接收由任务轮询。 */
static uint32_t send_ms, pending_ms, pending_mask; /**< 调度时刻、入队时刻与发送槽位。 */
static uint8_t sequence, cancelling, pending_active; /**< 命令序号与过期许可取消状态。 */
static uint8_t pending_kind, fric_sequence; /**< 当前帧0=Pitch、1=摩擦轮许可、2=拨弹电流。 */
static uint32_t fric_send_ms, dial_send_ms; /**< 摩擦轮许可及拨弹电流发送时刻，毫秒。 */
static uint8_t pending_standby; /**< 当前发送槽是否属于本次待命请求。 */
static uint8_t status_lease_valid; /**< 最近摘要属于当前通信阶段，Bus-Off撤销旧租约。 */
static uint32_t init_retry_ms; /**< 初始化重试节拍，毫秒。 */
static uint32_t recovery_started_ms; /**< 本次退避开始时刻，毫秒。 */
static uint32_t tx_failure_started_ms; /**< 连续发送失败开始时刻，毫秒。 */
static uint8_t tx_failure_active; /**< 连续失败恢复计时有效。 */
static uint32_t queue_busy_ms; /**< 未归属当前命令的硬件队列阻塞起点，毫秒。 */
static uint8_t queue_busy; /**< 队列持续占用计时有效。 */
static uint8_t bus_off_event, recovery_flushed, recovery_timeout_recorded; /**< 待消费诊断事件、硬件旧帧清空和超时已记录标记。 */
/* Exported variables --------------------------------------------------------*/
pitch_link_t pitch_link; /**< 两板连接统计及协议观察对象。 */
pitch_link_diag_t pitch_link_diag; /**< FIFO接收、发送失败快照及Bus-Off恢复状态。 */

/* Private function prototypes -----------------------------------------------*/
static void retry_init(void); /**< 重试本外设初始化，保留接收和故障统计。 */
/* Private functions ---------------------------------------------------------*/
/** @brief 锁存具体LEC与首次Bus-Off；已有失败现场直接复用，避免重复读取清除ECR日志。 */
static void retain_can_error(uint32_t now, uint32_t psr, const pitch_can_error_sample_t *captured)
{
    uint32_t code = psr & FDCAN_PSR_LEC;
    int specific = code >= 1U && code <= 6U;
    uint8_t bus_off = (psr & FDCAN_PSR_BO) != 0U;
    if (specific || (bus_off && !pitch_link_diag.bus_off)) {
        pitch_can_error_sample_t sample = captured ? *captured :
            (pitch_can_error_sample_t){now, code, pitch_can.Instance->CCCR, psr, pitch_can.Instance->ECR};
        if (specific) {
            ++pitch_link_diag.error_samples[code];
            if (!pitch_link_diag.first_error.code) pitch_link_diag.first_error = sample;
            pitch_link_diag.last_error = sample;
        }
        if (bus_off && !pitch_link_diag.bus_off) {
            if (!pitch_link_diag.bus_off_events) pitch_link_diag.first_bus_off = sample;
            ++pitch_link_diag.bus_off_events;
        }
    }
    pitch_link_diag.bus_off = bus_off;
}

/** @brief 在发送失败时保存硬件状态，PSR读取会清除硬件LEC历史。 */
static void capture_failure(uint32_t now)
{
    if (!tx_failure_active) { tx_failure_active = 1U; tx_failure_started_ms = now; }
    ++pitch_link_diag.consecutive_tx_errors;
    pitch_link_diag.failure_duration_ms = now - tx_failure_started_ms;
    pitch_link_diag.failure_ms = now;
    pitch_link_diag.failure_cccr = pitch_can.Instance->CCCR;
    pitch_link_diag.failure_psr = pitch_can.Instance->PSR;
    pitch_link_diag.failure_ecr = pitch_can.Instance->ECR;
    pitch_can_error_sample_t sample = {now, pitch_link_diag.failure_psr & FDCAN_PSR_LEC,
        pitch_link_diag.failure_cccr, pitch_link_diag.failure_psr, pitch_link_diag.failure_ecr};
    retain_can_error(now, pitch_link_diag.failure_psr, &sample);
}

/** @brief 只恢复CAN2并撤销摘要新鲜度；不改变整车会话、IMU参考或ACTIVE请求。 */
static void begin_recovery(uint32_t now, uint32_t psr)
{
    pitch_link_diag.failure_ms = now;
    pitch_link_diag.failure_cccr = pitch_can.Instance->CCCR;
    pitch_link_diag.failure_psr = psr;
    pitch_link_diag.failure_ecr = pitch_can.Instance->ECR;
    pitch_link_diag.recovery_state = PITCH_LINK_CAN_BACKOFF;
    recovery_started_ms = now;
    recovery_flushed = recovery_timeout_recorded = 0U;
    bus_off_event = 1U;
    status_lease_valid = 0U;
    pitch_link.online = 0U;
    tx_failure_active = 0U;
    queue_busy = 0U;
}

/** @brief 在CAN2初始化状态下清除旧TX/RX，再按节拍启动并等待硬件恢复。 */
static void poll_recovery(uint32_t now)
{
    if (pitch_link_diag.recovery_state == PITCH_LINK_CAN_BACKOFF) {
        if (!recovery_flushed) {
            if (!(pitch_can.Instance->CCCR & FDCAN_CCCR_INIT)) {
                SET_BIT(pitch_can.Instance->CCCR, FDCAN_CCCR_INIT);
                return;
            }
            pitch_link_diag.discarded_rx += HAL_FDCAN_GetRxFifoFillLevel(&pitch_can, FDCAN_RX_FIFO0);
            if (pending_mask || pitch_can.Instance->TXBRP) {
                ++pitch_link.errors;
                ++pitch_link.aborted;
            }
            CLEAR_BIT(pitch_can.Instance->CCCR, FDCAN_CCCR_CCE);
            SET_BIT(pitch_can.Instance->CCCR, FDCAN_CCCR_CCE);
            pending_mask = 0U;
            pending_active = pending_standby = cancelling = pitch_link.pending = 0U;
            pitch_can.LatestTxFifoQRequest = 0U;
            recovery_flushed = 1U;
        }
        if (now - recovery_started_ms < PITCH_LINK_RECOVERY_BACKOFF_MS ||
            (pitch_link_diag.recovery_attempts && now - pitch_link_diag.last_recovery_ms < PITCH_LINK_RECOVERY_RETRY_MS)) return;
        if (!(pitch_can.Instance->CCCR & FDCAN_CCCR_CCE) || pitch_can.Instance->TXBRP) return;
        pitch_link_diag.recovery_state = PITCH_LINK_CAN_SYNCHRONIZING;
        pitch_link_diag.last_recovery_ms = now;
        ++pitch_link_diag.recovery_attempts;
        CLEAR_BIT(pitch_can.Instance->CCCR, FDCAN_CCCR_INIT);
    } else if (pitch_link_diag.recovery_state == PITCH_LINK_CAN_SYNCHRONIZING &&
        !recovery_timeout_recorded && now - pitch_link_diag.last_recovery_ms >= PITCH_LINK_RECOVERY_TIMEOUT_MS) {
        ++pitch_link_diag.recovery_timeouts;
        recovery_timeout_recorded = 1U;
    }
}

/** @brief 构造机械/陀螺仪/小陀螺同一整车许可下的俯仰输入，抬头为正。 */
static board_pitch_command_t input_command(uint32_t now)
{
    const rc_sensor_info_t *rc = rc_sensor.info;
    board_pitch_command_t cmd = {0};
    int gyro = Car_IsGyroMode(car.car_mode);
    float rate_limit = gyro ? BOARD_PITCH_MAX_RATE_DPS : 20.0f;
    if (car.selected_mode == gyro_car || gyro) cmd.flags |= BOARD_PITCH_GYRO;
    if (pitch_link.standby_requested) {
        cmd.flags |= BOARD_PITCH_RESET;
        cmd.sequence = sequence;
        return cmd;
    }
    int neutral = abs(rc->ch0) <= RC_ARM_NEUTRAL && abs(rc->ch1) <= RC_ARM_NEUTRAL &&
        abs(rc->ch2) <= RC_ARM_NEUTRAL && abs(rc->ch3) <= RC_ARM_NEUTRAL;
    if (RC_Sensor_Online(now)) cmd.flags |= BOARD_PITCH_RC_ONLINE;
    if (car.car_ctrl == KEY_CTRL) {
        cmd.flags |= BOARD_PITCH_KEY;
        if (rc->key_v & RC_KEY_CTRL) cmd.flags |= BOARD_PITCH_CTRL;
        if ((rc->key_v & RC_KEY_MOVEMENT) || rc->mouse_vx || rc->mouse_vy) neutral = 0;
        cmd.rate_dps = PITCH_LINK_MOUSE_DIRECTION * (gyro ? rc->mouse_y * GYRO_MOUSE_PITCH_GAIN : (float)rc->mouse_vy * PITCH_LINK_MOUSE_GAIN);
    } else if (abs(rc->ch1) > PITCH_LINK_RC_DEADZONE) {
        cmd.rate_dps = gyro ? (float)rc->ch1 * GYRO_RC_PITCH_GAIN : (float)rc->ch1 / PITCH_LINK_RC_MAX * 20.0f;
    }
    if (car.car_ctrl == RC_CTRL && gyro_control.turning) cmd.rate_dps = 0.0f;
    if (neutral) cmd.flags |= BOARD_PITCH_NEUTRAL;
    if ((gyro || car.car_mode == mec_car) && mec_output_enable == 1U && RC_Sensor_Online(now)) cmd.flags |= BOARD_PITCH_ACTIVE;
    if (!(cmd.flags & BOARD_PITCH_ACTIVE) || ((cmd.flags & BOARD_PITCH_CTRL) && !gyro)) cmd.rate_dps = 0.0f;
    cmd.rate_dps = fmaxf(-rate_limit, fminf(rate_limit, cmd.rate_dps));
    cmd.sequence = sequence;
    return cmd;
}

/** @brief 精确校验经典标准8字节摘要后更新时间与在线状态。 */
void Pitch_Link_Receive(uint32_t now)
{
    if (pitch_link.init_ok && (pitch_can.State == HAL_FDCAN_STATE_ERROR || pitch_can.State == HAL_FDCAN_STATE_READY)) {
        pitch_link.init_ok = 0U;
        ++pitch_link_diag.init_errors;
        init_retry_ms = now - PITCH_LINK_RECOVERY_RETRY_MS;
    }
    if (!pitch_link.init_ok) {
        if (now - init_retry_ms >= PITCH_LINK_RECOVERY_RETRY_MS) {
            ++pitch_link_diag.init_retries;
            retry_init();
        }
        return;
    }
    if (tx_failure_active && pitch_link_diag.recovery_state == PITCH_LINK_CAN_RUNNING) {
        pitch_link_diag.failure_duration_ms = now - tx_failure_started_ms;
        if (pitch_link_diag.failure_duration_ms >= PITCH_LINK_RECOVERY_RETRY_MS)
            begin_recovery(now, pitch_can.Instance->PSR);
    }
    uint32_t psr = pitch_can.Instance->PSR;
    uint32_t cccr = pitch_can.Instance->CCCR;
    if (pitch_link_diag.recovery_state == PITCH_LINK_CAN_SYNCHRONIZING &&
        !(cccr & FDCAN_CCCR_INIT) &&
        (psr & FDCAN_PSR_LEC) == FDCAN_PROTOCOL_ERROR_BIT0) {
        ++pitch_link_diag.recovery_progress_samples;
        retain_can_error(now, psr & ~FDCAN_PSR_LEC, NULL);
    } else retain_can_error(now, psr, NULL);
    if ((psr & FDCAN_PSR_BO) && (pitch_link_diag.recovery_state == PITCH_LINK_CAN_RUNNING ||
        (pitch_link_diag.recovery_state == PITCH_LINK_CAN_SYNCHRONIZING && (cccr & FDCAN_CCCR_INIT))))
        begin_recovery(now, psr);
    if (pitch_link_diag.recovery_state == PITCH_LINK_CAN_SYNCHRONIZING &&
        !(psr & FDCAN_PSR_BO) && !(cccr & FDCAN_CCCR_INIT)) {
        pitch_link_diag.recovery_state = PITCH_LINK_CAN_RUNNING;
        pitch_link_diag.last_recovered_ms = now;
        ++pitch_link_diag.recoveries;
        send_ms = now - PITCH_LINK_PERIOD_MS;
    }
    if (pitch_link_diag.recovery_state != PITCH_LINK_CAN_RUNNING) {
        pitch_link.online = 0U;
        poll_recovery(now);
        return;
    }
    board_imu_status_t imu;
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    while (HAL_FDCAN_GetRxFifoFillLevel(&pitch_can, FDCAN_RX_FIFO0)) {
        if (HAL_FDCAN_GetRxMessage(&pitch_can, FDCAN_RX_FIFO0, &header, data) != HAL_OK) { ++pitch_link_diag.read_errors; break; }
        ++pitch_link_diag.raw_frames;
        pitch_link_diag.last_ms = now; pitch_link_diag.last_id = header.Identifier;
        memcpy(pitch_link_diag.last_raw, data, 8U);
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength != FDCAN_DLC_BYTES_8) {
            ++pitch_link.bad_frames; ++pitch_link_diag.format_errors; continue;
        }
        if (header.Identifier == BOARD_PITCH_STATUS_ID) ++pitch_link_diag.status_frames;
        if (header.Identifier == BOARD_IMU_STATUS_ID) ++pitch_link_diag.imu_frames;
        if (Shoot_Receive(header.Identifier, data, 8U, now)) continue;
        if (header.Identifier == BOARD_PITCH_STATUS_ID && Board_Pitch_DecodeStatus(data, 8U, &pitch_link.status)) {
            ++pitch_link.frames; pitch_link.last_ms = now;
            status_lease_valid = 1U;
            if (pitch_link.standby_requested && pitch_link.standby_confirmed)
                pitch_link.standby_ack = pitch_link.status.state == BOARD_PITCH_STANDBY_STATE && !pitch_link.status.motor_state;
        } else if (header.Identifier == BOARD_IMU_STATUS_ID && Board_Imu_Decode(data, 8U, &imu)) {
            pitch_link_diag.last_imu = imu;
            Gyro_ReceiveImu(&imu, now);
        } else { ++pitch_link.bad_frames; ++pitch_link_diag.decode_errors; }
    }
    pitch_link_diag.status_age_ms = now - pitch_link.last_ms;
    if (!pitch_link.frames || pitch_link_diag.status_age_ms > PITCH_LINK_TIMEOUT_MS) ++pitch_link_diag.status_timeouts;
    pitch_link.online = status_lease_valid && pitch_link.frames && now - pitch_link.last_ms <= PITCH_LINK_TIMEOUT_MS;
}

/* Exported functions --------------------------------------------------------*/
/** @brief 延用CAN2引脚与1Mbps，并避开已有CAN1的共享RAM配置。 */
static void retry_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    FDCAN_FilterTypeDef filter = {0};
    init_retry_ms = HAL_GetTick();
    __HAL_RCC_FDCAN_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &gpio);
    pitch_can.Instance = FDCAN2;
    pitch_can.Init.FrameFormat = FDCAN_FRAME_CLASSIC; pitch_can.Init.Mode = FDCAN_MODE_NORMAL;
    pitch_can.Init.AutoRetransmission = DISABLE; pitch_can.Init.TransmitPause = DISABLE;
    pitch_can.Init.ProtocolException = DISABLE;
    pitch_can.Init.NominalPrescaler = 4U; pitch_can.Init.NominalSyncJumpWidth = 1U;
    pitch_can.Init.NominalTimeSeg1 = 13U; pitch_can.Init.NominalTimeSeg2 = 2U;
    pitch_can.Init.DataPrescaler = 4U; pitch_can.Init.DataSyncJumpWidth = 1U;
    pitch_can.Init.DataTimeSeg1 = 13U; pitch_can.Init.DataTimeSeg2 = 2U;
    pitch_can.Init.MessageRAMOffset = PITCH_LINK_RAM_OFFSET;
    pitch_can.Init.StdFiltersNbr = 2U; pitch_can.Init.RxFifo0ElmtsNbr = 8U;
    pitch_can.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    pitch_can.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8; pitch_can.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    pitch_can.Init.TxFifoQueueElmtsNbr = 1U; pitch_can.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    pitch_can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&pitch_can) != HAL_OK) { ++pitch_link_diag.init_errors; return; }
    filter.IdType = FDCAN_STANDARD_ID; filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE; filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = BOARD_PITCH_STATUS_ID; filter.FilterID2 = BOARD_FRIC_STATUS_ID;
    if (HAL_FDCAN_ConfigFilter(&pitch_can, &filter) != HAL_OK) { ++pitch_link_diag.init_errors; return; }
    filter.FilterIndex = 1U; filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID1 = SHOOT_DIAL_FEEDBACK_ID; filter.FilterID2 = 0x7FFU;
    if (HAL_FDCAN_ConfigFilter(&pitch_can, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&pitch_can, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&pitch_can) != HAL_OK) { ++pitch_link_diag.init_errors; return; }
    pitch_link.init_ok = 1U; send_ms = HAL_GetTick();
    fric_send_ms = dial_send_ms = send_ms; pending_kind = 0U;
    pending_mask = 0U; cancelling = pending_active = 0U;
    pending_standby = bus_off_event = recovery_flushed = recovery_timeout_recorded = 0U;
    status_lease_valid = 0U;
    recovery_started_ms = 0U;
    pitch_link_diag.recovery_state = PITCH_LINK_CAN_RUNNING;
}

/** @brief 初次初始化清空统计，后续失败在收发任务持续重试且保留诊断。 */
void Pitch_Link_Init(void)
{
    tx_failure_active = 0U;
    queue_busy = 0U;
    sequence = 0U;
    fric_sequence = 0U;
    pitch_link = (pitch_link_t){0};
    pitch_link_diag = (pitch_link_diag_t){0};
    retry_init();
}

/** @brief 每次检测到Bus-Off只向控制任务报告一次，不清除硬件错误历史。 */
int Pitch_Link_TakeBusOff(void)
{
    int event = bus_off_event;
    bus_off_event = 0U;
    return event;
}

/** @brief 遥控离线期间持续请求上板待命，不等待其ACK作为重新在线的门槛。 */
void Pitch_Link_RequestStandby(uint32_t now)
{
    pitch_link.standby_requested = 1U;
    pitch_link.standby_confirmed = pitch_link.standby_ack = 0U;
    pending_standby = 0U;
    send_ms = now - PITCH_LINK_PERIOD_MS;
}

/** @brief 只检查本次待命请求及遥控重新在线，不使用摘要或TXOK作为软件门槛。 */
int Pitch_Link_StandbyReady(uint32_t now)
{
    return pitch_link.standby_requested && RC_Sensor_Online(now);
}

/** @brief 结束已确认请求，保留递增发送序号。 */
void Pitch_Link_FinishStandby(void)
{
    pitch_link.standby_requested = 0U;
}

/** @brief 遥控离线/退出时取消旧许可帧，CAN2故障不阻塞CAN1控制。 */
void Pitch_Link_Update(uint32_t now)
{
    uint8_t data[8];
    FDCAN_TxHeaderTypeDef header = {0};
    Pitch_Link_Receive(now);
    if (!pitch_link.init_ok) return;
    pitch_link.command = input_command(now);
    board_fric_command_t fric_command = Shoot_FricCommand(now, fric_sequence);
    if (pitch_link_diag.recovery_state != PITCH_LINK_CAN_RUNNING || pitch_link_diag.bus_off ||
        (pitch_can.Instance->CCCR & FDCAN_CCCR_INIT)) return;
    if (pending_mask) {
        if (HAL_FDCAN_IsTxBufferMessagePending(&pitch_can, pending_mask)) {
            int stale = pending_active && (pending_kind == 0U ? !(pitch_link.command.flags & BOARD_PITCH_ACTIVE) :
                !fric_command.enabled || (pending_kind == 2U && !shoot.feedback_online));
            if (!cancelling && (stale ||
                now - pending_ms >= PITCH_LINK_TX_TIMEOUT_MS)) {
                if (HAL_FDCAN_AbortTxRequest(&pitch_can, pending_mask) == HAL_OK) { cancelling = 1U; ++pitch_link.aborted; }
                else { ++pitch_link.errors; capture_failure(now); }
            }
            if (now - pending_ms >= PITCH_LINK_RECOVERY_RETRY_MS) begin_recovery(now, pitch_can.Instance->PSR);
            return;
        }
        if (!cancelling && (pitch_can.Instance->TXBTO & pending_mask)) {
            if (pending_kind == 1U) ++shoot.fric_confirmed;
            else if (pending_kind == 2U) ++shoot.dial_confirmed;
            else ++pitch_link.confirmed;
            tx_failure_active = 0U;
            pitch_link_diag.consecutive_tx_errors = 0U;
            if (pending_standby && pitch_link.standby_requested) pitch_link.standby_confirmed = 1U;
        }
        else {
            if (pending_kind == 1U) ++shoot.fric_errors;
            else if (pending_kind == 2U) ++shoot.dial_errors;
            else ++pitch_link.errors;
            if (!cancelling || pending_kind == 0U) capture_failure(now);
        }
        pending_mask = 0U; pitch_link.pending = cancelling = 0U;
    }
    int pitch_due = now - send_ms >= PITCH_LINK_PERIOD_MS;
    int fric_due = (shoot.input_seen || shoot.fric_queued) && now - fric_send_ms >= BOARD_FRIC_PERIOD_MS;
    int dial_due = shoot.feedback_frames && now - dial_send_ms >= SHOOT_DIAL_PERIOD_MS;
    if (!pitch_due && !fric_due && !dial_due) return;
    if (!HAL_FDCAN_GetTxFifoFreeLevel(&pitch_can)) {
        if (!queue_busy) { queue_busy = 1U; queue_busy_ms = now; }
        if (now - queue_busy_ms >= PITCH_LINK_RECOVERY_RETRY_MS) {
            ++pitch_link.errors; capture_failure(now);
            begin_recovery(now, pitch_can.Instance->PSR);
        }
        return;
    }
    queue_busy = 0U;
    if (pitch_due) {
        if (!Board_Pitch_EncodeCommand(&pitch_link.command, data)) return;
        pending_kind = 0U; header.Identifier = BOARD_PITCH_COMMAND_ID; send_ms = now;
    } else if (fric_due) {
        if (!Board_Fric_EncodeCommand(&fric_command, data)) return;
        pending_kind = 1U; header.Identifier = BOARD_FRIC_COMMAND_ID; fric_send_ms = now;
    } else {
        Shoot_PackCurrent(data); pending_kind = 2U; header.Identifier = SHOOT_DIAL_CURRENT_ID; dial_send_ms = now;
    }
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME; header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE; header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN; header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&pitch_can, &header, data) == HAL_OK) {
        pending_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&pitch_can);
        pitch_link.pending = 1U; pending_ms = now;
        pending_active = pending_kind ? fric_command.enabled : (pitch_link.command.flags & BOARD_PITCH_ACTIVE);
        pending_standby = pending_kind == 0U && (pitch_link.command.flags & BOARD_PITCH_RESET) != 0U;
        if (pending_kind == 1U) { ++shoot.fric_queued; ++fric_sequence; }
        else if (pending_kind == 2U) ++shoot.dial_queued;
        else { ++pitch_link.queued; ++sequence; }
    } else {
        if (pending_kind == 1U) ++shoot.fric_errors;
        else if (pending_kind == 2U) ++shoot.dial_errors;
        else ++pitch_link.errors;
        capture_failure(now);
    }
}
