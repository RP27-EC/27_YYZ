/**
 * @file    drv_pitch_link.c
 * @brief   CAN2轮询板间Pitch帧，不占用底盘与Yaw的CAN1发送槽。
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_pitch_link.h"
#include "pitch_link_config.h"
#include "main.h"
#include "rc_sensor.h"
#include "carctrl.h"
#include "chassis_config.h"
#include <math.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
static FDCAN_HandleTypeDef pitch_can; /**< 独立CAN2句柄；接收由任务轮询。 */
static uint32_t send_ms, pending_ms, pending_mask; /**< 调度时刻、入队时刻与发送槽位。 */
static uint8_t sequence, cancelling, pending_active; /**< 命令序号与过期许可取消状态。 */
/* Exported variables --------------------------------------------------------*/
pitch_link_t pitch_link; /**< 两板连接统计及协议观察对象。 */

/* Private functions ---------------------------------------------------------*/
/** @brief 构造底盘同一机械许可下的俯仰输入，抬头为正。 */
static board_pitch_command_t input_command(uint32_t now)
{
    const rc_sensor_info_t *rc = rc_sensor.info;
    board_pitch_command_t cmd = {0};
    int neutral = abs(rc->ch0) <= RC_ARM_NEUTRAL && abs(rc->ch1) <= RC_ARM_NEUTRAL &&
        abs(rc->ch2) <= RC_ARM_NEUTRAL && abs(rc->ch3) <= RC_ARM_NEUTRAL;
    if (RC_Sensor_Online(now)) cmd.flags |= BOARD_PITCH_RC_ONLINE;
    if (car.car_ctrl == KEY_CTRL) {
        cmd.flags |= BOARD_PITCH_KEY;
        if (rc->key_v & RC_KEY_CTRL) cmd.flags |= BOARD_PITCH_CTRL;
        if ((rc->key_v & RC_KEY_MOVEMENT) || rc->mouse_vx || rc->mouse_vy) neutral = 0;
        cmd.rate_dps = PITCH_LINK_MOUSE_DIRECTION * (float)rc->mouse_vy * PITCH_LINK_MOUSE_GAIN;
    } else if (abs(rc->ch1) > PITCH_LINK_RC_DEADZONE) {
        cmd.rate_dps = (float)rc->ch1 / PITCH_LINK_RC_MAX * BOARD_PITCH_MAX_RATE_DPS;
    }
    if (neutral) cmd.flags |= BOARD_PITCH_NEUTRAL;
    if (car.car_mode == mec_car && mec_output_enable == 1U && rc->s2.value == RC_SW_MID && RC_Sensor_Online(now)) cmd.flags |= BOARD_PITCH_ACTIVE;
    if (!(cmd.flags & BOARD_PITCH_ACTIVE) || (cmd.flags & BOARD_PITCH_CTRL)) cmd.rate_dps = 0.0f;
    cmd.rate_dps = fmaxf(-BOARD_PITCH_MAX_RATE_DPS, fminf(BOARD_PITCH_MAX_RATE_DPS, cmd.rate_dps));
    cmd.sequence = sequence;
    return cmd;
}

/** @brief 精确校验经典标准8字节摘要后更新时间与在线状态。 */
static void receive(uint32_t now)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(&pitch_can, FDCAN_RX_FIFO0)) {
        if (HAL_FDCAN_GetRxMessage(&pitch_can, FDCAN_RX_FIFO0, &header, data) != HAL_OK) break;
        if (header.Identifier == BOARD_PITCH_STATUS_ID && header.IdType == FDCAN_STANDARD_ID &&
            header.RxFrameType == FDCAN_DATA_FRAME && header.FDFormat == FDCAN_CLASSIC_CAN &&
            header.DataLength == FDCAN_DLC_BYTES_8 && Board_Pitch_DecodeStatus(data, 8U, &pitch_link.status)) {
            ++pitch_link.frames; pitch_link.last_ms = now;
        } else ++pitch_link.bad_frames;
    }
    pitch_link.online = pitch_link.frames && now - pitch_link.last_ms <= PITCH_LINK_TIMEOUT_MS;
}

/* Exported functions --------------------------------------------------------*/
/** @brief 延用国赛CAN2引脚与1Mbps，并避开已有CAN1的共享RAM配置。 */
void Pitch_Link_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    FDCAN_FilterTypeDef filter = {0};
    pitch_link = (pitch_link_t){0};
    __HAL_RCC_FDCAN_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &gpio);
    pitch_can.Instance = FDCAN2;
    pitch_can.Init.FrameFormat = FDCAN_FRAME_CLASSIC; pitch_can.Init.Mode = FDCAN_MODE_NORMAL;
    pitch_can.Init.AutoRetransmission = DISABLE; pitch_can.Init.TransmitPause = DISABLE;
    pitch_can.Init.ProtocolException = DISABLE;
    pitch_can.Init.NominalPrescaler = 5U; pitch_can.Init.NominalSyncJumpWidth = 1U;
    pitch_can.Init.NominalTimeSeg1 = 13U; pitch_can.Init.NominalTimeSeg2 = 2U;
    pitch_can.Init.DataPrescaler = 5U; pitch_can.Init.DataSyncJumpWidth = 1U;
    pitch_can.Init.DataTimeSeg1 = 13U; pitch_can.Init.DataTimeSeg2 = 2U;
    pitch_can.Init.MessageRAMOffset = PITCH_LINK_RAM_OFFSET;
    pitch_can.Init.StdFiltersNbr = 1U; pitch_can.Init.RxFifo0ElmtsNbr = 8U;
    pitch_can.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    pitch_can.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8; pitch_can.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    pitch_can.Init.TxFifoQueueElmtsNbr = 1U; pitch_can.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    pitch_can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&pitch_can) != HAL_OK) return;
    filter.IdType = FDCAN_STANDARD_ID; filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK; filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = BOARD_PITCH_STATUS_ID; filter.FilterID2 = 0x7FFU;
    if (HAL_FDCAN_ConfigFilter(&pitch_can, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&pitch_can, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&pitch_can) != HAL_OK) return;
    pitch_link.init_ok = 1U; send_ms = HAL_GetTick();
    pending_mask = 0U; sequence = cancelling = pending_active = 0U;
}

/** @brief 遥控离线/退出时取消旧许可帧，CAN2故障不阻塞CAN1控制。 */
void Pitch_Link_Update(uint32_t now)
{
    uint8_t data[8];
    FDCAN_TxHeaderTypeDef header = {0};
    if (!pitch_link.init_ok) return;
    receive(now);
    pitch_link.command = input_command(now);
    if (pending_mask) {
        if (HAL_FDCAN_IsTxBufferMessagePending(&pitch_can, pending_mask)) {
            if (!cancelling && ((pending_active && !(pitch_link.command.flags & BOARD_PITCH_ACTIVE)) ||
                now - pending_ms >= PITCH_LINK_TX_TIMEOUT_MS)) {
                if (HAL_FDCAN_AbortTxRequest(&pitch_can, pending_mask) == HAL_OK) { cancelling = 1U; ++pitch_link.aborted; }
            }
            return;
        }
        if (!cancelling && (pitch_can.Instance->TXBTO & pending_mask)) ++pitch_link.confirmed;
        else ++pitch_link.errors;
        pending_mask = 0U; pitch_link.pending = cancelling = 0U;
    }
    if (now - send_ms < PITCH_LINK_PERIOD_MS || !HAL_FDCAN_GetTxFifoFreeLevel(&pitch_can)) return;
    if (!Board_Pitch_EncodeCommand(&pitch_link.command, data)) return;
    header.Identifier = BOARD_PITCH_COMMAND_ID; header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME; header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE; header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN; header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    send_ms = now;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&pitch_can, &header, data) == HAL_OK) {
        pending_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&pitch_can);
        pitch_link.pending = 1U; pending_ms = now;
        pending_active = pitch_link.command.flags & BOARD_PITCH_ACTIVE;
        ++pitch_link.queued; ++sequence;
    } else ++pitch_link.errors;
}
