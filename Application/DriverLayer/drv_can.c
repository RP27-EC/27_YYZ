/**
 * @file    drv_can.c
 * @brief   H723四轮与遥控Yaw共享CAN发送，保留独立测试及优先清零。
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "drv_can.h"
#include "drv_status.h"
#include "motor.h"
#include "carctrl.h"
#include "yaw_probe.h"
#include "gimbal.h"
#include "rc_sensor.h"
#include <stdlib.h>

/* Private macro -------------------------------------------------------------*/
#define CAN_STANDARD_ID_MASK 0x7FFU /**< 标准CAN帧11位编号的精确匹配掩码。 */

/* Private variables ---------------------------------------------------------*/
static int previous_tx_ok = 1;         /**< 上一次底盘发送入队是否成功。 */
static uint32_t probe_tx_mask;         /**< 当前Yaw帧占用的发送缓冲区位。 */
static uint32_t probe_tx_started_ms;   /**< 当前Yaw帧入队时刻，毫秒。 */
static uint8_t probe_abort_requested; /**< 当前Yaw帧是否已请求取消。 */
static uint8_t yaw_tx_motion;         /**< 当前发送槽为运动帧时为1，否则为只读查询。 */
static int16_t yaw_tx_current;        /**< 当前运动帧原始电流，用于撤销和清零统计。 */
static uint8_t yaw_tx_command;        /**< 当前控制帧命令，区分电流与状态1查询。 */

/* Exported variables --------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;           /**< 共享底盘/Yaw的CAN1外设句柄。 */

/* Private functions ---------------------------------------------------------*/
/** @brief 构造指定编号的经典CAN标准8字节数据帧头。 */
static FDCAN_TxHeaderTypeDef classic_header(uint32_t identifier)
{
    return (FDCAN_TxHeaderTypeDef){
        .Identifier = identifier, .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME, .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS
    };
}

/** @brief 确认底盘为停止模式且四轮电流命令均为零。 */
static int chassis_stopped(void)
{
    if (car.car_mode != sleep_car) { return 0; }
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) {
        if (rm_motor[i].base_info.motor_out != 0) { return 0; }
    }
    return 1;
}

/** @brief 按入口模式记录条件；遥控Yaw允许四轮运动，独立测试要求底盘停止。 */
static uint32_t interlock_blocks(uint32_t now, int can_ready)
{
    const rc_sensor_info_t *rc = rc_sensor.info;
    uint32_t blocks = 0;
    if (Gimbal_Yaw_RemoteMode()) {
        if (mec_output_enable != 1U) { blocks |= GIMBAL_BLOCK_CHASSIS_ENABLE; }
        if (car.car_mode != mec_car) { blocks |= GIMBAL_BLOCK_CHASSIS_MODE; }
        if (!RC_Sensor_Online(now)) { blocks |= GIMBAL_BLOCK_REMOTE; }
        if (rc->s2.value != RC_SW_MID) { blocks |= GIMBAL_BLOCK_SWITCH; }
        if (!can_ready) { blocks |= GIMBAL_BLOCK_CAN; }
        return blocks;
    }
    if (mec_output_enable != 0) { blocks |= GIMBAL_BLOCK_CHASSIS_ENABLE; }
    if (car.car_mode != sleep_car) { blocks |= GIMBAL_BLOCK_CHASSIS_MODE; }
    for (unsigned i = 0; i < CHAS_MOTOR_COUNT; ++i) {
        if (rm_motor[i].base_info.motor_out != 0) { blocks |= GIMBAL_BLOCK_WHEEL_OUTPUT; }
    }
    if (!RC_Sensor_Online(now)) { blocks |= GIMBAL_BLOCK_REMOTE; }
    if (rc->s2.value != RC_SW_UP && rc->s2.value != RC_SW_DOWN) { blocks |= GIMBAL_BLOCK_SWITCH; }
    if (abs(rc->ch0) > RC_ARM_NEUTRAL || abs(rc->ch1) > RC_ARM_NEUTRAL ||
        abs(rc->ch2) > RC_ARM_NEUTRAL || abs(rc->ch3) > RC_ARM_NEUTRAL) { blocks |= GIMBAL_BLOCK_STICKS; }
    if (rc->key_v & RC_KEY_MOVEMENT) { blocks |= GIMBAL_BLOCK_KEYS; }
    if (!can_ready) { blocks |= GIMBAL_BLOCK_CAN; }
    return blocks;
}

/** @brief 跟踪Yaw发送完成/取消，退出时优先撤销未发送的非零运动帧。 */
static int query_pending(uint32_t now_ms, int allowed)
{
    if (!probe_tx_mask) { return 0; }
    if (!HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, probe_tx_mask)) {
        int success = (hfdcan1.Instance->TXBTO & probe_tx_mask) != 0;
        if (yaw_tx_motion) {
            if (success || !probe_abort_requested) { Gimbal_Yaw_TxComplete(yaw_tx_current, yaw_tx_command, success); }
            gimbal_yaw.tx_pending = 0;
        } else {
            if (success) { ++yaw_probe.tx_confirmed; }
            else if (!probe_abort_requested) { ++yaw_probe.tx_errors; }
            yaw_probe.tx_pending = 0;
        }
        probe_tx_mask = 0;
        probe_abort_requested = 0;
        return 0;
    }
    int timed_out = (uint32_t)(now_ms - probe_tx_started_ms) >= YAW_PROBE_TX_TIMEOUT_MS;
    int revoke = yaw_tx_motion ? (yaw_tx_command != GIMBAL_YAW_TORQUE_COMMAND || yaw_tx_current != 0) &&
        Gimbal_Yaw_NeedsZero() : !allowed;
    if ((revoke || timed_out) && !probe_abort_requested) {
        if (HAL_FDCAN_AbortTxRequest(&hfdcan1, probe_tx_mask) == HAL_OK) {
            probe_abort_requested = 1;
            if (yaw_tx_motion) { if (timed_out) { Gimbal_Yaw_TxError(); } }
            else {
                ++yaw_probe.tx_aborted;
                if (timed_out) { ++yaw_probe.tx_errors; }
            }
        } else {
            if (yaw_tx_motion) { Gimbal_Yaw_TxError(); }
            else { ++yaw_probe.tx_errors; }
        }
    }
    return 1;
}

/* Exported functions --------------------------------------------------------*/
/** @brief 配置CAN1的PD0/PD1引脚和接收中断。 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *can)
{
    if (can->Instance != FDCAN1) { return; }
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef pin = {0};
    pin.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    pin.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &pin);
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

/** @brief 初始化1Mbit/s CAN1，接收四轮和0x142，保持单槽无自动重发。 */
void Drv_CAN_Init(void)
{
    previous_tx_ok = 1;
    probe_tx_mask = 0;
    probe_abort_requested = 0;
    yaw_tx_motion = 0;
    yaw_tx_current = 0;
    yaw_tx_command = 0;
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.NominalPrescaler = 5;
    hfdcan1.Init.NominalSyncJumpWidth = 2;
    hfdcan1.Init.NominalTimeSeg1 = 13;
    hfdcan1.Init.NominalTimeSeg2 = 2;
    hfdcan1.Init.DataPrescaler = 5;
    hfdcan1.Init.DataSyncJumpWidth = 2;
    hfdcan1.Init.DataTimeSeg1 = 13;
    hfdcan1.Init.DataTimeSeg2 = 2;
    hfdcan1.Init.StdFiltersNbr = 2;
    hfdcan1.Init.RxFifo0ElmtsNbr = 16;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) { Error_Handler(); }
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x201;
    filter.FilterID2 = 0x204;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) { Error_Handler(); }
    filter.FilterIndex = 1;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID1 = YAW_PROBE_CAN_ID;
    filter.FilterID2 = CAN_STANDARD_ID_MASK;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan1) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
}

/** @brief 将CAN1中断交给HAL处理。 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/** @brief 校验接收帧格式，分别缓存Yaw或四轮反馈。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *can, uint32_t flags)
{
    (void)flags;
    if (can != &hfdcan1) { return; }
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    while (HAL_FDCAN_GetRxFifoFillLevel(can, FDCAN_RX_FIFO0)) {
        if (HAL_FDCAN_GetRxMessage(can, FDCAN_RX_FIFO0, &header, data) != HAL_OK) { break; }
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength != FDCAN_DLC_BYTES_8) { continue; }
        if (header.Identifier == YAW_PROBE_CAN_ID) {
            Yaw_Probe_Receive(header.Identifier, data, 8, HAL_GetTick());
        } else { Motor_Receive(header.Identifier, data, 8, HAL_GetTick()); }
    }
}

/** @brief 更新CAN故障观察量，并返回底盘通信是否正常。 */
int Drv_CAN_Ready(void)
{
    FDCAN_ProtocolStatusTypeDef status = {0};
    int can_ok = HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) == HAL_OK;
    mec_io.can_bus_off = status.BusOff;
    mec_io.can_error_passive = status.ErrorPassive;
    mec_io.can_last_error = status.LastErrorCode;
    return can_ok && !status.BusOff && !status.ErrorPassive && previous_tx_ok;
}

/** @brief 调度Yaw并优先退出清零，整车模式在Yaw限频空档让出四轮发送。 */
int Drv_CAN_PollYaw(uint32_t now_ms)
{
    int can_ready = Drv_CAN_Ready();
    gimbal_yaw.interlock_block_reason = interlock_blocks(now_ms, can_ready);
    Gimbal_Yaw_Update(now_ms, gimbal_yaw.interlock_block_reason == 0);
    int shared = Gimbal_Yaw_RemoteMode();
    int allowed = Yaw_Probe_Allowed(shared || mec_output_enable == 0, shared || chassis_stopped(), can_ready);
    if (Gimbal_Yaw_OwnsBus()) { allowed = 0; }
    if (query_pending(now_ms, allowed)) { return 1; }
    uint8_t data[8];
    int motion = Gimbal_Yaw_OwnsBus();
    if (motion) {
        if (!Gimbal_Yaw_MakeCommand(now_ms, data)) { return shared ? 0 : 1; }
    } else if (!allowed || !Yaw_Probe_MakeQuery(now_ms, data)) { return 0; }
    if (hfdcan1.Instance->TXBRP) {
        if (shared && motion && !Gimbal_Yaw_NeedsZero()) { return 1; }
        if (motion) {
            if (HAL_FDCAN_AbortTxRequest(&hfdcan1, hfdcan1.Instance->TXBRP) != HAL_OK) { Gimbal_Yaw_TxError(); }
        } else { ++yaw_probe.tx_busy_skips; }
        return motion;
    }
    FDCAN_TxHeaderTypeDef tx = classic_header(YAW_PROBE_CAN_ID);
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data) != HAL_OK) {
        if (motion) { Gimbal_Yaw_TxError(); }
        else { ++yaw_probe.tx_errors; }
        return motion;
    }
    probe_tx_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
    probe_tx_started_ms = now_ms;
    probe_abort_requested = 0;
    yaw_tx_motion = (uint8_t)motion;
    if (motion) {
        yaw_tx_current = (int16_t)((uint16_t)data[4] | (uint16_t)data[5] << 8);
        yaw_tx_command = data[0];
        gimbal_yaw.tx_pending = 1;
        Gimbal_Yaw_Queued(now_ms, yaw_tx_current, yaw_tx_command);
    } else {
        yaw_probe.tx_pending = 1;
        Yaw_Probe_QueryQueued(now_ms);
    }
    return 1;
}

/** @brief 发送四轮0x200电流报文，保持无反馈不发和无积压策略。 */
void CAN_Send(void)
{
    if (probe_tx_mask || (Gimbal_Yaw_OwnsBus() && !Gimbal_Yaw_RemoteMode()) || !Motor_HasFeedback()) { return; }
    if (hfdcan1.Instance->TXBRP) {
        HAL_FDCAN_AbortTxRequest(&hfdcan1, hfdcan1.Instance->TXBRP);
        previous_tx_ok = 0;
        ++mec_io.tx_errors;
        return;
    }
    uint8_t data[8];
    Motor_PackCurrent(data);
    FDCAN_TxHeaderTypeDef tx = classic_header(0x200);
    previous_tx_ok = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data) == HAL_OK;
    if (previous_tx_ok) { ++mec_io.tx_queued; }
    else { ++mec_io.tx_errors; }
}
