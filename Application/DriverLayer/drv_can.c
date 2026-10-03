#include "main.h"
#include "drv_can.h"
#include "drv_status.h"
#include "motor.h"
FDCAN_HandleTypeDef hfdcan1;
static int previous_tx_ok = 1;
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


void Drv_CAN_Init(void)
{
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE; /* Never queue/replay old motor commands. */
    hfdcan1.Init.NominalPrescaler = 5;
    hfdcan1.Init.NominalSyncJumpWidth = 2;
    hfdcan1.Init.NominalTimeSeg1 = 13;
    hfdcan1.Init.NominalTimeSeg2 = 2; /* 80 MHz / 5 / (1+13+2) = 1 Mbit/s. */
    hfdcan1.Init.DataPrescaler = 5;
    hfdcan1.Init.DataSyncJumpWidth = 2;
    hfdcan1.Init.DataTimeSeg1 = 13;
    hfdcan1.Init.DataTimeSeg2 = 2;
    hfdcan1.Init.StdFiltersNbr = 1;
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
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan1) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
}

void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan1); }
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
        Motor_Receive(header.Identifier, data, 8, HAL_GetTick());
    }
}
int Drv_CAN_Ready(void)
{
    FDCAN_ProtocolStatusTypeDef status = {0};
    int can_ok = HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) == HAL_OK;
    mec_io.can_bus_off = status.BusOff;
    mec_io.can_error_passive = status.ErrorPassive;
    mec_io.can_last_error = status.LastErrorCode;
    return can_ok && !status.BusOff && !status.ErrorPassive && previous_tx_ok;
}
void CAN_Send(void)
{
    /* Listen until the first feedback; preserve the existing no-backlog policy. */
    if (!Motor_HasFeedback()) { return; }
    /* No backlog: cancel a previous pending frame before accepting another. */
    if (hfdcan1.Instance->TXBRP) {
        HAL_FDCAN_AbortTxRequest(&hfdcan1, hfdcan1.Instance->TXBRP);
        previous_tx_ok = 0;
        ++mec_io.tx_errors;
        return;
    }
    uint8_t data[8];
    Motor_PackCurrent(data);
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = 0x200;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    previous_tx_ok = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data) == HAL_OK;
    if (previous_tx_ok) { ++mec_io.tx_queued; }
    else { ++mec_io.tx_errors; }
}
