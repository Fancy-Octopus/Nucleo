/*
 * can_queue.c
 *
 *  Created on: Apr 22, 2026
 *      Author: gruetzmacherg
 */

#include "can_queue.h"

static CanQueue_t       canQueue;
static volatile uint8_t txInProgress = 0;

/* --------------------------------------------------------------------------
 * Hardware initialisation
 * --------------------------------------------------------------------------
 * Call once per FDCAN interface after HAL_FDCAN_Init() has already been run
 * (either by MX generated code or your own ALT init function).
 *
 * Example usage in main.c:
 *   ALT_MX_FDCAN1_Init(&hfdcan1);
 *   CAN_HardwareInit(&hfdcan1, FDCAN1_IT0_IRQn, 5);
 *   CanQueue_Init();
 * -------------------------------------------------------------------------- */
void CAN_HardwareInit(FDCAN_HandleTypeDef *hfdcan, IRQn_Type it0IrqN, uint32_t irqPriority)
{
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0);

    HAL_NVIC_SetPriority(it0IrqN, irqPriority, 0);
    HAL_NVIC_EnableIRQ(it0IrqN);
}

void CAN_HardwareStart(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_FDCAN_Start(hfdcan);
}

/* --------------------------------------------------------------------------
 * IRQ handlers - one per physical FDCAN interface in use.
 * HAL_FDCAN_IRQHandler dispatches to HAL_FDCAN_TxFifoEmptyCallback below.
 * -------------------------------------------------------------------------- */
void FDCAN1_IT0_IRQHandler(void)
{
    extern FDCAN_HandleTypeDef hfdcan1;
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/* Uncomment if a second interface is used:
void FDCAN2_IT0_IRQHandler(void)
{
    extern FDCAN_HandleTypeDef hfdcan2;
    HAL_FDCAN_IRQHandler(&hfdcan2);
}
*/

/* Called by the HAL after the IRQ is serviced */
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    CanQueue_TxNext();
}

/* --------------------------------------------------------------------------
 * Queue management
 * -------------------------------------------------------------------------- */
void CanQueue_Init(void)
{
    canQueue.head  = 0;
    canQueue.tail  = 0;
    canQueue.count = 0;
    txInProgress   = 0;
}

/* Private - direct HAL transmit, never called outside this file */
static void CAN_TransmitRaw(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier          = id;
    TxHeader.IdType              = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = len;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
}

uint8_t CanQueue_Enqueue(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len)
{
    if(canQueue.count >= CAN_QUEUE_MAX_MESSAGES) return 0;
    if(len > CAN_MAX_DATA_BYTES) return 0;

    CanMessage_t *msg = &canQueue.buffer[canQueue.tail];
    msg->hfdcan = hfdcan;
    msg->id     = id;
    msg->len    = len;
    for(uint8_t i = 0; i < len; i++) msg->data[i] = data[i];

    canQueue.tail = (canQueue.tail + 1) % CAN_QUEUE_MAX_MESSAGES;
    canQueue.count++;

    if(!txInProgress) CanQueue_TxNext();

    return 0xFF;
}

void CanQueue_TxNext(void)
{
    if(canQueue.count == 0)
    {
        txInProgress = 0;
        return;
    }

    CanMessage_t *msg = &canQueue.buffer[canQueue.head];

    if(HAL_FDCAN_GetTxFifoFreeLevel(msg->hfdcan) == 0) return;

    txInProgress = 1;

    CAN_TransmitRaw(msg->hfdcan, msg->id, msg->data, msg->len);

    canQueue.head = (canQueue.head + 1) % CAN_QUEUE_MAX_MESSAGES;
    canQueue.count--;
}

uint8_t CanQueue_IsBusy(void)
{
    return (canQueue.count > 0 || txInProgress);
}

void CanQueue_Poll(void)
{
    if(canQueue.count == 0) return;

    CanMessage_t *msg = &canQueue.buffer[canQueue.head];

    if(HAL_FDCAN_GetTxFifoFreeLevel(msg->hfdcan) > 0)
    {
        CanQueue_TxNext();
    }
}

/* --------------------------------------------------------------------------
 * Public transmit entry point
 * All CAN transmission in the project must go through here.
 * -------------------------------------------------------------------------- */
void can_transmit_eid(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len)
{
    CanQueue_Enqueue(hfdcan, id, data, len);
}
