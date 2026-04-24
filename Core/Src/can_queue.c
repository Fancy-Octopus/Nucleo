/*
 * can_queue.c
 *
 *  Created on: Apr 22, 2026
 *      Author: gruetzmacherg
 */


#include "can_queue.h"

static CanQueue_t        canQueue;
static volatile uint8_t  txInProgress = 0;

void CanQueue_Init(void)
{
    canQueue.head  = 0;
    canQueue.tail  = 0;
    canQueue.count = 0;
    txInProgress   = 0;
}

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

