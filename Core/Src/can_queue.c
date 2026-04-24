/*
 * can_queue.c
 *
 *  Created on: Apr 22, 2026
 *      Author: gruetzmacherg
 */

#include "can_queue.h"

/* ==========================================================================
 * Module state
 * ========================================================================== */
static CanQueue_t            canQueue;
static volatile uint8_t      txInProgress      = 0;
static FDCAN_HandleTypeDef  *registeredHandle   = NULL;

/* Error tracking */
static volatile CAN_ErrorStats_t errorStats;
static volatile uint8_t          recoveryPending = 0;

/* Recovery uses its own schedule so it is independent of VescPoll timing */
static schedule_t                recoverySchedule;
static volatile uint8_t          recoveryScheduleReady = 0;

/* ==========================================================================
 * Internal helpers
 * ========================================================================== */

/* Samples the HAL error counters and accumulates deltas into errorStats.
 * Safe to call from both ISR and main context. */
static void CAN_SampleErrorCounters(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_ErrorCountersTypeDef counters;
    HAL_FDCAN_GetErrorCounters(hfdcan, &counters);

    if(counters.TxErrorCnt > errorStats.lastTxErrCnt)
        errorStats.txErrors += (counters.TxErrorCnt - errorStats.lastTxErrCnt);

    if(counters.RxErrorCnt > errorStats.lastRxErrCnt)
        errorStats.rxErrors += (counters.RxErrorCnt - errorStats.lastRxErrCnt);

    errorStats.lastTxErrCnt = counters.TxErrorCnt;
    errorStats.lastRxErrCnt = counters.RxErrorCnt;
}

/* Handles a detected Bus Off condition. Safe to call from either context. */
static void CAN_HandleBusOff(FDCAN_HandleTypeDef *hfdcan)
{
    errorStats.busState = CAN_BUS_OFF;
    errorStats.busOffCount++;
    txInProgress        = 0;
    recoveryPending     = 1;

    /* Arm the recovery schedule timer from this moment */
    SetScheduledTime(&recoverySchedule, CAN_BUS_OFF_RECOVERY_MS);
    recoveryScheduleReady = 1;

    CAN_BusOffCallback(hfdcan);
}

/* Handles a detected Error Passive condition. Safe to call from either context. */
static void CAN_HandleErrorPassive(FDCAN_HandleTypeDef *hfdcan)
{
    errorStats.busState = CAN_ERROR_PASS;
    errorStats.errorPassCount++;

    /* Error Passive does not require a stop/start recovery but is worth
     * logging. Transmission continues using passive error flags.
     * If your application needs to slow down or halt TX during Error
     * Passive, add that logic in CAN_ErrorPassiveCallback(). */
    CAN_ErrorPassiveCallback(hfdcan);
}

/* ==========================================================================
 * Hardware initialisation
 * ========================================================================== */
void CAN_HardwareInit(FDCAN_HandleTypeDef *hfdcan, IRQn_Type it0IrqN, uint32_t irqPriority)
{
    registeredHandle = hfdcan;

#if CAN_USE_INTERRUPTS
    HAL_NVIC_SetPriority(it0IrqN, irqPriority, 0);
    HAL_NVIC_EnableIRQ(it0IrqN);

    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0);

    HAL_FDCAN_ActivateNotification(hfdcan,
        FDCAN_IT_BUS_OFF            |
        FDCAN_IT_ERROR_PASSIVE      |
        FDCAN_IT_ERROR_WARNING      |
        FDCAN_IT_ARB_PROTOCOL_ERROR |
        FDCAN_IT_DATA_PROTOCOL_ERROR,
        0);
#endif
    /* When CAN_USE_INTERRUPTS=0 no NVIC or notifications are configured.
     * Error detection and queue draining are handled entirely by the
     * polling functions called from the main loop. */
}

void CAN_HardwareStart(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_FDCAN_Start(hfdcan);
}

/* ==========================================================================
 * IRQ handler and HAL callbacks
 * Compiled only when interrupt mode is enabled.
 * ========================================================================== */
#if CAN_USE_INTERRUPTS

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(registeredHandle);
}

/* Fired by HAL when the TX FIFO drains - chains the next queued message */
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    CanQueue_TxNext();
}

/* Fired by HAL on any error status change */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    CAN_SampleErrorCounters(hfdcan);

    if(ErrorStatusITs & FDCAN_IT_BUS_OFF)
    {
        CAN_HandleBusOff(hfdcan);
        /* Recovery is NOT performed here - CAN_BusRecoveryPoll() handles it
         * in main context to avoid blocking HAL calls inside the ISR. */
    }
    else if(ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE)
    {
        CAN_HandleErrorPassive(hfdcan);
    }
    else if(ErrorStatusITs & FDCAN_IT_ERROR_WARNING)
    {
        errorStats.busState = CAN_ERROR_WARN;
        errorStats.errorWarnCount++;
    }
}

#endif /* CAN_USE_INTERRUPTS */

/* ==========================================================================
 * Queue management
 * ========================================================================== */
void CanQueue_Init(void)
{
    canQueue.head         = 0;
    canQueue.tail         = 0;
    canQueue.count        = 0;
    txInProgress          = 0;
    recoveryPending       = 0;
    recoveryScheduleReady = 0;

    errorStats.txErrors       = 0;
    errorStats.rxErrors       = 0;
    errorStats.busOffCount    = 0;
    errorStats.errorWarnCount = 0;
    errorStats.errorPassCount = 0;
    errorStats.lastTxErrCnt   = 0;
    errorStats.lastRxErrCnt   = 0;
    errorStats.busState       = CAN_BUS_OK;
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
    if(errorStats.busState == CAN_BUS_OFF)      return 0;
    if(canQueue.count >= CAN_QUEUE_MAX_MESSAGES) return 0;
    if(len > CAN_MAX_DATA_BYTES)                 return 0;

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

/* Safe to call in both interrupt and polling modes. In polling mode this is
 * the sole mechanism that drains the queue. In interrupt mode it acts as a
 * safety drain in case the TX FIFO empty callback is missed. */
void CanQueue_Poll(void)
{
    if(errorStats.busState == CAN_BUS_OFF) return;
    if(canQueue.count == 0)                return;

    /* In polling mode also sample error counters here since there is no ISR */
#if !CAN_USE_INTERRUPTS
    if(registeredHandle != NULL)
    {
        CAN_SampleErrorCounters(registeredHandle);

        /* Check hardware PSR register for Bus Off and Error Passive flags */
        uint32_t psr = registeredHandle->Instance->PSR;

        if(psr & FDCAN_PSR_BO)
        {
            if(errorStats.busState != CAN_BUS_OFF)
                CAN_HandleBusOff(registeredHandle);
            return;
        }
        else if(psr & FDCAN_PSR_EP)
        {
            if(errorStats.busState != CAN_ERROR_PASS)
                CAN_HandleErrorPassive(registeredHandle);
        }
        else if(psr & FDCAN_PSR_EW)
        {
            if(errorStats.busState != CAN_ERROR_WARN)
            {
                errorStats.busState = CAN_ERROR_WARN;
                errorStats.errorWarnCount++;
            }
        }
        else
        {
            /* Bus recovered on its own - clear passive/warning state */
            if(errorStats.busState == CAN_ERROR_WARN ||
               errorStats.busState == CAN_ERROR_PASS)
            {
                errorStats.busState = CAN_BUS_OK;
            }
        }
    }
#endif

    CanMessage_t *msg = &canQueue.buffer[canQueue.head];
    if(HAL_FDCAN_GetTxFifoFreeLevel(msg->hfdcan) > 0)
    {
        CanQueue_TxNext();
    }
}

/* ==========================================================================
 * Bus Off recovery
 *
 * Must be called from the main loop. Uses its own schedule_t timer so it
 * is completely independent of VescPoll() timing. Safe to call as often
 * as desired - it returns immediately if no recovery is pending or if the
 * recovery timer has not yet elapsed.
 * ========================================================================== */
void CAN_BusRecoveryPoll(void)
{
    if(!recoveryPending)       return;
    if(!recoveryScheduleReady) return;
    if(registeredHandle == NULL) return;

    if(!ScheduleReady(recoverySchedule)) return;

    recoveryPending       = 0;
    recoveryScheduleReady = 0;

    HAL_FDCAN_Stop(registeredHandle);
    HAL_FDCAN_Start(registeredHandle);

    errorStats.busState = CAN_BUS_OK;
    txInProgress        = 0;
}

/* ==========================================================================
 * Error statistics API
 * ========================================================================== */
void CAN_GetErrorStats(CAN_ErrorStats_t *out)
{
    if(out == NULL) return;
    *out = errorStats;
}

void CAN_ClearErrorStats(void)
{
    errorStats.txErrors       = 0;
    errorStats.rxErrors       = 0;
    errorStats.busOffCount    = 0;
    errorStats.errorWarnCount = 0;
    errorStats.errorPassCount = 0;
    errorStats.lastTxErrCnt   = 0;
    errorStats.lastRxErrCnt   = 0;
    /* busState intentionally not cleared - reflects live hardware state */
}

/* ==========================================================================
 * Public transmit entry point
 * ========================================================================== */
void can_transmit_eid(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len)
{
    CanQueue_Enqueue(hfdcan, id, data, len);
}

/* ==========================================================================
 * Weak application callbacks
 * ========================================================================== */
__weak void CAN_BusOffCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}

__weak void CAN_ErrorPassiveCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}
