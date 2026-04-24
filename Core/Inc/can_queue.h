/*
 * can_queue.h
 *
 *  Created on: Apr 22, 2026
 *      Author: gruetzmacherg
 */

#ifndef INC_CAN_QUEUE_H_
#define INC_CAN_QUEUE_H_

#include <stdint.h>
#include "stm32h5xx_hal.h"
#include "luna_wait.h"  // schedule_t, ScheduleReady, ResetSchedule, SetScheduledTime

/* ==========================================================================
 * Build configuration
 * ==========================================================================
 *
 * CAN_USE_INTERRUPTS
 *   1 - TX chaining and error detection are driven by FDCAN interrupts.
 *       CAN_BusRecoveryPoll() and CanQueue_Poll() still run in main context
 *       and must be called from the main loop.
 *   0 - No FDCAN interrupts are enabled. Everything is driven by
 *       CanQueue_Poll() and CAN_BusRecoveryPoll() in the main loop.
 *       Suitable for disconnected-bus testing or bare polling environments.
 */
#define CAN_USE_INTERRUPTS          1

/* Milliseconds between Bus Off recovery attempts.
 * Acts as a natural backoff when the bus is disconnected. */
#define CAN_BUS_OFF_RECOVERY_MS     1000

/* Maximum number of messages held in the circular TX queue. */
#define CAN_QUEUE_MAX_MESSAGES      16

/* Maximum data bytes per CAN frame (classic CAN = 8). */
#define CAN_MAX_DATA_BYTES          8

/* ==========================================================================
 * Types
 * ========================================================================== */
typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t             id;
    uint8_t              data[CAN_MAX_DATA_BYTES];
    uint8_t              len;
} CanMessage_t;

typedef struct {
    CanMessage_t     buffer[CAN_QUEUE_MAX_MESSAGES];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} CanQueue_t;

typedef enum {
    CAN_BUS_OK     = 0,
    CAN_ERROR_WARN = 1,
    CAN_ERROR_PASS = 2,
    CAN_BUS_OFF    = 3,
} CAN_BusState_t;

typedef struct {
    uint32_t       txErrors;        // cumulative TX error counter increments
    uint32_t       rxErrors;        // cumulative RX error counter increments
    uint32_t       busOffCount;     // number of Bus Off events
    uint32_t       errorWarnCount;  // number of error warning threshold crossings
    uint32_t       errorPassCount;  // number of error passive threshold crossings
    uint32_t       lastTxErrCnt;    // raw HAL TX error counter at last sample
    uint32_t       lastRxErrCnt;    // raw HAL RX error counter at last sample
    CAN_BusState_t busState;        // current bus state
} CAN_ErrorStats_t;

/* ==========================================================================
 * Initialisation
 *
 * Required call sequence in main.c:
 *
 *   ALT_MX_FDCAN1_Init(&hfdcan1);
 *   CAN_HardwareInit(&hfdcan1, FDCAN1_IT0_IRQn, 5);  // IRQn/priority ignored
 *                                                     // when CAN_USE_INTERRUPTS=0
 *   CanQueue_Init();
 *   VescInit(&hfdcan1);   // configures filters, then calls CAN_HardwareStart()
 *
 * Main loop (always required regardless of CAN_USE_INTERRUPTS):
 *
 *   CAN_BusRecoveryPoll();
 *   CanQueue_Poll();       // only needed when CAN_USE_INTERRUPTS=0, but
 *                          // harmless to call in either mode
 * ========================================================================== */
void CAN_HardwareInit(FDCAN_HandleTypeDef *hfdcan, IRQn_Type it0IrqN, uint32_t irqPriority);
void CAN_HardwareStart(FDCAN_HandleTypeDef *hfdcan);
void CanQueue_Init(void);

/* ==========================================================================
 * Queue API
 * ========================================================================== */
uint8_t CanQueue_Enqueue(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len);
void    CanQueue_TxNext(void);
uint8_t CanQueue_IsBusy(void);

/* Must be called from the main loop.
 * When CAN_USE_INTERRUPTS=1 this acts as a safety drain in case the FIFO
 * empty callback is missed. When CAN_USE_INTERRUPTS=0 this is the sole
 * mechanism that drains the queue. */
void    CanQueue_Poll(void);

/* ==========================================================================
 * Transmit entry point
 * All CAN transmission in the project must go through here.
 * ========================================================================== */
void can_transmit_eid(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len);

/* ==========================================================================
 * Bus recovery
 * Must be called from the main loop. Uses its own internal schedule timer
 * so it is safe to call as frequently as desired.
 * ========================================================================== */
void CAN_BusRecoveryPoll(void);

/* ==========================================================================
 * Error statistics
 * Call CAN_GetErrorStats() from your CLI callback to retrieve counters.
 * ========================================================================== */
void CAN_GetErrorStats(CAN_ErrorStats_t *out);
void CAN_ClearErrorStats(void);

/* ==========================================================================
 * Weak application callbacks
 * Override anywhere in the project to react to bus state changes.
 * When CAN_USE_INTERRUPTS=1 these are called from interrupt context -
 * keep implementations short. When CAN_USE_INTERRUPTS=0 they are called
 * from CAN_BusRecoveryPoll() in main context.
 * ========================================================================== */
void CAN_BusOffCallback(FDCAN_HandleTypeDef *hfdcan);
void CAN_ErrorPassiveCallback(FDCAN_HandleTypeDef *hfdcan);

#endif /* INC_CAN_QUEUE_H_ */
