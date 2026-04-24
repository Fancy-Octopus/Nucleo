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

#define CAN_QUEUE_MAX_MESSAGES  16
#define CAN_MAX_DATA_BYTES      8

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;        // destination interface
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

/* Hardware initialisation - call once per FDCAN interface before CanQueue_Init.
 * Activates the TX FIFO empty notification, configures the NVIC, and starts
 * the peripheral. irqPriority is the preempt priority passed to HAL_NVIC_SetPriority. */
void    CAN_HardwareInit(FDCAN_HandleTypeDef *hfdcan, IRQn_Type it0IrqN, uint32_t irqPriority);

/* Starts up the target can hardware interface */
void CAN_HardwareStart(FDCAN_HandleTypeDef *hfdcan);

/* Queue management */
void    CanQueue_Init(void);
uint8_t CanQueue_Enqueue(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len);
void    CanQueue_TxNext(void);
uint8_t CanQueue_IsBusy(void);
void    CanQueue_Poll(void);

/* Generic low level transmit - routes through the queue.
 * This is the single entry point for all CAN transmission. */
void    can_transmit_eid(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len);

#endif /* INC_CAN_QUEUE_H_ */
