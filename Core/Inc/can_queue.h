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
    FDCAN_HandleTypeDef *hfdcan;            // destination interface
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

void    CanQueue_Init(void);
uint8_t CanQueue_Enqueue(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len);
void    CanQueue_TxNext(void);
uint8_t CanQueue_IsBusy(void);
void CanQueue_Poll(void);

#endif /* INC_CAN_QUEUE_H_ */
