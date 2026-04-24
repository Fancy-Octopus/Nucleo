/*
 * vesc_can.h
 *
 *  Created on: Apr 19, 2026
 *      Author: gruetzmacherg
 */

#ifndef INC_VESC_CAN_H_
#define INC_VESC_CAN_H_

#include "stm32h5xx_hal.h"
#include "can_queue.h"

/* Performs the FDCAN1 peripheral register configuration.
 * Pass the handle that was declared in main.c.
 * Does NOT start the peripheral - call CAN_HardwareInit() afterwards. */
void ALT_MX_FDCAN1_Init(FDCAN_HandleTypeDef *hfdcan1);

/* Stores the FDCAN handle for use by VESC transmit functions and
 * configures RX filters. Call after CAN_HardwareInit(). */
int  VescInit(FDCAN_HandleTypeDef *hfdcan);

/* Call from the main polling loop */
void VescPoll(void);

/* VESC FDCAN command set */
void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay);
void comm_can_set_current_brake(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);
void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay);
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);
void comm_can_set_handbrake(uint8_t controller_id, float current);
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);

#endif /* INC_VESC_CAN_H_ */
