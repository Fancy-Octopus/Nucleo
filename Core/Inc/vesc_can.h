/*
 * vesc_can.h
 *
 *  Created on: Apr 19, 2026
 *      Author: gruetzmacherg
 */

#ifndef INC_VESC_CAN_H_
#define INC_VESC_CAN_H_

#include "main.h"

void ALT_MX_FDCAN1_Init(FDCAN_HandleTypeDef *hfdcan1);
int VescInit(FDCAN_HandleTypeDef *hfdcan);
void VescPoll(void);

#endif /* INC_VESC_CAN_H_ */
