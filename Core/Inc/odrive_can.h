/*
 * odrive_can.h
 *
 *  Created on: Feb 9, 2025
 *      Author: strongp
 */

#ifndef INC_ODRIVE_CAN_H_
#define INC_ODRIVE_CAN_H_


int OdriveInit(FDCAN_HandleTypeDef* caninst);
int OdrivePoll(void);

void PrintODriveConnStatus(void);


#endif /* INC_ODRIVE_CAN_H_ */
