/*
 * luna_wait.h
 *
 *  Created on: Mar 30, 2025
 *      Author: strongp
 */

#ifndef INC_LUNA_WAIT_H_
#define INC_LUNA_WAIT_H_

#include "main.h"
#include "stdint.h"

typedef struct{
	uint32_t waitVal;
	uint32_t waitStamp;
} schedule_t;

void SetScheduledTime(schedule_t* waitDev,uint32_t msec);
void ResetSchedule(schedule_t* waitDev);
uint8_t ScheduleReady(schedule_t waitDev);


#endif /* INC_LUNA_WAIT_H_ */
