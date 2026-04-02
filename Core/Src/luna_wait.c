/*
 * luna_wait.c
 *
 *  Created on: Mar 30, 2025
 *      Author: strongp
 */
#include "luna_wait.h"

void SetScheduledTime(schedule_t* waitDev,uint32_t msec){
	waitDev->waitStamp = HAL_GetTick() + msec;
	waitDev->waitVal = msec;
}

void ResetSchedule(schedule_t* waitDev){
	waitDev->waitStamp = HAL_GetTick() + waitDev->waitVal;
}

uint8_t ScheduleReady(schedule_t waitDev){
	if ((waitDev.waitStamp - HAL_GetTick()) > waitDev.waitVal){
		return 0xFF;
	}
	return 0;
}
