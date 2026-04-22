/*
 * rover_controller.c
 *
 *  Created on: Mar 30, 2025
 *      Author: strongp
 */

#include "rover_controller.h"
#include "main.h"
#include "luna_wait.h"

#define ROVER_COMMAND_INTERVAL_MAX 10000

static rover_state_t roverState = ROVER_IDLE;
static rover_state_t defaultState = ROVER_IDLE;

rover_t rover;

IWDG_HandleTypeDef hiwdg;

static uint32_t avgLoopTime;

static schedule_t commandExpiration;

rover_state_t RequestRoverState(rover_state_t reqState){
	roverState = reqState;
	if(reqState==ROVER_IDLE || reqState==ROVER_READY){
		defaultState = reqState;
	}
	ResetSchedule(&commandExpiration);
	return reqState;
}

rover_state_t CurrentRoverState(void){
	return roverState;
}

void ControllerInit(void){
	//Check if reset was from watchdog
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0x00u){
		printf("WATCHDOG RESET OCCURED\r\n");

	}
	__HAL_RCC_CLEAR_RESET_FLAGS();

	//Init watchdog
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
	hiwdg.Init.Reload = 200; //With prescalar 32, watchdog time is Reload*2 (ms)
	hiwdg.Init.EWI = 0;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK){
		Error_Handler();
	}

	//Set wait time
	SetScheduledTime(&commandExpiration,ROVER_COMMAND_INTERVAL_MAX);
}

void ControllerPoll(void){
	//Feed the watchdog
	HAL_IWDG_Refresh(&hiwdg);

	//Get main loop time
	static uint32_t loopTime[10] = {0};
	static uint8_t index = 0;
	static uint32_t lastTime = 0;
	loopTime[index%10] = HAL_GetTick()-lastTime;
	avgLoopTime = 0;
	for(int i=0;i<10;i++){
		avgLoopTime += loopTime[i];
	}
	avgLoopTime = avgLoopTime/10;
	index++;
	lastTime = HAL_GetTick();

	//Check if it has been too long since last update
	if(ScheduleReady(commandExpiration)){
		roverState = defaultState;
	}


}

uint32_t GetAvgLoopTime(void){
	return avgLoopTime;
}


