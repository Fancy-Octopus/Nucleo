/*
 * luna_steering.c
 *
 *  Created on: Apr 19, 2025
 *      Author: strongp
 */

#include "main.h"
#include "luna_steering.h"
#include "rover_controller.h"

static rover_state_t state = ROVER_IDLE;

void SteeringPoll(void){
	rover_state_t nextState = CurrentRoverState();

	if(state != nextState){
		switch(nextState){
			case ROVER_TURN_RIGHT:
			case ROVER_TURN_LEFT:
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,1);
			break;
			case ROVER_FORWARD:
			case ROVER_BACKWARD:
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,0);
			break;
			default:
			break;
		}
		state = nextState;
	}
}
