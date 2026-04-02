/*
 * rover_controller.h
 *
 *  Created on: Mar 30, 2025
 *      Author: strongp
 */

#ifndef INC_ROVER_CONTROLLER_H_
#define INC_ROVER_CONTROLLER_H_

#include "stdint.h"

typedef enum {
	ROVER_IDLE=0U,
	ROVER_FORWARD=1U,
	ROVER_BACKWARD=2U,
	ROVER_TURN_RIGHT=3U,
	ROVER_TURN_LEFT=4U,
	ROVER_WINCH_DOWN=5U,
	ROVER_WINCH_UP=6U,
	ROVER_DIG_FORWARD=7U,
	ROVER_DEPOSIT_FORWARD=8U,
	ROVER_READY=9U,
	ROVER_ESTOP=10U,
	ROVER_DIG_BACKWARD=11U,
	ROVER_DEPOSIT_BACKWARD=12U
} rover_state_t;

typedef enum {WHEEL_OK, WHEEL_SPEED_MISMATCH, WHEEL_NO_COMMS} wheel_err_t;
typedef enum {WINCH_OK, WINCH_NO_COMMS} winch_err_t;
typedef enum {STEERING_OK, STEERING_NO_COMMS} steering_err_t;
typedef enum {NET_OK, NET_NO_COMMS} net_err_t;

typedef struct{
	wheel_err_t wheel_err;
	winch_err_t winch_err;
	steering_err_t steering_err;
	net_err_t net_err;
} rover_t;

extern rover_t rover;

rover_state_t RequestRoverState(rover_state_t reqState);
rover_state_t CurrentRoverState(void);
void ControllerInit(void);
void ControllerPoll(void);
uint32_t GetAvgLoopTime(void);

#endif /* INC_ROVER_CONTROLLER_H_ */
