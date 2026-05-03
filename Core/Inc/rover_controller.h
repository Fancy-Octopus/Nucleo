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
	ROVER_DEPOSIT_BACKWARD=12U,
	/* Continuous-control commands for autonomy. Unlike the discrete states
	 * above (each with its own hardcoded motor behavior in *_can.c), these
	 * commands carry payload bytes specifying per-motor targets. They do
	 * NOT change roverState — they call into handler stubs that write
	 * directly to motor controllers. The 1-second command timeout in
	 * rover_controller.c still applies: if no command arrives for 1s,
	 * roverState reverts to defaultState (ROVER_IDLE) which stops motors. */
	ROVER_SET_DRIVE_VEL=13U,         /* payload: 6 bytes int8, cm/s per drive motor */
	ROVER_SET_STEERING_ANGLES=14U,   /* payload: 4 bytes int8, degrees per corner   */
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
