/*
 * error_handle.c
 *
 *  Created on: Mar 11, 2025
 *      Author: patelc
 */

//Essentially main
#include "stdint.h"
#include "stdio.h"
#include "main.h"

char uart_buffer[80];

void odrive_error_decoder(int error_code);

void error_poll(int errorcode){


	if (errorcode == 0){
		return;
	}

	odrive_error_decoder(errorcode);


	//print errors with timestamp (over uart) (and terminal)
	//if you want, log them
	//Figure out if there is an error
}



//////////////////////////////////////////////////////////////////////
// Function That Should called
void uart_send_error(int error){



	//create buffer to send with timestamp and error code in it
	//send over uart pins, (NOT THE TERMINAL) (UART5)
	snprintf(uart_buffer, sizeof(uart_buffer), "text here" );//look it up
}

void get_timestamp(void) {
	//gives timestamp in "00:00:00" form with hr:min:sec

	uint32_t ticks_ms = HAL_GetTick();    //function return time is ms
	uint32_t ms_to_sec = ticks_ms / 1000;  // gives total time in ms to s
	uint32_t hours_elap = (ms_to_sec) / 3600;  // gives the amount of hours elapsed
	uint32_t minutes_elap = (ms_to_sec % 3600) / 60;  //mod to find minutes
	uint32_t second_elap = ms_to_sec % 60;  // mod to find secs left

	printf("%u:%u:%u\n", (unsigned int)hours_elap, \
	       (unsigned int)minutes_elap, (unsigned int)second_elap);
}

void odrive_error_decoder(int error_code) {
	switch (error_code) {
	case 0x1: // INITIALIZING
		printf("The system is initializing or reconfiguring.\n");
		break;

	case 0x2: // SYSTEM_LEVEL
		printf("Unexpected system error such as memory corruption, stack overflow, frozen thread, assert fail, etc. This error indicates a firmware bug.\n");
		break;

	case 0x4: // TIMING_ERROR
		printf("An internal hard timing requirement was violated. This usually means the device is computationally overloaded, either due to a specific user configuration or a firmware bug.\n");
		break;

	case 0x8: // MISSING_ESTIMATE
		printf("The position estimate, velocity estimate, or phase estimate was needed but invalid. Check encoder calibration, homing, or encoder connections.\n");
		break;

	case 0x10: // BAD_CONFIG
		printf("The ODrive configuration is invalid or incomplete. Verify motor direction, torque limits, brake resistor settings, and motor parameters.\n");
		break;

	case 0x20: // DRV_FAULT
		printf("The gate driver chip reported an error. This may indicate hardware damage if it occurs during normal operation.\n");
		break;

	case 0x40: // MISSING_INPUT
		printf("No value was provided for input_pos, input_vel, or input_torque. Check PWM input configuration and connections.\n");
		break;

	case 0x100: // DC_BUS_OVER_VOLTAGE
		printf("The DC voltage exceeded the configured limit. Check brake resistor connections and resistance value.\n");
		break;

	case 0x200: // DC_BUS_UNDER_VOLTAGE
		printf("The DC voltage fell below the configured limit. Ensure power leads are securely connected and the PSU is adequate.\n");
		break;

	case 0x400: // DC_BUS_OVER_CURRENT
		printf("Too much DC current was pulled. Check motor and board current limits.\n");
		break;

	case 0x800: // DC_BUS_OVER_REGEN_CURRENT
		printf("Too much DC current was regenerated. Ensure the brake resistor can handle the braking current.\n");
		break;

	case 0x1000: // CURRENT_LIMIT_VIOLATION
		printf("The motor current exceeded the hard limit. Adjust the margin between soft and hard current limits.\n");
		break;

	case 0x2000: // MOTOR_OVER_TEMP
		printf("The motor temperature exceeded the upper limit. Check the motor thermistor and cooling.\n");
		break;

	case 0x4000: // INVERTER_OVER_TEMP
		printf("The inverter temperature exceeded the upper limit. Check the inverter thermistor and cooling.\n");
		break;

	case 0x8000: // VELOCITY_LIMIT_VIOLATION
		printf("The estimated velocity exceeded the configured limit.\n");
		break;

	case 0x10000: // POSITION_LIMIT_VIOLATION
		printf("The position limit was violated.\n");
		break;

	case 0x1000000: // WATCHDOG_TIMER_EXPIRED
		printf("The watchdog timer expired. Ensure the watchdog is fed within the configured timeout.\n");
		break;

	case 0x2000000: // ESTOP_REQUESTED
		printf("An estop was requested from an external source. Check CAN messages or endstop status.\n");
		break;

	case 0x4000000: // SPINOUT_DETECTED
		printf("A spinout situation was detected. Check for discrepancies between electrical and mechanical power.\n");
		break;

	case 0x8000000: // BRAKE_RESISTOR_DISARMED
		printf("The brake resistor was disarmed due to another component failure. Fix the root cause and clear errors.\n");
		break;

	case 0x10000000: // THERMISTOR_DISCONNECTED
		printf("The motor thermistor is enabled but disconnected. Check the thermistor connection.\n");
		break;

	case 0x40000000: // CALIBRATION_ERROR
		printf("A calibration procedure failed. Check the procedure_result for details.\n");
		break;

	default:
		printf("Currently no odrive error");
		break;
	}
}
