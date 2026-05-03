/*
 * luna_continuous_control.c
 *
 *  Created on: May 3, 2026
 *      Author: noahlwe (autonomy team)
 *
 *  See luna_continuous_control.h for the interface contract.
 */

#include "luna_continuous_control.h"
#include "stdio.h"

void HandleSetDriveVel(const int8_t * vels)
{
	/* TODO(electrical): replace this with the actual SetSpeed calls into
	 * odrive_can.c. The values arrive in cm/s as signed int8; ODrive's
	 * SetSpeed(odrv, axis, float vel) takes a float. Determine the unit
	 * scale that matches the existing BASE_SPEED=12.5 hardcode and apply
	 * it here. */
	printf("HandleSetDriveVel: %d %d %d %d %d %d cm/s\r\n",
	       vels[0], vels[1], vels[2], vels[3], vels[4], vels[5]);
}

void HandleSetSteeringAngles(const int8_t * angles)
{
	/* TODO(electrical): wire to the steering FSM (currently being ported
	 * from LunaNucleoNema17Testing). Replace this print with calls to
	 * the per-wheel angle setpoint API once integrated. */
	printf("HandleSetSteeringAngles: FL=%d FR=%d BL=%d BR=%d deg\r\n",
	       angles[0], angles[1], angles[2], angles[3]);
}
