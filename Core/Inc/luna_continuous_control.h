/*
 * luna_continuous_control.h
 *
 *  Created on: May 3, 2026
 *      Author: noahlwe (autonomy team)
 *
 *  Stub handlers for continuous-control commands sent from the autonomy
 *  stack via TCP. Currently print the requested values via printf;
 *  electrical team to wire these to actual motor primitives.
 *
 *  See Core/Src/tcp_server.c for the call sites.
 *  See rover_controller.h for the corresponding rover_state_t enum
 *  values (ROVER_SET_DRIVE_VEL, ROVER_SET_STEERING_ANGLES).
 */

#ifndef INC_LUNA_CONTINUOUS_CONTROL_H_
#define INC_LUNA_CONTINUOUS_CONTROL_H_

#include "stdint.h"

/**
 * @brief Set per-drive-motor target velocities.
 * @param vels  Array of 6 signed bytes, each value is a velocity in cm/s
 *              for one drive motor. Convention TBD with electrical for
 *              motor-to-index mapping. Initial proposal:
 *                vels[0] = front-right outer  (odrvfr axis 0)
 *                vels[1] = front-right inner  (odrvfr axis 1)
 *                vels[2] = back-right inner   (odrvbr axis 1)
 *                vels[3] = front-left inner   (odrvfl axis 1)
 *                vels[4] = back-left outer    (odrvbl axis 0)
 *                vels[5] = back-left inner    (odrvbl axis 1)
 *              Matches the ordering in odrive_can.c ROVER_FORWARD case.
 *
 * @note This handler should NOT update roverState. The autonomy stack
 *       sends these commands at ~20 Hz; if commands stop, the existing
 *       1s command-expiration timer in rover_controller.c reverts the
 *       state to ROVER_IDLE, which OdrivePoll picks up to stop motors.
 *
 * @note TODO(electrical): replace printf with calls to the existing
 *       SetSpeed(&odrvXX, axis, vel_float) primitive from odrive_can.c.
 *       The conversion from int8 cm/s to ODrive velocity units may need
 *       a scale factor matching what BASE_SPEED=12.5 currently represents.
 */
void HandleSetDriveVel(const int8_t * vels);

/**
 * @brief Set per-corner steering target angles.
 * @param angles  Array of 4 signed bytes, each value is an angle in
 *                degrees for one corner. Convention TBD with electrical:
 *                  angles[0] = front-left  (FL)
 *                  angles[1] = front-right (FR)
 *                  angles[2] = back-left   (BL)
 *                  angles[3] = back-right  (BR)
 *                Range nominally [-90, +90] degrees from straight, but
 *                autonomy currently only commands within ±51° (per the
 *                kinematics module's STEERING_LIMIT_DEG).
 *
 * @note TODO(electrical): wire this to the steering FSM ported from
 *       LunaNucleoNema17Testing. Currently luna_steering.c only toggles
 *       a single GPIO pin for "turn mode on/off" — it does not support
 *       per-wheel angle targets.
 */
void HandleSetSteeringAngles(const int8_t * angles);

#endif /* INC_LUNA_CONTINUOUS_CONTROL_H_ */
