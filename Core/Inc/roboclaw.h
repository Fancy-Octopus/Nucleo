/*
 * roboclaw.h
 *
 *  Created on: Mar 15, 2025
 *      Author: strongp
 *  Updated: May 2026
 */

#ifndef INC_ROBOCLAW_H_
#define INC_ROBOCLAW_H_

#include <stdint.h>

void    RoboclawInit(void);
void    RoboclawPoll(void);

/* Returns the last-read encoder count for motor 1 or 2.
 * Updated every 500 ms by RoboclawPoll(). */
int32_t RoboclawGetEncoder(uint8_t motor);

/* Sends RESETENC (cmd 20) to zero both encoder counters. */
void    RoboclawResetEncoders(void);

#endif /* INC_ROBOCLAW_H_ */
