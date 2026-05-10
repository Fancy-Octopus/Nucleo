/*
 * tcp_server.h
 *
 *  Created on: Feb 1, 2025
 *      Author: gruetzmacherg
 *  Updated: May 2026
 *
 * ---- COMMAND / QUERY packet (client → H5) ---------------------------------
 *   [0xAA][type:u8][payload floats, IEEE 754 LE...]
 *
 *   Rover command types (type = rover_state_t):
 *     Tier 1 (0x00-0x13):  0 floats →  2 bytes
 *     Tier 2 (0x14-0x1F):  1 float  →  6 bytes  (drive_speed / spin_speed / arm_speed)
 *     Tier 3 (0x20-0x2F):  5 floats → 22 bytes  (drive_speed, steer_fl/fr/rl/rr)
 *     Tier 4 (0x30-0x3F):  8 floats → 34 bytes  (wheel[0..3].{speed, angle})
 *     Force T1 (0x40-0x43): 0 floats →  2 bytes
 *     Force T2 (0x44-0x45): 1 float  →  6 bytes
 *     Force T3 (0x46):      5 floats → 22 bytes
 *     Force T4 (0x47):      8 floats → 34 bytes
 *
 *   Telemetry query types (type ≥ 0x80, 0 payload floats → always 2 bytes):
 *     QUERY_ALL      0x80  — rover state + flags + all steering angles
 *     QUERY_STEERING 0x81  — steering angles, link status, hstate
 *     QUERY_DRIVE    0x82  — rover state + drive gate flags only
 *     QUERY_STATE    0x83  — current rover state byte only
 *
 * ---- TELEMETRY packet (H5 → client) ---------------------------------------
 *   [0xAB][telem_type:u8][payload...]
 *   Query responses mirror the query type byte that triggered them.
 *   ACK_CMD is sent automatically after every rover command packet.
 *
 *   TELEM_ALL      0x80  [state:u8][flags:u8][FL:f32][FR:f32][RL:f32][RR:f32][hstate:u8]  21 bytes
 *   TELEM_STEERING 0x81  [FL:f32][FR:f32][RL:f32][RR:f32][steer_flags:u8][hstate:u8]      20 bytes
 *   TELEM_DRIVE    0x82  [state:u8][flags:u8]                                               4 bytes
 *   TELEM_STATE    0x83  [state:u8]                                                         3 bytes
 *   ACK_CMD        0x84  [accepted:u8][reason:u8][current_state:u8]                         5 bytes
 *
 *   flags (TELEM_ALL / TELEM_DRIVE): bit0=wheels_ready  bit1=force_state  bit2=link_active
 *   steer_flags (TELEM_STEERING):    bit0=link_active  bit1=homed  bit2=homing
 *   accepted:  0x01=accepted  0x00=denied
 *   reason:    0x00=OK  0x01=ESTOP active  0x02=invalid state ID  0x03=unrecognised context state
 * ---------------------------------------------------------------------------
 */

#ifndef APP_TCP_TEST_H_
#define APP_TCP_TEST_H_

#include "main.h"

/* ---- Telemetry query types (sent by client, payload = 0 floats) ---- */
#define QUERY_ALL       0x80U
#define QUERY_STEERING  0x81U
#define QUERY_DRIVE     0x82U
#define QUERY_STATE     0x83U

/* ---- Command acknowledgement (H5 → client, unsolicited after every command) ---- */
#define ACK_CMD              0x84U
#define ACK_REASON_OK        0x00U  /* command accepted */
#define ACK_REASON_ESTOP     0x01U  /* denied — ESTOP is active */
#define ACK_REASON_INVALID_ID  0x02U  /* denied — state byte is not a defined rover state */
#define ACK_REASON_NO_CONTEXT  0x03U  /* denied — state byte falls in a context-bearing tier but is unrecognised */

/* ---- Telemetry response start byte ---- */
#define TELEM_START     0xABU

void    tcp_server_init(void);
void    tcp_task(void);
uint8_t GetTcpClientCount(void);

#endif /* APP_TCP_TEST_H_ */
