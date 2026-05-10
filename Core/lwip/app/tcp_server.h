/*
 * tcp_server.h
 *
 *  Created on: Feb 1, 2025
 *      Author: gruetzmacherg
 *  Updated: May 2026
 *
 * ---- COMMAND / QUERY packet (client → H5) ---------------------------------
 *   [0xAA][type:u8][payload floats, IEEE 754 LE...][CRC8]
 *
 *   Rover command types (type = rover_state_t, 0x00–0x3F):
 *     Tier 1 (0x00-0x0F):  0 floats →  3 bytes
 *     Tier 2 (0x10-0x1F):  1 float  →  7 bytes  (drive_speed / spin_speed / arm_speed)
 *     Tier 3 (0x20-0x2F):  5 floats → 24 bytes  (drive_speed, steer_fl/fr/rl/rr)
 *     Tier 4 (0x30-0x3F):  8 floats → 36 bytes  (wheel[0..3].{speed, angle})
 *
 *   Telemetry query types (type ≥ 0x80, 0 payload floats → always 3 bytes):
 *     QUERY_ALL      0x80  — rover state + flags + all steering angles
 *     QUERY_STEERING 0x81  — steering angles, link status, hstate
 *     QUERY_DRIVE    0x82  — rover state + drive gate flags only
 *
 *   CRC8 polynomial 0x07 over all bytes preceding the CRC byte.
 *
 * ---- TELEMETRY packet (H5 → client, only in response to a query) ----------
 *   [0xAB][telem_type:u8][payload...][CRC8]
 *   telem_type mirrors the query type that triggered it.
 *
 *   TELEM_ALL      0x80  [state:u8][flags:u8][FL:f32][FR:f32][RL:f32][RR:f32][hstate:u8][CRC8]  22 bytes
 *   TELEM_STEERING 0x81  [FL:f32][FR:f32][RL:f32][RR:f32][steer_flags:u8][hstate:u8][CRC8]      21 bytes
 *   TELEM_DRIVE    0x82  [state:u8][flags:u8][CRC8]                                               5 bytes
 *
 *   flags (TELEM_ALL / TELEM_DRIVE): bit0=drive_enabled  bit1=steering_bypassed  bit2=link_active
 *   steer_flags (TELEM_STEERING):    bit0=link_active  bit1=homed  bit2=homing
 * ---------------------------------------------------------------------------
 */

#ifndef APP_TCP_TEST_H_
#define APP_TCP_TEST_H_

#include "main.h"

/* ---- Telemetry query types (sent by client, payload = 0 floats) ---- */
#define QUERY_ALL       0x80U
#define QUERY_STEERING  0x81U
#define QUERY_DRIVE     0x82U

/* ---- Telemetry response start byte ---- */
#define TELEM_START     0xABU

void    tcp_server_init(void);
void    tcp_task(void);
uint8_t GetTcpClientCount(void);

#endif /* APP_TCP_TEST_H_ */
