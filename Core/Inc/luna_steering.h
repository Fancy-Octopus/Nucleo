/*
 * luna_steering.h
 *
 *  Created on: Apr 19, 2025
 *      Author: strongp
 */

#ifndef INC_LUNA_STEERING_H_
#define INC_LUNA_STEERING_H_

#include <stdint.h>
#include "rover_controller.h"

/* ---- Packet framing ---- */
#define STEERING_START_BYTE       0xAAU
#define STEERING_CMD_TYPE         0x01U
#define STEERING_STATUS_TYPE      0x02U
#define STEERING_HOME_TYPE        0x03U
#define STEERING_CMD_LEN          19U   /* [start][type][FL:4][FR:4][RL:4][RR:4][crc] */
#define STEERING_STATUS_LEN       20U   /* [start][type][FL:4][FR:4][RL:4][RR:4][flags][crc] */
#define STEERING_HOME_LEN         3U    /* [start][type][crc] */
#define STEERING_RESET_TYPE       0x04U
#define STEERING_RESET_LEN        4U    /* [start][type][idx:u8][crc] */
#define STEERING_RESET_ALL        0xFFU /* idx = reset all four drivers */
#define STEERING_QUERY_TYPE       0x05U
#define STEERING_QUERY_LEN        3U    /* [start][type][crc] */
#define STEERING_DISABLE_TYPE     0x06U
#define STEERING_DISABLE_LEN      4U    /* [start][type][idx:u8][crc] */
#define STEERING_ENABLE_TYPE      0x07U
#define STEERING_ENABLE_LEN       4U    /* [start][type][idx:u8][crc] */

/* How often SteeringPoll() proactively refreshes the cached status (ms). */
#define STEERING_REFRESH_MS       200U

/* Link-loss detection: if no valid packet arrives within this window the
 * link is considered lost and cached angles are cleared. */
#define STEERING_LINK_TIMEOUT_MS  1000U

/* ---- Drive-gate thresholds (used by rover_controller) ---- */

/* "At target" — wheels considered settled, drive gate opens */
#define STEERING_SETTLED_DEG      2.0f

/* "Small enough change" — proceed without waiting for convergence.
 * Handles near-real-time curve-following where angle deltas are small. */
#define STEERING_PROCEED_DEG      10.0f

/* Timeout before drive gate opens regardless of wheel position.
 * Prevents total motion lockout when a wheel is stuck. */
#define STEERING_STUCK_MS         3000U

/* ---- Status flags (from L4 flags byte) ---- */
#define STEERING_FLAG_HOMING      0x01U
#define STEERING_FLAG_HOMED       0x02U

/* ---- Angle constants (degrees) ---- */
#define STEERING_ANGLE_STRAIGHT   0.0f

#define STEERING_SPIN_FL          51.4f
#define STEERING_SPIN_FR         -51.4f
#define STEERING_SPIN_RL         -51.4f
#define STEERING_SPIN_RR          51.4f

/* ---- Steering sub-state (H5 side) ---- */
typedef enum {
    STEERING_STATE_IDLE = 0,   /* no command pending, link may or may not be up */
    STEERING_STATE_MOVING,     /* angles sent, waiting for convergence           */
    STEERING_STATE_AT_TARGET,  /* all wheels within STEERING_SETTLED_DEG         */
    STEERING_STATE_RESETTING,  /* RST pulse in progress                          */
    STEERING_STATE_DISABLED,   /* all drivers force-disabled                     */
    STEERING_STATE_LINK_LOST,  /* no reply within STEERING_LINK_TIMEOUT_MS       */
} steering_hstate_t;

/* ---- Status struct ---- */
typedef struct {
    float   fl, fr, rl, rr;  /* angles reported by the L4 stepper controller */
    uint8_t flags;            /* L4 status flags (STEERING_FLAG_*)            */
    uint8_t valid;            /* 1 if an unread packet is available           */
} steering_status_t;

/* ---- Diagnostic counters ---- */
typedef struct {
    uint32_t tx_sent;
    uint32_t tx_errors;
    uint32_t rx_bytes;
    uint32_t rx_good;
    uint32_t rx_bad_crc;
    uint32_t uart_errors;
    uint32_t link_lost_count;
    uint32_t last_rx_tick;
    uint8_t  init_ok;
    uint8_t  ever_connected;
    uint8_t  link_active;
    steering_hstate_t hstate;   /* H5-side sub-state */
} steering_diag_t;

/* ---- Public API ---- */
void              SteeringInit(void);
void              SteeringPoll(void);


/* Send specific angles to the L4 */
void              SetSteeringAngles(float fl, float fr, float rl, float rr);

/* Query L4 for current status without changing angles */
void              SteeringRequestStatus(void);

/* Homing sequence */
void              SteeringHome(void);

/* Hardware reset pulse on driver(s); idx 0-3 or STEERING_RESET_ALL */
void              SteeringResetDriver(uint8_t idx);

/* Force-disable driver(s) via EN pin — persists until ForceEnable called */
void              SteeringForceDisable(uint8_t idx);

/* Re-enable driver(s) after a force-disable */
void              SteeringForceEnable(uint8_t idx);

/* Returns 1 if all active wheels are within tolerance_deg of last target */
uint8_t           SteeringWheelsReady(float tolerance_deg);

/* Returns a copy of the latest status; clears the valid flag */
steering_status_t GetSteeringStatus(void);

/* Returns the latest status WITHOUT clearing the valid flag (non-consuming) */
steering_status_t GetSteeringStatusPeek(void);

steering_diag_t   GetSteeringDiag(void);

#endif /* INC_LUNA_STEERING_H_ */
