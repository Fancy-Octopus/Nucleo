/*
 * luna_steering.h
 *
 *  Created on: Apr 19, 2025
 *      Author: strongp
 */

#ifndef INC_LUNA_STEERING_H_
#define INC_LUNA_STEERING_H_

#include <stdint.h>

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
#define STEERING_QUERY_LEN        3U    /* [start][type][crc] — requests status without changing angles */

/* How often SteeringPoll() proactively refreshes the cached status (ms).
 * Any consumer (CLI, TCP, telemetry) can call GetSteeringStatus() and always
 * get data no older than this without issuing its own request. */
#define STEERING_REFRESH_MS       200U

/* ---- Status flags (from L4 flags byte) ---- */
#define STEERING_FLAG_HOMING      0x01U  /* homing sequence active */
#define STEERING_FLAG_HOMED       0x02U  /* successfully homed at least once */

/* ---- Angle constants (degrees) ---- */
#define STEERING_ANGLE_STRAIGHT   0.0f

#define STEERING_SPIN_FL          45.0f
#define STEERING_SPIN_FR         -45.0f
#define STEERING_SPIN_RL         -45.0f
#define STEERING_SPIN_RR          45.0f

/* ---- Status struct ---- */
typedef struct {
    float   fl, fr, rl, rr;  /* angles reported by the L4 stepper controller */
    uint8_t flags;            /* L4 status flags (STEERING_FLAG_*) */
    uint8_t valid;            /* 1 if an unread packet is available, cleared on read */
} steering_status_t;

/* ---- Diagnostic counters ---- */
typedef struct {
    uint32_t tx_sent;        /* packets where HAL_UART_Transmit returned HAL_OK */
    uint32_t tx_errors;      /* packets where HAL_UART_Transmit returned an error */
    uint32_t rx_bytes;       /* total raw bytes received from L4 */
    uint32_t rx_good;        /* status packets with valid CRC accepted */
    uint32_t rx_bad_crc;     /* status packets with bad CRC (framing looked valid) */
    uint32_t uart_errors;    /* UART-level framing / overrun / noise errors */
    uint8_t  init_ok;        /* 1 if HAL_UART_Init succeeded during SteeringInit */
    uint8_t  ever_connected; /* 1 if at least one valid status packet has been received */
} steering_diag_t;

/* ---- Public API ---- */
void              SteeringInit(void);
void              SteeringPoll(void);
void              SetSteeringAngles(float fl, float fr, float rl, float rr);
void              SteeringRequestStatus(void); /* asks L4 for current angles without changing them */
void              SteeringHome(void);
void              SteeringResetDriver(uint8_t idx); /* 0-3 or STEERING_RESET_ALL */
steering_status_t GetSteeringStatus(void);
steering_diag_t   GetSteeringDiag(void);

#endif /* INC_LUNA_STEERING_H_ */
