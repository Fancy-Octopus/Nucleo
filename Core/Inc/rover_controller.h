/*
 * rover_controller.h
 *
 *  Created on: Mar 30, 2025
 *      Author: strongp
 */

#ifndef INC_ROVER_CONTROLLER_H_
#define INC_ROVER_CONTROLLER_H_

#include "stdint.h"

/* ---- Rover states -------------------------------------------------------
 *
 * Tier 1 (0-15):  no context required — fixed default behaviour
 * Tier 2 (16-31): one scalar in rover_context_t
 * Tier 3 (32-47): speed + 4 steering angles in rover_context_t
 * Tier 4 (48-63): per-wheel speed + angle in rover_context_t
 *
 * Old names kept as compile-time aliases so existing callers build without
 * changes.  Values 0-12 are unchanged — the legacy TCP single-byte protocol
 * continues to work during the transition period.
 * ---------------------------------------------------------------------- */
typedef enum {
    /* --- Tier 1 --- */
    ROVER_IDLE             = 0U,
    ROVER_FORWARD          = 1U,   /* straight fwd, default wheel speed      */
    ROVER_BACKWARD         = 2U,   /* straight bwd, default wheel speed      */
    ROVER_SPIN_RIGHT       = 3U,   /* spin-in-place CW,  default speed       */
    ROVER_SPIN_LEFT        = 4U,   /* spin-in-place CCW, default speed       */
    ROVER_ARMS_DOWN        = 5U,   /* lower both arms at default arm speed   */
    ROVER_ARMS_UP          = 6U,   /* raise both arms at default arm speed   */
    ROVER_DIG_FORWARD      = 7U,   /* drums fwd, wheels zero                 */
    ROVER_DEPOSIT_FORWARD  = 8U,
    ROVER_READY            = 9U,   /* deprecated — treated as ROVER_IDLE     */
    ROVER_ESTOP            = 10U,  /* all stop, steering disabled, sticky    */
    ROVER_DIG_BACKWARD     = 11U,
    ROVER_DEPOSIT_BACKWARD = 12U,

    /* --- Tier 1 deprecated aliases (old names, same values) --- */
    ROVER_TURN_RIGHT       = 3U,   /* → ROVER_SPIN_RIGHT  */
    ROVER_TURN_LEFT        = 4U,   /* → ROVER_SPIN_LEFT   */
    ROVER_WINCH_DOWN       = 5U,   /* → ROVER_ARMS_DOWN   */
    ROVER_WINCH_UP         = 6U,   /* → ROVER_ARMS_UP     */

    /* --- Tier 2 --- */
    ROVER_DRIVE            = 16U,  /* fwd/bwd at context.drive_speed (ERPM)  */
    ROVER_SPIN             = 17U,  /* spin-in-place at context.spin_speed    */
    ROVER_ARM_MOVE         = 18U,  /* RoboClaw at context.arm_speed          */

    /* --- Tier 3 --- */
    ROVER_TRAVERSE         = 32U,  /* context.drive_speed + steer angles     */
    ROVER_STEER_ONLY       = 33U,  /* steer to context angles, no drive      */

    /* --- Tier 4 --- */
    ROVER_WHEEL_CONTROL    = 48U,  /* context.wheel[4].{speed, angle}        */

    /* ---- Force variants (64-71) -----------------------------------------
     * Identical to their counterparts but the wheel-motor subsystem bypasses
     * the steering-settled gate and drives immediately.
     * -------------------------------------------------------------------- */
    ROVER_FORWARD_FORCE        = 64U,
    ROVER_BACKWARD_FORCE       = 65U,
    ROVER_SPIN_RIGHT_FORCE     = 66U,
    ROVER_SPIN_LEFT_FORCE      = 67U,
    ROVER_DRIVE_FORCE          = 68U,  /* context.drive_speed                */
    ROVER_SPIN_FORCE           = 69U,  /* context.spin_speed                 */
    ROVER_TRAVERSE_FORCE       = 70U,  /* context.drive_speed + steer angles */
    ROVER_WHEEL_CONTROL_FORCE  = 71U,  /* context.wheel[4].{speed, angle}    */

} rover_state_t;

/* ---- Per-wheel entry for Tier 4 ---------------------------------------- */
typedef struct {
    float speed;   /* ERPM, signed (+fwd / -bwd from wheel's perspective)   */
    float angle;   /* degrees                                               */
} rover_wheel_cmd_t;

/* ---- Context struct -------------------------------------------------------
 * Carries parameters alongside a state change request.
 * Fields unused by the requested tier may be left zeroed.
 * ---------------------------------------------------------------------- */
typedef struct {
    float             drive_speed;         /* ERPM, signed — DRIVE/TRAVERSE  */
    float             spin_speed;          /* ERPM, signed — SPIN            */
    float             arm_speed;           /* 0-127 signed  — ARM_MOVE       */
    float             steer_fl, steer_fr,
                      steer_rl, steer_rr;  /* degrees — TRAVERSE/STEER_ONLY  */
    rover_wheel_cmd_t wheel[4];            /* FL=0 FR=1 BL=2 BR=3            */
} rover_context_t;

/* ---- Error / status types ----------------------------------------------- */
typedef enum { WHEEL_OK, WHEEL_SPEED_MISMATCH, WHEEL_NO_COMMS } wheel_err_t;
typedef enum { ARM_OK,   ARM_NO_COMMS }                          arm_err_t;
typedef enum { STEERING_OK, STEERING_NO_COMMS }                  steering_err_t;
typedef enum { NET_OK,   NET_NO_COMMS }                          net_err_t;

typedef struct {
    wheel_err_t    wheel_err;
    arm_err_t      arm_err;
    steering_err_t steering_err;
    net_err_t      net_err;
} rover_t;

extern rover_t rover;

/* ---- Public API --------------------------------------------------------- */

/* Legacy single-state request — context zeroed, backward-compatible */
rover_state_t          RequestRoverState(rover_state_t reqState);

/* Primary request — attach context data for Tier 2-4 states */
rover_state_t          RequestRoverStateCtx(rover_state_t reqState,
                                             const rover_context_t *ctx);

/* Set the state the rover reverts to when the command watchdog fires */
void                   SetDefaultState(rover_state_t safeState);

rover_state_t          CurrentRoverState(void);
const rover_context_t *CurrentRoverContext(void);

void                   ControllerInit(void);
void                   ControllerPoll(void);
uint32_t               GetAvgLoopTime(void);

#endif /* INC_ROVER_CONTROLLER_H_ */
