/*
 * rover_controller.c
 *
 *  Created on: Mar 30, 2025
 *      Author: strongp
 */

#include "rover_controller.h"
#include "luna_steering.h"
#include "main.h"
#include "luna_wait.h"
#include <string.h>

#define ROVER_COMMAND_INTERVAL_MAX  1000U   /* ms — command watchdog           */

/* ---- Private state ---------------------------------------------------- */
static rover_state_t    roverState   = ROVER_IDLE;
static rover_state_t    defaultState = ROVER_IDLE;
static rover_context_t  roverCtx;

/* Drive gate — cleared while steering is settling to a new target */
static uint8_t          driveEnabled      = 1;
static uint8_t          steeringBypassed  = 0;
static schedule_t       steeringStuckTimer;
static uint8_t          steeringTimerArmed = 0;

/* ESTOP is sticky — only ROVER_IDLE can exit it */
static uint8_t          estopActive = 0;

rover_t rover;

IWDG_HandleTypeDef hiwdg;

static uint32_t avgLoopTime;
static schedule_t commandExpiration;

/* ---- Helpers ---------------------------------------------------------- */

/* Largest absolute angle delta across all four wheels between the new
 * target angles and the most recently reported stepper positions. */
static float SteeringMaxDelta(float fl, float fr, float rl, float rr)
{
    steering_status_t s = GetSteeringStatusPeek();
    float d0 = fl - s.fl; if (d0 < 0.0f) d0 = -d0;
    float d1 = fr - s.fr; if (d1 < 0.0f) d1 = -d1;
    float d2 = rl - s.rl; if (d2 < 0.0f) d2 = -d2;
    float d3 = rr - s.rr; if (d3 < 0.0f) d3 = -d3;
    float max = d0;
    if (d1 > max) max = d1;
    if (d2 > max) max = d2;
    if (d3 > max) max = d3;
    return max;
}

/* Arm the drive gate for states that require steering to settle first. */
static void ArmSteeringGate(float fl, float fr, float rl, float rr)
{
    float delta = SteeringMaxDelta(fl, fr, rl, rr);
    if (delta < STEERING_PROCEED_DEG) {
        driveEnabled     = 1;
        steeringBypassed = 0;
        steeringTimerArmed = 0;
    } else {
        driveEnabled     = 0;
        steeringBypassed = 0;
        SetScheduledTime(&steeringStuckTimer, STEERING_STUCK_MS);
        steeringTimerArmed = 1;
    }
}

/* ---- Public API ------------------------------------------------------- */

rover_state_t RequestRoverState(rover_state_t reqState)
{
    rover_context_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    return RequestRoverStateCtx(reqState, &ctx);
}

rover_state_t RequestRoverStateCtx(rover_state_t reqState,
                                    const rover_context_t *ctx)
{
    /* ESTOP is only exitable by an explicit IDLE request */
    if (estopActive && reqState != ROVER_IDLE && reqState != ROVER_ESTOP)
        return roverState;

    if (reqState == ROVER_ESTOP) {
        estopActive      = 1;
        driveEnabled     = 0;
        steeringBypassed = 0;
        steeringTimerArmed = 0;
        SteeringForceDisable(STEERING_RESET_ALL);
        roverState = ROVER_ESTOP;
        ResetSchedule(&commandExpiration);
        return roverState;
    }

    if (reqState == ROVER_IDLE || reqState == ROVER_READY) {
        estopActive  = 0;
        defaultState = ROVER_IDLE;
        driveEnabled = 1;
        steeringTimerArmed = 0;
        /* Re-enable steering drivers after ESTOP or idle */
        SteeringForceEnable(STEERING_RESET_ALL);
    }

    roverState = reqState;
    if (ctx) roverCtx = *ctx;

    /* Update the watchdog-revert default for safe states */
    if (reqState == ROVER_IDLE)
        defaultState = ROVER_IDLE;

    ResetSchedule(&commandExpiration);

    /* ---- Drive gate logic per tier ---- */
    switch (reqState) {
    /* Tier 1: straight fwd/bwd — no steering change, always drive */
    case ROVER_FORWARD:
    case ROVER_BACKWARD:
        driveEnabled     = 1;
        steeringBypassed = 0;
        steeringTimerArmed = 0;
        SetSteeringAngles(0.0f, 0.0f, 0.0f, 0.0f);
        break;

    /* Tier 1: spin — needs steering to reach ±45° first */
    case ROVER_SPIN_RIGHT:
        ArmSteeringGate(STEERING_SPIN_FL,  STEERING_SPIN_FR,
                        STEERING_SPIN_RL,  STEERING_SPIN_RR);
        SetSteeringAngles(STEERING_SPIN_FL,  STEERING_SPIN_FR,
                          STEERING_SPIN_RL,  STEERING_SPIN_RR);
        break;

    case ROVER_SPIN_LEFT:
        ArmSteeringGate(-STEERING_SPIN_FL, -STEERING_SPIN_FR,
                        -STEERING_SPIN_RL, -STEERING_SPIN_RR);
        SetSteeringAngles(-STEERING_SPIN_FL, -STEERING_SPIN_FR,
                          -STEERING_SPIN_RL, -STEERING_SPIN_RR);
        break;

    /* Tier 2: proportional drive — no steering change */
    case ROVER_DRIVE:
        driveEnabled     = 1;
        steeringBypassed = 0;
        steeringTimerArmed = 0;
        SetSteeringAngles(0.0f, 0.0f, 0.0f, 0.0f);
        break;

    /* Tier 2: proportional spin — use default steering angles */
    case ROVER_SPIN:
        ArmSteeringGate(STEERING_SPIN_FL,  STEERING_SPIN_FR,
                        STEERING_SPIN_RL,  STEERING_SPIN_RR);
        if (roverCtx.spin_speed >= 0.0f) {
            SetSteeringAngles(STEERING_SPIN_FL,  STEERING_SPIN_FR,
                              STEERING_SPIN_RL,  STEERING_SPIN_RR);
        } else {
            SetSteeringAngles(-STEERING_SPIN_FL, -STEERING_SPIN_FR,
                              -STEERING_SPIN_RL, -STEERING_SPIN_RR);
        }
        break;

    /* Tier 3: traverse — custom angles, waits for steering */
    case ROVER_TRAVERSE:
        ArmSteeringGate(roverCtx.steer_fl, roverCtx.steer_fr,
                        roverCtx.steer_rl, roverCtx.steer_rr);
        SetSteeringAngles(roverCtx.steer_fl, roverCtx.steer_fr,
                          roverCtx.steer_rl, roverCtx.steer_rr);
        break;

    /* Tier 3: steer-only — set angles, drive is NEVER enabled */
    case ROVER_STEER_ONLY:
        driveEnabled     = 0;
        steeringBypassed = 0;
        steeringTimerArmed = 0;
        SetSteeringAngles(roverCtx.steer_fl, roverCtx.steer_fr,
                          roverCtx.steer_rl, roverCtx.steer_rr);
        break;

    /* Tier 4: per-wheel — waits for steering */
    case ROVER_WHEEL_CONTROL:
        ArmSteeringGate(roverCtx.wheel[0].angle, roverCtx.wheel[1].angle,
                        roverCtx.wheel[2].angle, roverCtx.wheel[3].angle);
        SetSteeringAngles(roverCtx.wheel[0].angle, roverCtx.wheel[1].angle,
                          roverCtx.wheel[2].angle, roverCtx.wheel[3].angle);
        break;

    /* All other Tier 1 states: no steering, no gate */
    default:
        driveEnabled     = 1;
        steeringBypassed = 0;
        steeringTimerArmed = 0;
        break;
    }

    return roverState;
}

void SetDefaultState(rover_state_t safeState)
{
    /* Only truly safe states are accepted as the watchdog default */
    if (safeState == ROVER_IDLE)
        defaultState = ROVER_IDLE;
}

rover_state_t CurrentRoverState(void)
{
    return roverState;
}

const rover_context_t *CurrentRoverContext(void)
{
    return &roverCtx;
}

uint8_t RoverDriveEnabled(void)
{
    return driveEnabled;
}

uint8_t RoverSteeringBypassed(void)
{
    return steeringBypassed;
}

/* ---- Init / Poll ------------------------------------------------------ */

void ControllerInit(void)
{
    memset(&roverCtx, 0, sizeof(roverCtx));

    /* Check if reset was from watchdog */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0x00u)
        printf("WATCHDOG RESET OCCURRED\r\n");
    __HAL_RCC_CLEAR_RESET_FLAGS();

    /* Init watchdog — timeout = Reload × 2 ms = 200 × 2 = 400 ms */
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    hiwdg.Init.Reload    = 200;
    hiwdg.Init.EWI       = 0;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        Error_Handler();

    SetScheduledTime(&commandExpiration, ROVER_COMMAND_INTERVAL_MAX);
    SetScheduledTime(&steeringStuckTimer, STEERING_STUCK_MS);
}

void ControllerPoll(void)
{
    HAL_IWDG_Refresh(&hiwdg);

    /* Loop time — 10-sample moving average */
    static uint32_t loopTime[10] = {0};
    static uint8_t  index = 0;
    static uint32_t lastTime = 0;
    loopTime[index % 10] = HAL_GetTick() - lastTime;
    avgLoopTime = 0;
    for (int i = 0; i < 10; i++) avgLoopTime += loopTime[i];
    avgLoopTime /= 10;
    index++;
    lastTime = HAL_GetTick();

    /* Command watchdog — revert to safe default if no command received */
    if (ScheduleReady(commandExpiration)) {
        if (!estopActive)
            roverState = defaultState;
    }

    /* Drive gate — check if steering has settled */
    if (!driveEnabled && steeringTimerArmed && roverState != ROVER_STEER_ONLY) {
        if (SteeringWheelsReady(STEERING_SETTLED_DEG)) {
            driveEnabled     = 1;
            steeringBypassed = 0;
            steeringTimerArmed = 0;
        } else if (ScheduleReady(steeringStuckTimer)) {
            driveEnabled     = 1;
            steeringBypassed = 1;
            steeringTimerArmed = 0;
        }
    }
}

uint32_t GetAvgLoopTime(void)
{
    return avgLoopTime;
}
