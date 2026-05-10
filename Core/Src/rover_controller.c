/*
 * rover_controller.c
 *
 *  Created on: Mar 30, 2025
 *      Author: strongp
 */

#include "rover_controller.h"
#include "main.h"
#include "luna_wait.h"
#include <string.h>

#define ROVER_COMMAND_INTERVAL_MAX  1000U   /* ms — command watchdog           */

/* ---- Private state ---------------------------------------------------- */
static rover_state_t    roverState   = ROVER_IDLE;
static rover_state_t    defaultState = ROVER_IDLE;
static rover_context_t  roverCtx;

/* ESTOP is sticky — only ROVER_IDLE can exit it */
static uint8_t          estopActive = 0;

rover_t rover;

IWDG_HandleTypeDef hiwdg;

static uint32_t avgLoopTime;
static schedule_t commandExpiration;

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

    if (reqState == ROVER_ESTOP)
        estopActive = 1;
    else if (reqState == ROVER_IDLE || reqState == ROVER_READY)
        estopActive = 0;

    roverState = reqState;
    if (ctx) roverCtx = *ctx;

    if (reqState == ROVER_IDLE)
        defaultState = ROVER_IDLE;

    ResetSchedule(&commandExpiration);
    return roverState;
}

void SetDefaultState(rover_state_t safeState)
{
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
}

uint32_t GetAvgLoopTime(void)
{
    return avgLoopTime;
}
