/*
 * lunaterm.c
 *
 *  Created on: Mar 29, 2025
 *      Author: strongp
 *  Updated: May 2026
 */

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "rover_controller.h"
#include "luna_steering.h"

/* ---- Forward declarations ---- */
void ToggleUserLEDs(EmbeddedCli *cli, char *args, void *context);
void LunaTermSendChar(EmbeddedCli *embeddedCli, char c);
void PrintRoverState(EmbeddedCli *cli, char *args, void *context);
void WDReset(EmbeddedCli *cli, char *args, void *context);
void SetForwardState(EmbeddedCli *cli, char *args, void *context);
void SetBackwardState(EmbeddedCli *cli, char *args, void *context);
void SetReadyState(EmbeddedCli *cli, char *args, void *context);
void SetIdleState(EmbeddedCli *cli, char *args, void *context);
void SetEstop(EmbeddedCli *cli, char *args, void *context);
void SetArmsUp(EmbeddedCli *cli, char *args, void *context);
void SetArmsDown(EmbeddedCli *cli, char *args, void *context);
void SetSpinRight(EmbeddedCli *cli, char *args, void *context);
void SetSpinLeft(EmbeddedCli *cli, char *args, void *context);
void SetDigState(EmbeddedCli *cli, char *args, void *context);
void SetDrive(EmbeddedCli *cli, char *args, void *context);
void SetSpinSpeed(EmbeddedCli *cli, char *args, void *context);
void SetArmSpeed(EmbeddedCli *cli, char *args, void *context);
void SetSteerOnly(EmbeddedCli *cli, char *args, void *context);
void SteerHome(EmbeddedCli *cli, char *args, void *context);
void AvgLoopTime(EmbeddedCli *cli, char *args, void *context);
void PrintStepperStats(EmbeddedCli *cli, char *args, void *context);

EmbeddedCli *cli;

/* ---- Command bindings ---- */

static CliCommandBinding TogLed = {
    "tog-led", "Toggle user LEDs", 0, NULL, ToggleUserLEDs
};
static CliCommandBinding RoverStateCmd = {
    "get-state", "Print current rover state and drive gate status", 0, NULL, PrintRoverState
};
static CliCommandBinding WatchdogReset = {
    "wd-rst", "Force IWDG reset (stalls 2 s)", 0, NULL, WDReset
};
static CliCommandBinding SetRovForward = {
    "set-fwd", "Rover state -> FORWARD", 0, NULL, SetForwardState
};
static CliCommandBinding SetRovBackward = {
    "set-bwd", "Rover state -> BACKWARD", 0, NULL, SetBackwardState
};
static CliCommandBinding SetRovRdy = {
    "set-rdy", "Rover state -> READY (deprecated, behaves as IDLE)", 0, NULL, SetReadyState
};
static CliCommandBinding SetRovIdle = {
    "set-idle", "Rover state -> IDLE (exits ESTOP)", 0, NULL, SetIdleState
};
static CliCommandBinding SetRovEstop = {
    "set-estop", "Rover state -> ESTOP (sticky, steering disabled)", 0, NULL, SetEstop
};
static CliCommandBinding ArmsUp = {
    "arms-up", "Raise both arms at default speed", 0, NULL, SetArmsUp
};
static CliCommandBinding ArmsDown = {
    "arms-down", "Lower both arms at default speed", 0, NULL, SetArmsDown
};
static CliCommandBinding SpinRight = {
    "spin-right", "Rover state -> SPIN_RIGHT (default speed)", 0, NULL, SetSpinRight
};
static CliCommandBinding SpinLeft = {
    "spin-left", "Rover state -> SPIN_LEFT (default speed)", 0, NULL, SetSpinLeft
};
static CliCommandBinding SetDig = {
    "set-dig", "Rover state -> DIG_FORWARD", 0, NULL, SetDigState
};
static CliCommandBinding DriveCmd = {
    /* tokenizeArgs=1 so args are split on whitespace */
    "set-drive", "Tier 2: drive at speed ERPM  (e.g. set-drive 3000)", 1, NULL, SetDrive
};
static CliCommandBinding SpinSpdCmd = {
    "set-spin", "Tier 2: spin-in-place at speed ERPM  (e.g. set-spin 1500)", 1, NULL, SetSpinSpeed
};
static CliCommandBinding ArmSpdCmd = {
    "set-arm", "Tier 2: move arm at signed speed -127..127  (e.g. set-arm -50)", 1, NULL, SetArmSpeed
};
static CliCommandBinding SteerOnlyCmd = {
    "set-steer", "Tier 3: steer wheels only  (e.g. set-steer 30 -30 -30 30)", 1, NULL, SetSteerOnly
};
static CliCommandBinding SteerHomeCmd = {
    "steer-home", "Send homing command to L4 stepper controller", 0, NULL, SteerHome
};
static CliCommandBinding GetLoopTime = {
    "loop-time", "Average main-loop execution time (ms)", 0, NULL, AvgLoopTime
};
static CliCommandBinding StepperStats = {
    "stats-steps", "Print steering / drive-gate diagnostic counters", 0, NULL, PrintStepperStats
};

/* ---- Init ---- */

void LunaTermInit(void)
{
    EmbeddedCliConfig cliConfig = *embeddedCliDefaultConfig();
    cliConfig.maxBindingCount = 24;
    cli = embeddedCliNew(&cliConfig);
    cli->writeChar = LunaTermSendChar;

    embeddedCliAddBinding(cli, TogLed);
    embeddedCliAddBinding(cli, RoverStateCmd);
    embeddedCliAddBinding(cli, WatchdogReset);
    embeddedCliAddBinding(cli, SetRovForward);
    embeddedCliAddBinding(cli, SetRovBackward);
    embeddedCliAddBinding(cli, SetRovRdy);
    embeddedCliAddBinding(cli, SetRovIdle);
    embeddedCliAddBinding(cli, SetRovEstop);
    embeddedCliAddBinding(cli, ArmsUp);
    embeddedCliAddBinding(cli, ArmsDown);
    embeddedCliAddBinding(cli, SpinRight);
    embeddedCliAddBinding(cli, SpinLeft);
    embeddedCliAddBinding(cli, SetDig);
    embeddedCliAddBinding(cli, DriveCmd);
    embeddedCliAddBinding(cli, SpinSpdCmd);
    embeddedCliAddBinding(cli, ArmSpdCmd);
    embeddedCliAddBinding(cli, SteerOnlyCmd);
    embeddedCliAddBinding(cli, SteerHomeCmd);
    embeddedCliAddBinding(cli, GetLoopTime);
    embeddedCliAddBinding(cli, StepperStats);

    printf("\r\n");
}

/* ---- Poll ---- */

void LunaTermPoll(void)
{
    static uint8_t byteIn;
    byteIn = 0;
    HAL_UART_Receive(&hcom_uart[COM1], &byteIn, 1, 5);
    if (byteIn != 0) embeddedCliReceiveChar(cli, byteIn);
    embeddedCliProcess(cli);
}

void LunaTermSendChar(EmbeddedCli *embeddedCli, char c)
{
    uint8_t data = c;
    HAL_UART_Transmit(&hcom_uart[COM1], &data, 1, 5);
}

/* ---- Helpers ---- */

static const char *HstateName(steering_hstate_t h)
{
    switch (h) {
    case STEERING_STATE_IDLE:       return "IDLE";
    case STEERING_STATE_MOVING:     return "MOVING";
    case STEERING_STATE_AT_TARGET:  return "AT_TARGET";
    case STEERING_STATE_RESETTING:  return "RESETTING";
    case STEERING_STATE_DISABLED:   return "DISABLED";
    case STEERING_STATE_LINK_LOST:  return "LINK_LOST";
    default:                        return "?";
    }
}

/* ---- Command handlers ---- */

void ToggleUserLEDs(EmbeddedCli *cli, char *args, void *context)
{
    BSP_LED_Toggle(LED_GREEN);
    BSP_LED_Toggle(LED_YELLOW);
    BSP_LED_Toggle(LED_RED);
}

void PrintRoverState(EmbeddedCli *cli, char *args, void *context)
{
    rover_state_t          state = CurrentRoverState();
    const rover_context_t *ctx   = CurrentRoverContext();

    const char *name;
    switch (state) {
    case ROVER_IDLE:             name = "IDLE";             break;
    case ROVER_FORWARD:          name = "FORWARD";          break;
    case ROVER_BACKWARD:         name = "BACKWARD";         break;
    case ROVER_SPIN_RIGHT:       name = "SPIN_RIGHT";       break;
    case ROVER_SPIN_LEFT:        name = "SPIN_LEFT";        break;
    case ROVER_ARMS_DOWN:        name = "ARMS_DOWN";        break;
    case ROVER_ARMS_UP:          name = "ARMS_UP";          break;
    case ROVER_DIG_FORWARD:      name = "DIG_FORWARD";      break;
    case ROVER_DEPOSIT_FORWARD:  name = "DEPOSIT_FORWARD";  break;
    case ROVER_READY:            name = "READY (->IDLE)";   break;
    case ROVER_ESTOP:            name = "ESTOP";            break;
    case ROVER_DIG_BACKWARD:     name = "DIG_BACKWARD";     break;
    case ROVER_DEPOSIT_BACKWARD: name = "DEPOSIT_BACKWARD"; break;
    case ROVER_DRIVE:            name = "DRIVE (T2)";       break;
    case ROVER_SPIN:             name = "SPIN (T2)";        break;
    case ROVER_ARM_MOVE:         name = "ARM_MOVE (T2)";    break;
    case ROVER_TRAVERSE:         name = "TRAVERSE (T3)";    break;
    case ROVER_STEER_ONLY:       name = "STEER_ONLY (T3)";  break;
    case ROVER_WHEEL_CONTROL:    name = "WHEEL_CONTROL (T4)"; break;
    default:                     name = "UNKNOWN";          break;
    }
    printf("State:    %s (%u)\r\n", name, (unsigned)state);
    printf("Drive:    %s%s\r\n",
           RoverDriveEnabled()      ? "ENABLED"  : "GATED",
           RoverSteeringBypassed()  ? " (bypass)" : "");

    /* Print context fields relevant to the current tier */
    if (state == ROVER_DRIVE || state == ROVER_TRAVERSE) {
        printf("DriveSpd: %+.1f ERPM\r\n", (double)ctx->drive_speed);
    }
    if (state == ROVER_SPIN) {
        printf("SpinSpd:  %+.1f ERPM\r\n", (double)ctx->spin_speed);
    }
    if (state == ROVER_ARM_MOVE) {
        printf("ArmSpd:   %+.0f\r\n", (double)ctx->arm_speed);
    }
    if (state == ROVER_TRAVERSE || state == ROVER_STEER_ONLY || state == ROVER_WHEEL_CONTROL) {
        printf("Steer:    FL=%+6.2f  FR=%+6.2f  RL=%+6.2f  RR=%+6.2f (deg)\r\n",
               (double)ctx->steer_fl, (double)ctx->steer_fr,
               (double)ctx->steer_rl, (double)ctx->steer_rr);
    }
}

void WDReset(EmbeddedCli *cli, char *args, void *context)
{
    HAL_Delay(2000);
}

void SetForwardState(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_FORWARD);
}

void SetBackwardState(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_BACKWARD);
}

void SetReadyState(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_READY);
}

void SetIdleState(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_IDLE);
}

void SetEstop(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_ESTOP);
}

void SetArmsUp(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_ARMS_UP);
}

void SetArmsDown(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_ARMS_DOWN);
}

void SetSpinRight(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_SPIN_RIGHT);
}

void SetSpinLeft(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_SPIN_LEFT);
}

void SetDigState(EmbeddedCli *cli, char *args, void *context)
{
    RequestRoverState(ROVER_DIG_FORWARD);
}

void SetDrive(EmbeddedCli *cli, char *args, void *context)
{
    const char *t = embeddedCliGetToken(args, 1);
    if (!t) { printf("Usage: set-drive <erpm>\r\n"); return; }
    rover_context_t ctx = {0};
    ctx.drive_speed = strtof(t, NULL);
    RequestRoverStateCtx(ROVER_DRIVE, &ctx);
    printf("DRIVE -> %.1f ERPM\r\n", (double)ctx.drive_speed);
}

void SetSpinSpeed(EmbeddedCli *cli, char *args, void *context)
{
    const char *t = embeddedCliGetToken(args, 1);
    if (!t) { printf("Usage: set-spin <erpm>\r\n"); return; }
    rover_context_t ctx = {0};
    ctx.spin_speed = strtof(t, NULL);
    RequestRoverStateCtx(ROVER_SPIN, &ctx);
    printf("SPIN -> %.1f ERPM\r\n", (double)ctx.spin_speed);
}

void SetArmSpeed(EmbeddedCli *cli, char *args, void *context)
{
    const char *t = embeddedCliGetToken(args, 1);
    if (!t) { printf("Usage: set-arm <speed -127..127>\r\n"); return; }
    rover_context_t ctx = {0};
    ctx.arm_speed = strtof(t, NULL);
    RequestRoverStateCtx(ROVER_ARM_MOVE, &ctx);
    printf("ARM_MOVE -> %.0f\r\n", (double)ctx.arm_speed);
}

void SetSteerOnly(EmbeddedCli *cli, char *args, void *context)
{
    const char *t1 = embeddedCliGetToken(args, 1);
    const char *t2 = embeddedCliGetToken(args, 2);
    const char *t3 = embeddedCliGetToken(args, 3);
    const char *t4 = embeddedCliGetToken(args, 4);
    if (!t1 || !t2 || !t3 || !t4) {
        printf("Usage: set-steer <fl> <fr> <rl> <rr> (degrees)\r\n");
        return;
    }
    rover_context_t ctx = {0};
    ctx.steer_fl = strtof(t1, NULL);
    ctx.steer_fr = strtof(t2, NULL);
    ctx.steer_rl = strtof(t3, NULL);
    ctx.steer_rr = strtof(t4, NULL);
    RequestRoverStateCtx(ROVER_STEER_ONLY, &ctx);
    printf("STEER_ONLY -> FL=%+.2f FR=%+.2f RL=%+.2f RR=%+.2f\r\n",
           (double)ctx.steer_fl, (double)ctx.steer_fr,
           (double)ctx.steer_rl, (double)ctx.steer_rr);
}

void SteerHome(EmbeddedCli *cli, char *args, void *context)
{
    SteeringHome();
    printf("Homing command sent\r\n");
}

void AvgLoopTime(EmbeddedCli *cli, char *args, void *context)
{
    printf("Avg loop time: %u ms\r\n", (unsigned int)GetAvgLoopTime());
}

void PrintStepperStats(EmbeddedCli *cli, char *args, void *context)
{
    steering_status_t s = GetSteeringStatusPeek();
    steering_diag_t   d = GetSteeringDiag();

    /* ---- Link health ---- */
    printf("Init:     %s\r\n", d.init_ok ? "OK" : "FAILED");
    if (!d.ever_connected) {
        printf("Link:     NO REPLY from L4\r\n");
    } else if (d.link_active) {
        printf("Link:     OK  (hstate=%s)\r\n", HstateName(d.hstate));
    } else {
        printf("Link:     LOST  (hstate=%s, %lu drop(s))\r\n",
               HstateName(d.hstate), (unsigned long)d.link_lost_count);
    }

    printf("TX ok: %lu  TX err: %lu  RX bytes: %lu  RX good: %lu  "
           "Bad CRC: %lu  UART err: %lu\r\n",
           (unsigned long)d.tx_sent,   (unsigned long)d.tx_errors,
           (unsigned long)d.rx_bytes,  (unsigned long)d.rx_good,
           (unsigned long)d.rx_bad_crc, (unsigned long)d.uart_errors);

    /* ---- Drive gate ---- */
    printf("Drive:    %s%s\r\n",
           RoverDriveEnabled()     ? "ENABLED"  : "GATED",
           RoverSteeringBypassed() ? " (bypass)" : "");

    /* ---- Wheel angles ---- */
    if (d.link_active) {
        printf("Actual:   FL=%+6.2f  FR=%+6.2f  RL=%+6.2f  RR=%+6.2f (deg)\r\n",
               (double)(s.fl + 0.0f), (double)(s.fr + 0.0f),
               (double)(s.rl + 0.0f), (double)(s.rr + 0.0f));
        printf("Flags:    %s%s\r\n",
               (s.flags & STEERING_FLAG_HOMING) ? "HOMING " : "",
               (s.flags & STEERING_FLAG_HOMED)  ? "HOMED"   : "not-homed");
    } else {
        printf("Actual:   (no data)\r\n");
    }
}
