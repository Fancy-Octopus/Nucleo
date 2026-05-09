/*
 * vesc_can.c
 *
 *  Created on: Apr 19, 2026
 *      Author: gruetzmacherg
 */

#include "vesc_can.h"
#include "string.h"
#include "lunautils.h"
#include "luna_wait.h"
#include "rover_controller.h"

/* --------------------------------------------------------------------------
 * VESC node IDs
 * -------------------------------------------------------------------------- */
#define VESC_FL (uint8_t) 10   /* Front-Left  corner wheel */
#define VESC_FR (uint8_t) 11   /* Front-Right corner wheel */
#define VESC_BL (uint8_t) 12   /* Back-Left   corner wheel */
#define VESC_BR (uint8_t) 13   /* Back-Right  corner wheel */
#define VESC_FD (uint8_t) 5    /* Front dig drum           */
#define VESC_BD (uint8_t) 6    /* Back  dig drum           */

#define VESC_WHEEL_SPEED 5000.0f   /* default ERPM for Tier 1 wheel states  */
#define VESC_DRUM_SPEED  7000.0f   /* default ERPM for dig/deposit states   */
#define VESC_POLL_TIME   100        /* ms — control loop rate                */

/* --------------------------------------------------------------------------
 * VESC CAN packet types
 * -------------------------------------------------------------------------- */
typedef enum {
    CAN_PACKET_SET_DUTY                  = 0,
    CAN_PACKET_SET_CURRENT               = 1,
    CAN_PACKET_SET_CURRENT_BRAKE         = 2,
    CAN_PACKET_SET_RPM                   = 3,
    CAN_PACKET_SET_POS                   = 4,
    CAN_PACKET_SET_CURRENT_REL           = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL     = 11,
    CAN_PACKET_SET_CURRENT_HANDBRAKE     = 12,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
    CAN_PACKET_MAKE_ENUM_32_BITS         = 0xFFFFFFFF,
} CAN_PACKET_ID;

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */
static FDCAN_HandleTypeDef *vescCanHandle = NULL;
static schedule_t           canbus_schedule;

/* --------------------------------------------------------------------------
 * Buffer helpers (private)
 * -------------------------------------------------------------------------- */
static void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

static void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

static void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index)
{
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

static void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index)
{
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

/* --------------------------------------------------------------------------
 * Hardware register configuration
 * -------------------------------------------------------------------------- */
void ALT_MX_FDCAN1_Init(FDCAN_HandleTypeDef *hfdcan1)
{
    hfdcan1->Instance                  = FDCAN1;
    hfdcan1->Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1->Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan1->Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1->Init.AutoRetransmission   = ENABLE;
    hfdcan1->Init.TransmitPause        = DISABLE;
    hfdcan1->Init.ProtocolException    = DISABLE;
    hfdcan1->Init.NominalPrescaler     = 5;
    hfdcan1->Init.NominalSyncJumpWidth = 2;
    hfdcan1->Init.NominalTimeSeg1      = 7;
    hfdcan1->Init.NominalTimeSeg2      = 2;
    hfdcan1->Init.DataPrescaler        = 1;
    hfdcan1->Init.DataSyncJumpWidth    = 1;
    hfdcan1->Init.DataTimeSeg1         = 1;
    hfdcan1->Init.DataTimeSeg2         = 1;
    hfdcan1->Init.StdFiltersNbr        = 1;
    hfdcan1->Init.ExtFiltersNbr        = 1;
    hfdcan1->Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(hfdcan1) != HAL_OK)
        Error_Handler();
}

/* --------------------------------------------------------------------------
 * VESC initialisation
 * -------------------------------------------------------------------------- */
int VescInit(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig;

    vescCanHandle = hfdcan;

    sFilterConfig.IdType       = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0;
    sFilterConfig.FilterID2    = 0;

    if (HAL_FDCAN_ConfigFilter(vescCanHandle, &sFilterConfig) != HAL_OK) return 1;

    if (HAL_FDCAN_ConfigGlobalFilter(vescCanHandle,
            FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) return 1;

    SetScheduledTime(&canbus_schedule, VESC_POLL_TIME);

    return 0;
}

/* --------------------------------------------------------------------------
 * Helper: zero all wheels and drums
 * -------------------------------------------------------------------------- */
static void AllStop(void)
{
    comm_can_set_rpm(VESC_FL, 0.0f);
    comm_can_set_rpm(VESC_FR, 0.0f);
    comm_can_set_rpm(VESC_BL, 0.0f);
    comm_can_set_rpm(VESC_BR, 0.0f);
    comm_can_set_rpm(VESC_FD, 0.0f);
    comm_can_set_rpm(VESC_BD, 0.0f);
}

/* --------------------------------------------------------------------------
 * Polling loop entry point
 * -------------------------------------------------------------------------- */
void VescPoll(void)
{
    if (vescCanHandle == NULL)           return;
    if (!ScheduleReady(canbus_schedule)) return;
    ResetSchedule(&canbus_schedule);

    rover_state_t          state = CurrentRoverState();
    const rover_context_t *ctx   = CurrentRoverContext();
    uint8_t                drive = RoverDriveEnabled();

    switch (state)
    {
    /* ---- Tier 1: all-stop states ---- */
    case ROVER_IDLE:
    case ROVER_READY:
    case ROVER_STEER_ONLY:
        AllStop();
        break;

    /* ---- Tier 1: ESTOP — zero everything ---- */
    case ROVER_ESTOP:
        AllStop();
        break;

    /* ---- Tier 1: straight forward ---- */
    case ROVER_FORWARD:
        comm_can_set_rpm(VESC_FL,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FR,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BL,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BR,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 1: straight backward ---- */
    case ROVER_BACKWARD:
        comm_can_set_rpm(VESC_FL, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FR, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BL, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BR, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 1: spin-in-place right — wait for steering ---- */
    case ROVER_SPIN_RIGHT:
        if (drive) {
            comm_can_set_rpm(VESC_FL,  VESC_WHEEL_SPEED);
            comm_can_set_rpm(VESC_BL,  VESC_WHEEL_SPEED);
            comm_can_set_rpm(VESC_FR, -VESC_WHEEL_SPEED);
            comm_can_set_rpm(VESC_BR, -VESC_WHEEL_SPEED);
        } else {
            comm_can_set_rpm(VESC_FL, 0.0f);
            comm_can_set_rpm(VESC_FR, 0.0f);
            comm_can_set_rpm(VESC_BL, 0.0f);
            comm_can_set_rpm(VESC_BR, 0.0f);
        }
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 1: spin-in-place left — wait for steering ---- */
    case ROVER_SPIN_LEFT:
        if (drive) {
            comm_can_set_rpm(VESC_FL, -VESC_WHEEL_SPEED);
            comm_can_set_rpm(VESC_BL, -VESC_WHEEL_SPEED);
            comm_can_set_rpm(VESC_FR,  VESC_WHEEL_SPEED);
            comm_can_set_rpm(VESC_BR,  VESC_WHEEL_SPEED);
        } else {
            comm_can_set_rpm(VESC_FL, 0.0f);
            comm_can_set_rpm(VESC_FR, 0.0f);
            comm_can_set_rpm(VESC_BL, 0.0f);
            comm_can_set_rpm(VESC_BR, 0.0f);
        }
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 1: dig states ---- */
    case ROVER_DIG_FORWARD:
    case ROVER_DIG_BACKWARD:
        comm_can_set_rpm(VESC_FL, 0.0f);
        comm_can_set_rpm(VESC_FR, 0.0f);
        comm_can_set_rpm(VESC_BL, 0.0f);
        comm_can_set_rpm(VESC_BR, 0.0f);
        comm_can_set_rpm(VESC_FD, VESC_DRUM_SPEED);
        comm_can_set_rpm(VESC_BD, VESC_DRUM_SPEED);
        break;

    case ROVER_DEPOSIT_FORWARD:
        comm_can_set_rpm(VESC_FL, 0.0f);
        comm_can_set_rpm(VESC_FR, 0.0f);
        comm_can_set_rpm(VESC_BL, 0.0f);
        comm_can_set_rpm(VESC_BR, 0.0f);
        comm_can_set_rpm(VESC_FD, -VESC_DRUM_SPEED);
        comm_can_set_rpm(VESC_BD,  0.0f);
        break;

    case ROVER_DEPOSIT_BACKWARD:
        comm_can_set_rpm(VESC_FL, 0.0f);
        comm_can_set_rpm(VESC_FR, 0.0f);
        comm_can_set_rpm(VESC_BL, 0.0f);
        comm_can_set_rpm(VESC_BR, 0.0f);
        comm_can_set_rpm(VESC_FD,  0.0f);
        comm_can_set_rpm(VESC_BD, -VESC_DRUM_SPEED);
        break;

    /* ---- Tier 2: proportional drive ---- */
    case ROVER_DRIVE:
        comm_can_set_rpm(VESC_FL, ctx->drive_speed);
        comm_can_set_rpm(VESC_FR, ctx->drive_speed);
        comm_can_set_rpm(VESC_BL, ctx->drive_speed);
        comm_can_set_rpm(VESC_BR, ctx->drive_speed);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 2: proportional spin — wait for steering ---- */
    case ROVER_SPIN: {
        float spd = (ctx->spin_speed >= 0.0f) ?  ctx->spin_speed
                                                : -ctx->spin_speed;
        float sign = (ctx->spin_speed >= 0.0f) ? 1.0f : -1.0f;
        if (drive) {
            comm_can_set_rpm(VESC_FL,  sign * spd);
            comm_can_set_rpm(VESC_BL,  sign * spd);
            comm_can_set_rpm(VESC_FR, -sign * spd);
            comm_can_set_rpm(VESC_BR, -sign * spd);
        } else {
            comm_can_set_rpm(VESC_FL, 0.0f);
            comm_can_set_rpm(VESC_FR, 0.0f);
            comm_can_set_rpm(VESC_BL, 0.0f);
            comm_can_set_rpm(VESC_BR, 0.0f);
        }
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;
    }

    /* ---- Tier 3: traverse — all wheels same speed, custom steering ---- */
    case ROVER_TRAVERSE:
        if (drive) {
            comm_can_set_rpm(VESC_FL, ctx->drive_speed);
            comm_can_set_rpm(VESC_FR, ctx->drive_speed);
            comm_can_set_rpm(VESC_BL, ctx->drive_speed);
            comm_can_set_rpm(VESC_BR, ctx->drive_speed);
        } else {
            comm_can_set_rpm(VESC_FL, 0.0f);
            comm_can_set_rpm(VESC_FR, 0.0f);
            comm_can_set_rpm(VESC_BL, 0.0f);
            comm_can_set_rpm(VESC_BR, 0.0f);
        }
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 4: per-wheel speed and angle ---- */
    case ROVER_WHEEL_CONTROL:
        if (drive) {
            comm_can_set_rpm(VESC_FL, ctx->wheel[0].speed);
            comm_can_set_rpm(VESC_FR, ctx->wheel[1].speed);
            comm_can_set_rpm(VESC_BL, ctx->wheel[2].speed);
            comm_can_set_rpm(VESC_BR, ctx->wheel[3].speed);
        } else {
            comm_can_set_rpm(VESC_FL, 0.0f);
            comm_can_set_rpm(VESC_FR, 0.0f);
            comm_can_set_rpm(VESC_BL, 0.0f);
            comm_can_set_rpm(VESC_BR, 0.0f);
        }
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    default:
        AllStop();
        break;
    }
}

/* --------------------------------------------------------------------------
 * VESC CAN command implementations
 * -------------------------------------------------------------------------- */
void comm_can_set_duty(uint8_t controller_id, float duty)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[6];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
    buffer_append_float16(buffer, off_delay, 1e3f, &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_current_brake(uint8_t controller_id, float current)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, pos, 1000000.0f, &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

void comm_can_set_current_rel(uint8_t controller_id, float current_rel)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 100000.0f, &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
}

void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[6];
    buffer_append_float32(buffer, current_rel, 100000.0f, &send_index);
    buffer_append_float16(buffer, off_delay, 1e3f, &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
}

void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 100000.0f, &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
}

void comm_can_set_handbrake(uint8_t controller_id, float current)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current, 1000.0f, &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
}

void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel)
{
    if (vescCanHandle == NULL) return;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 100000.0f, &send_index);
    can_transmit_eid(vescCanHandle, controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index);
}
