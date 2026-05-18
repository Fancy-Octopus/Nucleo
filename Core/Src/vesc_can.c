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
#include "luna_steering.h"

/* --------------------------------------------------------------------------
 * VESC node IDs
 * -------------------------------------------------------------------------- */
#define VESC_FL (uint8_t) 12   /* Front-Left  corner wheel */
#define VESC_FR (uint8_t) 13   /* Front-Right corner wheel */
#define VESC_BL (uint8_t) 10   /* Back-Left   corner wheel */
#define VESC_BR (uint8_t) 11   /* Back-Right  corner wheel */
#define VESC_FD (uint8_t) 5    /* Front dig drum           */
#define VESC_BD (uint8_t) 6    /* Back  dig drum           */

#define VESC_WHEEL_SPEED 5000.0f   /* default ERPM for Tier 1 wheel states  */
#define VESC_DRUM_SPEED  15000.0f   /* default ERPM for dig/deposit states   */
#define VESC_POLL_TIME   100        /* ms — control loop rate                */

/* --------------------------------------------------------------------------
 * VESC node ID table — must match VESC_COUNT in the header.
 * Index 0 = FD, 1 = BD, 2 = BL, 3 = BR, 4 = FL, 5 = FR.
 * -------------------------------------------------------------------------- */
static const uint8_t kVescNodeIds[VESC_COUNT] = { 5, 6, 10, 11, 12, 13 };

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */
static FDCAN_HandleTypeDef *vescCanHandle = NULL;
static schedule_t           canbus_schedule;
static vesc_t               vescData[VESC_COUNT];

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
 * Buffer helpers — decode (big-endian reads matching VESC firmware)
 * -------------------------------------------------------------------------- */
static int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index)
{
    int16_t v = (int16_t)((uint16_t)buffer[*index    ] << 8 |
                           (uint16_t)buffer[*index + 1]);
    *index += 2;
    return v;
}

__attribute__((unused))
static uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index)
{
    uint16_t v = (uint16_t)((uint16_t)buffer[*index    ] << 8 |
                              (uint16_t)buffer[*index + 1]);
    *index += 2;
    return v;
}

static int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index)
{
    int32_t v = (int32_t)((uint32_t)buffer[*index    ] << 24 |
                           (uint32_t)buffer[*index + 1] << 16 |
                           (uint32_t)buffer[*index + 2] <<  8 |
                           (uint32_t)buffer[*index + 3]);
    *index += 4;
    return v;
}

static uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index)
{
    uint32_t v = ((uint32_t)buffer[*index    ] << 24 |
                  (uint32_t)buffer[*index + 1] << 16 |
                  (uint32_t)buffer[*index + 2] <<  8 |
                  (uint32_t)buffer[*index + 3]);
    *index += 4;
    return v;
}

/* --------------------------------------------------------------------------
 * RX helpers
 * -------------------------------------------------------------------------- */

/* Returns pointer into vescData for the given node_id, or NULL if unknown. */
static vesc_t *vesc_node_lookup(uint8_t node_id)
{
    for (uint8_t i = 0; i < VESC_COUNT; i++)
    {
        if (vescData[i].node_id == node_id)
            return &vescData[i];
    }
    return NULL;
}

static void decode_status1(vesc_t *v, const uint8_t *d)
{
    int32_t idx = 0;
    v->rpm     = buffer_get_int32(d, &idx);
    v->current = (float)buffer_get_int16(d, &idx) / 10.0f;
    v->duty    = (float)buffer_get_int16(d, &idx) / 1000.0f;
}

static void decode_status2(vesc_t *v, const uint8_t *d)
{
    int32_t idx = 0;
    v->amp_hours         = (float)buffer_get_uint32(d, &idx) / 10000.0f;
    v->amp_hours_charged = (float)buffer_get_uint32(d, &idx) / 10000.0f;
}

static void decode_status3(vesc_t *v, const uint8_t *d)
{
    int32_t idx = 0;
    v->watt_hours         = (float)buffer_get_uint32(d, &idx) / 10000.0f;
    v->watt_hours_charged = (float)buffer_get_uint32(d, &idx) / 10000.0f;
}

static void decode_status4(vesc_t *v, const uint8_t *d)
{
    int32_t idx = 0;
    v->temp_fet   = (float)buffer_get_int16(d, &idx) / 10.0f;
    v->temp_motor = (float)buffer_get_int16(d, &idx) / 10.0f;
    v->current_in = (float)buffer_get_int16(d, &idx) / 10.0f;
    v->pid_pos    = (float)buffer_get_int16(d, &idx) / 50.0f;
}

static void decode_status5(vesc_t *v, const uint8_t *d)
{
    int32_t idx = 0;
    v->tacho = buffer_get_int32(d, &idx);
    v->v_in  = (float)buffer_get_int16(d, &idx) / 10.0f;
    (void)buffer_get_int16(d, &idx);  /* reserved / motor_id field */
}

static void decode_status6(vesc_t *v, const uint8_t *d)
{
    int32_t idx = 0;
    v->adc_1 = (float)buffer_get_int16(d, &idx) / 1000.0f;
    v->adc_2 = (float)buffer_get_int16(d, &idx) / 1000.0f;
    v->adc_3 = (float)buffer_get_int16(d, &idx) / 1000.0f;
    v->ppm   = (float)buffer_get_int16(d, &idx) / 1000.0f;
}

/* --------------------------------------------------------------------------
 * CAN_RxMessageCallback — overrides the __weak default in can_queue.c.
 * Called from interrupt context (CAN_USE_INTERRUPTS=1) or from CanQueue_Poll
 * (CAN_USE_INTERRUPTS=0) for every received frame.
 *
 * VESC extended ID format: bits[7:0]  = controller node ID
 *                          bits[15:8] = CAN_PACKET_ID
 * -------------------------------------------------------------------------- */
void CAN_RxMessageCallback(CanRxMessage_t *msg)
{
    if (msg->idType != FDCAN_EXTENDED_ID) return;

    uint8_t        node_id    = (uint8_t)(msg->id & 0xFF);
    CAN_PACKET_ID  packet_id  = (CAN_PACKET_ID)((msg->id >> 8) & 0xFF);

    vesc_t *v = vesc_node_lookup(node_id);
    if (v == NULL) return;

    switch (packet_id)
    {
    case CAN_PACKET_STATUS:   decode_status1(v, msg->data); break;
    case CAN_PACKET_STATUS_2: decode_status2(v, msg->data); break;
    case CAN_PACKET_STATUS_3: decode_status3(v, msg->data); break;
    case CAN_PACKET_STATUS_4: decode_status4(v, msg->data); break;
    case CAN_PACKET_STATUS_5: decode_status5(v, msg->data); break;
    case CAN_PACKET_STATUS_6: decode_status6(v, msg->data); break;
    default: return;  /* not a status packet — ignore */
    }

    v->connected    = 1;
    v->last_rx_tick = HAL_GetTick();
}

/* --------------------------------------------------------------------------
 * Data accessors
 * -------------------------------------------------------------------------- */
const vesc_t *VescGetNode(uint8_t node_id)
{
    return vesc_node_lookup(node_id);
}

int VescGetReport(uint8_t node_id, vesc_t *out)
{
    if (out == NULL) return -1;
    const vesc_t *v = vesc_node_lookup(node_id);
    if (v == NULL) return -1;
    *out = *v;
    return 0;
}

void VescGetAllReports(vesc_t out[VESC_COUNT])
{
    for (uint8_t i = 0; i < VESC_COUNT; i++)
        out[i] = vescData[i];
}

void VescGetStatus1All(vesc_status1_t out[VESC_COUNT])
{
    for (uint8_t i = 0; i < VESC_COUNT; i++)
    {
        out[i].node_id   = vescData[i].node_id;
        out[i].rpm       = vescData[i].rpm;
        out[i].current   = vescData[i].current;
        out[i].duty      = vescData[i].duty;
        out[i].connected = vescData[i].connected;
    }
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

    for (uint8_t i = 0; i < VESC_COUNT; i++)
    {
        vesc_t *v       = &vescData[i];
        memset(v, 0, sizeof(vesc_t));
        v->node_id = kVescNodeIds[i];
    }

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

    switch (state)
    {
    /* ---- Stop states ---- */
    case ROVER_IDLE:
    case ROVER_READY:
    case ROVER_ESTOP:
    case ROVER_STEER_ONLY:
        AllStop();
        break;

    /* ---- VESC output suppressed — do not send any CAN commands ---- */
    case ROVER_PROGRAM_VESCS:
        break;

    /* ---- Tier 1: straight forward — wait for steering ---- */
    case ROVER_FORWARD:
        if (!SteeringWheelsReady(STEERING_SETTLED_DEG)) { AllStop(); break; }
        /* fall through */
    case ROVER_FORWARD_FORCE:
        comm_can_set_rpm(VESC_FL,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FR,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BL,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BR,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 1: straight backward — wait for steering ---- */
    case ROVER_BACKWARD:
        if (!SteeringWheelsReady(STEERING_SETTLED_DEG)) { AllStop(); break; }
        /* fall through */
    case ROVER_BACKWARD_FORCE:
        comm_can_set_rpm(VESC_FL, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FR, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BL, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BR, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 1: spin-in-place right — wait for steering ---- */
    case ROVER_SPIN_RIGHT:
        if (!SteeringWheelsReady(STEERING_SETTLED_DEG)) { AllStop(); break; }
        /* fall through */
    case ROVER_SPIN_RIGHT_FORCE:
        comm_can_set_rpm(VESC_FL,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BL,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FR, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BR, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 1: spin-in-place left — wait for steering ---- */
    case ROVER_SPIN_LEFT:
        if (!SteeringWheelsReady(STEERING_SETTLED_DEG)) { AllStop(); break; }
        /* fall through */
    case ROVER_SPIN_LEFT_FORCE:
        comm_can_set_rpm(VESC_FL, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BL, -VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FR,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_BR,  VESC_WHEEL_SPEED);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 1: dig/deposit — wheels stopped, drums run ---- */
    case ROVER_DIG_FRONT:
        comm_can_set_rpm(VESC_FL, 0.0f); comm_can_set_rpm(VESC_FR, 0.0f);
        comm_can_set_rpm(VESC_BL, 0.0f); comm_can_set_rpm(VESC_BR, 0.0f);
        comm_can_set_rpm(VESC_FD,  VESC_DRUM_SPEED);
        comm_can_set_rpm(VESC_BD,  0.0f);
        break;

    case ROVER_DIG_BACK:
        comm_can_set_rpm(VESC_FL, 0.0f); comm_can_set_rpm(VESC_FR, 0.0f);
        comm_can_set_rpm(VESC_BL, 0.0f); comm_can_set_rpm(VESC_BR, 0.0f);
        comm_can_set_rpm(VESC_FD,  0.0f);
        comm_can_set_rpm(VESC_BD,  VESC_DRUM_SPEED);
        break;

    case ROVER_DIG_BOTH:
        comm_can_set_rpm(VESC_FL, 0.0f); comm_can_set_rpm(VESC_FR, 0.0f);
        comm_can_set_rpm(VESC_BL, 0.0f); comm_can_set_rpm(VESC_BR, 0.0f);
        comm_can_set_rpm(VESC_FD,  VESC_DRUM_SPEED);
        comm_can_set_rpm(VESC_BD,  VESC_DRUM_SPEED);
        break;

    case ROVER_DEPOSIT_FRONT:
        comm_can_set_rpm(VESC_FL, 0.0f); comm_can_set_rpm(VESC_FR, 0.0f);
        comm_can_set_rpm(VESC_BL, 0.0f); comm_can_set_rpm(VESC_BR, 0.0f);
        comm_can_set_rpm(VESC_FD, -VESC_DRUM_SPEED);
        comm_can_set_rpm(VESC_BD,  0.0f);
        break;

    case ROVER_DEPOSIT_BACK:
        comm_can_set_rpm(VESC_FL, 0.0f); comm_can_set_rpm(VESC_FR, 0.0f);
        comm_can_set_rpm(VESC_BL, 0.0f); comm_can_set_rpm(VESC_BR, 0.0f);
        comm_can_set_rpm(VESC_FD,  0.0f);
        comm_can_set_rpm(VESC_BD, -VESC_DRUM_SPEED);
        break;

    case ROVER_DEPOSIT_BOTH:
        comm_can_set_rpm(VESC_FL, 0.0f); comm_can_set_rpm(VESC_FR, 0.0f);
        comm_can_set_rpm(VESC_BL, 0.0f); comm_can_set_rpm(VESC_BR, 0.0f);
        comm_can_set_rpm(VESC_FD, -VESC_DRUM_SPEED);
        comm_can_set_rpm(VESC_BD, -VESC_DRUM_SPEED);
        break;

    /* ---- Tier 2: proportional drive — wait for steering ---- */
    case ROVER_DRIVE:
        if (!SteeringWheelsReady(STEERING_SETTLED_DEG)) { AllStop(); break; }
        /* fall through */
    case ROVER_DRIVE_FORCE:
        comm_can_set_rpm(VESC_FL, ctx->drive_speed);
        comm_can_set_rpm(VESC_FR, ctx->drive_speed);
        comm_can_set_rpm(VESC_BL, ctx->drive_speed);
        comm_can_set_rpm(VESC_BR, ctx->drive_speed);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 2: proportional spin — wait for steering ---- */
    case ROVER_SPIN: {
        if (!SteeringWheelsReady(STEERING_SETTLED_DEG)) { AllStop(); break; }
        /* fall through — C requires a statement before fall-through label */
    }
        /* fall through */
    case ROVER_SPIN_FORCE: {
        float spd  = (ctx->spin_speed >= 0.0f) ?  ctx->spin_speed : -ctx->spin_speed;
        float sign = (ctx->spin_speed >= 0.0f) ?  1.0f            : -1.0f;
        comm_can_set_rpm(VESC_FL,  sign * spd);
        comm_can_set_rpm(VESC_BL,  sign * spd);
        comm_can_set_rpm(VESC_FR, -sign * spd);
        comm_can_set_rpm(VESC_BR, -sign * spd);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;
    }

    /* ---- Tier 3: traverse — all wheels same speed, custom steering ---- */
    case ROVER_TRAVERSE:
        if (!SteeringWheelsReady(STEERING_SETTLED_DEG)) { AllStop(); break; }
        /* fall through */
    case ROVER_TRAVERSE_FORCE:
        comm_can_set_rpm(VESC_FL, ctx->drive_speed);
        comm_can_set_rpm(VESC_FR, ctx->drive_speed);
        comm_can_set_rpm(VESC_BL, ctx->drive_speed);
        comm_can_set_rpm(VESC_BR, ctx->drive_speed);
        comm_can_set_rpm(VESC_FD, 0.0f);
        comm_can_set_rpm(VESC_BD, 0.0f);
        break;

    /* ---- Tier 4: per-wheel speed and angle ---- */
    case ROVER_WHEEL_CONTROL:
        if (!SteeringWheelsReady(STEERING_SETTLED_DEG)) { AllStop(); break; }
        /* fall through */
    case ROVER_WHEEL_CONTROL_FORCE:
        comm_can_set_rpm(VESC_FL, ctx->wheel[0].speed);
        comm_can_set_rpm(VESC_FR, ctx->wheel[1].speed);
        comm_can_set_rpm(VESC_BL, ctx->wheel[2].speed);
        comm_can_set_rpm(VESC_BR, ctx->wheel[3].speed);
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
