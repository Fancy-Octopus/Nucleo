/*
 * vesc_can.h
 *
 *  Created on: Apr 19, 2026
 *      Author: gruetzmacherg
 */

#ifndef INC_VESC_CAN_H_
#define INC_VESC_CAN_H_

#include "stm32h5xx_hal.h"
#include "can_queue.h"

/* Number of VESC controllers tracked.
 * Fixed internal order: FD(5), BD(6), BL(10), BR(11), FL(12), FR(13). */
#define VESC_COUNT  6U

/* --------------------------------------------------------------------------
 * Full VESC CAN packet-type enumeration
 * Matches vedderb/bldc datatypes.h.  All known packet IDs are present even
 * if only the STATUS packets are decoded on the receive path.
 * -------------------------------------------------------------------------- */
typedef enum {
    CAN_PACKET_SET_DUTY                        = 0,
    CAN_PACKET_SET_CURRENT                     = 1,
    CAN_PACKET_SET_CURRENT_BRAKE               = 2,
    CAN_PACKET_SET_RPM                         = 3,
    CAN_PACKET_SET_POS                         = 4,
    CAN_PACKET_FILL_RX_BUFFER                  = 5,
    CAN_PACKET_FILL_RX_BUFFER_LONG             = 6,
    CAN_PACKET_PROCESS_RX_BUFFER               = 7,
    CAN_PACKET_PROCESS_SHORT_BUFFER            = 8,
    CAN_PACKET_STATUS                          = 9,   /* STATUS 1: rpm, current, duty */
    CAN_PACKET_SET_CURRENT_REL                 = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL           = 11,
    CAN_PACKET_SET_CURRENT_HANDBRAKE           = 12,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL       = 13,
    CAN_PACKET_STATUS_2                        = 14,  /* STATUS 2: amp_hours, amp_hours_charged */
    CAN_PACKET_STATUS_3                        = 15,  /* STATUS 3: watt_hours, watt_hours_charged */
    CAN_PACKET_STATUS_4                        = 16,  /* STATUS 4: temp_fet, temp_motor, current_in, pid_pos */
    CAN_PACKET_PING                            = 17,
    CAN_PACKET_PONG                            = 18,
    CAN_PACKET_DETECT_APPLY_ALL_FOC            = 19,
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES        = 20,
    CAN_PACKET_CONF_CURRENT_LIMITS             = 21,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS       = 22,
    CAN_PACKET_CONF_CURRENT_LIMITS_IN          = 23,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN    = 24,
    CAN_PACKET_CONF_FOC_ERPMS                  = 25,
    CAN_PACKET_CONF_STORE_FOC_ERPMS            = 26,
    CAN_PACKET_STATUS_5                        = 27,  /* STATUS 5: tacho_value, v_in */
    CAN_PACKET_POLL_TS5700N8501_STATUS         = 28,
    CAN_PACKET_CONF_BATTERY_CUT                = 29,
    CAN_PACKET_CONF_STORE_BATTERY_CUT          = 30,
    CAN_PACKET_SHUTDOWN                        = 31,
    CAN_PACKET_IO_BOARD_ADC_1_TO_4             = 32,
    CAN_PACKET_IO_BOARD_ADC_5_TO_8             = 33,
    CAN_PACKET_IO_BOARD_ADC_9_TO_12            = 34,
    CAN_PACKET_IO_BOARD_DIGITAL_IN             = 35,
    CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL     = 36,
    CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM         = 37,
    CAN_PACKET_BMS_V_TOT                       = 38,
    CAN_PACKET_BMS_I                           = 39,
    CAN_PACKET_BMS_AH_WH                       = 40,
    CAN_PACKET_BMS_V_CELL                      = 41,
    CAN_PACKET_BMS_BAL                         = 42,
    CAN_PACKET_BMS_TEMPS                       = 43,
    CAN_PACKET_BMS_HUM                         = 44,
    CAN_PACKET_BMS_SOC_SOH_TEMP_STAT           = 45,
    CAN_PACKET_PSW_STAT                        = 46,
    CAN_PACKET_PSW_SWITCH                      = 47,
    CAN_PACKET_BMS_HW_DATA_1                   = 48,
    CAN_PACKET_BMS_HW_DATA_2                   = 49,
    CAN_PACKET_BMS_HW_DATA_3                   = 50,
    CAN_PACKET_BMS_HW_DATA_4                   = 51,
    CAN_PACKET_BMS_HW_DATA_5                   = 52,
    CAN_PACKET_BMS_AH_WH_CHG_TOTAL             = 53,
    CAN_PACKET_BMS_AH_WH_DIS_TOTAL             = 54,
    CAN_PACKET_UPDATE_PID_POS_OFFSET           = 55,
    CAN_PACKET_POLL_ROTOR_POS                  = 56,
    CAN_PACKET_NOTIFY_BOOT                     = 57,
    CAN_PACKET_STATUS_6                        = 58,  /* STATUS 6: adc_1, adc_2, adc_3, ppm */
    CAN_PACKET_GNSS_TIME                       = 59,
    CAN_PACKET_GNSS_LAT                        = 60,
    CAN_PACKET_GNSS_LON                        = 61,
    CAN_PACKET_GNSS_ALT_SPEED_HDOP             = 62,
    CAN_PACKET_MAKE_ENUM_32_BITS               = 0xFFFFFFFF,
} CAN_PACKET_ID;

/* --------------------------------------------------------------------------
 * Per-VESC live data.
 * Fields are populated as STATUS packets arrive; the comment on each field
 * identifies which STATUS packet writes it.
 * 'connected' is 0 until at least one status packet has been received.
 * 'last_rx_tick' is HAL_GetTick() of the most recent status packet.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t  node_id;

    /* STATUS 1 (CAN_PACKET_STATUS) */
    int32_t  rpm;               /* electrical RPM */
    float    current;           /* motor current (A) */
    float    duty;              /* duty cycle  (-1.0 to +1.0) */

    /* STATUS 2 (CAN_PACKET_STATUS_2) */
    float    amp_hours;
    float    amp_hours_charged;

    /* STATUS 3 (CAN_PACKET_STATUS_3) */
    float    watt_hours;
    float    watt_hours_charged;

    /* STATUS 4 (CAN_PACKET_STATUS_4) */
    float    temp_fet;          /* MOSFET temperature (°C) */
    float    temp_motor;        /* motor temperature  (°C) */
    float    current_in;        /* input/battery current (A) */
    float    pid_pos;           /* PID position (deg) */

    /* STATUS 5 (CAN_PACKET_STATUS_5) */
    int32_t  tacho;             /* tachometer value (counts) */
    float    v_in;              /* input voltage (V) */

    /* STATUS 6 (CAN_PACKET_STATUS_6) */
    float    adc_1;
    float    adc_2;
    float    adc_3;
    float    ppm;

    uint8_t  connected;         /* 1 once any status packet received */
    uint32_t last_rx_tick;      /* HAL_GetTick() of most recent status packet */
} vesc_t;

/* --------------------------------------------------------------------------
 * Initialisation — call sequence in main.c:
 *
 *   ALT_MX_FDCAN1_Init(&hfdcan1);
 *   CAN_HardwareInit(&hfdcan1, FDCAN1_IT0_IRQn, 5);
 *   CanQueue_Init();
 *   VescInit(&hfdcan1);
 *
 * Main loop:
 *   VescPoll();
 *   CAN_BusRecoveryPoll();
 *   CanQueue_Poll();
 * -------------------------------------------------------------------------- */
void ALT_MX_FDCAN1_Init(FDCAN_HandleTypeDef *hfdcan1);
int  VescInit(FDCAN_HandleTypeDef *hfdcan);

/* Call from the main polling loop. Drives both TX control and RX processing. */
void VescPoll(void);

/* --------------------------------------------------------------------------
 * Compact STATUS-1 snapshot — rpm, current, duty for all six controllers.
 * Suitable for telemetry and CLI display without exposing the full vesc_t.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t node_id;
    int32_t rpm;
    float   current;   /* motor current (A) */
    float   duty;      /* duty cycle (-1.0 to +1.0) */
    uint8_t connected; /* 1 once any status packet received */
} vesc_status1_t;

/* Fills out[VESC_COUNT] in fixed order: FD(5), BD(6), BL(10), BR(11), FL(12), FR(13). */
void VescGetStatus1All(vesc_status1_t out[VESC_COUNT]);

/* --------------------------------------------------------------------------
 * Data accessors
 *
 * VescGetNode        — returns a read-only pointer to the internal vesc_t for
 *                      the given node ID, or NULL if not tracked.  The pointer
 *                      is stable for the lifetime of the program.
 *
 * VescGetReport      — copies the current state for one node into caller-
 *                      owned storage.  Returns 0 on success, -1 if node_id
 *                      is not one of the tracked controllers.
 *
 * VescGetAllReports  — copies all VESC_COUNT entries into caller-owned storage
 *                      in fixed order: FD(5), BD(6), BL(10), BR(11), FL(12), FR(13).
 * -------------------------------------------------------------------------- */
const vesc_t *VescGetNode(uint8_t node_id);
int           VescGetReport(uint8_t node_id, vesc_t *out);
void          VescGetAllReports(vesc_t out[VESC_COUNT]);

/* --------------------------------------------------------------------------
 * VESC CAN TX command set
 * -------------------------------------------------------------------------- */
void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay);
void comm_can_set_current_brake(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);
void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay);
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);
void comm_can_set_handbrake(uint8_t controller_id, float current);
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);

#endif /* INC_VESC_CAN_H_ */
