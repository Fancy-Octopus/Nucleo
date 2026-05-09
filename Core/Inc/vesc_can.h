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

/* Number of VESC controllers tracked. Order: FD(5), BD(6), FL(10), FR(11), BL(12), BR(13). */
#define VESC_COUNT  6U

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

/* Call from the main polling loop */
void VescPoll(void);

/* --------------------------------------------------------------------------
 * Received VESC status — populated from CAN_PACKET_STATUS through STATUS_5.
 * last_rx_tick is HAL_GetTick() of the most recent packet from this node.
 * connected is 0 until at least one status packet has been received.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t  node_id;
    int32_t  rpm;           /* electrical RPM */
    float    current;       /* motor current (A) */
    float    duty;          /* duty cycle (-1.0 to +1.0) */
    float    amp_hours;
    float    amp_hours_charged;
    float    watt_hours;
    float    watt_hours_charged;
    float    temp_fet;      /* MOSFET temperature (°C) */
    float    temp_motor;    /* motor temperature (°C) */
    float    current_in;    /* input/battery current (A) */
    float    v_in;          /* input voltage (V) */
    int32_t  tacho;         /* tachometer value (counts) */
    uint8_t  connected;     /* 1 once any status packet received */
    uint32_t last_rx_tick;
} vesc_report_t;

/* --------------------------------------------------------------------------
 * Named VESC controller — bundles a human-readable label with live status.
 * Iterate vesc_controller_t[VESC_COUNT] to access all controllers by name.
 * -------------------------------------------------------------------------- */
typedef struct {
    const char   *name;    /* e.g. "FL", "BR", "FD" — points to static storage */
    vesc_report_t status;
} vesc_controller_t;

/* Fill *out with the cached status for the given VESC node ID.
 * Returns 0 on success, -1 if node_id is not one of the tracked controllers. */
int VescGetReport(uint8_t node_id, vesc_report_t *out);

/* Fill out[VESC_COUNT] with all controllers in fixed order:
 * FD(5), BD(6), FL(10), FR(11), BL(12), BR(13). */
void VescGetAllReports(vesc_report_t out[VESC_COUNT]);

/* Fill out[VESC_COUNT] with name + status for all controllers. */
void VescGetAllControllers(vesc_controller_t out[VESC_COUNT]);

/* --------------------------------------------------------------------------
 * VESC CAN command set
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
