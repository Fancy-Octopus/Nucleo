/*
 * luna_steering.c
 *
 *  Created on: Apr 19, 2025
 *      Author: strongp
 */

#include "main.h"
#include "luna_steering.h"
#include "rover_controller.h"
#include <string.h>

/* ---- Private types ---- */
typedef enum {
    RX_WAIT_START = 0,
    RX_WAIT_TYPE,
    RX_PAYLOAD,
    RX_CRC
} rx_state_t;

/* ---- Private globals ---- */
static UART_HandleTypeDef steeringUart;
static steering_status_t  lastStatus;
static steering_diag_t    diag;
static rover_state_t      lastRoverState = ROVER_IDLE;
static uint32_t           lastRefreshTick = 0;

/* RX state machine */
static rx_state_t  rxState      = RX_WAIT_START;
static uint8_t     rxBuf[STEERING_STATUS_LEN];
static uint8_t     rxIdx        = 0;

/* ---- Private helpers ---- */

static uint8_t Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc <<= 1;
        }
    }
    return crc;
}

static void SendPacket(const uint8_t *pkt, uint8_t len)
{
    if (HAL_UART_Transmit(&steeringUart, (uint8_t *)pkt, len, 10) == HAL_OK)
        diag.tx_sent++;
    else
        diag.tx_errors++;
}

static void SendAngles(float fl, float fr, float rl, float rr)
{
    uint8_t pkt[STEERING_CMD_LEN];
    pkt[0] = STEERING_START_BYTE;
    pkt[1] = STEERING_CMD_TYPE;
    memcpy(&pkt[2],  &fl, 4);
    memcpy(&pkt[6],  &fr, 4);
    memcpy(&pkt[10], &rl, 4);
    memcpy(&pkt[14], &rr, 4);
    pkt[18] = Crc8(pkt, 18);
    SendPacket(pkt, STEERING_CMD_LEN);
}

static void SendQuery(void)
{
    uint8_t pkt[STEERING_QUERY_LEN];
    pkt[0] = STEERING_START_BYTE;
    pkt[1] = STEERING_QUERY_TYPE;
    pkt[2] = Crc8(pkt, 2);
    SendPacket(pkt, STEERING_QUERY_LEN);
}

static void SendHomeCommand(void)
{
    uint8_t pkt[STEERING_HOME_LEN];
    pkt[0] = STEERING_START_BYTE;
    pkt[1] = STEERING_HOME_TYPE;
    pkt[2] = Crc8(pkt, 2);
    SendPacket(pkt, STEERING_HOME_LEN);
}

static void SendResetCommand(uint8_t idx)
{
    uint8_t pkt[STEERING_RESET_LEN];
    pkt[0] = STEERING_START_BYTE;
    pkt[1] = STEERING_RESET_TYPE;
    pkt[2] = idx;
    pkt[3] = Crc8(pkt, 3);
    SendPacket(pkt, STEERING_RESET_LEN);
}

static void ProcessStatusPacket(const uint8_t *buf)
{
    /* buf[0]=0xAA, buf[1]=0x02, buf[2..17]=floats, buf[18]=flags, buf[19]=crc */
    if (Crc8(buf, STEERING_STATUS_LEN - 1) != buf[STEERING_STATUS_LEN - 1]) {
        diag.rx_bad_crc++;
        return;
    }

    memcpy(&lastStatus.fl, &buf[2],  4);
    memcpy(&lastStatus.fr, &buf[6],  4);
    memcpy(&lastStatus.rl, &buf[10], 4);
    memcpy(&lastStatus.rr, &buf[14], 4);
    lastStatus.flags    = buf[18];
    lastStatus.valid    = 1;
    diag.rx_good++;
    diag.ever_connected = 1;
}

static void DrainRx(void)
{
    uint8_t b;
    HAL_StatusTypeDef ret;

    while ((ret = HAL_UART_Receive(&steeringUart, &b, 1, 0)) == HAL_OK) {
        diag.rx_bytes++;
        switch (rxState) {
        case RX_WAIT_START:
            if (b == STEERING_START_BYTE) {
                rxBuf[0] = b;
                rxState  = RX_WAIT_TYPE;
            }
            break;

        case RX_WAIT_TYPE:
            if (b == STEERING_STATUS_TYPE) {
                rxBuf[1] = b;
                rxIdx    = 2;
                rxState  = RX_PAYLOAD;
            } else {
                rxState = RX_WAIT_START;
            }
            break;

        case RX_PAYLOAD:
            rxBuf[rxIdx++] = b;
            /* 18 bytes of payload: indices 2..19 minus the final CRC byte */
            if (rxIdx == STEERING_STATUS_LEN - 1)
                rxState = RX_CRC;
            break;

        case RX_CRC:
            rxBuf[rxIdx] = b;
            ProcessStatusPacket(rxBuf);
            rxState = RX_WAIT_START;
            break;
        }
    }

    /* Accumulate UART hardware errors (framing, overrun, noise).
     * HAL sets ErrorCode when any of these fire; clear it so we don't double-count. */
    if (steeringUart.ErrorCode != HAL_UART_ERROR_NONE) {
        diag.uart_errors++;
        steeringUart.ErrorCode = HAL_UART_ERROR_NONE;
        /* Re-arm the peripheral — overrun/framing errors can stall the receiver */
        __HAL_UART_CLEAR_OREFLAG(&steeringUart);
        __HAL_UART_CLEAR_FEFLAG(&steeringUart);
        __HAL_UART_CLEAR_NEFLAG(&steeringUart);
    }
}

/* ---- Public API ---- */

void SteeringInit(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PC12 — UART5 TX, AF8 */
    gpio.Pin       = GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* PD2 — UART5 RX, AF8 */
    gpio.Pin       = GPIO_PIN_2;
    gpio.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &gpio);

    steeringUart.Instance            = UART5;
    steeringUart.Init.BaudRate       = 460800;
    steeringUart.Init.Mode           = UART_MODE_TX_RX;
    steeringUart.Init.Parity         = UART_PARITY_NONE;
    steeringUart.Init.WordLength     = UART_WORDLENGTH_8B;
    steeringUart.Init.StopBits       = UART_STOPBITS_1;
    steeringUart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    steeringUart.Init.OverSampling   = UART_OVERSAMPLING_8;
    steeringUart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    diag.init_ok = (HAL_UART_Init(&steeringUart) == HAL_OK) ? 1 : 0;

    /* Send initial straight-angle command */
    SendAngles(STEERING_ANGLE_STRAIGHT, STEERING_ANGLE_STRAIGHT,
               STEERING_ANGLE_STRAIGHT, STEERING_ANGLE_STRAIGHT);
}

void SteeringPoll(void)
{
    DrainRx();

    /* Periodic status refresh — keeps cached data fresh for any consumer
     * (CLI, TCP, telemetry) without requiring an explicit request. */
    uint32_t now = HAL_GetTick();
    if (now - lastRefreshTick >= STEERING_REFRESH_MS) {
        lastRefreshTick = now;
        SendQuery();
    }

    rover_state_t state = CurrentRoverState();
    if (state == lastRoverState)
        return;
    lastRoverState = state;

    switch (state) {
    case ROVER_TURN_RIGHT:
        SendAngles(STEERING_SPIN_FL, STEERING_SPIN_FR,
                   STEERING_SPIN_RL, STEERING_SPIN_RR);
        break;
    case ROVER_TURN_LEFT:
        SendAngles(-STEERING_SPIN_FL, -STEERING_SPIN_FR,
                   -STEERING_SPIN_RL, -STEERING_SPIN_RR);
        break;
    default:
        SendAngles(STEERING_ANGLE_STRAIGHT, STEERING_ANGLE_STRAIGHT,
                   STEERING_ANGLE_STRAIGHT, STEERING_ANGLE_STRAIGHT);
        break;
    }
}

void SetSteeringAngles(float fl, float fr, float rl, float rr)
{
    SendAngles(fl, fr, rl, rr);
}

void SteeringRequestStatus(void)
{
    SendQuery();
}

void SteeringHome(void)
{
    SendHomeCommand();
}

void SteeringResetDriver(uint8_t idx)
{
    SendResetCommand(idx);
}

steering_status_t GetSteeringStatus(void)
{
    steering_status_t s = lastStatus;
    lastStatus.valid = 0;
    return s;
}

steering_diag_t GetSteeringDiag(void)
{
    return diag;
}
