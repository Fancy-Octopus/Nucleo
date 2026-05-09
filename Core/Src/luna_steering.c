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

/* Last angles commanded to the L4 — used by SteeringWheelsReady() */
static float targetFl = 0.0f, targetFr = 0.0f,
             targetRl = 0.0f, targetRr = 0.0f;

/* RX interrupt state machine */
static volatile uint8_t steeringRxByte;
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

static void SendAnglesRaw(float fl, float fr, float rl, float rr)
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

static void SendDisableCommand(uint8_t idx)
{
    uint8_t pkt[STEERING_DISABLE_LEN];
    pkt[0] = STEERING_START_BYTE;
    pkt[1] = STEERING_DISABLE_TYPE;
    pkt[2] = idx;
    pkt[3] = Crc8(pkt, 3);
    SendPacket(pkt, STEERING_DISABLE_LEN);
}

static void SendEnableCommand(uint8_t idx)
{
    uint8_t pkt[STEERING_ENABLE_LEN];
    pkt[0] = STEERING_START_BYTE;
    pkt[1] = STEERING_ENABLE_TYPE;
    pkt[2] = idx;
    pkt[3] = Crc8(pkt, 3);
    SendPacket(pkt, STEERING_ENABLE_LEN);
}

static void ProcessStatusPacket(const uint8_t *buf)
{
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
    diag.link_active    = 1;
    diag.last_rx_tick   = HAL_GetTick();

    /* Update sub-state based on link recovery */
    if (diag.hstate == STEERING_STATE_LINK_LOST)
        diag.hstate = STEERING_STATE_IDLE;
}

/* ---- IRQ handlers ---- */

void UART5_IRQHandler(void)
{
    HAL_UART_IRQHandler(&steeringUart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5)
        return;

    uint8_t b = steeringRxByte;
    diag.rx_bytes++;

    switch (rxState) {
    case RX_WAIT_START:
        if (b == STEERING_START_BYTE) { rxBuf[0] = b; rxState = RX_WAIT_TYPE; }
        break;
    case RX_WAIT_TYPE:
        if (b == STEERING_STATUS_TYPE) {
            rxBuf[1] = b; rxIdx = 2; rxState = RX_PAYLOAD;
        } else {
            rxState = RX_WAIT_START;
        }
        break;
    case RX_PAYLOAD:
        rxBuf[rxIdx++] = b;
        if (rxIdx == STEERING_STATUS_LEN - 1) rxState = RX_CRC;
        break;
    case RX_CRC:
        rxBuf[rxIdx] = b;
        ProcessStatusPacket(rxBuf);
        rxState = RX_WAIT_START;
        break;
    }

    HAL_UART_Receive_IT(&steeringUart, (uint8_t *)&steeringRxByte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5)
        return;

    diag.uart_errors++;
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    HAL_UART_Receive_IT(&steeringUart, (uint8_t *)&steeringRxByte, 1);
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

    diag.hstate = STEERING_STATE_IDLE;

    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
    HAL_UART_Receive_IT(&steeringUart, (uint8_t *)&steeringRxByte, 1);

    SendAnglesRaw(STEERING_ANGLE_STRAIGHT, STEERING_ANGLE_STRAIGHT,
                  STEERING_ANGLE_STRAIGHT, STEERING_ANGLE_STRAIGHT);
}

void SteeringPoll(void)
{
    uint32_t now = HAL_GetTick();

    /* Link-loss detection */
    if (diag.link_active &&
        (now - diag.last_rx_tick >= STEERING_LINK_TIMEOUT_MS)) {
        diag.link_active = 0;
        diag.link_lost_count++;
        memset(&lastStatus, 0, sizeof(lastStatus));
        diag.hstate = STEERING_STATE_LINK_LOST;
    }

    /* Periodic status refresh */
    if (now - lastRefreshTick >= STEERING_REFRESH_MS) {
        lastRefreshTick = now;
        SendQuery();
    }

    /* Update H5 sub-state based on wheel positions (only when link is up) */
    if (diag.link_active && diag.hstate == STEERING_STATE_MOVING) {
        if (SteeringWheelsReady(STEERING_SETTLED_DEG))
            diag.hstate = STEERING_STATE_AT_TARGET;
    }

    /* Dispatch angle commands on rover state change */
    rover_state_t state = CurrentRoverState();
    if (state == lastRoverState)
        return;
    lastRoverState = state;

    /* rover_controller.c already called SetSteeringAngles() for states that
     * need custom angles (TRAVERSE, WHEEL_CONTROL, STEER_ONLY, SPIN_*).
     * Here we only handle states that need explicit straight-ahead reset. */
    switch (state) {
    case ROVER_IDLE:
    case ROVER_READY:
    case ROVER_ESTOP:
        SendAnglesRaw(STEERING_ANGLE_STRAIGHT, STEERING_ANGLE_STRAIGHT,
                      STEERING_ANGLE_STRAIGHT, STEERING_ANGLE_STRAIGHT);
        targetFl = targetFr = targetRl = targetRr = 0.0f;
        break;
    default:
        break;
    }
}

void SetSteeringAngles(float fl, float fr, float rl, float rr)
{
    targetFl = fl; targetFr = fr; targetRl = rl; targetRr = rr;
    SendAnglesRaw(fl, fr, rl, rr);

    if (diag.hstate != STEERING_STATE_DISABLED &&
        diag.hstate != STEERING_STATE_LINK_LOST)
        diag.hstate = STEERING_STATE_MOVING;
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
    diag.hstate = STEERING_STATE_RESETTING;
}

void SteeringForceDisable(uint8_t idx)
{
    SendDisableCommand(idx);
    diag.hstate = STEERING_STATE_DISABLED;
}

void SteeringForceEnable(uint8_t idx)
{
    SendEnableCommand(idx);
    if (diag.hstate == STEERING_STATE_DISABLED)
        diag.hstate = STEERING_STATE_IDLE;
}

uint8_t SteeringWheelsReady(float tolerance_deg)
{
    if (!diag.link_active || !diag.ever_connected)
        return 0;

    float d0 = lastStatus.fl - targetFl; if (d0 < 0.0f) d0 = -d0;
    float d1 = lastStatus.fr - targetFr; if (d1 < 0.0f) d1 = -d1;
    float d2 = lastStatus.rl - targetRl; if (d2 < 0.0f) d2 = -d2;
    float d3 = lastStatus.rr - targetRr; if (d3 < 0.0f) d3 = -d3;
    return (d0 <= tolerance_deg && d1 <= tolerance_deg &&
            d2 <= tolerance_deg && d3 <= tolerance_deg) ? 1U : 0U;
}

steering_status_t GetSteeringStatus(void)
{
    steering_status_t s = lastStatus;
    lastStatus.valid = 0;
    return s;
}

steering_status_t GetSteeringStatusPeek(void)
{
    return lastStatus;
}

steering_diag_t GetSteeringDiag(void)
{
    return diag;
}
