/*
 * roboclaw.c
 *
 *  Created on: Mar 15, 2025
 *      Author: strongp
 *  Updated: May 2026
 *
 * Controls two linear actuator arms via a RoboClaw motor controller.
 * UART4: PA0 (TX, AF8) / PC11 (RX, AF6) at 115200 baud.
 *
 * M1 = arm 1, M2 = arm 2.  Both are driven together for ROVER_ARMS_UP/DOWN.
 * ARM_SPEED sets the default 0-127 speed value for Tier 1 states.
 * ROVER_ARM_MOVE (Tier 2) accepts context.arm_speed (-127..+127, signed).
 *
 * Encoder reads (GETENCODERS, cmd 78) run every ENC_POLL_MS milliseconds.
 * Encoders can be zeroed via RoboclawResetEncoders().
 */

#include "main.h"
#include "stdint.h"
#include "string.h"
#include "rover_controller.h"
#include "lunautils.h"
#include "luna_wait.h"

/* Default arm speed for Tier 1 ARMS_UP / ARMS_DOWN (0-127) */
#define ARM_SPEED        127

#define ROBOCLAW_ADDRESS 0x80
#define ENC_POLL_MS      500    /* encoder read interval (ms) */
#define TX_TIMEOUT_MS    5      /* UART TX timeout */
#define RX_TIMEOUT_MS    20     /* UART RX timeout for read responses */

/* RoboClaw packet command IDs */
enum {
    M1FORWARD              = 0,
    M1BACKWARD             = 1,
    SETMINMB               = 2,
    SETMAXMB               = 3,
    M2FORWARD              = 4,
    M2BACKWARD             = 5,
    M17BIT                 = 6,
    M27BIT                 = 7,
    MIXEDFORWARD           = 8,
    MIXEDBACKWARD          = 9,
    MIXEDRIGHT             = 10,
    MIXEDLEFT              = 11,
    MIXEDFB                = 12,
    MIXEDLR                = 13,
    GETM1ENC               = 16,
    GETM2ENC               = 17,
    GETM1SPEED             = 18,
    GETM2SPEED             = 19,
    RESETENC               = 20,
    GETVERSION             = 21,
    SETM1ENCCOUNT          = 22,
    SETM2ENCCOUNT          = 23,
    GETMBATT               = 24,
    GETLBATT               = 25,
    SETMINLB               = 26,
    SETMAXLB               = 27,
    SETM1PID               = 28,
    SETM2PID               = 29,
    GETM1ISPEED            = 30,
    GETM2ISPEED            = 31,
    M1DUTY                 = 32,
    M2DUTY                 = 33,
    MIXEDDUTY              = 34,
    M1SPEED                = 35,
    M2SPEED                = 36,
    MIXEDSPEED             = 37,
    M1SPEEDACCEL           = 38,
    M2SPEEDACCEL           = 39,
    MIXEDSPEEDACCEL        = 40,
    M1SPEEDDIST            = 41,
    M2SPEEDDIST            = 42,
    MIXEDSPEEDDIST         = 43,
    M1SPEEDACCELDIST       = 44,
    M2SPEEDACCELDIST       = 45,
    MIXEDSPEEDACCELDIST    = 46,
    GETBUFFERS             = 47,
    GETPWMS                = 48,
    GETCURRENTS            = 49,
    MIXEDSPEED2ACCEL       = 50,
    MIXEDSPEED2ACCELDIST   = 51,
    M1DUTYACCEL            = 52,
    M2DUTYACCEL            = 53,
    MIXEDDUTYACCEL         = 54,
    READM1PID              = 55,
    READM2PID              = 56,
    SETMAINVOLTAGES        = 57,
    SETLOGICVOLTAGES       = 58,
    GETMINMAXMAINVOLTAGES  = 59,
    GETMINMAXLOGICVOLTAGES = 60,
    SETM1POSPID            = 61,
    SETM2POSPID            = 62,
    READM1POSPID           = 63,
    READM2POSPID           = 64,
    M1SPEEDACCELDECCELPOS  = 65,
    M2SPEEDACCELDECCELPOS  = 66,
    MIXEDSPEEDACCELDECCELPOS = 67,
    SETM1DEFAULTACCEL      = 68,
    SETM2DEFAULTACCEL      = 69,
    SETPINFUNCTIONS        = 74,
    GETPINFUNCTIONS        = 75,
    SETDEADBAND            = 76,
    GETDEADBAND            = 77,
    GETENCODERS            = 78,
    GETISPEEDS             = 79,
    RESTOREDEFAULTS        = 80,
    GETTEMP                = 82,
    GETTEMP2               = 83,
    GETERROR               = 90,
    GETENCODERMODE         = 91,
    SETM1ENCODERMODE       = 92,
    SETM2ENCODERMODE       = 93,
    WRITENVM               = 94,
    READNVM                = 95,
    SETCONFIG              = 98,
    GETCONFIG              = 99,
    SETM1MAXCURRENT        = 133,
    SETM2MAXCURRENT        = 134,
    GETM1MAXCURRENT        = 135,
    GETM2MAXCURRENT        = 136,
    SETPWMMODE             = 148,
    GETPWMMODE             = 149,
    FLAGBOOTLOADER         = 255
};

/* ---- Private declarations ---- */
static void    ArmMotor1(uint8_t speed, uint8_t dir);
static void    ArmMotor2(uint8_t speed, uint8_t dir);
static void    BothArms(uint8_t speed, uint8_t dir);
static void    RoboclawCrcUpdate(uint8_t data, uint16_t *crc);
static void    RoboclawSendMessage(uint8_t addr, uint8_t command,
                                   uint8_t *data, uint8_t datalen);
static int     RoboclawSendRead(uint8_t addr, uint8_t cmd,
                                uint8_t *rxbuf, uint8_t rxlen);
static void    RoboclawReadEncoders(void);

/* ---- Module state ---- */
static UART_HandleTypeDef roboclawUart;
static schedule_t         enc_schedule;
static int32_t            arm_enc_m1 = 0;
static int32_t            arm_enc_m2 = 0;

/* ---- Init ---- */
void RoboclawInit(void)
{
    GPIO_InitTypeDef gpio_init;
    __HAL_RCC_UART4_CLK_ENABLE();

    /* PA0 — UART4 TX, AF8 */
    gpio_init.Pin       = GPIO_PIN_0;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* PC11 — UART4 RX, AF6 */
    gpio_init.Pin       = GPIO_PIN_11;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Alternate = GPIO_AF6_UART4;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    roboclawUart.Instance            = UART4;
    roboclawUart.Init.BaudRate       = 115200;
    roboclawUart.Init.Mode           = UART_MODE_TX_RX;
    roboclawUart.Init.Parity         = COM_PARITY_NONE;
    roboclawUart.Init.WordLength     = COM_WORDLENGTH_8B;
    roboclawUart.Init.StopBits       = COM_STOPBITS_1;
    roboclawUart.Init.HwFlowCtl      = COM_HWCONTROL_NONE;
    roboclawUart.Init.OverSampling   = UART_OVERSAMPLING_8;
    roboclawUart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    HAL_UART_Init(&roboclawUart);

    SetScheduledTime(&enc_schedule, ENC_POLL_MS);
}

/* ---- Poll ---- */
void RoboclawPoll(void)
{
    rover_state_t          state = CurrentRoverState();
    const rover_context_t *ctx   = CurrentRoverContext();

    switch (state) {
    case ROVER_ARMS_UP:
        BothArms(ARM_SPEED, 0);
        break;

    case ROVER_ARMS_DOWN:
        BothArms(ARM_SPEED, 1);
        break;

    case ROVER_ARM_MOVE: {
        /* context.arm_speed: positive = up, negative = down, 0 = stop */
        int16_t spd = (int16_t)ctx->arm_speed;
        if (spd > 127)  spd =  127;
        if (spd < -127) spd = -127;
        if (spd > 0) {
            BothArms((uint8_t)spd, 0);
        } else if (spd < 0) {
            BothArms((uint8_t)(-spd), 1);
        } else {
            BothArms(0, 0);
        }
        break;
    }

    default:
        BothArms(0, 0);
        break;
    }

    /* Periodic encoder read — runs independently of motor state */
    if (ScheduleReady(enc_schedule)) {
        ResetSchedule(&enc_schedule);
        RoboclawReadEncoders();
    }
}

/* ---- Public getters ---- */

int32_t RoboclawGetEncoder(uint8_t motor)
{
    return (motor == 2) ? arm_enc_m2 : arm_enc_m1;
}

void RoboclawResetEncoders(void)
{
    /* RESETENC is a write command (no data payload) — sends [addr][cmd][CRC] */
    RoboclawSendMessage(ROBOCLAW_ADDRESS, RESETENC, NULL, 0);
    arm_enc_m1 = 0;
    arm_enc_m2 = 0;
}

/* ---- Private helpers ---- */

static void RoboclawCrcUpdate(uint8_t data, uint16_t *crc)
{
    *crc = *crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; i++) {
        if (*crc & 0x8000)
            *crc = (*crc << 1) ^ 0x1021;
        else
            *crc <<= 1;
    }
}

static void RoboclawSendMessage(uint8_t addr, uint8_t command,
                                uint8_t *data, uint8_t datalen)
{
    uint8_t  i = 0;
    uint16_t crc = 0;
    uint8_t  txbuf[32];

    RoboclawCrcUpdate(addr, &crc);
    RoboclawCrcUpdate(command, &crc);
    for (int x = 0; x < datalen; x++)
        RoboclawCrcUpdate(*(data + x), &crc);

    txbuf[i++] = addr;
    txbuf[i++] = command;
    for (int j = 0; j < datalen; j++)
        txbuf[i++] = *(data + j);
    txbuf[i++] = (uint8_t)((crc & 0xFF00) >> 8);
    txbuf[i++] = (uint8_t)((crc & 0x00FF));

    HAL_UART_Transmit(&roboclawUart, txbuf, i, TX_TIMEOUT_MS);
}

/* Send a read command: [addr][cmd] only — no CRC in the request.
 * The RoboClaw's response carries its own CRC over addr+cmd+data.
 * Returns 0 on success, -1 on TX/RX timeout. */
static int RoboclawSendRead(uint8_t addr, uint8_t cmd,
                            uint8_t *rxbuf, uint8_t rxlen)
{
    uint8_t txbuf[2] = {addr, cmd};
    if (HAL_UART_Transmit(&roboclawUart, txbuf, 2, TX_TIMEOUT_MS) != HAL_OK)
        return -1;
    if (HAL_UART_Receive(&roboclawUart, rxbuf, rxlen, RX_TIMEOUT_MS) != HAL_OK)
        return -1;
    return 0;
}

/* GETENCODERS (cmd 78):
 *   TX: [addr][78]  (2 bytes, no CRC)
 *   RX: [M1:i32 BE][M2:i32 BE][CRC16_hi][CRC16_lo]  = 10 bytes
 *   Response CRC covers: addr + cmd + 8 data bytes */
static void RoboclawReadEncoders(void)
{
    uint8_t rx[10];
    if (RoboclawSendRead(ROBOCLAW_ADDRESS, GETENCODERS, rx, sizeof(rx)) != 0)
        return;

    uint16_t crc = 0;
    RoboclawCrcUpdate(ROBOCLAW_ADDRESS, &crc);
    RoboclawCrcUpdate(GETENCODERS, &crc);
    for (uint8_t i = 0; i < 8; i++)
        RoboclawCrcUpdate(rx[i], &crc);

    uint16_t received = ((uint16_t)rx[8] << 8) | rx[9];
    if (crc != received) return;

    arm_enc_m1 = (int32_t)(((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
                            ((uint32_t)rx[2] <<  8) |  (uint32_t)rx[3]);
    arm_enc_m2 = (int32_t)(((uint32_t)rx[4] << 24) | ((uint32_t)rx[5] << 16) |
                            ((uint32_t)rx[6] <<  8) |  (uint32_t)rx[7]);
}

/* dir=0 → forward/up, dir=1 → backward/down */
static void ArmMotor1(uint8_t speed, uint8_t dir)
{
    uint8_t data = speed;
    RoboclawSendMessage(ROBOCLAW_ADDRESS,
                        dir ? M1BACKWARD : M1FORWARD,
                        &data, sizeof(data));
}

static void ArmMotor2(uint8_t speed, uint8_t dir)
{
    uint8_t data = speed;
    RoboclawSendMessage(ROBOCLAW_ADDRESS,
                        dir ? M2BACKWARD : M2FORWARD,
                        &data, sizeof(data));
}

static void BothArms(uint8_t speed, uint8_t dir)
{
    ArmMotor1(speed, dir);
    ArmMotor2(speed, dir);
}
