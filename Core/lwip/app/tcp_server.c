/*
 * tcp_server.c
 *
 *  Created on: Feb 1, 2025
 *      Author: gruetzmacherg
 *  Updated: May 2026
 *
 * See tcp_server.h for the full protocol description.
 *
 * All telemetry is request-driven: a client sends a query packet and receives
 * a response only on its own connection. There is no periodic broadcast.
 */

#include "tcp_server.h"
#include "lwip/tcp.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "rover_controller.h"
#include "luna_steering.h"
#include "main.h"
#include <string.h>

#if LWIP_TCP

#define SERVER_PORT      5000
#define CLIENT_TIMEOUT   60000    /* drop idle client after this (ms) */
#define POLL_INTERVAL    2        /* lwIP poll interval (~500 ms units) */
#define CMD_START        0xAAU
#define CMD_BUF_SIZE     40U      /* ≥ largest command packet (Tier 4 = 36 bytes) */

/* ---- Per-connection reassembly state ---- */
typedef enum { RX_WAIT_START = 0, RX_WAIT_TYPE, RX_ACCUMULATE } rx_state_t;

struct client_conn {
    struct tcp_pcb    *pcb;
    u32_t              last_activity;
    struct client_conn*next;
    rx_state_t         rx_state;
    uint8_t            rx_buf[CMD_BUF_SIZE];
    uint8_t            rx_idx;
    uint8_t            rx_expected;
};

static struct tcp_pcb    *tcp_server_pcb;
static struct client_conn*client_list = NULL;

/* ---- Forward declarations ---- */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv  (void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t tcp_server_poll  (void *arg, struct tcp_pcb *tpcb);
static void  tcp_server_close (struct tcp_pcb *tpcb);
static void  add_client       (struct tcp_pcb *pcb);
static void  remove_client    (struct tcp_pcb *pcb);

/* ---- CRC-8 (polynomial 0x07, same as steering UART) ---- */
static uint8_t Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ---- Protocol helpers ---- */

/* Payload float count for a given type byte.
 * Query types (≥ 0x80) and unknown values: 0 floats (3-byte packet). */
static uint8_t PayloadFloats(uint8_t type)
{
    if (type < 16U) return 0;   /* Tier 1 rover state */
    if (type < 32U) return 1;   /* Tier 2 rover state */
    if (type < 48U) return 5;   /* Tier 3 rover state */
    if (type < 64U) return 8;   /* Tier 4 rover state */
    if (type < 68U) return 0;   /* Force Tier 1 (64-67): no payload */
    if (type < 70U) return 1;   /* Force Tier 2 (68-69): drive/spin speed */
    if (type < 71U) return 5;   /* Force Tier 3 (70): drive_speed + 4 angles */
    if (type < 72U) return 8;   /* Force Tier 4 (71): wheel[4] */
    return 0;                   /* query types or unknown → no payload */
}

static uint8_t CmdPacketLen(uint8_t type)
{
    return (uint8_t)(3U + PayloadFloats(type) * 4U);
}

/* ---- Rover command dispatch ---- */
static void DispatchRoverCommand(const uint8_t *buf)
{
    uint8_t state_byte = buf[1];
    uint8_t nf = PayloadFloats(state_byte);

    float floats[8] = {0};
    for (uint8_t i = 0; i < nf; i++)
        memcpy(&floats[i], &buf[2 + i * 4U], 4);

    rover_context_t ctx;
    memset(&ctx, 0, sizeof(ctx));

    if (state_byte >= 16U && state_byte < 32U) {
        switch ((rover_state_t)state_byte) {
        case ROVER_DRIVE:    ctx.drive_speed = floats[0]; break;
        case ROVER_SPIN:     ctx.spin_speed  = floats[0]; break;
        case ROVER_ARM_MOVE: ctx.arm_speed   = floats[0]; break;
        default:                                           break;
        }
    } else if (state_byte >= 32U && state_byte < 48U) {
        ctx.drive_speed = floats[0];
        ctx.steer_fl    = floats[1];
        ctx.steer_fr    = floats[2];
        ctx.steer_rl    = floats[3];
        ctx.steer_rr    = floats[4];
    } else if (state_byte >= 48U && state_byte < 64U) {
        for (uint8_t i = 0; i < 4U; i++) {
            ctx.wheel[i].speed = floats[i * 2U];
            ctx.wheel[i].angle = floats[i * 2U + 1U];
        }
    } else if (state_byte == 68U || state_byte == 69U) {
        /* Force Tier 2: ROVER_DRIVE_FORCE=68, ROVER_SPIN_FORCE=69 */
        if (state_byte == 68U) ctx.drive_speed = floats[0];
        else                   ctx.spin_speed  = floats[0];
    } else if (state_byte == 70U) {
        /* Force Tier 3: ROVER_TRAVERSE_FORCE */
        ctx.drive_speed = floats[0];
        ctx.steer_fl    = floats[1];
        ctx.steer_fr    = floats[2];
        ctx.steer_rl    = floats[3];
        ctx.steer_rr    = floats[4];
    } else if (state_byte == 71U) {
        /* Force Tier 4: ROVER_WHEEL_CONTROL_FORCE */
        for (uint8_t i = 0; i < 4U; i++) {
            ctx.wheel[i].speed = floats[i * 2U];
            ctx.wheel[i].angle = floats[i * 2U + 1U];
        }
    }

    RequestRoverStateCtx((rover_state_t)state_byte, &ctx);
}

/* ---- Telemetry builders ---- */

/* TELEM_ALL: [0xAB][0x80][state][flags][FL][FR][RL][RR][hstate][CRC] = 22 bytes
 * flags: bit0=wheels_ready  bit1=force_state  bit2=link_active */
static void SendTelemAll(struct tcp_pcb *pcb)
{
    steering_status_t s = GetSteeringStatusPeek();
    steering_diag_t   d = GetSteeringDiag();
    uint8_t           st = (uint8_t)CurrentRoverState();

    uint8_t flags = 0;
    if (SteeringWheelsReady(STEERING_SETTLED_DEG)) flags |= 0x01U;
    if (st >= 64U && st < 72U)                     flags |= 0x02U;
    if (d.link_active)                             flags |= 0x04U;

    uint8_t pkt[22];
    pkt[0]  = TELEM_START;
    pkt[1]  = QUERY_ALL;
    pkt[2]  = (uint8_t)CurrentRoverState();
    pkt[3]  = flags;
    memcpy(&pkt[4],  &s.fl, 4);
    memcpy(&pkt[8],  &s.fr, 4);
    memcpy(&pkt[12], &s.rl, 4);
    memcpy(&pkt[16], &s.rr, 4);
    pkt[20] = (uint8_t)d.hstate;
    pkt[21] = Crc8(pkt, 21);

    err_t err = tcp_write(pcb, pkt, sizeof(pkt), TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) tcp_output(pcb);
}

/* TELEM_STEERING: [0xAB][0x81][FL][FR][RL][RR][steer_flags][hstate][CRC] = 21 bytes
 * steer_flags: bit0=link_active  bit1=homed  bit2=homing */
static void SendTelemSteering(struct tcp_pcb *pcb)
{
    steering_status_t s = GetSteeringStatusPeek();
    steering_diag_t   d = GetSteeringDiag();

    uint8_t sf = 0;
    if (d.link_active)                       sf |= 0x01U;
    if (s.flags & STEERING_FLAG_HOMED)       sf |= 0x02U;
    if (s.flags & STEERING_FLAG_HOMING)      sf |= 0x04U;

    uint8_t pkt[21];
    pkt[0]  = TELEM_START;
    pkt[1]  = QUERY_STEERING;
    memcpy(&pkt[2],  &s.fl, 4);
    memcpy(&pkt[6],  &s.fr, 4);
    memcpy(&pkt[10], &s.rl, 4);
    memcpy(&pkt[14], &s.rr, 4);
    pkt[18] = sf;
    pkt[19] = (uint8_t)d.hstate;
    pkt[20] = Crc8(pkt, 20);

    err_t err = tcp_write(pcb, pkt, sizeof(pkt), TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) tcp_output(pcb);
}

/* TELEM_DRIVE: [0xAB][0x82][state][flags][CRC] = 5 bytes
 * flags: bit0=wheels_ready  bit1=force_state  bit2=link_active */
static void SendTelemDrive(struct tcp_pcb *pcb)
{
    steering_diag_t d  = GetSteeringDiag();
    uint8_t         st = (uint8_t)CurrentRoverState();

    uint8_t flags = 0;
    if (SteeringWheelsReady(STEERING_SETTLED_DEG)) flags |= 0x01U;
    if (st >= 64U && st < 72U)                     flags |= 0x02U;
    if (d.link_active)                             flags |= 0x04U;

    uint8_t pkt[5];
    pkt[0] = TELEM_START;
    pkt[1] = QUERY_DRIVE;
    pkt[2] = (uint8_t)CurrentRoverState();
    pkt[3] = flags;
    pkt[4] = Crc8(pkt, 4);

    err_t err = tcp_write(pcb, pkt, sizeof(pkt), TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) tcp_output(pcb);
}

/* ---- Per-byte reassembly ---- */
static void ProcessByte(struct client_conn *c, uint8_t b)
{
    switch (c->rx_state) {

    case RX_WAIT_START:
        if (b == CMD_START) {
            c->rx_buf[0] = b;
            c->rx_idx    = 1;
            c->rx_state  = RX_WAIT_TYPE;
        }
        break;

    case RX_WAIT_TYPE: {
        c->rx_buf[c->rx_idx++] = b;
        uint8_t pktlen = CmdPacketLen(b);
        if (pktlen > CMD_BUF_SIZE) {
            c->rx_state = RX_WAIT_START;
            break;
        }
        c->rx_expected = pktlen;
        if (c->rx_idx >= c->rx_expected)
            goto complete;
        c->rx_state = RX_ACCUMULATE;
        break;
    }

    case RX_ACCUMULATE:
        c->rx_buf[c->rx_idx++] = b;
        if (c->rx_idx < c->rx_expected) break;
        /* fall through */

    complete: {
        uint8_t type = c->rx_buf[1];
        if (Crc8(c->rx_buf, (uint8_t)(c->rx_expected - 1U)) !=
            c->rx_buf[c->rx_expected - 1U]) {
            /* Bad CRC — discard */
            c->rx_state = RX_WAIT_START;
            break;
        }
        if (type < 0x80U) {
            DispatchRoverCommand(c->rx_buf);
        } else {
            switch (type) {
            case QUERY_ALL:      SendTelemAll(c->pcb);      break;
            case QUERY_STEERING: SendTelemSteering(c->pcb); break;
            case QUERY_DRIVE:    SendTelemDrive(c->pcb);    break;
            default:                                        break;
            }
        }
        c->rx_state = RX_WAIT_START;
        break;
    }
    }
}

/* ---- lwIP callbacks ---- */

void tcp_server_init(void)
{
    tcp_server_pcb = tcp_new();
    if (tcp_server_pcb == NULL) return;
    if (tcp_bind(tcp_server_pcb, IP_ADDR_ANY, SERVER_PORT) != ERR_OK) return;
    tcp_server_pcb = tcp_listen(tcp_server_pcb);
    tcp_accept(tcp_server_pcb, tcp_server_accept);
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    if (newpcb == NULL || err != ERR_OK) return ERR_VAL;
    add_client(newpcb);
    tcp_recv(newpcb, tcp_server_recv);
    tcp_poll(newpcb, tcp_server_poll, POLL_INTERVAL);
    return ERR_OK;
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb,
                              struct pbuf *p, err_t err)
{
    struct client_conn *client = (struct client_conn *)arg;

    if (p == NULL) {
        remove_client(tpcb);
        tcp_server_close(tpcb);
        return ERR_OK;
    }

    if (client) client->last_activity = HAL_GetTick();

    tcp_recved(tpcb, p->tot_len);

    struct pbuf *q = p;
    while (q != NULL) {
        uint8_t *data = (uint8_t *)q->payload;
        for (uint16_t i = 0; i < q->len; i++)
            ProcessByte(client, data[i]);
        q = q->next;
    }

    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
    struct client_conn *client = (struct client_conn *)arg;
    if (client && (HAL_GetTick() - client->last_activity) > CLIENT_TIMEOUT) {
        remove_client(tpcb);
        tcp_server_close(tpcb);
    }
    return ERR_OK;
}

static void tcp_server_close(struct tcp_pcb *tpcb)
{
    tcp_arg(tpcb,  NULL);
    tcp_recv(tpcb, NULL);
    tcp_poll(tpcb, NULL, 0);
    tcp_close(tpcb);
}

static void add_client(struct tcp_pcb *pcb)
{
    struct client_conn *c = (struct client_conn *)mem_malloc(sizeof(struct client_conn));
    if (c == NULL) return;
    memset(c, 0, sizeof(*c));
    c->pcb           = pcb;
    c->last_activity = HAL_GetTick();
    c->rx_state      = RX_WAIT_START;
    c->next          = client_list;
    client_list      = c;
    tcp_arg(pcb, c);
}

static void remove_client(struct tcp_pcb *pcb)
{
    struct client_conn **cur = &client_list;
    while (*cur != NULL) {
        if ((*cur)->pcb == pcb) {
            struct client_conn *dead = *cur;
            *cur = (*cur)->next;
            mem_free(dead);
            return;
        }
        cur = &(*cur)->next;
    }
}

/* Stub — no periodic broadcast. Telemetry is request-driven per client. */
void tcp_task(void) {}

#endif /* LWIP_TCP */
