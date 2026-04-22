/*
 * tcp_server.c
 *
 *  Created on: Feb 1, 2025
 *      Author: gruetzmacherg
 */


#include "tcp_server.h"
#include "lwip/tcp.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "rover_controller.h"
#include "main.h"
#include <string.h>


#if LWIP_TCP

#define SERVER_PORT 5000
#define MAX_CLIENTS 5
#define DATA_INTERVAL 100     // time in ms to periodically send data
#define CLIENT_TIMEOUT 60000  // timeout in ms
#define POLL_INTERVAL 2       // polling interval in LWIP time units (~500ms)

/* Structure to keep track of client connection information
 * inside of a dynamically-linked list. Passed as argument
 * for LwIP callbacks */
struct client_conn
{
  struct tcp_pcb *pcb;       // pointer to client pcb
  u32_t last_activity;       // sys tick of last received packet
  struct client_conn *next;  // pointer to next client
};

static struct tcp_pcb *tcp_server_pcb;          // pointer to server pcb
static struct client_conn *client_list = NULL;  // initialize client_list to NULL
static u32_t last_send_time = 0;                // initialize timeout variable


err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
void tcp_server_send_data(void);
void tcp_server_close(struct tcp_pcb *tpcb);
void remove_client(struct tcp_pcb *pcb);
void add_client(struct tcp_pcb *pcb);


/**
 * @brief   API function to initialize the tcp server. Meant to be
 *          called in main upon startup.
 * @param   none
 * @retval  none
 */
void tcp_server_init()
{
  /* create new pcb for server */
  tcp_server_pcb = tcp_new();

  if (tcp_server_pcb != NULL)
  {
    err_t err;

    /* bind server pcb to target port */
    err = tcp_bind(tcp_server_pcb, IP_ADDR_ANY, SERVER_PORT);

    if (err == ERR_OK)
    {
      /* set server to tcp_listen state */
      tcp_server_pcb = tcp_listen(tcp_server_pcb);

      /* setup tcp_accept callback */
      tcp_accept(tcp_server_pcb, tcp_server_accept);
    }
  }
}


/**
 * @brief   Function to handle new client connections. Passed to
 *          LwIP for callbacks.
 * @param   arg: unused
 * @param   newpcb: pointer to newly created pcb
 * @param   err: LwIP error status
 * @retval  err_t: returns LwIP error status
 */
err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  /* check for errors and return if present */
  if (newpcb == NULL || err != ERR_OK) return ERR_VAL;

  add_client(newpcb);

  /* setup receive and poll callbacks */
  tcp_recv(newpcb, tcp_server_recv);
  tcp_poll(newpcb, tcp_server_poll, POLL_INTERVAL);

  return ERR_OK;
}


/**
 * @brief   Function to handle received data. Passed to LwIP
 *          for callbacks.
 * @param   arg: hold client connection information
 * @param   tpcb: pointer to client pcb
 * @param   p: pointer to received pbuf
 * @param   err: LwIP error status
 * @retval  err_t: returns LwIP error status
 */
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct client_conn *client;
  client = (struct client_conn *)arg;

  /* check for empty TCP frame */
  if (p == NULL)
  {
    remove_client(tpcb);
    tcp_server_close(tpcb);
    return ERR_OK;
  }

  if (client)
  {
    client->last_activity = HAL_GetTick(); /* update last activity */
  }

  /* acknowledge received packet */
  tcp_recved(tpcb, p->tot_len);

  /* Read data frame and update robot state */
  uint8_t *payload = p->payload;
  rover_state_t frame = (rover_state_t) *payload;
  rover_state_t return_state;

  char msg[32];

  switch(frame)
  {
  case ROVER_IDLE:
    snprintf(msg, sizeof(msg), "Command: Hold State\n");
    return_state = RequestRoverState(ROVER_IDLE);
    break;
  case ROVER_FORWARD:
    snprintf(msg, sizeof(msg), "Command: Forward\n");
    return_state = RequestRoverState(ROVER_FORWARD);
    break;
  case ROVER_BACKWARD:
    snprintf(msg, sizeof(msg), "Command: Backward\n");
    return_state = RequestRoverState(ROVER_BACKWARD);
    break;
  case ROVER_TURN_RIGHT:
    snprintf(msg, sizeof(msg), "Command: Right\n");
    return_state = RequestRoverState(ROVER_TURN_RIGHT);
    break;
  case ROVER_TURN_LEFT:
    snprintf(msg, sizeof(msg), "Command: Left\n");
    return_state = RequestRoverState(ROVER_TURN_LEFT);
    break;
  case ROVER_WINCH_DOWN:
    snprintf(msg, sizeof(msg), "Command: Winch Down\n");
    return_state = RequestRoverState(ROVER_WINCH_DOWN);
    break;
  case ROVER_WINCH_UP:
    snprintf(msg, sizeof(msg), "Command: Winch Up");
    return_state = RequestRoverState(ROVER_WINCH_UP);
    break;
  case ROVER_DIG_FORWARD:
    snprintf(msg, sizeof(msg), "Command: Dig forward");
    return_state = RequestRoverState(ROVER_DIG_FORWARD);
    break;
  case ROVER_DEPOSIT_FORWARD:
    snprintf(msg, sizeof(msg), "Command: Deposit forward");
    return_state = RequestRoverState(ROVER_DEPOSIT_FORWARD);
    break;
  case ROVER_DIG_BACKWARD:
    snprintf(msg, sizeof(msg), "Command: Dig backward");
    return_state = RequestRoverState(ROVER_DIG_BACKWARD);
    break;
  case ROVER_DEPOSIT_BACKWARD:
    snprintf(msg, sizeof(msg), "Command: Deposit backward");
    return_state = RequestRoverState(ROVER_DEPOSIT_BACKWARD);
    break;
  case ROVER_READY:
    snprintf(msg, sizeof(msg), "Command: Ready");
    return_state = RequestRoverState(ROVER_READY);
    break;
  case ROVER_ESTOP:
    snprintf(msg, sizeof(msg), "Command: Stop");
    return_state = RequestRoverState(ROVER_ESTOP);
  default:
    snprintf(msg, sizeof(msg), "Error: Invalid Command\n");
    break;
  }

  /* Check for invalid rover state return value */
  //if (return_state == NULL) {
  //  snprintf(msg, sizeof(msg), "Error: Command Failed");
  //}

  /* enqueue copy of state data to be sent */
  err_t ret_err;
  ret_err = tcp_write(tpcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);

  /* if data is good send to client */
  if (ret_err == ERR_OK) tcp_output(tpcb);

  pbuf_free(p);

  return ERR_OK;
}


/**
 * @brief   Server polling function to check for inactive clients.
 *          Passed to LwIP for callbacks.
 * @param   arg: holds client connection information
 * @param   tpcb: pointer to client pcb
 * @retval  err_t: returns LwIP error status
 */
err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
  struct client_conn *client = (struct client_conn *)arg;
  u32_t current_time = HAL_GetTick();

  /* check for inactive clients */
  if (client && (current_time - client->last_activity) > CLIENT_TIMEOUT)
  {
    /* remove inactive client and terminate connection */
    remove_client(tpcb);
    tcp_server_close(tpcb);
  }

  return ERR_OK;
}


/**
 * @brief   Function for periodically sending data to all
 *          connected clients.
 * @param   none
 * @retval  none
 */
void tcp_server_send_data()
{
  /* check for time since last send and return if*/
  u32_t current_time = HAL_GetTick();
  if (current_time - last_send_time < DATA_INTERVAL) return;

  /* cycle through connected clients and send data */
  struct client_conn *conn = client_list;
  while (conn != NULL)
  {
    /* Placeholder payload to test connection */
    const char *msg = "Test Data\n";

    /* enqueue copy of data data to be sent */
    err_t err;
    err = tcp_write(conn->pcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);

    /* if data is good send to client */
    if (err == ERR_OK) tcp_output(conn->pcb);

    /* move to next client in list */
    conn = conn->next;
  }

  /* update timestamp for last send */
  current_time = HAL_GetTick();
  last_send_time = current_time;
}


/**
 * @brief   Function to close a client connection and deallocate
 *          the pcb for it.
 * @param   tpcb: pointer to client pcb that is to be disconnected
 * @retval  none
 */
void tcp_server_close(struct tcp_pcb *tpcb)
{
  /* clear callbacks and arguments handling the pcb */
  tcp_arg(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);

  /* close the connection and deallocate pcb */
  tcp_close(tpcb);
}


/**
 * @brief   Function to add a client to the linked list
 * @param   pcb: pointer to client pcb
 * @retval  none
 */
void add_client(struct tcp_pcb *pcb)
{
  struct client_conn *new_client = (struct client_conn *)mem_malloc(sizeof(struct client_conn));
  if (new_client == NULL) return;

  /* assign pcb to new client */
  new_client->pcb = pcb;

  /* initialize last activity timestamp */
  new_client->last_activity = HAL_GetTick();

  /* assign current client list to linked next-pointer */
  new_client->next = client_list;

  /* assign the new client as the start of the list */
  client_list = new_client;

  /* associate client structure with the pcb for LwIP callbacks */
  tcp_arg(pcb, new_client);
}


/**
 * @brief   Function to remove a client from the linked list
 * @param   pcb: pointer to client pcb
 * @retval  none
 */
void remove_client(struct tcp_pcb *pcb)
{
  /* create pointer to start of client list */
  struct client_conn **current = &client_list;

  /* cycle through the client list */
  while (*current != NULL)
  {
    /* check for pcb we want to remove */
    if ((*current)->pcb == pcb)
    {
      struct client_conn *to_remove = *current;  /* assign client for removal */
      *current = (*current)->next;               /* patch hole in list */
      mem_free(to_remove);                       /* release element memory */
      return;
    }
    current = &((*current)->next);  /* set sweep to address of next element */
  }
}


/**
 * @brief   API function to be called in main to pass priority
 *          to the tcp server.
 * @param   none
 * @retval  none
 */
void tcp_task()
{
  tcp_server_send_data();
}


#endif /* LWIP_TCP */
