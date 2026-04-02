/*
 * lunanet_wrapper.c
 *
 *  Created on: Mar 17, 2025
 *      Author: strongp
 */

//LWIP
#include "lwip.h"
#include "ethernetif.h"
#include "lwip/opt.h"
#include "lwip/init.h"
#include "netif/etharp.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
//#include "tcp_echoserver.h"
#include "tcp_server.h"
#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif

extern struct netif gnetif;

/**
 * @brief  Setup the network interface
 *   None
 * @retval None
 */
static void Netif_Config(void) {
      ip_addr_t ipaddr;
      ip_addr_t netmask;
      ip_addr_t gw;
#if LWIP_DHCP
      ip_addr_set_zero_ip4(&ipaddr);
      ip_addr_set_zero_ip4(&netmask);
      ip_addr_set_zero_ip4(&gw);
#else
  /* IP address default setting */
  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif
      /* add the network interface */
      netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init,
                   &ethernet_input);
      /*  Registers the default network interface */
      netif_set_default(&gnetif);
#if LWIP_NETIF_LINK_CALLBACK
      netif_set_link_callback(&gnetif, ethernet_link_status_updated);
      dhcp_start(&gnetif);
      ethernet_link_status_updated(&gnetif);
#endif
}

int NetworkInit(void){
  lwip_init();
  Netif_Config();
  //tcp_echoserver_init();
  tcp_server_init();
  return 0;
}

int NetworkPoll(void){
  /* Read a received packet from the Ethernet buffers and send it
     to the lwIP for handling */
  ethernetif_input(&gnetif);
  // Handle timeouts
  sys_check_timeouts();
  /* Pass priority to the TCP server */
  //tcp_task();
#if LWIP_NETIF_LINK_CALLBACK
  Ethernet_Link_Periodic_Handle(&gnetif);
#endif
#if LWIP_DHCP
  DHCP_Periodic_Handle(&gnetif);
#endif
}

int MDNS_Init(void) {
#if LWIP_MDNS_RESPONDER
  mdns_resp_init();      // Start up mDNS system
  mdns_resp_add_netif(&gnetif, "davy");  // Add hostname to network interface
#endif
  return 0;
}

