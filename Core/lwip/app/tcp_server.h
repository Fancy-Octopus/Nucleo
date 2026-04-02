/*
 * tcp_server.h
 *
 *  Created on: Feb 1, 2025
 *      Author: gruetzmacherg
 */

#ifndef APP_TCP_TEST_H_
#define APP_TCP_TEST_H_


/**
 * @brief   API function to initialize the tcp server. Meant to be
 *          called in main upon startup.
 * @param   none
 * @retval  none
 */
void tcp_server_init(void);


/**
 * @brief   API function to be called in main to pass priority
 *          to the tcp server.
 * @param   none
 * @retval  none
 */
void tcp_task(void);


#endif /* APP_TCP_TEST_H_ */
