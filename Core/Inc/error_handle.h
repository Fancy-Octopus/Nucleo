/*
 * error_handle.h
 *
 *  Created on: Mar 11, 2025
 *      Author: patelc
 */

#ifndef INC_ERROR_HANDLE_H_
#define INC_ERROR_HANDLE_H_

void error_poll(int err);
void get_timestamp(void);
int uart_send_error(int error);

void odrive_error_decoder(int error_code);


#endif /* INC_ERROR_HANDLE_H_ */
