/*
* uart.h
 *
 *  Created on: 14/04/2013
 *      Author: pgo
 */

#include <derivative.h>

#ifndef UART_H_
#define UART_H_

#ifdef __cplusplus
extern "C" {
#endif

void uart_initialise(int baudRate);
void uart_txChar(int ch);
int  uart_rxChar(void);

#ifdef __cplusplus
}
#endif
#endif /* UART_H_ */
