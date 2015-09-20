/**
 * @file    uart.h
 * @brief   Basic UART routines
 * @date    13 June 2015
 */
#include <derivative.h>

#ifndef UART_H_
#define UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialises the UART
 *
 * @param baudRate - the baud rate to use e.g. @ref DEFAULT_BAUD_RATE
 */
void uart_initialise(int baudRate);
/**
 * Transmits a single character over the UART (blocking)
 *
 * @param ch - character to send
 */
void uart_txChar(int ch);
/**
 * Receives a single character over the UART (blocking)
 *
 * @return - character received
 */
int  uart_rxChar(void);
/**
 * Receives a single character over the UART (non-blocking)
 *
 * @return - character received or EOF if none available
 */
int uart_getCh(void);

#ifdef __cplusplus
}
#endif
#endif /* UART_H_ */
