/**
 * @file    console.h
 * @brief   Basic UART routines for console
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
void console_initialise(int baudRate);
/**
 * Transmits a single character over the UART (blocking)
 *
 * @param ch - character to send
 */
void console_txChar(int ch);
/**
 * Receives a single character over the UART (blocking)
 *
 * @return - character received
 */
int  console_rxChar(void);

#ifdef __cplusplus
}
#endif
#endif /* UART_H_ */
