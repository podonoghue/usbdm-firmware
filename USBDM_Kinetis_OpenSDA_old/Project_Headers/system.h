/**
 * @file     system.h
 * @brief    System initialisation routines
 * @version  V4.11.1.70
 * @date     13 Nov 2012
 */
#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock; //!< System core clock frequency in Hz
extern uint32_t SystemBusClock;  //!< System bus clock frequency Hz

/**
 *  @brief Low-level initialize the system
 *
 *  Low level setup of the microcontroller system. \n
 *  Called very early in the initialisation. \n
 *  May NOT use globals etc (as will be overwritten by BSS initialization)
 */
void SystemInitLowLevel(void);
/**
 * @brief Initialize the system
 *
 * Setup the microcontroller system.
 */
void SystemInit(void);
/**
 * @brief Update SystemCoreClock variable
 *
 * Updates the SystemCoreClock & SystemBusClock variables with current core Clock retrieved from CPU registers.
 */
void SystemCoreClockUpdate(void);

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------

//   <o> UART default baud rate
//   <i> Used by default UART setup for stdio
//   <i> Note: Manually change Custom value
//     <110=> 110
//     <300=> 300
//     <600=> 600
//     <1200=> 1200
//     <2400=> 2400
//     <4800=> 4800
//     <9600=> 9600
//     <14400=> 14400
//     <19200=> 19200
//     <28800=> 28800
//     <38400=> 38400
//     <56000=> 56000
//     <57600=> 57600
//     <115200=> 115200
//     <115200=> Restore default
//     <115200=> Custom

#ifndef DEFAULT_BAUD_RATE
/**
 * Default baud rate for UART used for stdio
 */
#define DEFAULT_BAUD_RATE 115200 
#endif

#if ((DEBUG&USB_PUTS_DEBUG) != 0)
#include "uart.h"
#define PUTS(x) puts(x)
#define PRINTF(...) printf (__VA_ARGS__)
#else
#define PUTS(x)
#define PRINTF(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_H_ */
