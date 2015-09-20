/*
 * system.h
 *
 *  Created on: Nov 6, 2012
 *      Author: podonoghue
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock; // Hz
extern uint32_t SystemBusClock;  // Hz

/*!
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
/*!
 * @brief Update SystemCoreClock variable
 *
 * Updates the SystemCoreClock & SystemBusClock variables with current core Clock retrieved from CPU registers.
 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_H_ */
