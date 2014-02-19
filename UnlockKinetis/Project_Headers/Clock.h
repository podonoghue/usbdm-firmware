/*
 * clock.h
 *
 *  Created on: Nov 6, 2012
 *      Author: podonoghue
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include <stdint.h>

extern const uint32_t SystemCoreClock; // Hz
extern const uint32_t PeripheralClock; // Hz
void initClock(void);

#endif /* CLOCK_H_ */
