/*
 * clock.h
 *
 *  Created on: Nov 6, 2012
 *      Author: podonoghue
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const uint32_t SystemCoreClock; // Hz
extern const uint32_t PeripheralClock; // Hz
void initClock(void);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_H_ */
