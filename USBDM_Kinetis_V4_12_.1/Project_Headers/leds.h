/*
 * leds.h
 *
 *  Created on: 14/04/2013
 *      Author: pgo
 */

#ifndef LEDS_H_
#define LEDS_H_

#ifdef __cplusplus
extern "C" {
#endif

void greenLedOn(void);
void greenLedOff(void);
void greenLedToggle(void);
void redLedOn(void);
void redLedOff(void);
void redLedToggle(void);
void blueLedOn(void);
void blueLedOff(void);
void blueLedToggle(void);
void blueLedEnable(void);
void blueLedDisable(void);
void orangeLedOn(void);
void orangeLedOff(void);
void orangeLedToggle(void);

void led_initialise(void);

#ifdef __cplusplus
}
#endif

#endif /* LEDS_H_ */
