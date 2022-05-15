/**
 * @file    leds.c
 * @brief   Basic LED control for demo boards
 * @date    13 June 2015
 * 
 * Generated from leds.c
 */
#include "derivative.h"
#include "utilities.h"
#include "Board_LEDs.h"
#include "leds.h"

/*
 * ====================================================================
 */

#ifdef LED_GREEN_PORT
void greenLedOn(void) {
   PCOR(LED_GREEN_PORT) = (1<<LED_GREEN_NUM);
}
void greenLedOff(void) {
   PSOR(LED_GREEN_PORT) = (1<<LED_GREEN_NUM);
}
#endif

// Define at least a dummy function as used by default main()
void greenLedToggle(void) {
#ifdef LED_GREEN_PORT
   PTOR(LED_GREEN_PORT) = (1<<LED_GREEN_NUM);
#endif
}

#ifdef LED_RED_PORT
void redLedOn(void) {
   PCOR(LED_RED_PORT) = (1<<LED_RED_NUM);
}
void redLedOff(void) {
   PSOR(LED_RED_PORT) = (1<<LED_RED_NUM);
}
void redLedToggle(void) {
   PTOR(LED_RED_PORT) = (1<<LED_RED_NUM);
}
#endif

#ifdef LED_BLUE_PORT
void blueLedOn(void) {
   PCOR(LED_BLUE_PORT) = (1<<LED_BLUE_NUM);
}
void blueLedOff(void) {
   PSOR(LED_BLUE_PORT) = (1<<LED_BLUE_NUM);
}
void blueLedToggle(void) {
   PTOR(LED_BLUE_PORT) = (1<<LED_BLUE_NUM);
}
void blueLedEnable(void) {
   PDDR(LED_BLUE_PORT) |= (1<<LED_BLUE_NUM);
}
void blueLedDisable(void) {
   PDDR(LED_BLUE_PORT) &= ~(1<<LED_BLUE_NUM);
}
#endif

#ifdef LED_ORANGE_PORT
void orangeLedOn(void) {
   PCOR(LED_ORANGE_PORT) = (1<<LED_ORANGE_NUM);
}
void orangeLedOff(void) {
   PSOR(LED_ORANGE_PORT) = (1<<LED_ORANGE_NUM);
}
void orangeLedToggle(void) {
   PTOR(LED_ORANGE_PORT) = (1<<LED_ORANGE_NUM);
}
#endif

#ifndef PORT_PCR_DSE_MASK
//! Dummy definition for devices without DSE feature
#define PORT_PCR_DSE_MASK (0)
#endif

void led_initialise(void) {
#ifdef LED_GREEN_PORT
   SIM->SCGC5 |=  PORT_CLOCK_MASK(LED_GREEN_PORT);
   greenLedOff();
   PDDR(LED_GREEN_PORT)               |= (1<<LED_GREEN_NUM);
   PCR(LED_GREEN_PORT, LED_GREEN_NUM)  = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;
#endif

#ifdef LED_RED_PORT
   SIM->SCGC5 |=  PORT_CLOCK_MASK(LED_RED_PORT);
   redLedOff();
   PDDR(LED_RED_PORT)             |= (1<<LED_RED_NUM);
   PCR(LED_RED_PORT, LED_RED_NUM)  = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;
#endif

#ifdef LED_BLUE_PORT
   SIM->SCGC5 |=  PORT_CLOCK_MASK(LED_BLUE_PORT);
   blueLedOff();
   PDDR(LED_BLUE_PORT)              |= (1<<LED_BLUE_NUM);
   PCR(LED_BLUE_PORT, LED_BLUE_NUM)  = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;
#endif

#ifdef LED_ORANGE_PORT
   SIM->SCGC5 |=  PORT_CLOCK_MASK(LED_ORANGE_PORT);
   orangeLedOff();
   PDDR(LED_ORANGE_PORT)                |= (1<<LED_ORANGE_NUM);
   PCR(LED_ORANGE_PORT, LED_ORANGE_NUM)  = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;
#endif
}
