/*
 * sysinit-mkl.c
 *
 * Generic system initialization for Kinetis MKLxx family
 *
 *  Created on: 07/12/2012
 *      Author: podonoghue
 */

#include <stdint.h>
#include "derivative.h"

#define MKL25Z4



/* This definition is overridden if Clock initialisation is provided */
__attribute__((__weak__))
void SystemCoreClockUpdate(void) {
}

/* This is overridden if actual clock code is provided */
__attribute__((__weak__))
uint32_t SystemBusClock = 8000000;

/* Actual Vector table */
extern int const __vector_table[];

#ifndef SCB
   #define SCB_VTOR                 (*(uint32_t *)0xE000ED08)
   #define SCB_CCR                  (*(uint32_t *)0xE000ED14)
   #define SCB_CCR_DIV_0_TRP_MASK   (1<<4)
   #define SCB_CCR_UNALIGN_TRP_MASK (1<<3)
#else
   #define SCB_VTOR  (SCB->VTOR)
   #define SCB_CCR   (SCB->CCR)
#endif

#if !defined(SIM)
   #define SIM_COPC (*(uint32_t *)0x40048100)
#else
   #define SIM_COPC (SIM->COPC)
#endif


/* This definition is overridden if Clock initialisation is provided */
__attribute__((__weak__))
void clock_initialise() {
}

/* This definition is overridden if UART initialisation is provided */
__attribute__((__weak__))
void uart_initialise(int baudRate __attribute__((__unused__))) {
}

/* This definition is overridden if RTC initialisation is provided */
__attribute__((__weak__))
void rtc_initialise(void) {
}

// Dummy hook routine for when CMSIS is not used.
__attribute__((weak))
void software_init_hook (void) {
}

#ifdef __NO_STARTFILES__
#warning Due to limited RAM the C library standard initialisation is not called - BSS and DATA are still initialised
#endif

/*!
 *  @brief Low-level initialize the system
 *
 *  Low level setup of the microcontroller system. \n
 *  Called very early in the initialisation. \n
 *  May NOT use globals etc (as will be overwritten by BSS initialization)
 */
void SystemInitLowLevel(void) {
   /* This is generic initialization code */
   /* It may not be correct for a specific target */

#ifdef __VTOR_PRESENT
   /* Set the interrupt vector table position */
   SCB_VTOR = (uint32_t)__vector_table;
#endif

   // Disable watch-dog
   SIM_COPC = 0x00;
}

/**
 * @brief Initialize the system
 *
 * Setup the microcontroller system.
 */
void SystemInit(void) {
   /* This is generic initialization code */
   /* It may not be correct for a specific target */

   /* Use Clock initialisation - if present */
   clock_initialise();

   /* Use UART initialisation - if present */
   uart_initialise(DEFAULT_BAUD_RATE);

   /* Use RTC initialisation - if present */
   rtc_initialise();

}

