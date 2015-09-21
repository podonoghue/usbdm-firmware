/*
 * sysinit-mke.c
 *
 * Generic system initialization for Kinetis MKExx family
 *
 *  Created on: 18/08/2013
 *      Author: podonoghue
 */

#include <stdint.h>
#include "derivative.h"

#define MKE02Z4



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

#if !defined(WDOG)

/* WDOG timer register */
typedef struct {
   volatile uint8_t  CS1;
   volatile uint8_t  CS2;
   volatile uint16_t CNT;
   volatile uint16_t TOVAL;
   volatile uint16_t WIN;
} WatchDog;

#define WDOG (*(volatile WatchDog*) 0x40052000)

#define WDOG_CS1_EN_MASK       (1<<7)
#define WDOG_CS1_INT_MASK      (1<<6)
#define WDOG_CS1_UPDATE_MASK   (1<<5)
#define WDOG_CS1_DBG_MASK      (1<<2)
#define WDOG_CS1_WAIT_MASK     (1<<1)
#define WDOG_CS1_STOP_MASK     (1<<0)

#define WDOG_CS2_CLK(x)  ((x)<<0)

#define WDOG_CS1     WDOG.CS1
#define WDOG_CS2     WDOG.CS2
#define WDOG_CNT     WDOG.CNT
#define WDOG_TOVAL   WDOG.TOVAL

#endif

#define WDOG_KEY1    (0x20C5)
#define WDOG_KEY2    (0x28D9)

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
   WDOG->CNT    = WDOG_KEY1;               // Write the 1st unlock word
   WDOG->CNT    = WDOG_KEY2;               // Write the 2nd unlock word
   WDOG->TOVAL  = 1010;                    // Setting time-out value
   WDOG->CS2    = WDOG_CS2_CLK(1);         // Setting 1-kHz clock source
   WDOG->CS1    = WDOG_CS1_UPDATE_MASK;    // Disable watchdog
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

