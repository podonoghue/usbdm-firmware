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

#define DEVICE_SUBFAMILY_CortexM0



/* This definition is overridden if Clock initialisation is provided */
__attribute__((__weak__))
void SystemCoreClockUpdate(void) {
}

/* Actual Vector table */
extern int const __vector_table[];

#ifndef SCB_VTOR
#define SCB_VTOR (*(uint32_t *)0xE000ED08)
#endif

#if !defined(WDOG_CS1)

/* WDOG timer register */
typedef struct {
   volatile uint8_t cs1;
   volatile uint8_t cs2;
   volatile uint16_t cnt;
   volatile uint16_t toval;
   volatile uint16_t win;
} WatchDog;

#define WDOG (*(volatile WatchDog*) 0x40052000)

#define WDOG_CS1_EN_MASK       (1<<7)
#define WDOG_CS1_INT_MASK      (1<<6)
#define WDOG_CS1_UPDATE_MASK   (1<<5)
#define WDOG_CS1_DBG_MASK      (1<<2)
#define WDOG_CS1_WAIT_MASK     (1<<1)
#define WDOG_CS1_STOP_MASK     (1<<0)

#define WDOG_CS2_CLK(x)  ((x)<<0)

#define WDOG_CS1     WDOG.cs1
#define WDOG_CS2     WDOG.cs2
#define WDOG_CNT     WDOG.cnt
#define WDOG_TOVAL   WDOG.toval

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

   /* Set the interrupt vector table position */
   SCB_VTOR = (uint32_t)__vector_table;

   // Disable watch-dog
   WDOG_CNT    = WDOG_KEY1;               // Write the 1st unlock word
   WDOG_CNT    = WDOG_KEY2;               // Write the 2nd unlock word
   WDOG_TOVAL  = 1010;                    // Setting time-out value
   WDOG_CS2    = WDOG_CS2_CLK(1);         // Setting 1-kHz clock source
   WDOG_CS1    = WDOG_CS1_UPDATE_MASK;    // Disable watchdog
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
   uart_initialise(19200);

   /* Use RTC initialisation - if present */
   rtc_initialise();

}

