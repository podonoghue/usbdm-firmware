/*
 *  @file system.c
 *
 *  Derived from  system-kinetis.c
 *
 * Generic system initialization for Kinetis family
 *
 *  Created on: 25/5/2017
 */

#include <stdint.h>
#include "derivative.h"

/* This definition is overridden if Clock initialisation is provided */
__attribute__((__weak__))
void SystemCoreClockUpdate(void) {
}

/* These are overridden if actual clock code is provided */
__attribute__((__weak__))
uint32_t SystemCoreClock = 4000000;
__attribute__((__weak__))
uint32_t SystemBusClock = 8000000;

/* Actual Vector table */
extern int const __vector_table[];

/* This definition is overridden if Clock initialisation is provided */
__attribute__((__weak__))
void clock_initialise() {
}

/* This definition is overridden if UART initialisation is provided */
__attribute__((__weak__))
void console_initialise() {
}

/* This definition is overridden if RTC initialisation is provided */
__attribute__((__weak__))
void rtc_initialise(void) {
}

// Dummy hook routine for when CMSIS is not used.
__attribute__((__weak__))
void software_init_hook (void) {
}

#ifdef __NO_STARTFILES__
#warning Due to limited RAM the C library standard initialisation is not called - BSS and DATA are still initialised
#endif

/**
 *  @brief Low-level initialize the system
 *
 *  Low level setup of the microcontroller system. \n
 *  Called very early in the initialisation. \n
 *  May NOT use globals etc (as will be overwritten by BSS initialization)
 */
void SystemInitLowLevel(void) {
   /*
    * This is generic initialization code
    * It may not be correct for a specific target
    */

#ifdef __VTOR_PRESENT
   /* Set the interrupt vector table position */
   SCB->VTOR = (uint32_t)__vector_table;
#endif

#ifdef SCB_CCR_DIV_0_TRP_Msk
   /* Enable trapping of divide by zero */
   SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
#endif

#ifdef RCM_MR_BOOTROM
   // Clear Boot ROM flag
   RCM->MR = RCM_MR_BOOTROM(3);
#endif

#if defined(SIM_COPC_COPT_MASK)
   // Disable watch-dog
   SIM->COPC = SIM_COPC_COPT(0);
#endif

#ifdef WDOG_CS_UPDATE
   /* Unlocking Watchdog word */
#define WDOG_UPDATE_KEY  (0xD928C520U)

   // Disable watch-dog
   WDOG->CNT    = WDOG_UPDATE_KEY; // Write the unlock word
   WDOG->TOVAL  = -1;              // Setting time-out value
   WDOG->CS     =
         WDOG_CS_CLK(1) |        // Setting 1-kHz clock source
         WDOG_CS_UPDATE(1);      // Allow future update
#endif

#ifdef WDOG_CS1_UPDATE_MASK
   /* Unlocking Watchdog sequence words*/
#define WDOG_KEY1    (0x20C5)
#define WDOG_KEY2    (0x28D9)

   /* Disable watch-dog */
   WDOG->CNT    = WDOG_KEY1;               // Write the 1st unlock word
   WDOG->CNT    = WDOG_KEY2;               // Write the 2nd unlock word
   WDOG->TOVAL  = -1;                      // Setting time-out value
   WDOG->CS2    = WDOG_CS2_CLK(1);         // Setting 1-kHz clock source
   WDOG->CS1    = WDOG_CS1_UPDATE(1);      // Disable watchdog and allow future changes
#endif

#ifdef WDOG_UNLOCK_WDOGUNLOCK_MASK
   /* Unlocking Watchdog sequence words*/
#define WDOG_KEY1   (0xC520)
#define WDOG_KEY2   (0xD928)

   /* Disable watch-dog */
   WDOG->UNLOCK  = WDOG_UNLOCK_WDOGUNLOCK(WDOG_KEY1);
   WDOG->UNLOCK  = WDOG_UNLOCK_WDOGUNLOCK(WDOG_KEY2);
   __DSB();
   WDOG->STCTRLH =
         WDOG_STCTRLH_WDOGEN(0)|          // Disable WDOG
         WDOG_STCTRLH_ALLOWUPDATE(1)|     // Allow future updates
         WDOG_STCTRLH_CLKSRC(0);          // WDOG clk=LPO
#endif
}

/**
 * @brief Initialize the system
 *
 * Setup the microcontroller system.
 */
__attribute__ ((constructor))
void SystemInit(void) {
   /*
    * This is generic initialization code
    * It may not be correct for a specific target
    */

   /* Use Clock initialisation - if present */
   clock_initialise();

   /* Use UART initialisation - if present */
   console_initialise();

   /* Use RTC initialisation - if present */
   rtc_initialise();

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
   /* Initialise FPU if present & in use */
   __asm__ (
         "  .equ CPACR, 0xE000ED88     \n"
         "                             \n"
         "  LDR.W R0, =CPACR           \n"  // CPACR address
         "  LDR R1, [R0]               \n"  // Read CPACR
         "  ORR R1, R1, #(0xF << 20)   \n"  // Enable CP10 and CP11 coprocessors
         "  STR R1, [R0]               \n"  // Write back the modified value to the CPACR
         "  DSB                        \n"  // Wait for store to complete"
         "  ISB                        \n"  // Reset pipeline now the FPU is enabled
   );
#endif
}

// Code below assumes interrupts start out enabled!

/** Nesting count for interrupt disable */
static int disableInterruptCount = 0;

/**
 * Check interrupt status
 *
 * @return true if interrupts are enabled
 */
int areInterruptsEnabled() {
   return disableInterruptCount == 0;
}

/**
 * Disable interrupts
 *
 * This function keeps a count of the number of times interrupts is enabled/disabled so may be called in recursive routines
 */
void disableInterrupts() {
   __disable_irq();
   disableInterruptCount++;
}

/**
 * Enable interrupts
 *
 * This function keeps a count of the number of times interrupts is enabled/disabled so may be called in recursive routines
 *
 * @return true if interrupts are now enabled
 */
int enableInterrupts() {
   if (disableInterruptCount>0) {
      disableInterruptCount--;
   }
   if (disableInterruptCount == 0) {
      __enable_irq();
      return 1;
   }
   return 0;
}

