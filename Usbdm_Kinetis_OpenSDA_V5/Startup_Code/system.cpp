/*
 *  @file system.cpp (from Stationery/Packages/180.ARM_Peripherals/Startup_Code/)
 *
 * Generic system initialization for Kinetis family
 *
 *  Created on: 23/9/2017
 */

#include <stdint.h>
#include "derivative.h"
#include "pmc.h"
#include "sim.h"
#include "wdog.h"


/* This definition is overridden if Clock initialisation is provided */
__attribute__((__weak__))
void SystemCoreClockUpdate(void) {
}

   /**
    *  System Core Clock
    *  Clocks the ARM Cortex-M4 core and bus masters
    */
   uint32_t SystemCoreClock;
   
   /**
    *  System Bus Clock
    *  Clocks the bus slaves and peripherals
    *        - Must be &lt;= Core Clock frequency and an integer divisor
    */
   uint32_t SystemBusClock;
   


#ifdef __cplusplus
extern "C" {
#endif

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

// Dummy hook routine for when CMSIS is not used.
__attribute__((__weak__))
void software_init_hook () {
}

#ifdef __cplusplus
}
#endif

#ifdef __NO_STARTFILES__
#warning Due to limited RAM the C library standard initialisation is not called - BSS and DATA are still initialised
void* __dso_handle;
#endif

#if defined(KINETIS_BOOTLOADER_CHECK)
void checkICP();
#endif

/**
 *  @brief Low-level initialise the system
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

#if (__VTOR_PRESENT != 0)
   /* Set the interrupt vector table position */
   SCB->VTOR = (uint32_t)__vector_table;
#endif

#ifdef SCB_CCR_DIV_0_TRP_Msk
   /* Enable trapping of divide by zero */
   SCB->CCR = SCB->CCR | SCB_CCR_DIV_0_TRP_Msk;
#endif

#ifdef RCM_MR_BOOTROM
   // Clear Boot ROM flag
   RCM->MR = RCM_MR_BOOTROM(3);
#endif
   /*
    * Disable watchdog
    */
   USBDM::Wdog::disableWdog();
   
#if defined(KINETIS_BOOTLOADER_CHECK)
   /**
    * Hook for ICP code
    * Needed to be done before too much uC configuration
    */
   checkICP();
#endif
}

/**
 * @brief Initialise the system
 *
 * Setup the microcontroller system.
 */
__attribute__ ((constructor))
void SystemInit(void) {
   /*
    * This is generic initialisation code
    * It may not be correct for a specific target
    */

#ifdef PMC_REGSC_ACKISO
   USBDM::PmcInfo::releaseIsolation();
#endif

   /* Use Clock initialisation - if present */
   clock_initialise();
   
   /* Early system startup code for peripherals */
   /*  Initialise Sim */
   USBDM::Sim::defaultConfigure();

   /* Use UART initialisation - if present */
   console_initialise();

#if defined(__VFP_FP__) && !defined(__SOFTFP__)
//#warning "Using FP hardware"

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
/* System startup code for peripherals */


}
