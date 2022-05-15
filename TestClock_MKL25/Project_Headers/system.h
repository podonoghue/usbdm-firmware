/**
 * @file     system.h (180.ARM_Peripherals/Project_Headers/system.h)
 * @brief    System initialisation routines
 * @version  V4.11.1.70
 * @date     13 Nov 2012
 */
#ifndef INCLUDE_USBDM_SYSTEM_H_
#define INCLUDE_USBDM_SYSTEM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;    //!< System core clock frequency in Hz
extern uint32_t SystemBusClock;     //!< System bus clock frequency in Hz
extern uint32_t SystemFlexbusClock; //!< System Flexbus clock frequency in Hz

/**
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

#if defined(__CM3_REV) || defined(__CM4_REV) // Only available on Cortex M3 & M4
/**
 * Obtain lock
 *
 * @param lockVar Locking variable to use
 *
 * @note This is a spin-lock - don't use on interrupts
 */
static inline void lock(volatile uint32_t *lockVar) {
   do {
      // If not locked
      if (__LDREXW(lockVar) == 0) {
         // Try to obtain lock by writing 1
         if (__STREXW(1, lockVar) == 0) {
            // Succeeded
            // Do not start any other memory access
            // until memory barrier is completed
            __DMB();
            return;
         }
      }
   } while (1);
}

/**
 * Release lock
 *
 * @param addr Locking variable to use
 */
static inline void unlock(volatile uint32_t *lockVar) {
   // Ensure memory operations completed before
   __DMB();
   // Release lock
   *lockVar = 0;
}
#else
// Not available on Cortex M0
static inline void lock(volatile uint32_t * dummy) {(void)dummy;}
static inline void unlock(volatile uint32_t * dummy) {(void)dummy;}
#endif

#ifndef __cplusplus
/**
 * Enter critical section
 *
 * Disables interrupts for a critical section
 *
 * @param cpuSR Variable to hold interrupt state so it can be restored
 *
 * @code
 * uint8_t cpuSR;
 * ...
 * enterCriticalSection(&cpuSR);
 *  // Critical section
 * exitCriticalSection(&cpuSR);
 * @endcode
 */
static inline void enterCriticalSection(uint8_t *cpuSR) {
   __asm__ volatile (
         "  MRS   r0, PRIMASK       \n"   // Copy flags
         // It may be possible for a ISR to run here but it
         // would save/restore PRIMASK so this code is OK
         "  CPSID I                 \n"   // Disable interrupts
         "  STRB  r0, %[output]     \n"   // Save flags
         : [output] "=m" (cpuSR) : : "r0");
}

/**
 * Exit critical section
 *
 * Restores interrupt state saved by enterCriticalSection()
 *
 * @param cpuSR Variable to holding interrupt state to be restored
 */
static inline void exitCriticalSection(uint8_t *cpuSR) {
   __asm__ volatile (
         "  LDRB r0, %[input]    \n"  // Retrieve original flags
         "  MSR  PRIMASK,r0;     \n"  // Restore
         : :[input] "m" (cpuSR) : "r0");
}
#endif // __cplusplus

#ifdef __cplusplus
}
#endif


#endif /* INCLUDE_USBDM_SYSTEM_H_ */
