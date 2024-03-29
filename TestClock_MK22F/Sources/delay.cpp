/**
 * @file delay.cpp (180.ARM_DeviceOptions/Sources/delay.cpp)
 *
 * @brief Delay routines using Systick Counter
 *
 *
 *  Created on: 5 Nov 2015
 *      Author: podonoghue
 */
#include "math.h"
#include "delay.h"

#ifdef __CMSIS_RTOS
#include "cmsis_os.h"  // CMSIS RTX
#endif

/*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#ifdef __cplusplus
namespace USBDM {
#endif

#if defined(__CMSIS_RTOS)
/**
 * Simple delay routine
 *
 * @param[in] delayct How many ticks to busy-wait
 *
 * @note Does not suspend the thread - this is a busy-wait loop
 */
static void waitTicks(int64_t delayct) {

   // Get current tick
   uint32_t last = osKernelSysTick();

   while(delayct > 0) {
      // Decrement time elapsed
      // Note: This relies on the loop executing in less than the roll-over time
      // of the kernel counter
      uint32_t now = osKernelSysTick();
      delayct -= (uint32_t)(now-last);
      last = now;
   }
}

/**
 * Simple delay routine
 *
 * @param[in] usToWait How many microseconds to busy-wait
 *
 * @note Uses busy-waiting
 */
void waitUS(uint32_t usToWait) {
   // Convert duration to ticks
   waitTicks(osKernelSysTickMicroSec(usToWait));
}

/**
 * Simple delay routine
 *
 * @param[in]  msToWait How many milliseconds to busy-wait
 *
 * @note Uses busy-waiting
 */
void waitMS(uint32_t msToWait) {
   // Convert duration to ticks
   waitTicks(1000UL*osKernelSysTickMicroSec(msToWait));
}
#else

/**
 * Simple delay routine
 *
 * @param[in] delayct How many ticks to busy-wait
 *
 * @note This is a busy-wait loop
 */
static void waitTicks(int64_t delayct) {

   // Enable counter
   enableTimer();

   // Get current tick
   uint32_t last = getTicks();

   while(delayct > 0) {
      // Decrement time elapsed
      // Note: This relies on the loop executing in less than the roll-over time
      // of the counter i.e. (TIMER_MASK+1)/SystemCoreClock
      uint32_t now = getTicks();
      delayct -= (uint32_t)(TIMER_MASK&(last-now));
      last = now;
   }
}

/**
 * Simple delay routine
 *
 * @param[in] usToWait How many microseconds to busy-wait
 *
 * @note Uses busy-waiting
 */
void waitUS(uint32_t usToWait) {
   // Convert duration to ticks
   waitTicks(((uint64_t)usToWait * SystemCoreClock) / 1000000);
}

/**
 * Simple delay routine
 *
 * @param[in]  msToWait How many milliseconds to busy-wait
 *
 * @note Uses busy-waiting
 */
void waitMS(uint32_t msToWait) {
   // Convert duration to ticks
   waitTicks(((uint64_t)msToWait * SystemCoreClock) / 1000);
}
#endif // __CMSIS_RTOS

/**
 * Simple delay routine
 *
 * @param[in]  seconds How many seconds to busy-wait
 *
 * @note Limited to 2^32 ms (71,582 minutes)
 * @note Uses busy-waiting
 */
void wait(Seconds seconds) {
   // Convert duration to ticks
   waitTicks((int)round(float(seconds) * SystemCoreClock));
}

#ifdef __cplusplus

#if defined(__CMSIS_RTOS)
/**
 * Routine to wait for an event with timeout
 *
 * @param[in] delayct  How many ticks to busy-wait
 * @param[in] testFn   Polling function indicating if waited for event has occurred
 *
 * @return Indicate if event occurred: true=>event, false=>no event
 */
static bool waitTicks(int64_t delayct, bool testFn(void)) {
   // Get current tick
   uint32_t last = osKernelSysTick();

   while(delayct > 0) {
      if (testFn()) {
         return true;
      }
      // Decrement time elapsed
      // Note: This relies on the loop executing in less than the roll-over time
      // of the kernel counter
      uint32_t now = osKernelSysTick();
      delayct -= (uint32_t)(now-last);
      last = now;
   }
   return false;
}

/**
 * Routine to wait for a condition with timeout
 *
 * @param[in] usToWait How many microseconds to busy-wait
 * @param[in] testFn   Polling function indicating if waited for condition has occurred
 *
 * @return Indicate if event occurred: true=>event, false=>no event
 *
 * @note Limited to 2^32 us (4,294 s)
 * @note Uses busy-waiting
 */
bool waitUS(uint32_t usToWait, bool testFn(void)) {
   // Convert duration to ticks
   return waitTicks(osKernelSysTickMicroSec(usToWait), testFn);
}

/**
 * Routine to wait for an event with timeout
 *
 * @param[in] msToWait How many milliseconds to busy-wait
 * @param[in] testFn   Polling function indicating if waited for event has occurred
 *
 * @return Indicate if event occurred: true=>event, false=>no event
 *
 * Note: Accuracy is affected by execution time of function.
 */
bool waitMS(uint32_t msToWait, bool testFn(void)) {
   // Convert duration to ticks
   return waitTicks(1000*osKernelSysTickMicroSec(msToWait), testFn);
}

/**
 * Routine to wait for a condition with timeout
 *
 * @param[in] seconds  How many seconds to busy-wait
 * @param[in] testFn   Polling function indicating if waited for condition has occurred
 *
 * @return Indicate if event occurred: true=>event, false=>no event
 *
 * Note: Accuracy is affected by execution time of function.
 */
bool wait(float seconds, bool testFn(void)) {
   // Convert duration to ticks
   return waitTicks((int)round(seconds * SystemCoreClock), testFn);
}

#else
/**
 * Routine to wait for an event with timeout
 *
 * @param[in] delayct  How many ticks to busy-wait
 * @param[in] testFn   Polling function indicating if waited for event has occurred
 *
 * @return Indicate if event occurred: true=>event, false=>no event
 */
static bool waitTicks(int64_t delayct, bool testFn(void)) {

   // Enable counter
   enableTimer();

   // Get current tick
   uint32_t last = getTicks();

   while (delayct > 0) {
      if (testFn()) {
         return true;
      }
      // Decrement time elapsed
      // Note: This relies on the loop executing in less than the roll-over time
      // of the counter i.e. (TIMER_MASK+1)/SystemCoreClock
      uint32_t now = getTicks();
      delayct -= (uint32_t)(TIMER_MASK&(last-now));
      last = now;
   }
   return false;
}

/**
 * Routine to wait for an event with timeout
 *
 * @param[in] usToWait How many microseconds to busy-wait
 * @param[in] testFn   Polling function indicating if waited for event has occurred
 *
 * @return Indicate if event occurred. true=>event, false=>no event
 *
 * Note: Accuracy is affected by execution time of function.
 */
bool waitUS(uint32_t usToWait, bool testFn(void)) {
   // Convert duration to ticks
   return waitTicks(((uint64_t)usToWait * SystemCoreClock) / 1000000, testFn);
}

/**
 * Routine to wait for an event with timeout
 *
 * @param[in] msToWait How many milliseconds to busy-wait
 * @param[in] testFn   Polling function indicating if waited for event has occurred
 *
 * @return Indicate if event occurred: true=>event, false=>no event
 *
 * Note: Accuracy is affected by execution time of function.
 */
bool waitMS(uint32_t msToWait, bool testFn(void)) {
   // Convert duration to ticks
   return waitTicks(((uint64_t)msToWait * SystemCoreClock) / 1000, testFn);
}

/**
 * Routine to wait for an event with timeout
 *
 * @param[in] seconds  How many seconds to busy-wait
 * @param[in] testFn   Polling function indicating if waited for event has occurred
 *
 * @return Indicate if event occurred: true=>event, false=>no event
 *
 * Note: Accuracy is affected by execution time of function.
 */
bool wait(float seconds, bool testFn(void)) {
   // Convert duration to ticks
   return waitTicks((int)round(seconds * SystemCoreClock), testFn);
}
#endif // __CMSIS_RTOS

#endif // __cplusplus

#ifdef __cplusplus
} // End namespace USBDM
#endif

