/**
 * @file     wdog.h (180.ARM_Peripherals/Project_Headers/wdog.h)
 * @brief    External Watchdog Monitor
 *
 * @version  V4.12.1.230
 * @date     13 April 2016
 */

#ifndef HEADER_WDOG_H_
#define HEADER_WDOG_H_
 /*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include "derivative.h"
#include "pin_mapping.h"

namespace USBDM {

/**
 * @addtogroup WDOG_Group WDOG, Watchdog Timer
 * @brief Abstraction for Watchdog Timer
 * @{
 */

/**
 * Template class representing the Watchdog Monitor
 *
 * The Watchdog Timer (WDOG) keeps a watch on the system functioning and resets it in
 * case of its failure. Reasons for failure include run-away software code and the stoppage
 * of the system clock that in a safety critical system can lead to serious consequences. In
 * such cases, the watchdog brings the system into a safe state of operation. The watchdog
 * monitors the operation of the system by expecting periodic communication from the
 * software, generally known as servicing or refreshing the watchdog. If this periodic
 * refreshing does not occur, the watchdog resets the system.
 *
 * @tparam info      Information class for WDOG
 */
template<class Info>
class WdogBase_T : public Info {


protected:
// No private methods found

public:
   /**
    * Hardware instance pointer.
    *
    * @return Reference to WDOG hardware
    */
   static constexpr HardwarePtr<WDOG_Type> wdog = Info::baseAddress;

   /**
    * Writing the sequence of 0xA602 (WdogRefresh_1) followed by 0xB480 (WdogRefresh_2) within 20 bus clock
    * cycles refreshes the WDOG and prevents it from resetting the system. Writing a value other than
    * the above mentioned sequence or if the sequence is longer than 20 bus cycles, resets the system,
    * or if IRQRSTEN is set, it interrupts and then resets the system.
    *
    * @param wdogRefresh_1 1st value to write (WdogRefresh_1)
    * @param wdogRefresh_2 2nd value to write (WdogRefresh_2)
    *
    * @note This operation is time-critical so interrupts are disabled during refresh
    * @note Due to clock domain issues it is necessary to wait at least 5 clock
    *       cycles between attempted refreshes.  This is most significant when
    *       using the LPO clock source (i.e. at least 5 ms in that case).
    */
   static void refresh(WdogRefresh wdogRefresh_1, WdogRefresh wdogRefresh_2) {

      // Protect sequence from interrupts
      CriticalSection cs;

      wdog->REFRESH = wdogRefresh_1;
      wdog->REFRESH = wdogRefresh_2;
   }

protected:
#if false
   /**
    *
    * @param[in]     stctrlh   Used to obtain clock source (STCTRLH.CLKSRC)
    * @param[in,out] timeout   .seconds Timeout value in seconds -> .ticks   Timeout value in ticks
    * @param[in,out] window    .seconds  Window value in seconds -> .ticks    Window value in ticks
    * @param[out]    presc     Calculated prescale value (PRESC.PRESCVAL)
    *
    * @return Error code
    */
   static ErrorCode calculateTimingParameters(
         uint16_t       stctrlh,
         Seconds_Ticks &timeout,
         Seconds_Ticks &window,
         uint16_t      &presc) {

      float constexpr maxCount = ~1UL;

      if ((int)window.toTicks()>(int)timeout.toTicks()) {
         return E_ILLEGAL_PARAM;
      }
      uint32_t clockFrequency = WdogInfo::getInputClockFrequency((WdogClock)(stctrlh & WDOG_STCTRLH_CLKSRC_MASK));
      Seconds maxTime = maxCount/clockFrequency;

      for(int prescale=1; prescale<=8; prescale++) {
         float counterFrequency = clockFrequency/(float)prescale;
         maxTime = maxCount/clockFrequency;
         if (maxTime > timeout.toSeconds()) {
            timeout.fromTicks(Ticks(roundf(float(timeout.toSeconds())*counterFrequency)));
            window.fromTicks(Ticks(roundf(float(window.toSeconds())*counterFrequency)));
            presc = WDOG_PRESC_PRESCVAL(prescale-1);
            return E_NO_ERROR;
         }
      }
      return setErrorCode(E_TOO_LARGE);
   }
#endif

public:
   /**
    * Disable WDOG
    */
   static inline void disableWdog() {
   
      if (wdog->STCTRLH&WdogEnable_Enabled) {
   
         // Unlock before changing settings
         wdog->UNLOCK  = WdogUnlock_1;
         wdog->UNLOCK  = WdogUnlock_2;
   
         // Read-back to delay until change effected
         (void)(wdog->UNLOCK);
   
         // Disable watchdog
         wdog->STCTRLH = WdogEnable_Disabled|WdogAllowUpdate_Disabled;
      }
   }
   



#if false // /WDOG/generateSharedIrqInfo
   /**
    * Wrapper to allow the use of a class member as a callback function
    * @note Only usable with static objects.
    *
    * @tparam T         Type of the object containing the callback member function
    * @tparam callback  Member function pointer
    * @tparam object    Object containing the member function
    *
    * @return  Pointer to a function suitable for the use as a callback
    *
    * @code
    * class AClass {
    * public:
    *    int y;
    *
    *    // Member function used as callback
    *    // This function must match CallbackFunction
    *    void callback() {
    *       ...;
    *    }
    * };
    * ...
    * // Instance of class containing callback member function
    * static AClass aClass;
    * ...
    * // Wrap member function
    * auto fn = Wdog::wrapCallback<AClass, &AClass::callback, aClass>();
    * // Use as callback
    * Wdog::setCallback(fn);
    * @endcode
    */
   template<class T, void(T::*callback)(), T &object>
   static typename Info::CallbackFunction wrapCallback() {
      static typename Info::CallbackFunction fn = []() {
         (object.*callback)();
      };
      return fn;
   }

   /**
    * Wrapper to allow the use of a class member as a callback function
    * @note There is a considerable space and time overhead to using this method
    *
    * @tparam T         Type of the object containing the callback member function
    * @tparam callback  Member function pointer
    * @tparam object    Object containing the member function
    *
    * @return  Pointer to a function suitable for the use as a callback
    *
    * @code
    * class AClass {
    * public:
    *    int y;
    *
    *    // Member function used as callback
    *    // This function must match CallbackFunction
    *    void callback() {
    *       ...;
    *    }
    * };
    * ...
    * // Instance of class containing callback member function
    * AClass aClass;
    * ...
    * // Wrap member function
    * auto fn = Wdog::wrapCallback<AClass, &AClass::callback>(aClass);
    * // Use as callback
    * Wdog::setCallback(fn);
    * @endcode
    */
   template<class T, void(T::*callback)()>
   static typename Info::CallbackFunction wrapCallback(T &object) {
      static T &obj = object;
      static typename Info::CallbackFunction fn = []() {
         (obj.*callback)();
      };
      return fn;
   }
#endif

public:

   /**
    * Basic enable WDOG.
    *
    * Dummy function as always clocked.
    */
   static void enable() {
   }

   /**
    * Gets watchdog reset count.
    * This is a count of the number of watchdog timeout resets since power-on reset
    *
    * @return Count of timeout resets
    */
   static uint16_t getResetCount() {
      return wdog->RSTCNT;
   }

   /**
    * Gets watchdog timer value.
    *
    * @return current timer value
    */
   static Ticks getTimer() {
      return (wdog->TMROUTH<<16)|wdog->TMROUTL;
   }

   /**
    * Sets watchdog pre-scaler and time-out value in ticks.
    * The watchdog clock is divided by this value to provide the prescaled WDOG_CLK
    *
    * @param wdogPrescale This prescaler divides the input clock for the watchdog counter
    * @param timeout      The watchdog must be refreshed before the counter reaches this value
    * @param window       If windowed operation is enabled, then the watchdog can only be refreshed
    *        if the timer reaches a value greater than or equal to this window length value.
    *        A refresh outside of this window resets the system
    *
    * @note This is a protected operation which uses unlock
    */
   static void setTimeout(
            WdogPrescale wdogPrescale,
            Ticks        timeout,
            Ticks        window = 0_ticks) {

      // Disable interrupts while accessing watchdog
      CriticalSection cs;

      // Unlock before changing settings
      wdog->UNLOCK = WdogUnlock_1;
      wdog->UNLOCK = WdogUnlock_2;

      wdog->PRESC  = wdogPrescale;
      wdog->TOVALH = (unsigned)timeout>>16;
      wdog->TOVALL = (unsigned)timeout;
      wdog->WINH   = (unsigned)window>>16;
      wdog->WINL   = (unsigned)window;
   }

#if false // /WDOG/secondsSupport
   /**
    * Sets the watchdog time-out value in seconds.
    *
    * @param timeout The watchdog must be refreshed before this interval expires
    * @param window  Refresh of the watchdog may not be carried out before this interval has expired i.e.
    *        Refresh must occur within [window...timeout] if window mode is enabled.
    *        A refresh outside of this range resets the system
    *
    * @note This is a protected operation which uses unlock
    * @note This adjusts both the prescaler and the timeout value.
    */
   static ErrorCode setTimeout(
            Seconds timeout,
            Seconds window  = 0.0_s) {

      unsigned prescaler;
      uint64_t timerValue;
      uint32_t inputClockFreq = WdogInfo::getInputClockFrequency();

      for (prescaler = 1; prescaler++; ) {
         if (prescaler>8) {
            return setErrorCode(E_TOO_LARGE);
         }
         timerValue = (uint64_t)(((float)timeout*inputClockFreq)/prescaler);
         if (timerValue <= 0xFFFFFFFF) {
            break;
         }
      }
      uint64_t windowValue = (uint64_t)(((float)window*inputClockFreq)/prescaler);
      setTimeout((WdogPrescale)WDOG_PRESC_PRESCVAL(prescaler-1), (Ticks)timerValue, (Ticks)windowValue);
      return E_NO_ERROR;
   }
#endif

   /**
    * Lock watchdog register against further changes
    */
   static void lockRegisters() {
      // Protect sequence from interrupts
      CriticalSection cs;

      // Unlock before changing settings
      wdog->UNLOCK = WdogUnlock_1;
      wdog->UNLOCK = WdogUnlock_2;

      // Read-back to delay until change effected
      (void)(wdog->UNLOCK);

      wdog->STCTRLH = wdog->STCTRLH & ~WDOG_STCTRLH_ALLOWUPDATE_MASK;
   }

   /**
    * Disable interface to WDOG
    */
   static void disable() {
      Info::disableClock();
   }

   /**
    * Enable/disable interrupts
    *
    * @param[in]  enable        True => enable, False => disable
    *
    * @note This is a protected operation which requires unlock
    */
   static void enableInterrupt(bool enable=true) {
      // Protect sequence from interrupts
      CriticalSection cs;
      if (enable) {
         wdog->STCTRLH = wdog->STCTRLH | WDOG_STCTRLH_IRQRSTEN_MASK;
      }
      else {
         wdog->STCTRLH = wdog->STCTRLH & ~WDOG_STCTRLH_IRQRSTEN_MASK;
      }
   }
};

//template<class Info> typename WdogBase_T<Info>::CallbackFunction WdogBase_T<Info>::callback = WdogBase_T<Info>::unhandledCallback;

   /**
    * Class representing WDOG
    */
   class Wdog : public WdogBase_T<WdogInfo> {};
   

/**
 * End WDOG_Group
 * @}
 */
} // End namespace USBDM

#endif /* HEADER_WDOG_H_ */