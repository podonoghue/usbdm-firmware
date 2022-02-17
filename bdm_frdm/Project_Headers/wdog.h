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
 * Disables watchdog test mode until next reset
 */
enum WdogTestMode {
   WdogTestMode_Enabled  = WDOG_STCTRLH_DISTESTWDOG(0), //!< Not disabled
   WdogTestMode_Disabled = WDOG_STCTRLH_DISTESTWDOG(1), //!< Disabled until next reset
};

/**
 * Enable watchdog in WAIT mode
 */
enum WdogEnableInWait {
   WdogEnableInWait_Disabled  = WDOG_STCTRLH_WAITEN(0), //!< Disabled in WAIT mode
   WdogEnableInWait_Enabled   = WDOG_STCTRLH_WAITEN(1), //!< Enabled in WAIT mode
};

/**
 * Enable watchdog in STOP mode
 */
enum WdogEnableInStop {
   WdogEnableInStop_Disabled  = WDOG_STCTRLH_STOPEN(0), //!< Disabled in STOP mode
   WdogEnableInStop_Enabled   = WDOG_STCTRLH_STOPEN(1), //!< Enabled in STOP mode
};

/**
 * Enable watchdog in DEBUG mode
 */
enum WdogEnableInDebug {
   WdogEnableInDebug_Disabled  = WDOG_STCTRLH_DBGEN(0), //!< Disabled in DEBUG mode
   WdogEnableInDebug_Enabled   = WDOG_STCTRLH_DBGEN(1), //!< Enabled in DEBUG mode
};

/**
 * Enable update of watchdog protected registers until next reset
 */
enum WdogAllowUpdate {
   WdogAllowUpdate_Disabled  = WDOG_STCTRLH_ALLOWUPDATE(0), //!< Disallow update of protected registers
   WdogAllowUpdate_Enabled   = WDOG_STCTRLH_ALLOWUPDATE(1), //!< Enabled update of protected registers
};

/**
 * Enable watchdog windowing mode
 */
enum WdogWindow {
   WdogWindow_Disabled  = WDOG_STCTRLH_WINEN(0), //!< Disable windowing mode
   WdogWindow_Enabled   = WDOG_STCTRLH_WINEN(1), //!< Enabled windowing mode
};

/**
 * Controls watchdog Interrupts
 */
enum WdogInterrupt {
   WdogInterrupt_Disabled = WDOG_STCTRLH_IRQRSTEN(0), //!< Interrupts disabled
   WdogInterrupt_Enabled  = WDOG_STCTRLH_IRQRSTEN(1), //!< Interrupts enabled
};

/**
 * Watchdog clock source
 */
enum WdogClock {
   WdogClock_Lpo  = WDOG_STCTRLH_CLKSRC(0), //!< Watchdog clock = LPO
   WdogClock_Alt  = WDOG_STCTRLH_CLKSRC(1), //!< Watchdog clock = alternate clock source
};

/**
 * Watchdog enable/disable
 */
enum WdogEnable {
   WdogEnable_Disabled  = WDOG_STCTRLH_WDOGEN(0), //!< Watchdog disabled
   WdogEnable_Enabled   = WDOG_STCTRLH_WDOGEN(1), //!< Watchdog enabled
};

/** Watchdog Refresh value 1 */
static constexpr uint16_t WdogRefresh1 = 0xA602;

/** Watchdog Refresh value 2 */
static constexpr uint16_t WdogRefresh2 = 0xB480;

/** Watchdog Unlock value 1 */
static constexpr uint16_t WdogUnlock1 = 0xC520;

/** Watchdog Unlock value 2 */
static constexpr uint16_t WdogUnlock2 = 0xD928;

/**
 * Type definition for WDOG interrupt call back
 *
 * @param[in]  status Struct indicating interrupt source and state
 */
typedef void (*WdogCallbackFunction)();

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
class WdogBase_T {

protected:
   /**
    * Callback to catch unhandled interrupt
    */
   static void unhandledCallback() {
      setAndCheckErrorCode(E_NO_HANDLER);
   }

   /** Callback function for ISR */
   static WdogCallbackFunction callback;

public:
   /**
    * Hardware instance pointer.
    *
    * @return Reference to WDOG hardware
    */
   static constexpr HardwarePtr<WDOG_Type> wdog = Info::baseAddress;

   /**
    * IRQ handler
    */
   static void irqHandler() {
      // Call handler
      callback();
   }

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
    *    // This function must match WdogCallbackFunction
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
   static WdogCallbackFunction wrapCallback() {
      static WdogCallbackFunction fn = []() {
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
    *    // This function must match WdogCallbackFunction
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
   static WdogCallbackFunction wrapCallback(T &object) {
      static T &obj = object;
      static WdogCallbackFunction fn = []() {
         (obj.*callback)();
      };
      return fn;
   }

   /**
    * Set callback function.
    *
    * The callback may be executed prior to the WDOG reset.
    * This allows the system to save important information or log the watchdog event.
    *
    * @param[in]  theCallback Callback function to execute on interrupt
    */
   static void setCallback(WdogCallbackFunction theCallback) {
      static_assert(Info::irqHandlerInstalled, "WDOG not configured for interrupts");
      if (theCallback == nullptr) {
         theCallback = unhandledCallback;
      }
      callback = theCallback;
   }

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
    * @param prescaler [1.8]
    * @param ticks
    *
    * @note This is a protected operation which uses unlock
    */
   static void setTimeout(uint8_t prescaler, Ticks ticks) {
      // Disable interrupts while accessing watchdog
      CriticalSection cs;
      writeUnlock(WdogUnlock1, WdogUnlock2);
      wdog->PRESC = WDOG_PRESC_PRESCVAL(prescaler-1);
      wdog->TOVALH = (unsigned)ticks>>16;
      wdog->TOVALL = (unsigned)ticks;
   }

   /**
    * Sets the watchdog time-out value in seconds.
    *
    * @param seconds
    *
    * @note This is a protected operation which uses unlock
    * @note This adjusts both the prescaler and the timeout value.
    */
   static ErrorCode setTimeout(Seconds seconds) {
      unsigned prescaler;
      uint64_t timerValue;
      uint32_t inputClockFreq = WdogInfo::getInputClockFrequency();

      for (prescaler = 1; prescaler++; ) {
         if (prescaler>8) {
            return setErrorCode(E_TOO_LARGE);
         }
         timerValue = (uint64_t)(((float)seconds*inputClockFreq)/prescaler);
         if (timerValue <= 0xFFFF) {
            break;
         }
      }
      setTimeout(prescaler, (unsigned)timerValue);
      return E_NO_ERROR;
   }

   /**
    * Sets the watchdog window value.
    *
    * @param value
    *
    * @note This is a protected operation which requires unlock
    */
   static void setWindow(uint32_t value) {
      // Disable interrupts while accessing watchdog
      CriticalSection cs;
      writeUnlock(WdogUnlock1, WdogUnlock2);
      wdog->WINH = value>>16;
      wdog->WINL = value;
   }

   /**
    * Enable with default settings.
    * Includes configuring all pins
    */
   static void defaultConfigure() {
      enable();
      // Initialise hardware
   }

   /**
    * Writing the sequence of 0xA602 (WdogRefresh1) followed by 0xB480 (WdogRefresh2) within 20 bus clock
    * cycles refreshes the WDOG and prevents it from resetting the system. Writing a value other than
    * the above mentioned sequence or if the sequence is longer than 20 bus cycles, resets the system,
    * or if IRQRSTEN is set, it interrupts and then resets the system.
    *
    * @param wdogRefresh1 1st value to write (WdogRefresh1)
    * @param wdogRefresh2 2nd value to write (WdogRefresh2)
    */
   static void writeRefresh(uint16_t wdogRefresh1, uint16_t wdogRefresh2) {
      wdog->REFRESH = wdogRefresh1;
      wdog->REFRESH = wdogRefresh2;
   }

   /**
    * Writes the unlock sequence values to this register to makes
    * the watchdog write-once registers writable again.
    *
    * @param wdogUnlock1 1st value to write (WdogUnlock1)
    * @param wdogUnlock2 2nd value to write (WdogUnlock2)
    */
   static void writeUnlock(uint16_t wdogUnlock1, uint16_t wdogUnlock2) {
      wdog->UNLOCK = wdogUnlock1;
      wdog->UNLOCK = wdogUnlock2;
   }

   /**
    * Configure watchdog
    *
    * @note This is a protected operation which uses unlock
    * @note Register changes after unlock is enabled
    */
   static __attribute__((always_inline)) void configure(
         WdogEnable        wdogEnable,
         WdogClock         wdogClock,
         WdogWindow        wdogWindow          = WdogWindow_Disabled,
         WdogInterrupt     wdogInterrupt       = WdogInterrupt_Disabled,
         WdogEnableInDebug wdogEnableInDebug   = WdogEnableInDebug_Disabled,
         WdogEnableInStop  wdogEnableInStop    = WdogEnableInStop_Disabled,
         WdogEnableInWait  wdogEnableInWait    = WdogEnableInWait_Disabled ) {

      enable();

      // Protect sequence from interrupts
      CriticalSection cs;

      // Unlock before changing settings
      writeUnlock(WdogUnlock1, WdogUnlock2);

      wdog->STCTRLH = wdogEnable|wdogClock|wdogInterrupt|wdogWindow|WdogAllowUpdate_Enabled|wdogEnableInDebug|wdogEnableInStop|wdogEnableInWait;
   }

   /**
    * Lock watchdog register against further changes
    */
   static void lockRegisters() {
      // Protect sequence from interrupts
      CriticalSection cs;

      // Unlock before changing settings
      writeUnlock(WdogUnlock1, WdogUnlock2);
      wdog->STCTRLH = wdog->STCTRLH & ~WDOG_STCTRLH_ALLOWUPDATE_MASK;
   }

   /**
    * Disable interface to WDOG
    */
   static void disable() {
      Info::disableClock();
   }

   /**
    * Enable interrupts in NVIC
    * Any pending NVIC interrupts are first cleared.
    */
   static void enableNvicInterrupts() {
      enableNvicInterrupt(Info::irqNums[0]);
   }

   /**
    * Enable and set priority of interrupts in NVIC
    * Any pending NVIC interrupts are first cleared.
    *
    * @param[in]  nvicPriority  Interrupt priority
    */
   static void enableNvicInterrupts(NvicPriority nvicPriority) {
      enableNvicInterrupt(Info::irqNums[0], nvicPriority);
   }

   /**
    * Disable interrupts in NVIC
    */
   static void disableNvicInterrupts() {
      NVIC_DisableIRQ(Info::irqNums[0]);
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

template<class Info> WdogCallbackFunction WdogBase_T<Info>::callback = WdogBase_T<Info>::unhandledCallback;

#if defined(USBDM_WDOG_IS_DEFINED)
class Wdog : public WdogBase_T<WdogInfo> {};
#endif

/**
 * End WDOG_Group
 * @}
 */
} // End namespace USBDM

#endif /* HEADER_WDOG_H_ */
