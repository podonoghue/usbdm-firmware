/**
 * @file     pit.h (180.ARM_Peripherals/Project_Headers/pit-MK.h)
 *
 * @brief    Programmable Interrupt Timer interface
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */

#ifndef INCLUDE_USBDM_PIT_H_
#define INCLUDE_USBDM_PIT_H_
/*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include "math.h"
#include "derivative.h"
#include "system.h"
#include "pin_mapping.h"

#if true // /PIT/enablePeripheralSupport

namespace USBDM {

/**
 * @addtogroup PIT_Group PIT, Programmable Interrupt Timer
 * @brief Abstraction for Programmable Interrupt Timer
 * @{
 */

/**
 * Calculate a PIT channel number using an offset from an existing number
 *
 * @param pitChannelNum Base channel to use
 * @param offset  Offset from base channel
 *
 * @return  PIT channel number calculated from channel+offset
 */
constexpr PitChannelNum inline operator+(PitChannelNum pitChannelNum, unsigned offset) {
   return PitChannelNum(unsigned(pitChannelNum) + offset);
}

/**
 * Calculate a PIT channel number using an offset from an existing number
 *
 * @param pitChannelNum Base channel to use
 * @param offset  Offset from base channel
 *
 * @return  PIT channel number calculated from channel+offset
 */
constexpr PitChannelNum inline operator+(PitChannelNum pitChannelNum, int offset) {
   return PitChannelNum(unsigned(pitChannelNum) + unsigned(offset));
}

/**
 * @brief Class representing a Programmable Interrupt  Timer
 *
 * <b>Example</b>
 * @code
 *
 * @endcode
 */
template<class Info>
class PitBase_T : public Info {

private:
   /**
    * This class is not intended to be instantiated
    */
   PitBase_T(const PitBase_T&) = delete;
   PitBase_T(PitBase_T&&) = delete;

protected:

   /** Default TCTRL value for timer channel */
   static constexpr uint32_t PIT_TCTRL_DEFAULT_VALUE = (PIT_TCTRL_TEN_MASK);

   // Reduce clutter
   using Info::allocatedChannels;
   using Info::NumChannels;

// /PIT/protectedMethods not found
public:
   /// Defaulted constructor
   constexpr PitBase_T() = default;

// /PIT/publicMethods not found
   /**
    * Allocate PIT channel.
    *
    * @return PitChannelNum_None - No suitable channel available.  Error code set.
    * @return Channel number     - Number of allocated channel
    */
   static PitChannelNum allocateChannel() {
      CriticalSection cs;
      unsigned channelNum = __builtin_ffs(allocatedChannels);
      if ((channelNum == 0)||(--channelNum>=NumChannels)) {
         setErrorCode(E_NO_RESOURCE);
         return PitChannelNum_None;
      }
      allocatedChannels &= ~(1<<channelNum);
      return (PitChannelNum) channelNum;
   }

#if false // (/DMA/enablePeripheralSupport)
   /**
    * Allocate PIT channel associated with DMA channel.
    * This is a channel that may be used to throttle the associated DMA channel.
    *
    * @param dmaChannelNum DMA channel being used.
    *
    * @return PitChannelNum_None - No suitable channel available.  Error code set.
    * @return Channel number     - Number of allocated channel
    */
   static PitChannelNum allocateDmaAssociatedChannel(DmaChannelNum dmaChannelNum) {
      const uint32_t channelMask = (1<<dmaChannelNum);
      usbdm_assert(dmaChannelNum<NumChannels, "No PIT channel associated with DMA channel");
      CriticalSection cs;
      usbdm_assert((allocatedChannels & channelMask) != 0, "PIT channel already allocated");
      if ((allocatedChannels & channelMask) == 0) {
         setErrorCode(E_NO_RESOURCE);
         return PitChannelNum_None;
      }
      allocatedChannels &= ~channelMask;
      return (PitChannelNum) dmaChannelNum;
   }
#endif // (/DMA/enablePeripheralSupport)

   /**
    * Free PIT channel.
    * Disables the channel.
    *
    * @param pitChannelNum Channel to release
    */
   static void freeChannel(PitChannelNum pitChannelNum) {
      if (pitChannelNum == PitChannelNum_None) {
         return;
      }
      const uint32_t channelMask = (1<<pitChannelNum);
      usbdm_assert(pitChannelNum<NumChannels, "Illegal PIT channel");
      usbdm_assert((allocatedChannels & channelMask) == 0, "Freeing unallocated PIT channel");

      disableChannel(pitChannelNum);
      CriticalSection cs;
      allocatedChannels |= channelMask;
   }

   /**
    * Enable/disable channel interrupts
    *
    * @param[in]  pitChannelNum Channel being modified
    * @param[in]  enable  True => enable, False => disable
    */
   static void setChannelAction(PitChannelNum pitChannelNum, PitChannelAction pitChannelAction) {
      pit->CHANNEL[pitChannelNum].TCTRL = pit->CHANNEL[pitChannelNum].TCTRL | pitChannelAction;
   }

protected:
   /** Pointer to hardware */
   static constexpr HardwarePtr<PIT_Type> pit = Info::baseAddress;

public:
   /**
    * Basic enable of PIT
    */
   static void enable() {
      // Enable clock
      Info::enableClock();
      __DMB();
   }

   /**
    *   Disable the PIT (all channels)
    */
   static void disable() {
      pit->MCR = PIT_MCR_MDIS(1);
      Info::disableClock();
   }

   /**
    *  Enable the PIT channel
    *
    *  @param[in]  pitChannelNum   Channel to enable
    */
   static void enableChannel(const PitChannelNum pitChannelNum) {
      pit->CHANNEL[pitChannelNum].TCTRL = pit->CHANNEL[pitChannelNum].TCTRL | PIT_TCTRL_TEN_MASK;
   }

   /**
    *   Disable the PIT channel
    *
    *   @param[in]  pitChannelNum Channel to disable
    */
   static void disableChannel(PitChannelNum pitChannelNum) {

      // Disable timer channel
      pit->CHANNEL[pitChannelNum].TCTRL = pit->CHANNEL[pitChannelNum].TCTRL & ~PIT_TCTRL_TEN_MASK;
   }

   /**
    * Clear channel interrupt flag
    */
   static void clearInterruptFlag(PitChannelNum pitChannelNum)  {
      pit->CHANNEL[pitChannelNum].TFLG = PIT_TFLG_TIF_MASK;
   }

   /**
    *  Configure the PIT channel
    *
    *  @param[in]  pitChannelNum     Channel to configure
    *  @param[in]  tickInterval      Interval in timer ticks (usually bus clock period)
    *  @param[in]  pitChannelAction  Action to take on timer event
    *
    *  @note The timer channel is disabled before configuring so that period changes have
    *        immediate effect.
    */
   static void configureChannel(
         PitChannelNum     pitChannelNum,
         Ticks             tickInterval,
         PitChannelAction  pitChannelAction=PitChannelAction_None) {

      usbdm_assert((unsigned)tickInterval>0, "Interval too short");

      pit->CHANNEL[pitChannelNum].TCTRL = 0;
      pit->CHANNEL[pitChannelNum].LDVAL = (unsigned)tickInterval-1;
      pit->CHANNEL[pitChannelNum].TFLG  = PIT_TFLG_TIF_MASK;
      pit->CHANNEL[pitChannelNum].TCTRL = pitChannelAction|PIT_TCTRL_TEN(1);
   }

#if false // /PIT/secondsSupport
   /**
    *  Configure the PIT channel in seconds
    *
    *  @param[in]  pitChannelNum     Channel to configure
    *  @param[in]  intervalInSeconds Interval in seconds
    *  @param[in]  pitChannelAction  Action to take on timer event
    *
    *  @note The timer channel is disabled before configuring so that period changes have
    *        immediate effect.
    */
   static void configureChannel(
         PitChannelNum     pitChannelNum,
         Seconds           intervalInSeconds,
         PitChannelAction  pitChannelAction=PitChannelAction_None) {

      configureChannel(pitChannelNum, Info::convertSecondsToTicks(intervalInSeconds), pitChannelAction);
   }
#endif // /PIT/secondsSupport

   /**
    *  Configure the PIT channel in milliseconds
    *
    *  @param[in]  pitChannelNum    Channel to configure
    *  @param[in]  milliseconds     Interval in seconds
    *  @param[in]  pitChannelAction Action to take on timer event
    *
    *  @note The timer channel is disabled before configuring so that period changes have
    *        immediate effect.
    */
   static void configureChannelInMilliseconds(
         PitChannelNum     pitChannelNum,
         unsigned          milliseconds,
         PitChannelAction  pitChannelAction=PitChannelAction_None) {

      configureChannel(pitChannelNum, convertMillisecondsToTicks(milliseconds), pitChannelAction);
   }

   /**
    *  Configure the PIT channel in microseconds
    *
    *  @param[in]  pitChannelNum    Channel to configure
    *  @param[in]  microseconds     Interval in microseconds
    *  @param[in]  pitChannelAction Action to take on timer event
    *
    *  @note The timer channel is disabled before configuring so that period changes have
    *        immediate effect.
    */
   static void configureChannelInMicroseconds(
         PitChannelNum     pitChannelNum,
         unsigned          microseconds,
         PitChannelAction     pitChannelAction=PitChannelAction_None) {

      configureChannel(pitChannelNum, convertMicrosecondsToTicks(microseconds), pitChannelAction);
   }

   /**
    * Convert time in ticks to time in microseconds
    *
    * @param[in] ticks Time interval in ticks
    *
    * @return Time interval in microseconds
    */
   static unsigned convertTicksToMilliseconds(Ticks ticks) {
      return (unsigned)((1000UL * (unsigned)ticks)/Info::getClockFrequency());
   }

   /**
    * Convert time in ticks to time in milliseconds
    *
    * @param[in] ticks Time interval in ticks
    *
    * @return Time interval in milliseconds
    */
   static unsigned convertTicksToMicroseconds(Ticks ticks) {
      return (unsigned)((1000000UL * (unsigned)ticks)/Info::getClockFrequency());
   }

   /**
    * Converts time in milliseconds to time in ticks
    *
    * @param[in] milliseconds Time interval in milliseconds
    *
    * @return Time interval in ticks
    *
    * @note Will set error code if calculated value is unsuitable
    */
   static Ticks convertMillisecondsToTicks(unsigned milliseconds) {
      unsigned long intervalInTicks = milliseconds*(Info::getClockFrequency()/1000);
      usbdm_assert(intervalInTicks <= 0xFFFFFFFFUL, "Interval is too long");
      usbdm_assert(intervalInTicks > 0, "Interval is too short");
      if (intervalInTicks > 0xFFFFFFFFUL) {
         setErrorCode(E_TOO_LARGE);
      }
      if (intervalInTicks <= 0) {
         setErrorCode(E_TOO_SMALL);
      }
      return Ticks((unsigned)intervalInTicks);
   }

   /**
    * Converts time in microseconds to time in ticks
    *
    * @param[in] microseconds Time interval in microseconds
    *
    * @return Time interval in ticks
    *
    * @note Will set error code if calculated value is unsuitable
    */
   static Ticks convertMicrosecondsToTicks(unsigned microseconds) {
      unsigned long intervalInTicks = microseconds*(Info::getClockFrequency()/1000000);
      usbdm_assert(intervalInTicks <= 0xFFFFFFFFUL, "Interval is too long");
      usbdm_assert(intervalInTicks > 0, "Interval is too short");
      if (intervalInTicks > 0xFFFFFFFFUL) {
         setErrorCode(E_TOO_LARGE);
      }
      if (intervalInTicks <= 0) {
         setErrorCode(E_TOO_SMALL);
      }
      return Ticks((unsigned)intervalInTicks);
   }

   /**
    * Set period in Ticks
    *
    * @param[in]  pitChannelNum Channel being modified
    * @param[in]  ticks         Interval in ticks
    *
    * @note If the timer is currently enabled this value will be loaded on the next expiration.
    *       To have immediate effect it is necessary to use configureChannel().
    */
   static void setPeriod(PitChannelNum pitChannelNum, Ticks ticks) {
      pit->CHANNEL[pitChannelNum].LDVAL = (unsigned)ticks-1;
   }

   /**
    * Set period in microseconds
    *
    * @param[in]  pitChannelNum Channel being modified
    * @param[in]  microseconds  Interval in microseconds
    *
    * @note If the timer is currently enabled this value will be loaded on the next expiration.
    *       To have immediate effect it is necessary to use configureChannel().
    */
   static void setPeriodInMicroseconds(PitChannelNum pitChannelNum, uint32_t microseconds) {
      setPeriod(pitChannelNum, convertMicrosecondsToTicks(microseconds));
   }

   /**
    * Set period in milliseconds
    *
    * @param[in]  pitChannelNum Channel being modified
    * @param[in]  milliseconds  Interval in milliseconds
    *
    * @note If the timer is currently enabled this value will be loaded on the next expiration.
    *       To have immediate effect it is necessary to use configureChannel().
    */
   static void setPeriodInMilliseconds(PitChannelNum pitChannelNum, uint32_t milliseconds) {
      setPeriod(pitChannelNum, convertMillisecondsToTicks(milliseconds));
   }

#if false // /PIT/secondsSupport
   /**
    * Set period in seconds
    *
    * @param[in]  pitChannelNum Channel being modified
    * @param[in]  interval Interval in seconds
    *
    * @note If the timer is currently enabled this value will be loaded on the next expiration.
    *       To have immediate effect it is necessary to use configureChannel().
    */
   static void setPeriod(PitChannelNum pitChannelNum, Seconds interval) {
      setPeriod(pitChannelNum, Ticks((float)interval*Info::getClockFrequency()));
   }
#endif // /PIT/secondsSupport

   /**
    *  Use a PIT channel to implement a busy-wait delay
    *
    *  @param[in]  pitChannelNum   Channel to use
    *  @param[in]  interval        Interval to wait in timer ticks (usually bus clock period)
    *
    *  @note Function doesn't return until interval has expired
    */
   static void delay(PitChannelNum pitChannelNum, Ticks interval) {
      configureChannel(pitChannelNum, interval);
      while (pit->CHANNEL[pitChannelNum].TFLG == 0) {
         __NOP();
      }
      disableChannel(pitChannelNum);
   }

#if false // /PIT/secondsSupport
   /**
    *  Use a PIT channel to implement a busy-wait delay
    *
    *  @param[in]  pitChannelNum   Channel to use
    *  @param[in]  interval        Interval to wait in seconds
    *
    *  @note Function doesn't return until interval has expired
    */
   static void delay(PitChannelNum pitChannelNum, Seconds interval) {
      configureChannel(pitChannelNum, interval);
      while (pit->CHANNEL[pitChannelNum].TFLG == 0) {
         __NOP();
      }
      disableChannel(pitChannelNum);
   }
#endif // /PIT/secondsSupport

#if false // /PIT/irqHandlingMethod

   using CallbackFunction = typename Info::CallbackFunction;
   
   /**
    * Set interrupt callback function.
    *
    *  @param pitChannelNum   Channel to configure
    *  @param callback        Callback function to execute on interrupt
    *                         Use nullptr to remove callback.
    */
   static void setCallback(PitChannelNum pitChannelNum, CallbackFunction callback) {
      Info::setCallback(PitIrqNum(pitChannelNum), callback);
   }

   /**
    * Set interrupt callback function.
    *
    * @param pitIrqNum Used to identify peripheral interrupt
    * @param callback  Callback function to execute on interrupt
    *                  Use nullptr to remove callback.
    */
   static void setCallback(PitIrqNum pitIrqNum, CallbackFunction callback) {
      Info::setCallback(pitIrqNum, callback);
   }

   /**
    * Set one-shot timer callback in microseconds
    *
    *  @note It is necessary to enable NVIC interrupts beforehand
    *
    *  @param[in]  pitChannelNum     Channel to configure
    *  @param[in]  callback          Callback function to be executed on timeout
    *  @param[in]  microseconds      Interval in milliseconds
    */
   static void oneShotInMicroseconds(PitChannelNum pitChannelNum, CallbackFunction callback, uint32_t microseconds) {
      Info::setCallback(PitIrqNum(pitChannelNum), callback);
      configureChannelInMicroseconds(pitChannelNum, microseconds, PitChannelAction_Interrupt);
      CriticalSection cs;
      Info::clearOnEvent |= (1<<pitChannelNum);
   }

   /**
    * Set one-shot timer callback in milliseconds
    *
    *  @note It is necessary to enable NVIC interrupts beforehand
    *
    *  @param[in]  pitChannelNum     Channel to configure
    *  @param[in]  callback          Callback function to be executed on timeout
    *  @param[in]  milliseconds      Interval in milliseconds
    */
   static void oneShotInMilliseconds(PitChannelNum pitChannelNum, CallbackFunction callback, uint32_t milliseconds) {
      Info::setCallback(PitIrqNum(pitChannelNum), callback);
      configureChannelInMilliseconds(pitChannelNum, milliseconds, PitChannelAction_Interrupt);
      CriticalSection cs;
      Info::clearOnEvent |= (1<<pitChannelNum);
   }

   /**
    * Set one-shot timer callback in Ticks
    *
    *  @note It is necessary to enable NVIC interrupts beforehand
    *
    *  @param[in]  pitChannelNum     Channel to configure
    *  @param[in]  callback          Callback function to be executed on timeout
    *  @param[in]  tickInterval      Interval in timer ticks (usually bus clock period)
    */
   static void oneShot(PitChannelNum pitChannelNum, CallbackFunction callback, Ticks tickInterval) {
      Info::setCallback(PitIrqNum(pitChannelNum), callback);
      CriticalSection cs;
      configureChannel(pitChannelNum, tickInterval, PitChannelAction_Interrupt);
      Info::clearOnEvent |= (1<<pitChannelNum);
   }
   
#if false // /PIT/secondsSupport
   /**
    * Set one-shot timer callback.
    *
    *  @note It is necessary to enable NVIC interrupts beforehand
    *
    *  @param[in]  pitChannelNum     Channel to configure
    *  @param[in]  callback          Callback function to be executed on timeout
    *  @param[in]  interval          Interval in seconds until callback is executed
    */
   static void oneShot(PitChannelNum pitChannelNum, CallbackFunction callback, Seconds interval) {
      oneShot(pitChannelNum, callback, Info::convertSecondsToTicks(interval));
   }
#endif // /PIT/secondsSupport
#endif // /PIT/irqHandlingMethod

   /**
    * Class representing a PIT channel.
    * This version may be instantiated and passed as a reference
    */
   class PitChannel {

   public:

      // PIT Owning this channel
      using Owner = PitBase_T<Info>;
      using ChannelInit = typename Info::ChannelInit;

      constexpr PitChannel(PitChannelNum channel) : chan(channel) {}

      /** Timer channel number */
      const PitChannelNum chan;

      /**
       * Configure PIT channel from values specified in init.
       * The PIT shared hardware will be default initialised if necessary
       *
       * @param init Class containing initialisation values (channel number is ignored)
       */
       void configure(const ChannelInit &init) const {
         PitBase_T<Info>::configure(chan, init);
      }
   
      /**
       *  Enables and configures the PIT if not already done.
       *  This also disables all channel interrupts and channel reservations if newly configured.
       */
       void defaultConfigureIfNeeded() const {
         PitBase_T<Info>::defaultConfigureIfNeeded();
      }
   
      /**
       *  Configure the PIT channel
       *
       *  @param[in]  interval          Interval in timer ticks (usually bus clock)
       *  @param[in]  pitChannelAction     Whether to enable interrupts
       *
       *  @note The timer channel is disabled before configuring so that period changes
       *        have immediate effect.
       */
       void configure(
            Ticks             interval,
            PitChannelAction  pitChannelAction=PitChannelAction_None) const {
   
         PitBase_T<Info>::configureChannel(chan, interval, pitChannelAction);
      }
   
      /**
       * Clear channel interrupt flag
       */
       void clearInterruptFlag() const {
         PitBase_T<Info>::clearInterruptFlag(chan);
      }
   
#if false // /PIT/secondsSupport
      /**
       *  Configure the PIT channel
       *
       *  @param[in]  interval          Interval in seconds
       *  @param[in]  pitChannelAction     Whether to enable interrupts
       *
       *  @note The timer channel is disabled before configuring so that period changes
       *        have immediate effect.
       */
       void configure(
            Seconds           interval,
            PitChannelAction  pitChannelAction=PitChannelAction_None) const {
   
         PitBase_T<Info>::configureChannel(chan, interval, pitChannelAction);
      }
#endif
   
      /**
       *   Enable the PIT channel
       */
       void enable() const {
         PitBase_T<Info>::enableChannel(chan);
      }
   
      /**
       *   Disable the PIT channel
       */
       void disable() const {
         PitBase_T<Info>::disableChannel(chan);
      }
   
      /**
       * Set period in ticks
       *
       * @param[in]  interval Interval in ticks
       *
       * @note If the timer is currently enabled this value will be loaded on the next expiration.
       *       To have immediate effect it is necessary to use configure().
       */
       void setPeriod(Ticks interval) const {
         PitBase_T<Info>::setPeriod(chan, interval);
      }
   
      /**
       * Read channel counter
       *
       * @return Current down-counter value
       */
       uint32_t readCounter() const {
         return pit->CHANNEL[chan].CVAL;
      }
   
#if false // /PIT/secondsSupport
      /**
       * Set period in seconds
       *
       * @param[in]  interval Interval in seconds
       *
       * @note If the timer is currently enabled this value will be loaded on the next expiration.
       *       To have immediate effect it is necessary to use configure().
       */
       void setPeriod(Seconds interval) const {
         PitBase_T<Info>::setPeriod(chan, interval);
      }
#endif
      /**
       * Set period in microseconds
       *
       * @param[in]  microseconds Interval in microseconds
       *
       * @note If the timer is currently enabled this value will be loaded on the next expiration.
       *       To have immediate effect it is necessary to use configure().
       */
       void setPeriodInMicroseconds(uint32_t microseconds) const {
         uint64_t interval = ((uint64_t)microseconds*Info::getClockFrequency())/1000000;
         usbdm_assert(interval<0xFFFFFFFFUL,"Interval too long");
         PitBase_T<Info>::setPeriod(chan, Ticks(uint32_t(interval)));
      }
   
      /**
       *  Use a PIT channel to implement a busy-wait delay
       *
       *  @param[in]  interval  Interval to wait in timer ticks (usually bus clock period)
       *
       *  @note Function does not return until interval has expired
       */
       void delay(Ticks interval) const {
         PitBase_T<Info>::delay(chan, interval);
      }
   
#if false // /PIT/secondsSupport
      /**
       *  Use a PIT channel to implement a busy-wait delay
       *
       *  @param[in]  interval  Interval to wait in seconds
       *
       *  @note Function does not return until interval has expired
       */
       void delay(Seconds interval) const {
         PitBase_T<Info>::delay(chan, interval);
      }
#endif
   
      /**
       * Enable/disable channel interrupts
       *
       * @param[in]  enable  True => enable, False => disable
       */
       void setChannelAction(PitChannelAction pitChannelAction) const {
         PitBase_T<Info>::setChannelAction(chan, pitChannelAction);
      }
   
      /**
       * Enable interrupts in NVIC
       */
       void enableNvicInterrupts() const {
         Info::enableNvicInterrupts(PitIrqNum(chan));
      }
   
      /**
       * Enable and set priority of interrupts in NVIC
       * Any pending NVIC interrupts are first cleared.
       *
       * @param[in]  nvicPriority   Interrupt priority
       */
       void enableNvicInterrupts(NvicPriority nvicPriority) const {
         Info::enableNvicInterrupts(PitIrqNum(chan), nvicPriority);
      }
   
      /**
       * Disable interrupts in NVIC
       */
       void disableNvicInterrupts() const {
         Info::disableNvicInterrupts(PitIrqNum(chan));
      }
   

   };

   /**
    * Class representing a PIT channel
    * This version is a template and may not be instantiated.
    * It is a type only with static member methodes.
    *
    * @tparam channel Timer channel number
    */
   template <int channel>
   class Channel : public PitChannel, private Info {

   private:
      Channel(const Channel&) = delete;
      Channel(Channel&&) = delete;

   public:
      // Allow access to channel initialisation type
      using typename Info::ChannelInit;

      constexpr Channel() : PitChannel(CHANNEL) {};

      /** Timer channel number */
      static constexpr PitChannelNum CHANNEL = (PitChannelNum)channel;
      
      /**
       * Configure PIT channel from values specified in init.
       * The PIT shared hardware will be default initialised if necessary
       *
       * @param init Class containing initialisation values (channel number is ignored)
       */
      static void configure(const ChannelInit &init)  {
         PitBase_T<Info>::configure(CHANNEL, init);
      }
   
      /**
       *  Enables and configures the PIT if not already done.
       *  This also disables all channel interrupts and channel reservations if newly configured.
       */
      static void defaultConfigureIfNeeded()  {
         PitBase_T<Info>::defaultConfigureIfNeeded();
      }
   
      /**
       *  Configure the PIT channel
       *
       *  @param[in]  interval          Interval in timer ticks (usually bus clock)
       *  @param[in]  pitChannelAction     Whether to enable interrupts
       *
       *  @note The timer channel is disabled before configuring so that period changes
       *        have immediate effect.
       */
      static void configure(
            Ticks             interval,
            PitChannelAction  pitChannelAction=PitChannelAction_None)  {
   
         PitBase_T<Info>::configureChannel(CHANNEL, interval, pitChannelAction);
      }
   
      /**
       * Clear channel interrupt flag
       */
      static void clearInterruptFlag()  {
         PitBase_T<Info>::clearInterruptFlag(CHANNEL);
      }
   
#if false // /PIT/secondsSupport
      /**
       *  Configure the PIT channel
       *
       *  @param[in]  interval          Interval in seconds
       *  @param[in]  pitChannelAction     Whether to enable interrupts
       *
       *  @note The timer channel is disabled before configuring so that period changes
       *        have immediate effect.
       */
      static void configure(
            Seconds           interval,
            PitChannelAction  pitChannelAction=PitChannelAction_None)  {
   
         PitBase_T<Info>::configureChannel(CHANNEL, interval, pitChannelAction);
      }
#endif
   
      /**
       *   Enable the PIT channel
       */
      static void enable()  {
         PitBase_T<Info>::enableChannel(CHANNEL);
      }
   
      /**
       *   Disable the PIT channel
       */
      static void disable()  {
         PitBase_T<Info>::disableChannel(CHANNEL);
      }
   
      /**
       * Set period in ticks
       *
       * @param[in]  interval Interval in ticks
       *
       * @note If the timer is currently enabled this value will be loaded on the next expiration.
       *       To have immediate effect it is necessary to use configure().
       */
      static void setPeriod(Ticks interval)  {
         PitBase_T<Info>::setPeriod(CHANNEL, interval);
      }
   
      /**
       * Read channel counter
       *
       * @return Current down-counter value
       */
      static uint32_t readCounter()  {
         return pit->CHANNEL[CHANNEL].CVAL;
      }
   
#if false // /PIT/secondsSupport
      /**
       * Set period in seconds
       *
       * @param[in]  interval Interval in seconds
       *
       * @note If the timer is currently enabled this value will be loaded on the next expiration.
       *       To have immediate effect it is necessary to use configure().
       */
      static void setPeriod(Seconds interval)  {
         PitBase_T<Info>::setPeriod(CHANNEL, interval);
      }
#endif
      /**
       * Set period in microseconds
       *
       * @param[in]  microseconds Interval in microseconds
       *
       * @note If the timer is currently enabled this value will be loaded on the next expiration.
       *       To have immediate effect it is necessary to use configure().
       */
      static void setPeriodInMicroseconds(uint32_t microseconds)  {
         uint64_t interval = ((uint64_t)microseconds*Info::getClockFrequency())/1000000;
         usbdm_assert(interval<0xFFFFFFFFUL,"Interval too long");
         PitBase_T<Info>::setPeriod(CHANNEL, Ticks(uint32_t(interval)));
      }
   
      /**
       *  Use a PIT channel to implement a busy-wait delay
       *
       *  @param[in]  interval  Interval to wait in timer ticks (usually bus clock period)
       *
       *  @note Function does not return until interval has expired
       */
      static void delay(Ticks interval)  {
         PitBase_T<Info>::delay(CHANNEL, interval);
      }
   
#if false // /PIT/secondsSupport
      /**
       *  Use a PIT channel to implement a busy-wait delay
       *
       *  @param[in]  interval  Interval to wait in seconds
       *
       *  @note Function does not return until interval has expired
       */
      static void delay(Seconds interval)  {
         PitBase_T<Info>::delay(CHANNEL, interval);
      }
#endif
   
      /**
       * Enable/disable channel interrupts
       *
       * @param[in]  enable  True => enable, False => disable
       */
      static void setChannelAction(PitChannelAction pitChannelAction)  {
         PitBase_T<Info>::setChannelAction(CHANNEL, pitChannelAction);
      }
   
      /**
       * Enable interrupts in NVIC
       */
      static void enableNvicInterrupts()  {
         Info::enableNvicInterrupts(PitIrqNum(CHANNEL));
      }
   
      /**
       * Enable and set priority of interrupts in NVIC
       * Any pending NVIC interrupts are first cleared.
       *
       * @param[in]  nvicPriority   Interrupt priority
       */
      static void enableNvicInterrupts(NvicPriority nvicPriority)  {
         Info::enableNvicInterrupts(PitIrqNum(CHANNEL), nvicPriority);
      }
   
      /**
       * Disable interrupts in NVIC
       */
      static void disableNvicInterrupts()  {
         Info::disableNvicInterrupts(PitIrqNum(CHANNEL));
      }
   

   };
   
// /PIT/InitMethod Not found
}; // class PitBase_T

/**
* Class representing PIT
*/
class Pit : public PitBase_T<PitInfo> {};

   /**
    * @brief class representing a PIT channel
    */
   using PitChannel = Pit::PitChannel;
   

/**
 * @}
 */

} // End namespace USBDM

#endif // /PIT/enablePeripheralSupport

#endif /* INCLUDE_USBDM_PIT_H_ */
