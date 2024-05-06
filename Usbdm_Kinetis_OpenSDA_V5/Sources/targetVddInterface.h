/*
 * TargetVddInterface.h
 *
 *  Created on: 21Dec.,2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_TARGETVDDINTERFACE_H_
#define PROJECT_HEADERS_TARGETVDDINTERFACE_H_

#if HW_CAPABILITY&CAP_VDDCONTROL

#include <math.h>
#include "hardware.h"
#include "cmp.h"
#include "console.h"
#include "commands.h"

/**
 * State of VDD control interface
 */
enum VddState {
   VddState_None,         //!< Vdd Off
   VddState_Internal,     //!< Vdd Internal
   VddState_External,     //!< Vdd External
   VddState_Overloaded,   //!< Internal Vdd overloaded & off)
};

/**
 * Low-level interface to Target Vdd (Vbdm) control and sensing
 */
class TargetVddInterface {

private:
   /**
    * Represents the 2:1 resistor voltage divider on input
    */
   static constexpr int externalDivider = 2;

   /**
    * Vref of Comparator and ADC
    */
   static constexpr float Vref = 3.3f;

   /**
    * GPIO for Target Vdd enable pin
    */
   using Control = USBDM::GpioD<6>;

   /**
    * GPIO used to monitor power switch error indicator
    */
   using VddPowerFaultMonitor = USBDM::GpioD<7, USBDM::ActiveLow>;

   /**
    * Target Vdd state
    */
   static inline VddState vddState = VddState_None;

   /**
    * Dummy routine used if callback is not set
    */
   static void nullCallback(VddState) {
#ifdef DEBUG_BUILD
      __BKPT();
#endif
   }

   /**
    * Callback for Vdd changes
    */
   static inline void (*fCallback)(VddState) = nullCallback;

public:

   /**
    * Monitors Target Vdd (Vbdm) power switch overload
    * GPIO falling edge IRQ
    *
    * @param status Bit mask for entire port
    */
   static void powerFaultCallback() {

      if (VddPowerFaultMonitor::getAndClearInterruptState()) {

         // In case Vdd overload
         Control::off();

         // Fault (overload) detected
         vddState = VddState_Overloaded;

         // Notify callback
         fCallback(vddState);
      }
   }

   /**
    * Set callback to execute on Target Vdd (Vbdm) changes
    *
    * @param[in] callback Callback for target Vdd state changes (may be null)
    */
   static void setCallback(void (*callback)(VddState)) {
      if (callback == nullptr) {
         callback = nullCallback;
      }
      fCallback = callback;
   }

   /**
    * Initialise Vdd control and measurement interface
    *
    * @param[in] callback Callback for target Vdd state changes (may be null)
    */
   static void initialise(void (*callback)(VddState)) {
      using namespace USBDM;

      Control::setOutput();

      setCallback(callback);

      VddPowerFaultMonitor::setPinCallback(powerFaultCallback);
      VddPowerFaultMonitor::setInput(
            PinPull_Up,
            PinAction_IrqFalling,
            PinFilter_Passive);
      VddPowerFaultMonitor::enableNvicPinInterrupts(NvicPriority_High);

      vddState = VddState_None;
   }

   /**
    * Turn on Target Vdd
    *
    * @note This has no effect if in error state
    */
   static void vddOn() {
      if (vddState == VddState_Overloaded) {
         return;
      }
      vddState  = VddState_Internal;
      Control::on();
   }

   /**
    * Turn on Target Vdd @ 3.3V\n
    * Dummy routine as voltage level controlled by physical link
    *
    * @note This has no effect if in error state
    */
   static void vdd3V3On() {
      vddOn();
   }

   /**
    * Turn on Target Vdd @ 5V\n
    * Dummy routine as voltage level controlled by physical link
    *
    * @note This has no effect if in error state
    */
   static void vdd5VOn() {
      vddOn();
   }

   /**
    * Turn off Target Vdd
    *
    * @note - Will reset error state
    */
   static void vddOff() {
      Control::off();
      vddState  = VddState_None;
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    *
    * @return true  => Target Vdd =  ~3V3
    * @return false => Target Vdd != ~3V3
    */
   static bool isVddOK_3V3() {
      return (vddState != VddState_Overloaded);
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    *
    * @return true  => Target Vdd =  ~5V
    * @return false => Target Vdd != ~5V
    */
   static bool isVddOK_5V() {
      return (vddState != VddState_Overloaded);
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    *
    * @return true  => Target Vdd >= ~1.5V
    * @return false => Target Vdd <  ~1.5V
    */
   static bool isVddPresent() {
      if (isVddOK_3V3()) {
         return false;
      }
      else {
         return true;
      }
   }

   /**
    * Check if target Vdd has fallen to POR level\n
    * Also updates Target Vdd LED
    */
   static bool isVddLow() {
      return true;
   }

   /**
    * Get Vdd state
    *
    * @return Vdd state as VddState_None, VddState_Internal, VddState_External or VddState_Overloaded
    */
   static VddState getState() {
      return vddState;
   }

   /**
    * Update Vdd state
    *
    * @return Vdd state as VddState_None, VddState_Internal, VddState_External or VddState_Overloaded
    */
   static VddState checkVddState() {
      switch(vddState) {
         case VddState_Overloaded    :
            // No change - requires Vdd to be turned off to clear
            break;

         case VddState_Internal :
            if (!isVddOK_3V3() && !isVddOK_5V()) {
               // In case Vdd overload
               Control::off();
               // Power should be present in expected range!
               vddState = VddState_Overloaded;
            }
            break;

         case VddState_External :
         case VddState_None     :
            if (isVddOK_3V3()) {
               vddState = VddState_External;
            }
            else {
               vddState = VddState_None;
            }
            break;
      }
      return vddState;
   }
};
#endif // HW_CAPABILITY&CAP_VDDCONTROL

#endif /* PROJECT_HEADERS_TARGETVDDINTERFACE_H_ */
