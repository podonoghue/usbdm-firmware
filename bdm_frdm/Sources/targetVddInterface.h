/*
 * TargetVddInterface.h
 *
 *  Created on: 21Dec.,2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_TARGETVDDINTERFACE_H_
#define PROJECT_HEADERS_TARGETVDDINTERFACE_H_

#include <math.h>
#include "hardware.h"
#include "cmp.h"
#include "console.h"

/**
 * State of VDD control interface
 */
enum VddState {
   VddState_None,       //!< Vdd Off
   VddState_Internal,   //!< Vdd Internal
   VddState_External,   //!< Vdd External
   VddState_Error,      //!< Vdd in Error (overloaded & off)
};

/**
 * Low-level interface to Target Vdd (Vbdm) control and sensing
 */
class TargetVddInterface {

private:
   /**
    * GPIO for Target Vdd enable pin
    */
   using Control = USBDM::GpioD<6, USBDM::ActiveHigh>;

   /**
    * GPIO used to monitor power switch error indicator
    */
   using VddPowerFaultMonitor = USBDM::GpioD<7, USBDM::ActiveHigh>;

   /**
    * Callback for Vdd changes
    */
   static void (*fCallback)(VddState);

   /**
    * Target Vdd state
    */
   static VddState vddState;

   /**
    * Dummy routine used if callback is not set
    */
   static void nullCallback(VddState) {
#ifdef DEBUG_BUILD
      __BKPT();
#endif
   }

public:

   /**
    * Monitors Target Vdd (Vbdm) power switch overload
    * GPIO falling edge IRQ
    *
    * @param status Bit mask for entire port
    */
   static void powerFaultCallback(uint32_t status) {

      if ((VddPowerFaultMonitor::MASK & status) != 0) {

         // In case Vdd overload
         Control::off();

         // Fault (overload) detected
         vddState = VddState_Error;

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
      Control::setOutput();

      setCallback(callback);

      VddPowerFaultMonitor::setCallback(powerFaultCallback);
      VddPowerFaultMonitor::setInput(
            USBDM::PinPull_Up,
            USBDM::PinAction_IrqFalling,
            USBDM::PinFilter_Passive);
      VddPowerFaultMonitor::enableNvicInterrupts();

      vddState = VddState_None;

      if (isVddOK()) {
         vddState = VddState_External;
      }
   }

   /**
    * Turn on Target Vdd
    *
    * @note This has no effect if in error state
    */
   static void vddOn() {
      if (vddState == VddState_Error) {
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
      if (isVddOK()) {
         vddState = VddState_External;
      }
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd as an integer in the range 0-255 => 0-5V
    */
   static int readRawVoltage() {
      static constexpr int NominalVoltage = round(3.3*255/5.0);

      return isVddOK()?NominalVoltage:0;
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd in volts as a float
    */
   static float readVoltage() {
      static constexpr float NominalVoltage = 3.3;

      return isVddOK()?NominalVoltage:0;
   }

   /**
    * Check if target Vdd is present. \n
    * Also updates Target Vdd LED
    *
    * @param voltage Target voltage to check as ADC value (8-resolution)
    *
    * @return true  => Target Vdd >= voltage
    * @return false => Target Vdd < voltage
    */
   static bool isVddOK() {
      return (vddState == VddState_External) || (vddState == VddState_Internal);
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    *
    * @return true  => Target Vdd >= 3V3
    * @return false => Target Vdd < 3V3
    */
   static bool isVddOK_3V3() {
      return isVddOK();
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    *
    * @return true  => Target Vdd >= 5V
    * @return false => Target Vdd < 5V
    */
   static bool isVddOK_5V() {
      return isVddOK();
   }

   /**
    * Check if target Vdd has fallen to POR level\n
    * Also updates Target Vdd LED
    */
   static bool isVddLow() {
      return !isVddOK();
   }

   /**
    * Clear VDD change flag
    */
   static void clearVddChangeFlag() {

   }

//   /**
//    * Get Vdd state
//    *
//    * @return Vdd state as VddState_None, VddState_Internal, VddState_External or VddState_Error
//    */
//   static VddState getState() {
//      return vddState;
//   }

   /**
    * Update Vdd state
    *
    * @return Vdd state as VddState_None, VddState_Internal, VddState_External or VddState_Error
    */
   static VddState checkVddState() {
      switch(vddState) {
         case VddState_Error    :
            // No change - requires Vdd to be turned off to clear
            break;

         case VddState_Internal :
            if (!isVddOK()) {
               // Power should be present!
               vddState = VddState_Error;
            }
            break;

         case VddState_External :
         case VddState_None     :
            if (isVddOK()) {
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

#endif /* PROJECT_HEADERS_TARGETVDDINTERFACE_H_ */
