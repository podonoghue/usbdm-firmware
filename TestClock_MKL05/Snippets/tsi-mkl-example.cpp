/*
 ============================================================================
 * @file    tsi-mkl-example.cpp (180.ARM_Peripherals)
 * @brief   Basic C++ demo using TSI class
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "delay.h"
#include "tsi.h"

using namespace USBDM;

// LED connection - change as required
using Led1  = USBDM::GpioB<18, ActiveLow>;
using Led2  = USBDM::GpioB<19, ActiveLow>;

// Button connections
// Threshold value have to be determined experimentally for each button
using Button1 = Tsi0Button< 9, 7000>;
using Button2 = Tsi0Button<10, 7000>;

/**
 * This routine calculates usable thresholds for the two buttons
 * based upon the measured extreme values.
 *
 * It assumes the other TSI settings are reasonable for use.
 */
void calculateThresholds() {
   uint16_t b1Min, b1Max, b1Threshold, b2Min, b2Max, b2Threshold;

   // Configure for capacitive measurement mode
   Tsi0::configure(
         TsiLowPower_Enabled,
         TsiScanNumber_24,
         TsiElectrodePrescaler_8,
         TsiReferenceCharge_8uA,
         TsiExternalCharge_8uA,
         TsiDeltaVoltage_High);

   // Wait for fist successful conversion
   Button1::startScanAndWait();
   b1Min = b1Max = Button1::getCount();
   Button2::startScanAndWait();
   b2Min = b2Max = Button2::getCount();

   for(int count = 0;;count++) {
      waitMS(50);
      uint16_t b1,b2;
      Button1::startScanAndWait();
      b1 = Button1::getCount();
      if (b1<b1Min) {
         b1Min = b1;
      }
      if (b1>b1Max) {
         b1Max = b1;
      }
      b1Threshold = (b1Min+b1Max)/2;
      Led1::write((b1>b1Threshold));
      Button2::startScanAndWait();
      b2 = Button2::getCount();
      if (b2<b2Min) {
         b2Min = b2;
      }
      if (b2>b2Max) {
         b2Max = b2;
      }
      b2Threshold = (b2Min+b2Max)/2;
      Led2::write((b2>b2Threshold));
      console.setPadding(Padding_LeadingSpaces).setWidth(8).write(count).write(": ");
      console.write("Button1(").write(b1Threshold).write(", ").write(b1>b1Threshold).write("), ");
      console.write("Button2(").write(b2Threshold).write(", ").write(b2>b2Threshold).write(") ").writeln();
   }
}

/**
 * Poll the touch are as two buttons
 * LEDs reflect the button values
 */
void pollButtons() {
   Led1::setOutput();
   Led2::setOutput();
   
   // Configure for capacitive measurement mode
   Tsi0::configure(
         TsiLowPower_Enabled,
         TsiScanNumber_24,
         TsiElectrodePrescaler_8,
         TsiReferenceCharge_8uA,
         TsiExternalCharge_8uA,
         TsiDeltaVoltage_High);

   Button1::setInput();
   Button2::setInput();

   console.setPadding(Padding_LeadingSpaces).setWidth(8);
   for(;;) {
      console.write("Button1(").write(Button1::poll()).write(", ").write(Button1::getCount()).write(") ");
      console.write("Button2(").write(Button2::poll()).write(", ").write(Button2::getCount()).write(") ").writeln();
      Led1::write(Button1::poll());
      Led2::write(Button2::poll());
   }
}

int main() {
   console.write("SystemBusClock   = ").writeln(USBDM::SystemBusClock);
   console.write("SystemCoreClock  = ").writeln(USBDM::SystemCoreClock);

   Led1::setOutput();
   Led2::setOutput();

//   calculateThresholds();
   pollButtons();
}
