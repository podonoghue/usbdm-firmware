/**
 ============================================================================
 * @file    ftm-quadrature-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Demo using Ftm class to implement a Quadrature decoder
 *
 *  Created on: 28/5/2017
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"

/*
 * Note - Not all FTMs support quadrature decoder functions
 */
using namespace USBDM;

// Use FTM1 as the quadrature decoder
// Not all FTMs support this mode
using QuadDecoder = QuadDecoder1;

/**
 * Callback executed on timer overflow/underflow
 */
void callBack() {
   if (QuadDecoder::getOverflowDirection()) {
      // Indicates overflow while increasing
      console.writeln("Increasing");
   }
   else {
      // Indicates overflow while decreasing
      console.writeln("Decreasing");
   }
}

int main() {
   // Configure decoder
   QuadDecoder::configure(
         FtmPrescale_1,
         QuadratureMode_Phase_AB_Mode);

   // Connect QuadDecoder to pins and configure input characteristics
   QuadDecoder::setInput(PinPull_Up);

   // Change polarity if needed
//   QuadDecoder::setPolarity(ActiveLow);

   // Set pin filters
   QuadDecoder::enableFilter(15);

   // Reset position to zero
   // Movement will be +/- relative to this initial position
   QuadDecoder::resetPosition();

   // Set up callback for quadrature overflow or underflow
   QuadDecoder::setTimerOverflowCallback(callBack);
   QuadDecoder::enableTimerOverflowInterrupts();
   QuadDecoder::enableNvicInterrupts(NvicPriority_Normal);

   // Check if configuration failed
   USBDM::checkError();

   int16_t lastPosition = 0;
   for (;;) {
      int16_t position = QuadDecoder::getPosition();
      if (position != lastPosition) {
         // Report position
         console.write("Shaft position = ").writeln(position);
         lastPosition = position;
      }
   }

   return 0;
}
