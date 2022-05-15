/**
 ============================================================================
 * @file    ftm-servo-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Demo using Ftm class to implement a servo-motor controller
 *
 *  Created on: 10/6/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "ftm.h"

using namespace USBDM;

/**
 * Class representing a servo motor connected to a FTM channel
 * Note that all FTM channels will be set to same period
 *
 * @tparam FtmChannel Class representing a FTM channel e.g. Ftm0Channel<3>
 */
template <class FtmChannel>
class Servo {

private:
   using Timer = typename FtmChannel::Ftm;

   // Assumes servo is controlled by a [1,2] millisecond pulse repeated every 20 millisecond
   static constexpr Seconds SERVO_PERIOD = 20.0_ms;
   static constexpr Seconds SERVO_MIN    =  1.0_ms;
   static constexpr Seconds SERVO_MAX    =  2.0_ms;

public:
   /**
    * Enable servo
    * Position is centred
    */
   static void enable() {
      Timer::configure(FtmMode_LeftAlign, FtmClockSource_System);
      Timer::setPeriod(SERVO_PERIOD);
      FtmChannel::configure(FtmChMode_PwmHighTruePulses);
      FtmChannel::setOutput(PinDriveStrength_High);
      FtmChannel::setHighTime((SERVO_MIN+SERVO_MAX)/2);
   }

   /**
    * Set position
    *
    * @param position Position as an angle 0-180
    */
   static ErrorCode setPosition(unsigned position) {
      if (position>180) {
         return setErrorCode(E_ILLEGAL_PARAM);
      }
      return FtmChannel::setHighTime(SERVO_MIN+((position/180.0f)*(SERVO_MAX-SERVO_MIN)));
   }
};

// Instantiate servo on pin
// It will be necessary to map the pin to a FTM channel in Configure.usbdmProject
using Servo1 = Servo<Ftm0::Channel<0>>;

int main() {
   console.writeln("Starting\n");

   Servo1::enable();

   // Check if configuration failed
   USBDM::checkError();

   for (;;) {
      for(unsigned i=0; i<=180; i++) {
         Servo1::setPosition(i);
         waitMS(50);
         console.write("Position = ", i);
      }
   }
   return 0;
}
