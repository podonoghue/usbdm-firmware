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
   static constexpr float SERVO_PERIOD = 20.0 * ms;
   static constexpr float SERVO_MIN    =  1.0 * ms;
   static constexpr float SERVO_MAX    =  2.0 * ms;

public:
   /**
    * Enable servo
    * Position is centred
    */
   static void enable() {
      Timer::configure(FtmMode_LeftAlign, FtmClockSource_System);
      Timer::setPeriod(SERVO_PERIOD, true);
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
//using servo = Servo<ftm_D2>;
using servo = Servo<Ftm0Channel<7>>;

int main() {
   console.writeln("Starting\n");

   servo::enable();

   // Check if configuration failed
   USBDM::checkError();

   for (;;) {
      for(unsigned i=0; i<=180; i++) {
         servo::setPosition(i);
         waitMS(50);
         console.write("Position = ").writeln(i);
      }
   }
   return 0;
}
