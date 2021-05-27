/**
 ============================================================================
 * @file   rtc-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief  Real Time CLock (RTC) Example
 * @date   12 Jul 2018
 * Author podonoghue
 ============================================================================
 */
 /*
 * This example uses RTC interrupts.
 * Uses an LED for debug timing check.
 *
 * Note - Requires RTC interrupt handlers to be installed.
 ============================================================================
 */
#include <ctime>
#include "hardware.h"
#include "rtc.h"
#include "smc.h"

using namespace USBDM;

/**
 * Real Time Clock Example
 */
// LED connection - change as required
using Led = GpioA<2,ActiveLow>;

void reportTime(const char *msg, time_t rawtime) {
   char buffer[80];
   struct tm * timeinfo;
   timeinfo = localtime(&rawtime);
   strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S : ", timeinfo);
   console.write(buffer).writeln(msg).flushOutput();
}

/**
 * Callback alarm handler from RTC Alarm
 */
void alarmHandler(uint32_t timeSinceEpoch) {
   // Set repeat callback for 5 seconds from now
   Rtc::setAlarmTime(timeSinceEpoch+4);
   reportTime("Alarm !!!!!!", timeSinceEpoch);
}

/**
 * Callback seconds handler from RTC Alarm
 */
void secondsHandler(uint32_t timeSinceEpoch) {
   Led::toggle();
   reportTime("Tick", timeSinceEpoch);
}

int main() {
   console.writeln("Starting").flushOutput();

   // Enable RTC - done by startup code
//   Rtc::initialise();

   // Set callbacks
   Rtc::setSecondsCallback(secondsHandler);
   Rtc::enableSecondsInterrupts();

   Rtc::setAlarmCallback(alarmHandler);
   Rtc::setAlarmTime(Rtc::getTime()+5);
   Rtc::enableAlarmInterrupts();
   Rtc::enableNvicInterrupts(NvicPriority_Normal);

   Led::setOutput();
   for(;;) {
      // Sleep between events
      Smc::enterWaitMode();
//      Smc::enterStopMode(SmcStopMode_NormalStop);

      time_t rawtime;
      time (&rawtime);
      reportTime("Woke", rawtime);
   }
   return 0;
}
