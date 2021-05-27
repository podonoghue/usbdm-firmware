/**
 * @file     sht10-example.cpp
 * @brief    Interface for SHT10 Temperature and Humidity sensor
 *
 * @version  V4.12.1.210S
 * @date     15 November 2017
 */

#include <sht10.h>
#include "hardware.h"

using namespace USBDM;

// Create interface
static SHT10<GpioB<2>, GpioB<3>> sht10{};

int main() {

   // Check status register functions
   uint8_t status = sht10.readStatus();
   console.setWidth(8).setPadding(Padding_LeadingZeroes);
   console.write("Status = 0b").writeln(status,Radix_2);

   sht10.enableLowResolution(true);
   sht10.enableReloadOtp(false);
   sht10.enableHeater(true);

   status = sht10.readStatus();
   console.write("Status = 0b").writeln(status,Radix_2);

   sht10.enableLowResolution(false);
   sht10.enableReloadOtp(true);
   sht10.enableHeater(false);

   status = sht10.readStatus();
   console.write("Status = 0b").writeln(status,Radix_2);
   console.reset();

   console.write("Battery status = ").writeln(sht10.readBatteryLevel()?"OK":"Low");

   // Start over
   sht10.reset();

   int errCount = 0;

   for(int count=0; ; count++) {
      if (count%20 == 0) {
         // Toggle resolution every 20
         static bool enable = false;
         sht10.enableLowResolution(enable);
         console.writeln(enable?"Low resolution":"High resolution");
         enable = !enable;
      }
      if (count%100 == 0) {
         static int  heaterCycles = 0;
         static bool enable       = true;
         if (heaterCycles++<10) {
            // Limit cycles to reduce wear
            // Toggle heater
            enable = !enable;
            sht10.enableHeater(enable);
            console.writeln(enable?"Heater On":"Heater Off");
         }
         else if (enable) {
            console.writeln("Heater Off");
            sht10.enableHeater(false);
         }
      }
      sht10.startTemperatureMeasurement();
      static float temperature;
      static auto waitForT =[]() {
         return sht10.getTemperature(temperature);
      };
      if (!waitMS(400, waitForT)) {
         console.writeln("Failed measurement");
         sht10.resetInterface();
         errCount++;
         continue;
      }

      sht10.startHumidityMeasurement();
      static float humidity;
      static auto waitForH =[]() {
         return sht10.getHumidity(humidity);
      };
      if (!waitMS(400, waitForH)) {
         console.writeln("Failed measurement");
         sht10.resetInterface();
         errCount++;
         continue;
      }
      console.setWidth(2).setPadding(Padding_LeadingSpaces).write(errCount).reset();
      console.setWidth(6).setPadding(Padding_LeadingSpaces).write(count).reset();
      console.write(": Temperature ").write(temperature);
      console.write(", Humidity    ").writeln(humidity);
      console.reset();

      waitMS(100);
   }
}
