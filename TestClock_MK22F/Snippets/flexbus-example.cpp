/**
 ================================================================================
 * @file    flexbus-example.cpp (180.ARM_Peripherals/Sources/flexbus-example.cpp)
 * @brief   Basic demo of Flexbus use
 *
 *  Created on: 10/11/2021
 *      Author: podonoghue
 ================================================================================
 */
#include "hardware.h"
#include "flexbus.h"

using namespace USBDM;

int main() {
   /*
    *  Address where external memory is to be mapped to internal memory space
    *  See MCU description for suitable ranges
    *
    *  MK28F
    *       Address Range            Type of memory                                  Access
    * |-------------------------|------------------------------------------------|----------------
    * | 0x1800_0000-0x1BFF_FFFF | FlexBus (Aliased Area) Mapped to the same      | Cortex-M4 core
    * |                         |    access space of 0x9800_0000-0x9BFF_FFFF     |
    * |-------------------------|------------------------------------------------|----------------
    * | 0x6000_0000-0x66FF_FFFF | FlexBus (External Memory - Write-back)         | All masters
    * |-------------------------|------------------------------------------------|----------------
    * | 0x9800_0000-0x9FFF_FFFF | FlexBus (External Memory - Write-through)      | All masters
    * |-------------------------|------------------------------------------------|----------------
    * | 0xA000_0000-0xDFFF_FFFF | FlexBus (External Peripheral - Not executable) | All masters
    * |-------------------------|------------------------------------------------|----------------
    */
   static constexpr uint32_t BASE_ADDRESS1 = 0x6000'0000;
   static constexpr uint32_t BASE_ADDRESS2 = 0x9800'0000;

   Flexbus::enable();

   static const FlexbusEntry flexbusEntries[] = {
         {FlexbusRegion0, BASE_ADDRESS1, FlexbusSize_128kiB, FlexbusMode_ReadWrite, FlexbusWait_0, FlexbusPortSize_8bit, FlexbusAutoAck_Enabled},
         {FlexbusRegion1, BASE_ADDRESS2, FlexbusSize_256kiB, FlexbusMode_ReadWrite, FlexbusWait_0, FlexbusPortSize_8bit, FlexbusAutoAck_Enabled},
   };
   Flexbus::configureSelectRanges(flexbusEntries);
   Flexbus::configureMultiplexing(FlexbusGroup1_FB_ALE);

   static const MemoryAddressWrapper<uint8_t, BASE_ADDRESS1, 100> externalRam1;
   static const MemoryAddressWrapper<uint8_t, BASE_ADDRESS2, 100> externalRam2;

   static constexpr uint8_t value = 0x34;

   externalRam1[3] = value;
   console.writeln("Wrote ", value, ", Read back ", externalRam1[3]);

   externalRam2[3] = value;
   console.writeln("Wrote ", value, ", Read back ", externalRam2[3]);

   for(;;) {
   }
   return 0;
}
