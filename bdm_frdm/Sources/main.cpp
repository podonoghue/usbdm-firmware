/*
 ============================================================================
 * @file    main.cpp (180.ARM_Peripherals)
 * @brief   Basic C++ demo using GPIO class
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include <math.h>
#include <string.h>
#include <random>
#include <stdlib.h>
#include "hardware.h"
#include "interface.h"
#include "usb.h"
#include "targetVddInterface.h"
#include "resetInterface.h"
#include "delay.h"
#include "console.h"
#include "configure.h"
#include "commands.h"
#include "bdmCommon.h"
#include "cmdProcessingSWD.h"
#if HW_CAPABILITY & CAP_BDM
#include "bdm.h"
#include "cmdProcessingHCS.h"
#endif
using namespace USBDM;

#if 0
/** Check error code from USBDM API function
 *
 * @param rc       Error code to report
 * @param file     Filename to report
 * @param lineNum  Line number to report
 *
 * An error message is printed with line # and the program exited if
 * rc indicates any error
 */
void check(USBDM_ErrorCode rc , const char *file = NULL, unsigned lineNum = 0 ) {
   (void)rc;
   (void)file;
   (void)lineNum;
   if (rc == BDM_RC_OK) {
   //   console.WRITE("OK, [").WRITE(file).WRITE(":#").WRITE(lineNum).WRITELN("]");
      return;
   }
   console.WRITE("Failed, [").WRITE(file).WRITE(":#").WRITE(lineNum).WRITE("], Reason= ").WRITELN(rc);
   __BKPT();
}
/**
 * Convenience macro to add line number information to check()
 */
#define CHECK(x) check((x), __FILE__, __LINE__)

uint8_t buffer[20] = {
      0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x20
};

using namespace Hcs;
using namespace Cfv1;

/**
 *  Sets up dummy command for testing f_CMD_WRITE_MEM()
 */
USBDM_ErrorCode memWrite(uint32_t address, uint8_t opSize, uint8_t size, const uint8_t data[]) {
   //                          SIZE     #BYTES   Address
   uint8_t operation[] = {0,0, opSize,   size,   (uint8_t)(address>>24), (uint8_t)(address>>16), (uint8_t)(address>>8), (uint8_t)address};

   memcpy(commandBuffer,                   operation, sizeof(operation));
   memcpy(commandBuffer+sizeof(operation), data,      size);

   return f_CMD_WRITE_MEM();
}

/**
 *  Sets up dummy command for testing f_CMD_READ_MEM()
 */
USBDM_ErrorCode memRead(uint32_t address, uint8_t opSize, uint8_t size, uint8_t data[]) {
   //                          SIZE     #BYTES   Address
   uint8_t operation[] = {0,0, opSize,   size,   (uint8_t)(address>>24), (uint8_t)(address>>16), (uint8_t)(address>>8), (uint8_t)address};

   memcpy(commandBuffer, operation, sizeof(operation));
   USBDM_ErrorCode rc = f_CMD_READ_MEM();
   if (rc == BDM_RC_OK) {
      memcpy(data, commandBuffer+1, size);
   }
   return rc;
}

/**
 *
 * Examples
 *   SWD : testmem(0x20000000, 0x10000);
 *
 */
USBDM_ErrorCode testmem(uint32_t addressStart, uint32_t addrRange) {

//   console.WRITE("Connection speed = %ld Hz\n", getSpeed());

   CHECK(f_CMD_CONNECT());
//   CHECK(Swd::powerUp());
   console.WRITE("Connected\n");

   uint8_t randomData[sizeof(commandBuffer)];
   for (unsigned i=0; i<sizeof(randomData);i++) {
      randomData[i] = rand();
   }
   for(;;) {
      static const uint8_t sizes[] = {1,2,4};
      int sizeIndex    = rand()%3;
      uint8_t  opSize  = sizes[sizeIndex];
      uint8_t  size    = rand()%(sizeof(commandBuffer)-20)+1;
      uint32_t address = addressStart+rand()%(addrRange-size);

      uint32_t mask = ~((1<<sizeIndex)-1);
      address = address & mask;
      size    = size & mask;

      //                          SIZE     #BYTES   Address
      uint8_t operation[] = {0,0, opSize,   size,   (uint8_t)(address>>24), (uint8_t)(address>>16), (uint8_t)(address>>8), (uint8_t)address};

      console.write("Testing [").WRITE(address,Radix_16).WRITE("..").WRITE(address+size-1,Radix_16).WRITE("]:").WRITELN("-BW-L"[opSize]);

      memcpy(commandBuffer, operation, sizeof(operation));
      memcpy(commandBuffer+sizeof(operation), randomData, size);
      CHECK(f_CMD_WRITE_MEM());

      memset(commandBuffer, 0, sizeof(commandBuffer));
      memcpy(commandBuffer, operation, sizeof(operation));
      CHECK(f_CMD_READ_MEM());

      if (memcmp(commandBuffer+1, randomData, size) != 0) {
         CHECK(BDM_RC_CF_DATA_INVALID);
      }
   }
   return BDM_RC_OK;
}
#endif

#if 0
USBDM_ErrorCode recover() {
   Swd::initialise();
   USBDM_ErrorCode rc = Swd::lineReset();
   if (rc != BDM_RC_OK) {
      console.WRITE();
      console.WRITE("Failed lineReset(), rc = %d\n", rc);
      rc = Swd::connect();
      if (rc != BDM_RC_OK) {
         console.WRITE("Failed connect(), rc = %d\n", rc);
      }
   }
   if (rc == BDM_RC_OK) {
      console.WRITE("lineReset()/connect() done\n");
   }
   uint32_t status;
   rc = Swd::readReg(Swd::SWD_RD_DP_STATUS, status);
   if (rc != BDM_RC_OK) {
      console.WRITE("Failed SWD_RD_DP_STATUS, rc = %d\n", rc);
   }
   else {
      console.WRITE("DP Status = 0x%08lX\n", status);
   }
   rc = Swd::clearStickyBits();
   if (rc != BDM_RC_OK) {
      console.WRITE("Failed clearStickyBits(), rc = %d\n", rc);
   }
   else {
      console.WRITE("Cleared sticky bits\n");
   }
   return rc;
}

void testMassErase() {
   Swd::initialise();

   CHECK(Swd::f_CMD_CONNECT());
   CHECK(Swd::powerUp());

   console.WRITE("Connected\n");

   USBDM_ErrorCode rc = Swd::kinetisMassErase();
   if (rc != BDM_RC_OK) {
      console.WRITE("Failed massErase(), rc = %d\n", rc);
   }
   else {
      console.WRITE("OK massErase()\n");
   }
}

/**
 *
 */
USBDM_ErrorCode testmem() {

   Swd::initialise();

   console.WRITE("Connection speed = %ld Hz\n", Swd::getSpeed());

   CHECK(Swd::f_CMD_CONNECT());
   CHECK(Swd::powerUp());
   console.WRITE("Connected\n");

   uint8_t randomData[sizeof(commandBuffer)];
   for (unsigned i=0; i<sizeof(randomData);i++) {
      randomData[i] = rand();
   }
   for(;;) {
      static const uint8_t sizes[] = {1,2,4};
      int sizeIndex    = rand()%3;
      uint8_t  opSize  = sizes[sizeIndex];
      uint32_t address = 0x20000000+rand()%10000;
      uint8_t  size    = rand()%(sizeof(commandBuffer)-20)+1;

      uint32_t mask = ~((1<<sizeIndex)-1);
      address = address & mask;
      size    = size & mask;

      //                          SIZE     #BYTES   Address
      uint8_t operation[] = {0,0, opSize,   size,   (uint8_t)(address>>24), (uint8_t)(address>>16), (uint8_t)(address>>8), (uint8_t)address};

      console.write("Testing [").WRITE(address,Radix_16).WRITE("..").WRITE(address+size-1,Radix_16).WRITE("]:").WRITELN("-BW-L"[opSize]);

      memcpy(commandBuffer, operation, sizeof(operation));
      memcpy(commandBuffer+sizeof(operation), randomData, size);
      CHECK(Swd::f_CMD_WRITE_MEM());

      memset(commandBuffer, 0, sizeof(commandBuffer));
      memcpy(commandBuffer, operation, sizeof(operation));
      CHECK(Swd::f_CMD_READ_MEM());

      if (memcmp(commandBuffer+1, randomData, size) != 0) {
         return BDM_RC_CF_DATA_INVALID;
      }
   }
   return BDM_RC_OK;
}

USBDM_ErrorCode checkIDcode() {
   Swd::initialise();

   CHECK(Swd::f_CMD_CONNECT());
   CHECK(Swd::powerUp());
   console.WRITE("Connected\n");

   uint32_t idcode;
   USBDM_ErrorCode rc = Swd::readReg(Swd::SWD_RD_DP_IDCODE, idcode);
   if (rc != BDM_RC_OK) {
      console.WRITE("Failed SWD_RD_DP_IDCODE, rc = ").writeln(rc);
      return rc;
   }
   else {
      if (idcode != 0x2BA01477) {
         console.WRITE("Wrong IDCODE = 0x").writeln(idcode, Radix_16);
         return rc;
      }
      else {
         console.WRITE("IDCODE = 0x").writeln(idcode, Radix_16);
      }
   }
   return BDM_RC_OK;
}

void testReset() {
   for(;;) {
      ResetInterface::low();
      waitMS(100);
      ResetInterface::high();
      waitMS(100);
      ResetInterface::low();
      waitMS(100);
      ResetInterface::highZ();
      waitMS(100);
   }
}

void hcs08Testing () {
   // Need to initialise for debug UART0
   ::initialise();

   console.WRITE("SystemBusClock  = ").writeln(::SystemBusClock);
   console.WRITE("SystemCoreClock = ").writeln(::SystemCoreClock);
   console.WRITE("Target Vdd = ").writeln(TargetVdd::readVoltage());

   USBDM_ErrorCode rc;
   do {
      rc = setTarget(T_HCS08);
      if (rc != BDM_RC_OK) {
         continue;
      }
      rc = f_CMD_CONNECT();
      if (rc != BDM_RC_OK) {
         continue;
      }
      rc = f_CMD_HALT();
      if (rc != BDM_RC_OK) {
         continue;
      }
      testmem(0x80,1024);
   } while (true);

   for(;;) {
   }

}
#endif

/**
 *  Dummy callback function servicing the interrupt from Vdd changes
 */
static void targetVddSense(VddState) {
   console.WRITELN("Target Vdd Change");
}

void warmStart() {
   ResetInterface::initialise();
   UsbLed::initialise();

   Debug::initialise();

   InterfaceEnable::initialise();
   // The interface is always on
   InterfaceEnable::on();

//   console_initialise();

   checkError();
}

void coldStart() {
   warmStart();

   TargetVddInterface::initialise(targetVddSense);

   // Wait for Vbdm stable
   wait(10*ms);

   checkError();
}

#include "utilities.h"

char buff[100];

int main() {
//   hcs08Testing();

   // Need to coldStart voltage monitoring etc
   ::coldStart();

//   console.write("SystemBusClock  = ").write(::SystemBusClock/1000000.0).writeln(" MHz");
//   console.write("SystemCoreClock = ").write(::SystemCoreClock/1000000.0).writeln(" MHz");
//
//   console.write("HardwareId = ").writeln(HardwareId::getId());
//   console.write("Target Vdd = ").write(TargetVddInterface::readVoltage()).writeln(" V");

   console.WRITELN("\n\nStarting");

   UsbImplementation::initialise();
   checkError();

   for(;;) {
      // Wait for USB connection
      while(!UsbImplementation::isConfigured()) {
         __WFI();
      }
      // Process commands
      commandLoop();
      // Re-initialise
      ::warmStart();
   }
}
