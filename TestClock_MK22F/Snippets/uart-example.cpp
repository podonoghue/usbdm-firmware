/*
 ============================================================================
 * @file    uart-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo of using UART interface
 *
 *  Created on: 17 Jun 2018
 *      Author: podonoghue
 ============================================================================
 */
/*
 * See also console-example.cpp since the console is usually mapped to one of the USBDM::Uart's
 * and all functions demonstrated are available on a USBDM::Uart as well.
 */
#include "hardware.h"
#include "uart.h"


using namespace USBDM;

int main() {

   Uart0 uart;

   uart.configureAllPins();

   uart.setBaudRate(115200);

   char aSingleCharacter;
   char aLineOfText[60];
   uart.writeln("Hello from UART 0");

   uart.setEcho(EchoMode_Off);
   uart.writeln("Please type any key (which will not be echoed)");
   uart.read(aSingleCharacter);

   uart.setEcho(EchoMode_On);
   uart.writeln("Please press any key (which will be echoed)");
   uart.read(aSingleCharacter);
   uart.writeln();

   for(;;) {
      uart.setEcho(EchoMode_Off);
      uart.writeln("Please type a line of text terminated with enter (which will not be echoed)");
      uart.gets(aLineOfText, sizeof(aLineOfText));

      uart.setEcho(EchoMode_On);
      uart.writeln("Please type a line of text terminated with enter (which will be echoed)");
      uart.gets(aLineOfText, sizeof(aLineOfText));
   }

}
