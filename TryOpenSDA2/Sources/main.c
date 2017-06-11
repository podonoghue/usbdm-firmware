/**
 * Example OpenSDA format code i.e. can be loaded into P7E version of OpenSDA
 */
#include <stdio.h>

#include "derivative.h" /* include peripheral declarations */
#include "Configure.h"
#include "Clock.h"

//#define CLK_OUT_PTC3

void initPorts() {
   // Enable all port clocks
   SIM_SCGC5 |=   SIM_SCGC5_PORTA_MASK
                | SIM_SCGC5_PORTB_MASK
                | SIM_SCGC5_PORTC_MASK
                | SIM_SCGC5_PORTD_MASK
                | SIM_SCGC5_PORTE_MASK;
   
   ledInit();

#ifdef CLK_OUT_PTC3
   /* Enable the CLKOUT function on PTC3 (alt5 function) */
   PORTC_PCR3 = PORT_PCR_MUX(0x5);
   
   /* Select the CLKOUT in the SMI_SOPT2 mux to be bus clk*/
   SIM_SOPT2 |= SIM_SOPT2_CLKOUTSEL(2);
#endif
}

void delay(void) {
   int i;
   for(i=0; i<80000; i++) {
      asm("nop");
   }
}

int main(void) {
   initPorts();
   initClock();
   ledInit();
   for(;;) { 
      greenLedToggle();
      delay();
   }
   return 0;
}
