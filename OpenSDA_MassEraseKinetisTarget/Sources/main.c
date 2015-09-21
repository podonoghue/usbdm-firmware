#include "derivative.h" /* include peripheral declarations */

#include "BDMCommon.h"
#include "Commands.h"
#include "SPI.h"
#include "SWD.h"

void initPorts(void) {
   // Enable all port clocks
   SIM_SCGC5 |=   SIM_SCGC5_PORTA_MASK
                | SIM_SCGC5_PORTB_MASK
                | SIM_SCGC5_PORTC_MASK
                | SIM_SCGC5_PORTD_MASK
                | SIM_SCGC5_PORTE_MASK;
   ledInit();
}

int main(void)
{

   initPorts();
   initTimers();
   
   bdm_setTarget(T_ARM_SWD);
   spi_setSpeed(12000 /* kHz */);
   
   swd_reset_capture_mass_erase();

   for(;;) {
      
   }
}
