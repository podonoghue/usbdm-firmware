/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "derivative.h" /* include peripheral declarations */

void recover(void) {
   SIM_SCGC5  |= SIM_SCGC5_PORTA_MASK;
   PORTA_PCR0  = PORT_PCR_MUX(7);  // Restore SWD-CLK
   PORTA_PCR0  = PORT_PCR_MUX(7);  // Restore SWD-DATA   
}

void delay(void) {
   int i;
   for (i=0; i<100000; i++) {
      __asm("nop");
   }   
}

#define RED_LED   (1<<18)
#define GREEN_LED (1<<19)
#define RESCUE    (1<<0)

int main(void)  {
   // Just in case while testing
//   delay();

   SIM_SCGC5  |= SIM_SCGC5_PORTA_MASK;
   PORTA_PCR0  = PORT_PCR_MUX(1);  // Disable SWD-CLK
   PORTA_PCR0  = PORT_PCR_MUX(1);  // Disable SWD-DATA
      
   SIM_SCGC5   |= SIM_SCGC5_PORTB_MASK;
   PORTB_PCR18  = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;
   PORTB_PCR19  = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;
   PORTB_PCR0   = PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
   GPIOB_PDDR  |= RED_LED|GREEN_LED;
   GPIOB_PDDR  &= ~RESCUE;
   
	for(;;) {	  
	   if ((PORTA_PCR0&PORT_PCR_MUX(7)) != PORT_PCR_MUX(7)) {
         GPIOB_PTOR   = RED_LED;       
	      GPIOB_PSOR   = GREEN_LED;
	   }
	   else {
         GPIOB_PTOR   = GREEN_LED;
         GPIOB_PSOR   = RED_LED;       
	   }
	   delay();
	   if ((GPIOB_PDIR&RESCUE) == 0) {
	      recover();
	   }
	}
	return 0;
}
