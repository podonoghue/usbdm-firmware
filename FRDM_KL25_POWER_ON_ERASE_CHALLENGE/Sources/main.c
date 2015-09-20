/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "derivative.h" /* include peripheral declarations */

void recover(void) {
   SIM->SCGC5  |= SIM_SCGC5_PORTA_MASK;
   PORTA->PCR[0]  = PORT_PCR_MUX(7);  // Restore SWD-CLK
   PORTA->PCR[3]  = PORT_PCR_MUX(7);  // Restore SWD-DATA
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

   SIM->SCGC5  |= SIM_SCGC5_PORTA_MASK;
   PORTA->PCR[0]  = PORT_PCR_MUX(1);  // Disable SWD-CLK
   PORTA->PCR[3]  = PORT_PCR_MUX(1);  // Disable SWD-DATA
   PORTA->PCR[20] = PORT_PCR_MUX(1);  // Enable RESET as a GPIO

   GPIOA->PDDR |= ((1<<0)|(1<<3)|(1<<20));
   GPIOA->PCOR |= ((1<<0)|(1<<3)|(1<<20));

   SIM->SCGC5   |= SIM_SCGC5_PORTB_MASK;
   PORTB->PCR[18]  = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;
   PORTB->PCR[19]  = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;
   PORTB->PCR[0]   = PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
   GPIOB->PDDR  |= RED_LED|GREEN_LED;
   GPIOB->PDDR  &= ~RESCUE;


	for(;;) {
	   if ((PORTA->PCR[0]&PORT_PCR_MUX(7)) != PORT_PCR_MUX(7)) {
         GPIOB->PTOR   = RED_LED;
	      GPIOB->PSOR   = GREEN_LED;
	   }
	   else {
         GPIOB->PTOR   = GREEN_LED;
         GPIOB->PSOR   = RED_LED;
	   }
	   delay();
	   if ((GPIOB->PDIR&RESCUE) == 0) {
	      recover();
	   }
	}
	return 0;
}
