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

#define RED_LED   (1<<3)  // C.3
//#define GREEN_LED (1<<4)  // D.4
#define BLUE_LED  (1<<2)   // A.2
#define RESCUE    (1<<12) // A.12

int main(void)  {
   // Just in case while testing
   delay();

   SIM->SCGC5  |= SIM_SCGC5_PORTA_MASK;
   PORTA->PCR[0]  = PORT_PCR_MUX(1);  // Disable SWD-CLK
   PORTA->PCR[3]  = PORT_PCR_MUX(1);  // Disable SWD-DATA

   // Just to be difficult set all the SWD pins low
   GPIOA->PDDR |= ((1<<0)|(1<<3)|(1<<20));
   GPIOA->PCOR |= ((1<<0)|(1<<3)|(1<<20));

   SIM->SCGC5   |= SIM_SCGC5_PORTA_MASK|SIM_SCGC5_PORTC_MASK|SIM_SCGC5_PORTD_MASK;

   PORTC->PCR[3]   = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK; // RED
   PORTA->PCR[2]   = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK; // BLUE

   PORTA->PCR[12]  = PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK; // RESCUE

   GPIOC->PDDR  |= RED_LED;   // C.3
   GPIOA->PDDR  |= BLUE_LED;  // A.2

   GPIOA->PDDR  &= ~RESCUE;   // A.12

	for(;;) {
	   if ((PORTA->PCR[0]&PORT_PCR_MUX(7)) != PORT_PCR_MUX(7)) {
         GPIOC->PTOR   = RED_LED;
	      GPIOA->PSOR   = BLUE_LED;
	   }
	   else {
         GPIOA->PTOR   = BLUE_LED;
         GPIOC->PSOR   = RED_LED;
	   }
	   delay();
	   if ((GPIOA->PDIR&RESCUE) == 0) {
	      recover();
	   }
	}
	return 0;
}
