/*
 * uart.c
 *
 *  Created on: Nov 6, 2012
 *      Author: podonoghue
 */
#include "derivative.h" /* include peripheral declarations */
#include "Clock.h"

void initUart0(int baud) {
   register uint16_t ubd;//, brfa;
   
   // Enable clock to PTA (for UART0 pin muxing)
   SIM_SCGC5  |= SIM_SCGC5_PORTA_MASK;
   
   // Configure shared pins
   PORTA_PCR1  = PORT_PCR_MUX(2); // Rx (PFE?)
   PORTA_PCR2  = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK| PORT_PCR_SRE_MASK; // Tx

   // Enable clock to UART0
   SIM_SCGC4  |= SIM_SCGC4_UART0_MASK;
   
   // Disable the transmitter and receiver while changing settings.
   UART0_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

   // Configure the UART for 8-bit mode, no parity (default)
   UART0_C1 = 0;

   // Calculate baud settings
   ubd = (uint16_t)(SystemCoreClock/(baud * 16));

   // Set Baud rate register
   UART0_BDH = (UART0_BDH&~UART_BDH_SBR_MASK) | UART_BDH_SBR((ubd>>8));
   UART0_BDL = UART_BDL_SBR(ubd);

   // Determine fractional divider to get closer to the baud rate
//   brfa     = (uint8_t)(((sysclk*32000)/(baud * 16)) - (ubd * 32));
   UART0_C4 = 0x0F;

   // Enable receiver and transmitter
   UART0_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
}

void putchUart0(char ch) {
   while ((UART0_S1&UART_S1_TDRE_MASK) == 0) {
   }
   UART0_D = ch;
}

void putsUart0(char *ch) {
   while (*ch != '\0') {
      putchUart0(*ch++);
   }
}

char getchUart0(void) {
   while ((UART0_S1&UART_S1_RDRF_MASK) == 0) {
   }
   return UART0_D;
}

int InitializeUART(void) {
   initUart0(38400);	
   return 0;
}

int ReadUARTN(void* bytes, unsigned long limit) {
	unsigned count;
	for (count = 0; count < limit; count++) {
    	*( (char *)bytes + count ) = getchUart0();
  	}
	return 0;
}

int WriteUARTN(const void* bytes, unsigned long length) {
	unsigned count;
	for (count = 0; count < length; count++) {
		putchUart0(*( ((char *)bytes) + count));
	}
	return 0;
}

