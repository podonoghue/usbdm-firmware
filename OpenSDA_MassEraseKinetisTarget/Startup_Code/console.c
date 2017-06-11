/*
 * console-mk.c
 *
 *  Created on: 14/04/2013
 *      Author: pgo
 */

#include <derivative.h>
#include "system.h"
#include "clock_configure.h"
#include "console.h"

//#define USE_IRQ

#if defined(MCU_MK82F25615)
//=================================================================================
// UART to use
//
#define LPUART  LPUART0

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC2 |= SIM_SCGC2_LPUART0_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_LPUART0RXSRC_MASK|SIM_SOPT5_LPUART0TXSRC_MASK);

   // Clock source (OSCERCLK)
   SIM->SOPT2 &= (SIM->SOPT2&~SIM_SOPT2_LPUARTSRC_MASK)|SIM_SOPT2_LPUARTSRC(2);

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

   // Set Tx & Rx Pin function
   PORTB->PCR[16] = PORT_PCR_MUX(3);
   PORTB->PCR[17] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MKV31F51212)
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SYSTEM_UART0_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTB->PCR[16] = PORT_PCR_MUX(3);
   PORTB->PCR[17] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MK22F51212)
//=================================================================================
// UART to use
//
#define UART  UART1
#define UART_CLOCK SYSTEM_UART1_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTE->PCR[0] = PORT_PCR_MUX(3);
   PORTE->PCR[1] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MK21F12) || defined(MCU_MK22F12)|| defined(MCU_MK22FA12) || defined(MCU_MK20D5)
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SYSTEM_UART0_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTB->PCR[16] = PORT_PCR_MUX(3);
   PORTB->PCR[17] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MK20D10) || defined(MCU_MK20DZ10) || defined(MCU_MK40D10) || defined(MCU_MK40DZ10) || defined(MCU_MK20D7)
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SYSTEM_UART0_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTD->PCR[6] = PORT_PCR_MUX(3);
   PORTD->PCR[7] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MK60DZ10) || defined(MCU_MK60D10) || defined(MCU_MK60F12) ||\
      defined(MCU_MK10DZ10) || defined(MCU_MK10D10) || defined(MCU_MK10F12)
//=================================================================================
// UART to use
//
#define UART  UART5
#define UART_CLOCK SYSTEM_UART5_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC1 |= SIM_SCGC1_UART5_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

   // Set Tx & Rx pins in use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTE->PCR[8] = PORT_PCR_MUX(3);
   PORTE->PCR[9] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MK64F12) || defined(MCU_MK66F18)
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SYSTEM_UART0_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTB->PCR[16] = PORT_PCR_MUX(3);
   PORTB->PCR[17] = PORT_PCR_MUX(3);

#ifdef USE_IRQ
   // Enable IRQs in NVIC
   NVIC_EnableIRQ(UART0_RxTx_IRQn);
   NVIC_EnableIRQ(UART0_Error_IRQn);
#endif
}
#else
#error "Please modify before use"
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SYSTEM_UART0_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTD->PCR[6] = PORT_PCR_MUX(3);
   PORTD->PCR[7] = PORT_PCR_MUX(3);
}
#endif

#if defined(LPUART)
#define LPUART_FLAGS                \
   (LPUART_STAT_LBKDIF_MASK|        \
         LPUART_STAT_RXEDGIF_MASK|  \
         LPUART_STAT_IDLE_MASK|     \
         LPUART_STAT_OR_MASK|       \
         LPUART_STAT_NF_MASK|       \
         LPUART_STAT_FE_MASK|       \
         LPUART_STAT_PF_MASK)

/**
 * Set Console baud rate
 *
 * @param baudRate - the baud rate to use e.g. 19200
 */
void console_setBaudRate(int baudRate) {
#define OSR_VALUE (16)

   // Disable UART before changing registers
   LPUART->CTRL = 0;

   // Calculate UART clock setting (assume 8MHz clock!)
   int scaledBaudValue = (8000000)/(OSR_VALUE*baudRate);

   // Set Baud rate register
   LPUART->BAUD = LPUART_BAUD_SBR(scaledBaudValue)|LPUART_BAUD_OSR(OSR_VALUE-1);

   // Clear flags
   LPUART->STAT = LPUART_FLAGS;

   // Disable FIFO
   LPUART->FIFO = 0;

   #ifdef USE_IRQ
   // Enable UART Tx & Rx - with Rx IRQ
#error
#else
   // Enable UART Tx & Rx
   LPUART->CTRL = LPUART_CTRL_RE_MASK|LPUART_CTRL_TE_MASK;
#endif
}

/*
 * Transmits a single character over the UART (blocking)
 *
 * @param ch - character to send
 */
void console_txChar(int ch) {
   while ((LPUART->STAT & LPUART_STAT_TDRE_MASK) == 0) {
      // Wait for Tx buffer empty
      __asm__("nop");
   }
   LPUART->DATA = ch;
}

/*
 * Receives a single character over the UART (blocking)
 *
 * @return - character received
 */
int console_rxChar(void) {
   uint32_t status;

   // Wait for Rx buffer full
   do {
      status = LPUART->STAT & LPUART_FLAGS;
      // Clear & ignore pending errors
      if (status != 0) {
         // Clear flags
         LPUART->STAT = status;
      }
   }  while ((status & LPUART_STAT_RDRF_MASK) == 0);
   int ch = LPUART->DATA;
   if (ch == '\r') {
      ch = '\n';
   }
   return ch;
}
#elif defined(UART)
/**
 * Set Console baud rate
 *
 * @param baudRate - the baud rate to use e.g. 19200
 */
void console_setBaudRate(int baudRate) {
   // Disable UART before changing registers
   UART->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

   // Calculate UART clock setting (5-bit fraction at right)
   int scaledBaudValue = (2*UART_CLOCK)/(baudRate);

#ifdef UART_C4_BRFA_MASK
   // Set Baud rate register
   UART->BDH = (UART->BDH&~UART_BDH_SBR_MASK) | UART_BDH_SBR((scaledBaudValue>>(8+5)));
   UART->BDL = UART_BDL_SBR(scaledBaudValue>>5);
   // Fractional divider to get closer to the baud rate
   UART->C4 = (UART->C4&~UART_C4_BRFA_MASK) | UART_C4_BRFA(scaledBaudValue);
#else
   scaledBaudValue += 1<<4; // Round value
   // Set Baud rate register
   UART->BDH = (UART->BDH&~UART_BDH_SBR_MASK) | UART_BDH_SBR((scaledBaudValue>>(8+5)));
   UART->BDL = UART_BDL_SBR(scaledBaudValue>>5);
#endif

   UART->C1 = 0;

#ifdef USE_IRQ
   // Enable UART Tx & Rx - with Rx IRQ
   UART->C2 = UART_C2_TE_MASK|UART_C2_RE_MASK|UART_C2_RIE_MASK;
#else
   // Enable UART Tx & Rx
   UART->C2 = UART_C2_TE_MASK|UART_C2_RE_MASK;
#endif
}

/*
 * Transmits a single character over the UART (blocking)
 *
 * @param ch - character to send
 */
void console_txChar(int ch) {
   while ((UART->S1 & UART_S1_TDRE_MASK) == 0) {
      // Wait for Tx buffer empty
      __asm__("nop");
   }
   UART->D = ch;
}

/*
 * Receives a single character over the UART (blocking)
 *
 * @return - character received
 */
int console_rxChar(void) {
   uint8_t status;
   // Wait for Rx buffer full
   do {
      status = UART->S1;
      // Clear & ignore pending errors
      if ((status & (UART_S1_FE_MASK|UART_S1_OR_MASK|UART_S1_PF_MASK|UART_S1_NF_MASK)) != 0) {
         (void)UART->D;
      }
   }  while ((status & UART_S1_RDRF_MASK) == 0);
   int ch = UART->D;
   //   console_txChar(ch);
   if (ch == '\r') {
      ch = '\n';
   }
   return ch;
}
#else
#error "UART/LPUART not defined"
#if !defined(UART_CLOCK)
#error "UART_CLOCK not defined"
#endif
#endif

/*
 * Initialises the Console with default settings
 */
void console_initialise() {
   initDefaultUart();
   console_setBaudRate(DEFAULT_BAUD_RATE);
}

