/*
 * SPI.h
 *
 *  Created on: 07/08/2012
 *      Author: podonoghue
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <Common.h>

#define DEFAULT_SPI_FREQUENCY  (1000)    //!< Initial SPI frequency 1000 kHz

extern volatile uint8_t bitDelay;  //!< Required software delay used with SPI base Tx/Rx
extern volatile uint8_t rxTiming1; //!< bdm_Rx timing constant #1
extern volatile uint8_t txTiming1; //!< bdm_Tx timing constant #1

void spi_init(uint32_t ctar0, uint32_t ctar1);
void spi_finalise(void);
uint8_t spi_setSpeed(uint16_t freq);

uint8_t f_CMD_SPI_SET_SPEED(void);
uint8_t f_CMD_SPI_GET_SPEED(void);

extern uint32_t spiBaudValue;

//! Set SPI.CTAR0 value (non-persistent)
//! 
//! @param ctar 32-bit CTAR value (excluding baud)
//!
__forceinline
static inline void spi_setCTAR0(uint32_t ctar) {
   SPI0->CTAR[0] = spiBaudValue|ctar;
}

//! Set SPI.CTAR1 value (non-persistent)
//! 
//! @param ctar 32-bit CTAR value (excluding baud)
//!
__forceinline
static inline void spi_setCTAR1(uint32_t ctar) {
   SPI0->CTAR[1] = spiBaudValue|ctar;
}

#endif /* SPI_H_ */
