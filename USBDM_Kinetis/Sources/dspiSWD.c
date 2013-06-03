/*
 * sciColdfire.c
 *
 *  Created on: 02/03/2012
 *      Author: podonoghue
 */
#include "Common.h"
#include "Configure.h"
#include "SPI.h"
#include "dspiSWD.h"
#include "derivative.h" /* include peripheral declarations */
#include "Commands.h"

const int ctas_8bit  = 0;
const int ctas_16bit = 1;

//! Initialise SPI for SWD
//!
//! @return BDM_RC_OK => success
//!
uint8_t initDSPI_SWD(void) {
   spi_configure(SPI_CTAR_LSBFE_MASK|SPI_CTAR_FMSZ(8-1),      // 8-bit transfer
                 SPI_CTAR_LSBFE_MASK|SPI_CTAR_FMSZ(16-1));    // 16-bit transfer 
   return spi_setSpeed(0);
}

//! Transmit a 8-bit word with the target 
//!
//! @param send    - data to send
//!
//! @return BDM_RC_OK => success
//!
uint8_t spi_tx8(uint8_t data) {
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_EOQ_MASK|SPI_PUSHR_PCS(0x1)|SPI_PUSHR_TXDATA(data); 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   SPI0_SR = SPI_SR_EOQF_MASK;
   return BDM_RC_OK;
}

//! Transmit a 32-bit word with the target 
//!
//! @param send    - data to send
//!
//! @return BDM_RC_OK => success
//!
uint8_t spi_tx32(const uint8_t *data) {
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_PCS(0x1)|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(*data++);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_PCS(0x1)|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(*data++);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_PCS(0x1)|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(*data++);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_PCS(0x1)|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(*data++)|SPI_PUSHR_EOQ_MASK; 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   SPI0_SR = SPI_SR_EOQF_MASK;
   return BDM_RC_OK;
}

//! Receive a 32-bit word from the target 
//!
//! @param receive - data received
//!
//! @return BDM_RC_OK => success
//!
uint8_t spi_rx32(uint8_t *receive) {
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_PCS(0x1)|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_PCS(0x1)|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_PCS(0x1)|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_PCS(0x1)|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK; 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   SPI0_SR = SPI_SR_EOQF_MASK;
   *receive++  = SPI0_POPR;
   *receive++  = SPI0_POPR;
   *receive++  = SPI0_POPR;
   *receive++  = SPI0_POPR;
   return BDM_RC_OK;
}
