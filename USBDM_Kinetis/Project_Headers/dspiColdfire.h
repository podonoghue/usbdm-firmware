/*
 * sciColdfire.h
 *
 *  Created on: 02/03/2012
 *      Author: podonoghue
 */

#ifndef SPICOLDFIRE_H_
#define SPICOLDFIRE_H_
#include <stdint.h>

//! Initialise SPI for Coldfire BDM (17 bits as 1 + 16 transfer)
//!
//! @param freq - frequency to use
//!
uint8_t initDSPIColdfire(uint32_t freq /* kHz */);

//! Exchange a 17-bit word with the target 
//!
//! @param send    - data to send
//! @param receive - data received
//!
uint8_t rxtxDSPIColdfire(uint32_t send, uint32_t *receive);

#endif /* SPICOLDFIRE_H_ */
