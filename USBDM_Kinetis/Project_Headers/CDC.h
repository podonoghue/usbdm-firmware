/*
 * SCI.h
 *
 *  Created on: 29/09/2010
 *      Author: PODonoghue
 */

#ifndef SCI_H_
#define SCI_H_

#include <stdint.h>

//! Structure for SetLineCoding/GetLineCoding
//! This MUST agree with the USB format
//!
typedef struct {
	uint32_t dwDTERate;     //!< data rate (littleEndian format)
	uint8_t  bCharFormat;   //!< character format
	uint8_t  bParityType;   //!< parity type
	uint8_t  bDataBits;     //!< number of bits
} LineCodingStructure;

// Interrupt handlers
void cdc_txHandler(void);
void cdc_rxHandler(void);
//void cdcErrorHandler(void);

// SCI Tx Buffer 
uint8_t cdc_putTxBuffer(char *source, uint8_t size);
uint8_t cdc_txBufferIsFree(void);
void checkUsbCdcTxData(void);

// SCI Rx
uint8_t cdc_setRxBuffer(char *buffer);
uint8_t cdc_rxBufferItemCount(void);
void checkUsbCdcRxData(void);

void cdc_setLineCoding(const LineCodingStructure *lineCodingStructure);
const LineCodingStructure *cdc_getLineCoding(void);
void cdc_setControlLineState(uint8_t value);
void cdc_sendBreak(U16 length);

#define SERIAL_STATE_CHANGE (1<<7)
uint8_t cdc_getSerialState(void);

#endif /* SCI_H_ */
