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
void sciTxHandler(void);
void sciRxHandler(void);
void sciErrorHandler(void);

// SCI Tx Buffer 
uint8_t putTxBuffer(char *source, uint8_t size);
uint8_t sciTxBufferFree(void);
void checkUsbCdcTxData(void);

// SCI Rx
uint8_t setRxBuffer(char *buffer);
uint8_t rxBufferItems(void);
void checkUsbCdcRxData(void);

void sciSetLineCoding(const LineCodingStructure *lineCodingStructure);
const LineCodingStructure *sciGetLineCoding(void);
void sciSetControlLineState(uint8_t value);
void sciSendBreak(U16 length);

#define SERIAL_STATE_CHANGE (1<<7)
uint8_t getSerialState(void);

#endif /* SCI_H_ */
