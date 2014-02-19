/*
 * SWD.h
 *
 *  Created on: 04/08/2012
 *      Author: PODonoghue
 */

#ifndef SWD_H_
#define SWD_H_

#include "Common.h"
#include <stdint.h>

// Ack values displaced by offset introduced during read (left justified 8-bit value)
#define SWD_ACK_OK       (0x1)
#define SWD_ACK_WAIT     (0x2)
#define SWD_ACK_FAULT    (0x4)
#define SWD_ACK_PROTOCOL (0x7)

void swd_interfaceIdle(void);
void swd_init(void);
void swd_txIdle8(void);

uint8_t swd_test(uint8_t *returnSize, uint8_t *buff);
uint8_t swd_reset_capture_mass_erase(uint8_t *returnSize, uint8_t *buff);

uint8_t swd_sendCommandWithWait(uint8_t command);

uint8_t swd_connect(void);
uint8_t swd_readReg(uint8_t command, uint8_t *data);
uint8_t swd_writeReg(uint8_t command, const uint8_t *data);
uint8_t swd_readAPReg(const uint8_t *address, uint8_t *buff);
uint8_t swd_writeAPReg(const uint8_t *address, const uint8_t *buff);
uint8_t swd_clearStickyError(void);
uint8_t swd_abortAP(void);
#endif /* SWD_H_ */
