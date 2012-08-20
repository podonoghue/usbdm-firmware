/*
 * SWD.h
 *
 *  Created on: 04/08/2012
 *      Author: podonoghue
 */

#ifndef SWD_H_
#define SWD_H_

#include "Common.h"

// Ack values displaced by offset introduced during read (left justified 8-bit value)
#define SWD_ACK_OK       (0x1<<5)
#define SWD_ACK_WAIT     (0x2<<5)
#define SWD_ACK_FAULT    (0x4<<5)
#define SWD_ACK_PROTOCOL (0x7<<5)

void swd_interfaceIdle(void);
void swd_init(void);
void swd_txIdle8(void);

U8 swd_test(void);

U8 swd_sendCommandWithWait(U8 command);

U8 swd_connect(void);
U8 swd_readReg(U8 command, U8 *data);
U8 swd_writeReg(U8 command, const U8 *data);
U8 swd_writeAPReg(const U8 *address, const U8 *buff);
U8 swd_readAPReg(const U8 *address, U8 *buff);
U8 swd_clearStickyError(void);

#endif /* SWD_H_ */
