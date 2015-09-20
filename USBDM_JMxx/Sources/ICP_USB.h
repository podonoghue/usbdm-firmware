/*
 * ICP_USB.h
 *
 *  Created on: 25/08/2011
 *      Author: PODonoghue
 */

#ifndef ICP_USB_H_
#define ICP_USB_H_

#include "Common.h"

#pragma CODE_SEG BOOT_ROM
extern void initICP_USB(void);
extern void USBEventPollingLoop(void);
extern U8 icpReset(void);
#pragma CODE_SEG DEFAULT

#endif /* ICP_USB_H_ */
