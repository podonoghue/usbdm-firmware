#ifndef _USB_H_
#define _USB_H_

#include "Common.h"

extern void initUSB(void);

extern void receiveUSBCommand( uint8_t size, uint8_t *buffer);
extern void sendUSBResponse( uint8_t size, const uint8_t *buffer);

void USBInterruptHandler( void );

void usbPutChar(char ch);
void setBDMBusy(void);
#endif  // _USB_H_
