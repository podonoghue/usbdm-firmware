#ifndef _USB_H_
#define _USB_H_

#include "Common.h"
/* Public Functions */
void usb_init(void);

extern void usb_isr(void);
#endif