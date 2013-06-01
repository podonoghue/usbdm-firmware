/*! \file
    
   \verbatim
    Copyright (C) 2005  Daniel Malik

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   \endverbatim

   \verbatim
   Change Hisory
   -========================================================================
   |  5 Aug 2009 | Changed USB command/response structure              - pgo
   |  5 Aug 2009 | Moved SET_BOOT and GET_VERSION to USB module        - pgo
   | 17 May 2009 | Tested with USBCV13.exe from USB.ORG - now passes   - pgo
   | 17 May 2009 | Added GET_STATUS & GET_INTERFACE                    - pgo
   | 10 May 2009 | Changed String language to EN_AUS from GREEK!       - pgo
   | 30 Aug 2008 | Added VISTA option to remove bulk endpoints         - pgo
   | 10 Aug 2008 | Changed code so return pkt can be less than 8 bytes - pgo
   +========================================================================
   \endverbatim
*/

#include <string.h>
#include <hidef.h>      // For EnableInterrupts macro
#include "Derivative.h" // Include peripheral declarations
#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "LED.h"
#include "BDM.h"
#include "BDMCommon.h"
#include "CmdProcessing.h"
#include "USBDefs.h"
#include "USB.h"
#include "ICP.h"
#include "main.h"

#if (DEBUG&COMMAND_DEBUG)
#define commandExec() debugCommandExec()
#endif

U8 responseSize = 0;

/* Data transfer format: two possibilities exist 

1. Bulk transfer over EP2 (non standard for low speed devices!)
   ***  WARNING  This option is not available with Vista!                   ***
   ***  Inclusing this option will prevent the BDM interface working at all ***

Data format:  1 byte: size (of cmd+data)
              1 byte: cmd
              size-1 bytes: data
              
Returns at least one packet with status, but possibly more data depending on command executed.
When the data is returned the command has finished.
Start of transfer of another message into EP2 will discard any data ready to be transmitted back out of EP2

Performance: ~20kB/s, roundtrip (8B IN & 8B OUT) ~ 4ms

2. Control transfer over EP0 (standard approach and the only standard option for JB8/16)

Data format:  

  - Setup frame:  bmRequestType  = 0x41 if data is to be transfered to the device  
                  bmRequestType  = 0xC1 if data is to be transfered out of the device 
                  bRequest       = cmd
                  wValue.le.lo      = data0
                  wValue.le.hi      = data1
                  wIndex.le.lo      = data2
                  wIndex.le.hi      = data3
                  wLength.le.hi     = 0
                  wLength.le.lo     = # of bytes in data stage
                         
  cmd, data0-3 is always transferred from host to device. 
  If the command parameters fit into 4 bytes, bmRequestType = 0xC1 can be used to read results of the command being transferred
  If more than 4 bytes of parameters are needed, bmRequestType = 0x41 is used and data5,... is transferred in data stage
  New setup frame will discard any data ready to be transmitted out of the device

Performance: ~6.7kB/s, short command (5B IN, no data out) ~3ms, ave command (5B IN, 8B OUT) ~4ms, longer cmd (5B IN, 16B OUT) ~5ms

*/

/* global and static variables, buffers, etc. */

#pragma DATA_SEG __SHORT_SEG Z_PAGE
/* Private variables */
static unsigned char *usb_dptr;           /* pointer to first empty location in buffer (Rx) or to the first char to transmit (Tx) */
static unsigned char usb_ep0_dcntT;       /* data count to transmit */
static unsigned char usb_ep0_dcntR;       /* data count to receive */
static unsigned char usb_ep2_dcntT;       /* data count to transmit */
static unsigned char usb_ep2_dcntR;       /* data count to receive */

static unsigned char usb_ep0_zeroterm=0;  /* when non-zero it tells the EP0 Tx routine to terminate transfers by zero-length
                                             packet because the host has requested more data than available */
static unsigned char led_timer=0;         /* counter for timing the LED flashing */
static unsigned char USB_State = US_ATTACHED;
/* Public variables */
U8 led_state;                   /* led state variable (BLINK, ON, OFF) */
#pragma DATA_SEG DEFAULT

/* buffer for Rx and Tx of commands & results (size: 16x8byte frame + 5bytes from setup frame + 1 byte for status + 1 byte for endian byte swap = 135) */
/* data is always received starting at commandBuffer+1 and transmitted starting at commandBuffer+0 */
/* this is to make sure that status of the last command is preserved when receving the next command from the host */

#define ENDPTMAXSIZE    (8)    

/* USB descriptors */
const DeviceDescriptor DeviceDesc = {
   sizeof(DeviceDescriptor),               // Length
   DT_DEVICE,                              // Type [device]          [0x01]
   {0x10, 0x01},                           // USB spec rel. No.      [BCD = 1.10]
   0xFF,                                   // Class code             [none]
   0xFF,                                   // Sub Class code         [none]
   0xFF,                                   // Protocol               [none]
   ENDPTMAXSIZE,                           // EndPt 0 max packet size
   {VendorID&0xFF,  (VendorID>>8)&0xFF},   // Vendor ID
   {ProductID&0xFF, (ProductID>>8)&0xFF},  // Product Id
   {0x00, 0x01},                           // Device Release         [BCD = 1.00]
   1,                                      // String index of Manufacturer name
   2,                                      // String index of product desc.
   3,                                      // String index desc. serial #
   1                                       // Number of configurations
   };

// Configuration: Control EP0, Bulk IN EP2 & Bulk OUT EP2
// Bulk transfers are not allowed for low speed devices by the spec, but seem to work and much faster!
// Bulk EPs break the BDM on Vista (not even allowed in descriptor!)
// There will be several transactions per frame and the throughput is very high
// There are 2 options: either to use Bulk EP2 or Control EP0 (in case Bulk on EP2 does not work on the specific machine)
const struct {
  ConfigurationDescriptor ConfigDesc;  
  InterfaceDescriptor InterfaceDesc0;
  //EndpointDescriptor Endpoint2INDesc;  // For Vista
  //EndpointDescriptor Endpoint2OUTDesc; // For Vista
} config_data = {
   { // configDescriptor
      sizeof(ConfigurationDescriptor),    // bLength              = 9
      DT_CONFIGURATION,                   // bDescriptorType      = 2
      {sizeof(config_data),0x00},         // wTotalLength         = 25
      1,                                  // bNumInterfaces       = 1
      1,                                  // bConfigurationValue  = 1
      0,                                  // iConfiguration       = ""
      0x80,                               // bmAttributes         = Bus powered, no wakeup (yet?)
      150                                 // MaxPower             = 300 mA
   },
   { // interfaceDescriptor
      sizeof(InterfaceDescriptor),  // bLength              = 9
      DT_INTERFACE,                 // bDescriptorType      = 4
      0,                            // bInterfaceNumber     = 0
      0,                            // bAlternateSetting    = 0
#ifdef VISTA  
     0,					            // No of EPs used by this IF (excl. EP0) -- Vista doesn't allow bulk EPs
#else
      2,                            // bNumEndpoints        = 2
#endif // VISTA  
      0xFF,                         // bInterfaceClass      = (Vendor specific)
      0xFF,                         // bInterfaceSubClass   = (Vendor specific)
      0xFF,                         // bInterfaceProtocol   = (Vendor specific)
      0                             // iInterface desc      = ""
   },
#ifndef VISTA  // Vista doesn't allows Bulk endpoints (Won't even set configuration!)
   { /* EP2 IN descriptor */
     sizeof(EndpointDescriptor), // bLength           = 7
     DT_ENDPOINT,                // bDescriptorType   = 5
     0x82,				            // Endpoint Address (EP2) 
     0x02,                       // bmAttributes      = Bulk
     {0x08, 0x00},		         // Max. Endpoint Packet Size
     1                           // bInterval         = 1 ms
   },
   { /* EP2 OUT descriptor */
     sizeof(EndpointDescriptor), // bLength           = 7
     DT_ENDPOINT,                // bDescriptorType   = 5
     0x02,				            // Endpoint Address (EP2) 
     0x02,                       // bmAttributes      = Bulk
     {0x08, 0x00},		         // Max. Endpoint Packet Size 
     1                           // bInterval         = 1 ms
   }
#endif // VISTA  
};

static const char sd0[] = {4,  DT_STRING, 0x09, 0x0C, 0};  // Language IDs ( zero terminated for copy)
static const char sd1[] = "Freescale";		     // We are using Freescale VId so it is only fair to say that this is a Freescale product :-) */
static const char sd2[] = ProductDescription;  // Product version */
static const char sd3[] = "USBDM JB16-0001";   // Serial #
static const char *const stringDescriptors[]={sd0, sd1, sd2, sd3}; /* Pointers to the descriptors */

void setBDMBusy(void) {
// Dummy routine
}
//===============================================================================
//! Creates a valid string descriptor in UTF-16-LE from a limited UTF-8 string
//!
//! @param source - Zero terminated UTF-8 C string
//!
//! @param dest   - Where to place descriptor
//!
//! @note Only handles UTF-16 characters that fit in a single 'character'.
//!
static void utf8ToStringDescriptor(const U8 *source, U8 *dest) {
U8 *size = dest; // 1st byte is where to place descriptor size

    *dest++ = 2;         // 1st byte = descriptor size (2 bytes so far)
    *dest++ = DT_STRING; // 2nd byte = DT_STRING;
    
    while (*source != '\0') {
       U16 utf16Char;
       *size  += 2;         // Update size
       if (*source < 0x80) {  // 1-byte
          utf16Char = *source++;
       }
       else if ((*source &0xE0) == 0xC0){   // 2-byte
          utf16Char  = (0x1F&*source++)<<6; 
          utf16Char += (0x3F&*source++);
       } 
       else if ((*source &0xF0) == 0xE0){   // 3-byte
          utf16Char  = (0x0F&*source++)<<12; 
          utf16Char += (0x3F&*source++)<<6; 
          utf16Char += (0x3F&*source++);
       } 
       *dest++ = (char)utf16Char;       // Copy 16-bit UTF-16 value
       *dest++ = (char)(utf16Char>>8);  // in little-endian order
    }
}

/* Copies string from flash to the dest buffer */
unsigned char copystring(unsigned char descriptorIndex, unsigned char *dest) {

   if (descriptorIndex >= sizeof(stringDescriptors)/sizeof(stringDescriptors[0])) {
      // Error - Return empty string, should really stall
      *dest = '\0';
   }
   else if (descriptorIndex == 0) {
      // Language bytes just copy unchanged
      (void)memcpy(dest, sd0, sizeof(sd0));
   }
   else {
      // Strings are stored in limited UTF-8 and need conversion
      utf8ToStringDescriptor(stringDescriptors[descriptorIndex], dest);
   }
   return *dest;
}

/* 1 ms tick */
/* Handles general timing functions and blinks the LED */
void usb_1ms_tick(void) {
  if (led_timer) {  /* led is in the middle of blinking */
    led_timer--;    /* decrement the counter */
    if (led_timer==LED_OFF_TIME)
       GREEN_LED_OFF(); /* switch the led off when the counter hits the off treshold */
  } else {
    if (led_state==LED_ON) {
      GREEN_LED_ON();    /* switch it on */
    }
    else if (led_state==LED_OFF) {
      GREEN_LED_OFF();   /* switch it off */
    }
    else if (led_state==LED_BLINK) {
      GREEN_LED_ON();				  /* switch it on */
      led_state=LED_ON;            /* change state to permanent ON */
      led_timer=LED_BLINK_PERIOD;  /* blink the LED once before it settles into the ON state */
    }
  }
}

/* EP0 Tx */
void USB_ep0_tx(void) {
  UCR0_TX0E=0;      /* disable EP0 transmitter */
  UIR2_TXD0FR = 1;  /* clear the interrupt flag to make sure the packet transmits */
  if (usb_ep0_dcntT!=0xff) {
    /* transmit data from the buffer (must be in Zero Page) */
    UE0D0 = *(usb_dptr++); /* copy all 8 bytes, packet might be shorter than 8 bytes */
    UE0D1 = *(usb_dptr++);
    UE0D2 = *(usb_dptr++);
    UE0D3 = *(usb_dptr++);
    UE0D4 = *(usb_dptr++);
    UE0D5 = *(usb_dptr++);
    UE0D6 = *(usb_dptr++);
    UE0D7 = *(usb_dptr++);
    if ((usb_ep0_dcntT>8)||((usb_ep0_dcntT==8)&&(usb_ep0_zeroterm))) {
      UCR0 = ((UCR0^UCR0_T0SEQ_MASK)&UCR0_T0SEQ_MASK) | UCR0_TX0E_MASK | UCR0_RX0E_MASK + 8; /* enable transmission on EP0, toggle DATA0/1, length 8 (more data in buffer) */
      usb_ep0_dcntT-=8;
    }
    else {
      UCR0 = ((UCR0^UCR0_T0SEQ_MASK)&UCR0_T0SEQ_MASK) | UCR0_TX0E_MASK | UCR0_RX0E_MASK + usb_ep0_dcntT; /* enable transmission on EP0, toggle DATA0/1, length according to count */
      usb_ep0_dcntT = 0xff; /* no more transmission the next time */
      usb_ep0_zeroterm=0;   /* just finished transmission, switch zero-length termination off */      
    }
  } else {
    /* there is no data to transmit, but the interrupt occured anyway - this must be a special case or end of transmit condition */
    if ((USR0_SETUP)&&(((*(SetupPacket*)&UE0D0).bmRequestType&0x60)==0)) {
      /* the special case is a Setup frame of standard request which was received previously, now detemine what to do */
      switch ((*(SetupPacket*)&UE0D0).bRequest) {
        case SET_ADDRESS:
          UADDR = UADDR_USBEN_MASK | (*(SetupPacket*)&UE0D0).wValue.le.lo;  /* set the new address (confirmation of reception was just transmitted) */
          if ((*(SetupPacket*)&UE0D0).wValue.le.lo) 
             USB_State = US_ADDRESSED; 
          else 
             USB_State = US_DEFAULT; 
        case CLEAR_FEATURE:
        case SET_CONFIGURATION:
        case GET_CONFIGURATION:
          break;
      }
    }
  }
}

/* EP2 Tx */
void USB_ep2_tx(void) {
  UCR2_TX2E=0;      /* disable EP2 transmitter */
  UIR2_TXD2FR = 1;  /* clear the interrupt flag to make sure the packet transmits */
  if (usb_ep2_dcntT==0) return; /* no data to transmit - return without enabling the transmitter */
  /* transmit data from the buffer (must be in Zero Page) */
  UE2D0 = *(usb_dptr++); /* copy all 8 bytes, packet might be shorter than 8 bytes */
  UE2D1 = *(usb_dptr++);
  UE2D2 = *(usb_dptr++);
  UE2D3 = *(usb_dptr++);
  UE2D4 = *(usb_dptr++);
  UE2D5 = *(usb_dptr++);
  UE2D6 = *(usb_dptr++);
  UE2D7 = *(usb_dptr++);
  if (usb_ep2_dcntT>8) {
    UCR2 = ((UCR2^UCR2_T2SEQ_MASK)&UCR2_T2SEQ_MASK) | UCR2_TX2E_MASK | UCR2_RX2E_MASK + 8; /* enable transmission on EP2, toggle DATA0/1, length 8 (more data in buffer) */
    usb_ep2_dcntT-=8;
  } else {
    UCR2 = ((UCR2^UCR2_T2SEQ_MASK)&UCR2_T2SEQ_MASK) | UCR2_TX2E_MASK | UCR2_RX2E_MASK + usb_ep2_dcntT; /* enable transmission on EP2, toggle DATA0/1, length according to count */
    usb_ep2_dcntT = 0;  /* no more data to transmit */
  }
}

/* EP0 Rx */
void USB_ep0_rx(void) {
  unsigned char * ptr=&UE0D0;
  while ((ptr<((&UE0D0)+8))&&(usb_ep0_dcntR)) {
    *(usb_dptr++) = *(ptr++);  /* copy data */
    usb_ep0_dcntR--;           /* decrement count */   
  }
  if (usb_ep0_dcntR==0) {
    /* command reception complete */
    led_state = LED_BLINK;     /* blink the LED to indicate a command */
    responseSize = commandExec();
    usb_ep0_dcntT=0xff;   /* make sure the Tx routine does not interpret the subsequent IRQ incorrectly */
    usb_ep0_zeroterm=0;   /* switch zero-length termination off */      
    UCR0 = UCR0_T0SEQ_MASK | UCR0_TX0E_MASK | UCR0_RX0E_MASK; /* enable transmission on EP0, DATA1, length 0 (status packet for the control transfer) */
  }
}

/* EP2 Rx */
void USB_ep2_rx(void) {
  /* EP2 receives the messages in raw format (size of cmd+data, cmd, data0, data1, ...) */
  unsigned char * ptr=&UE2D0;
  if (usb_ep2_dcntR==0) {
    /* if the routine is called with no data to be received, it must be beginning of a message */
    UCR2_TX2E=0;      /* disable EP2 transmitter */
    usb_ep2_dcntR=UE2D0; 
    usb_dptr=commandBuffer;
    ptr = &UE2D1; /* skip copying the size */
  }
  while ((ptr<((&UE2D0)+8))&&(usb_ep2_dcntR)) {
    *(usb_dptr++) = *(ptr++);  /* copy data */
    usb_ep2_dcntR--;                /* decrement count */   
  }
  if (usb_ep2_dcntR==0) {
    /* command reception complete */  
    usb_ep2_dcntT = commandExec();
    usb_dptr = commandBuffer;
    USB_ep2_tx();     /* start transmitting the data */
  }
}

/* handles the set-up frame */
void USB_Setup(void) {
  usb_ep0_dcntT=0xff;   /* stop any data transfer in progress upon reception of a new setup packet */
  usb_ep0_zeroterm=0;   /* switch zero-length termination off */      
  if (USR0!=USR0_SETUP_MASK + 8) {  /* check whether it is valid setup request (DATA0 && size==8) */
    UCR3 |= UCR3_ISTALL0_MASK | UCR3_OSTALL0_MASK;   /* stall EP0 communication and wait for new setup */
  } else {
    UCR0_T0SEQ=0; /* each setup transaction needs to start with DATA1 response (DATA0/1 inverted in tx) */
    if (((*(SetupPacket*)&UE0D0).bmRequestType&(3<<5))==(0<<5)) { /* standard request? */
      switch ((*(SetupPacket*)&UE0D0).bRequest) {
         case GET_STATUS:   // Added - pgo
            switch((*(SetupPacket*)&UE0D0).bmRequestType) {
               case (EP_IN|RT_DEVICE) : // Device Status - response is always 0
               case (EP_IN|RT_INTERFACE) : // Interface Status - reserved - response is always 0
                  UE0D0 = 0;  // set up buffer for transfer 
                  UE0D1 = 0;
                  UCR0  = UCR0_T0SEQ_MASK | UCR0_TX0E_MASK | UCR0_RX0E_MASK + 2; /* enable transmission on EP0, DATA1, length 2 */
                  break;
               case (EP_IN|RT_ENDPOINT) : // Endpoint Status
               default : // Illegal
                  UCR3 |= UCR3_ISTALL0_MASK | UCR3_OSTALL0_MASK;   /* stall EP0 communication and wait for new setup */
                  break;
               }
            return;
         case GET_INTERFACE:  // Added - pgo
            // if (((*(SetupPacket*)&UE0D0).bmRequestType != (EP_IN|RT_INTERFACE)) || // NOT In,Standard,Interface
               // ((*(SetupPacket*)&UE0D0).wLength.word != 1) ||                     // NOT correct length
               // (USB_State != US_ADDRESSED)) {                    // NOT in addressed state
               // UCR3 |= UCR3_ISTALL0_MASK | UCR3_OSTALL0_MASK;   /* stall EP0 communication and wait for new setup */
               // return;
               // }
            // Only support one interface
            // if ((*(SetupPacket*)&UE0D0).wValue.word != config_data.InterfaceDesc0.bInterfaceNumber) {
               // UCR3 |= UCR3_ISTALL0_MASK | UCR3_OSTALL0_MASK;   /* stall EP0 communication and wait for new setup */
               // return;
               // }
            UE0D0 = config_data.InterfaceDesc0.bAlternateSetting;  // set up buffer for transfer
            UCR0  = UCR0_T0SEQ_MASK | UCR0_TX0E_MASK | UCR0_RX0E_MASK + 1; /* enable transmission on EP0, DATA1, length 1 */
            return;
         case SET_CONFIGURATION:
            if ((*(SetupPacket*)&UE0D0).wValue.le.lo) {
               /* non-zero configuration number */
               /* initialise EP2 */
               UCR3 |= UCR3_ENABLE2_MASK;  /* enable EP2 */            
               UCR2 = UCR2_T2SEQ_MASK | UCR2_RX2E_MASK; /* enable EP2 receiver, transaction starts with DATA0 and the flag is inverted before transmitting */
               usb_ep0_dcntT=0xff;         /* bring the state machine into a known idle state */
               usb_ep0_dcntR=0;
               usb_ep2_dcntT=0;
               usb_ep2_dcntR=0;
               USB_State = US_CONFIGURED;  /* change state */  
               } else {
               /* configuration zero means back to addressed state */
               USB_State = US_ADDRESSED;
               }
               /* next send back the confirmation... */
            case SET_INTERFACE:
               /* do nothing, just send back a confirmation */
            case SET_ADDRESS:
               /* Note: address of the device is going to change only after this transmission is completed */
            case CLEAR_FEATURE:
               /* clear feature does nothing, just sends a confirmation */
               UCR0 = UCR0_T0SEQ_MASK | UCR0_TX0E_MASK | UCR0_RX0E_MASK; /* enable transmission on EP0, DATA1, length 0 (confirmation of new address reception) */
               return;
            case GET_CONFIGURATION:
               if (USB_State == US_CONFIGURED) 
                  UE0D0 = 1; 
               else 
                  UE0D0 = 0;
               UCR0 = UCR0_T0SEQ_MASK | UCR0_TX0E_MASK | UCR0_RX0E_MASK + 1; /* enable transmission on EP0, DATA1, length 1 */
               return; // pgo
            case GET_DESCRIPTOR: 
               {
               unsigned char requested_length;
               if ((*(SetupPacket*)&UE0D0).wLength.le.hi)   /* no descriptor is longer than 255 bytes... */
                  requested_length = 255; 
               else
                  requested_length = (*(SetupPacket*)&UE0D0).wLength.le.lo;  
               /* which descriptor? */
               switch ((*(SetupPacket*)&UE0D0).wValue.le.hi) {
                  case DT_DEVICE:     /* device descriptor */
                     memcpy(commandBuffer, &DeviceDesc, sizeof(DeviceDescriptor));
                     usb_ep0_dcntT = sizeof(DeviceDescriptor); /* update count */
                     usb_dptr = commandBuffer;                 /* data to transmit */
                     if (usb_ep0_dcntT>=requested_length) {
                        usb_ep0_dcntT=requested_length;
                        } else {
                        usb_ep0_zeroterm=1;
                        }
                     USB_ep0_tx();     /* start transmitting the data */
                     break;
                  case DT_CONFIGURATION:
                     memcpy(commandBuffer, &config_data, sizeof(config_data));
                     usb_ep0_dcntT = sizeof(config_data); /* update count */
                     usb_dptr = commandBuffer;			 /* data to transmit */
                     if (usb_ep0_dcntT>=requested_length) {
                        usb_ep0_dcntT=requested_length;
                     } else {
                        usb_ep0_zeroterm=1;
                     }
                     USB_ep0_tx();     /* start transmitting the data */
                     break;
                  case DT_STRING:
                     usb_ep0_dcntT = copystring((*(SetupPacket*)&UE0D0).wValue.le.lo, commandBuffer);
                     usb_dptr = commandBuffer; /* data to transmit */
                     if (usb_ep0_dcntT>=requested_length) {
                        usb_ep0_dcntT=requested_length;
                     } else {
                        usb_ep0_zeroterm=1;
                     }
                     USB_ep0_tx();     /* start transmitting the data */
                     break;
                  } // switch
               } // case GET_DESCRIPTOR: 
            return;
         } // switch
    } else if (((*(SetupPacket*)&UE0D0).bmRequestType&(3<<5))==(2<<5)) { /* vendor specific request? */
      if ((*(SetupPacket*)&UE0D0).bmRequestType&0x80) {
        /* transfer dir device->host 
           (it will be a command which reads data from the target or
           a command which has 5 or less bytes) */  

        led_state = LED_BLINK;   /* blink the LED to indicate a command */

        if ((*(SetupPacket*)&UE0D0).bRequest == CMD_USBDM_GET_VER) {
          U8 versionConstant[5];
          versionConstant[0] = BDM_RC_OK; 
          versionConstant[1] = VERSION_SW;      // BDM SW/HW version
          versionConstant[2] = VERSION_HW;
          versionConstant[3] = ICP_Version_SW;  // ICP SW/HW version
          versionConstant[4] = ICP_Version_HW;
          usb_ep0_dcntT = sizeof(versionConstant); // # of bytes to return
          usb_dptr      = versionConstant;         // Point to data to return to the host
          USB_ep0_tx();                            // Start transmitting the data
          return;
        }
        else if ((*(SetupPacket*)&UE0D0).bRequest == CMD_USBDM_GET_COMMAND_RESPONSE) {
           // This is a transfer to obtain the command response.
           usb_ep0_dcntT=responseSize;
           if (usb_ep0_dcntT>(*(SetupPacket*)&UE0D0).wLength.le.lo)  // Don't send too much data 
              usb_ep0_dcntT = (*(SetupPacket*)&UE0D0).wLength.le.lo; 
           usb_dptr = commandBuffer; // Point to data to return to the host
           USB_ep0_tx();             // Start transmitting the data
           return;
        }
        else {
           // transfer 5 bytes + transfer size from the status frame to the buffer
           commandBuffer[1]=UE0D1;  /* cmd */
           commandBuffer[2]=UE0D2;  /* data0 */
           commandBuffer[3]=UE0D3;  /* data1 */
           commandBuffer[4]=UE0D4;  /* data2 */
           commandBuffer[5]=UE0D5;  /* data3 */
           usb_ep0_dcntT=commandExec();
           if (usb_ep0_dcntT>(*(SetupPacket*)&UE0D0).wLength.le.lo)  // Don't send too much data 
              usb_ep0_dcntT = (*(SetupPacket*)&UE0D0).wLength.le.lo; 
           usb_dptr = commandBuffer; /* point to data to return to the host */
           USB_ep0_tx();             /* start transmitting the data (or send an empty status frame in case there is no data to transmit) */
        }
      } else {
        /* transfer dir host->device (it will be a command which writes data to the target & expect no return data) */
        if ((*(SetupPacket*)&UE0D0).bRequest == CMD_USBDM_ICP_BOOT) {
          // Turn on bootloader & reset CPU!
          forceICPReset();  // Reboots to ICP mode - doesn't return
        }
        /* transfer size is 5+length (5 first bytes are transferred in the setup frame) */
        responseSize = 0;  // Indicate no response available yet
        commandBuffer[1]=UE0D1;  /* cmd */
        commandBuffer[2]=UE0D2;  /* data0 */
        commandBuffer[3]=UE0D3;  /* data1 */
        commandBuffer[4]=UE0D4;  /* data2 */
        commandBuffer[5]=UE0D5;  /* data3 */
        usb_ep0_dcntR=(*(SetupPacket*)&UE0D0).wLength.le.lo;  /* the host MUST NOT send more data than will fit into the buffer!! */
        if (usb_ep0_dcntR==0) {
          /* no more data to follow */
          USB_ep0_rx(); /* the routine will not copy any data, but will launch command execution and status frame transmit */
                        /* this is cleaner as this code is in one place only */
          return;
        }
        /* data to follow, set-up pointer */  
        usb_dptr = commandBuffer+6;  /* first 5 bytes stored already starting from position 1 */
      }
      return;
    }
    /* non-standard or unsupported request */
    UCR3 |= UCR3_ISTALL0_MASK | UCR3_OSTALL0_MASK;   /* stall EP0 communication and wait for new setup */
  }
}

/* USB interrupt service routine */
void interrupt usb_isr(void) {
  suspend_timer=0;              /* keep the device out of stop */
  if (UIR1_EOPF) {
    usb_1ms_tick();             /* call timing routine */
    UIR2_EOPFR = 1;             /* clear the flag */
  } else {
    if (led_timer<255) led_timer++; /* interrupt request has a different reason, perform correction of LED tick (it is not going to be accurate, but this is better than nothing) */
    if (UIR1_RXD0F) {           /* EP0 Rx complete */
      UCR0_RX0E=0;              /* deactivate receiver */
      if (USR0_SETUP) {         /* was it a setup frame? */
        USB_Setup();            /* handle the setup frame */        
      } else {                  /* other EP0 Rx */
        if (USR0_RP0SIZ) USB_ep0_rx();  /* EP0 receives either zero-length status frames or valid data */ 
      }
      UIR2_RXD0FR = 1;          /* clear the flag and enable further reception */
      UCR0_RX0E=1;              /* reactivate receiver */
    } else if (UIR1_TXD0F) {    /* EP0 Tx complete */
      USB_ep0_tx();             /* handle Tx on EP0 */
      UIR2_TXD0FR = 1;          /* clear the flag */
    } else if (UIR1_RXD2F) {    /* EP2 Rx complete */
      UCR2_RX2E=0;              /* deactivate receiver */
      USB_ep2_rx();             /* receive the data */ 
      UIR2_RXD2FR = 1;          /* clear the flag and enable further reception */
      UCR2_RX2E=1;              /* reactivate receiver */
    } else if (UIR1_TXD2F) {    /* EP2 Tx complete */
      USB_ep2_tx();             /* handle Tx on EP0 */
      UIR2_TXD2FR = 1;          /* clear the flag */
    } else if (UIR1_RSTF) {     /* bus reset */
      usb_init();               /* soft reset the USB interface */
      USB_State = US_DEFAULT;   /* change the state to reflect the reset has occured */
      UCR0 = UCR0_RX0E_MASK;    /* enable EP0 reception */
      UIR2_RSTFR = 1;           /* clear the flag */
    } else if (UIR1_RESUMF) {   /* resume interrupt */
      UIR0_SUSPND = 0;          /* clear the suspend flag */
      UIR2_RESUMFR = 1;         /* clear the interrupt flag */
      usb_ep0_dcntT=0xff;       /* special meaning - means no transmission in progress */
      usb_ep0_dcntR=0;          /* no data to receive */
      usb_ep2_dcntT=0;          /* no data to transmit */
      usb_ep2_dcntR=0;          /* no data to receive */
         led_timer=0;
      led_state=LED_ON;
    }
  }
}

/* initialise the USB peripheral */
void usb_init(void) {
  usb_ep0_dcntT=0xff;       /* special meaning - means no transmission in progress */
  usb_ep0_dcntR=0;          /* no data to receive */
  usb_ep2_dcntT=0;          /* no data to transmit */
  usb_ep2_dcntR=0;          /* no data to receive */
  led_timer=0;
  led_state=LED_ON;
  LED_INIT();               /* usb interrupt drives the LED, so make sure it is initialised */

#if USBPUP == PUP_AUTO
  // Auto detect if there is an external 1k5 PUP on PTE4/USBDM
  DDRE  |=  (1<<4); // Take PTE.4 low briefly (in case floating high)
  PTE   &= ~(1<<4);
  DDRE  &= ~(1<<4); // Float PTE.4 (PTE.4 is now an input to sample)
  asm {
    nop // Wait for PUP to pull PTE.4 high if present
    nop
  }
  if (PTE_PTE4 == 0) { // Not pulled high?
     UCR3 = UCR3_TX1STR_MASK|UCR3_PULLEN_MASK; // clear TX1ST & enable internal pull-up
  }
  else {
     UCR3 = UCR3_TX1STR_MASK;                   // clear TX1ST & disable internal pull-up
  }

#elif USBPUP == PUP_ON 
     UCR3 = UCR3_TX1STR_MASK|UCR3_PULLEN_MASK; // clear TX1ST & enable internal pull-up
#elif USBPUP == PUP_OFF  
     UCR3 = UCR3_TX1STR_MASK;                   // clear TX1ST & disable internal pull-up
#endif

  UIR0 = UIR0_RXD2IE_MASK | UIR0_RXD0IE_MASK | UIR0_TXD2IE_MASK | UIR0_TXD0IE_MASK | UIR0_EOPIE_MASK;  /* enable Rx&Tx interupts on EP0 & EP2, enable end of frame interrupt */
  UCR0 = 0;                 /* reset EP0 */
  UCR1 = 0;                 /* reset EP1 */
  UCR2 = 0;                 /* reset EP2 */
  UCR4 = 0;                 /* normal operation */

  UADDR = UADDR_USBEN_MASK; /* enable the USB module, assign address to the default value (0) */  

  USB_State = US_POWERED;	 /* must be powered when running this code... */
}
