/*! \file
    \brief Minimum USB stack & Flash programing routines for BOOT area
   
    \warning
    It is very important that this code does not use any library
    routines as they will end up in the USER ROM area which may be replaced. \n
    This code must be stand-alone.

    \warning
    This code allows the Flash memory to be read!  It is not suitable for
    devices where security of the Flash memory is of concern.  

   \verbatim
    JMxx ICP Code
    
    Copyright (C) 2008  Peter O'Donoghue

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
Change History
+================================================================================
|  12 Oct  2011 | Changed reboot to after USB transaction complete - pgo - ICP v2.5
|   8 Oct  2011 | Added wait for xtal stabilisation                - pgo - ICP v2.4
|               | Changed 8MHz auto threshold for JMxx devices     - pgo - ICP v2.4
|  30 Sep  2011 | Added crystal frequency auto-detection           - pgo - ICP v2.3
|  11 Sep  2011 | Changed ep0StartSetupTransaction() to use DATA1  - pgo - ICP v2.2
|   7 Aug  2011 | Rework to remove subtle buffer error             - pgo - ICP v2.0
|   7 Aug  2011 | Changed USB flag handling                        - pgo - ICP v1.7
|  17 Apr  2009 | Added USB std. req. GET_INTERFACE                - pgo - ICP v1.4
|   4 Feb  2009 | Made GET_VERSION more compatible                 - pgo 
|  23 July 2008 | Started coding                                   - pgo
+================================================================================
 \endverbatim
*/
#include <hidef.h>      // For EnableInterrupts macro
#include <stdio.h>

#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "USBDefs.h"
#include "ICP.h"
#include "ICP_USB.h"
#ifdef ICP_DEBUG
#include "SCI_Debug.h"
#endif

//! Maximum packet sizes for each endpoint & data transfer
#define ENDPT0MAXSIZE    (64) //!< Size of Endpoint #

#pragma DATA_SEG BOOT_RAM2
//! 
static U8 dataBuffer[ENDPT0MAXSIZE];

#pragma CONST_SEG BOOT_CONST
#ifndef ICP_DEBUG
#pragma CODE_SEG  BOOT_ROM
#endif
#pragma DATA_SEG __SHORT_SEG BOOT_RAM

// Return codes
// Note - can't be an enum as used in ASM code
#define ICP_RC_OK            (0) //!< No error
#define ICP_RC_ILLEGAL       (1) //!< Illegal operation
#define ICP_RC_FLASH_ERR     (2) //!< Flash programming error
#define ICP_RC_VERIFY_ERR    (3) //!< Flash verify error

//=================================================================================
// LED Port bit masks
// This supports LEDs on pins PTF.4 and PTF.5
//
#ifndef GREEN_LED_MASK
#define GREEN_LED_MASK  (PTFD_PTFD4_MASK) //!< GREEN LED Pin
#define RED_LED_MASK    (PTFD_PTFD5_MASK) //!< RED LED Pin
#define LED_PORT_DATA   (PTFD)            //!< Data Port for LEDs
#define LED_PORT_DDR    (PTFDD)           //!< DDR Port for LEDs
#endif

#pragma MESSAGE DISABLE C4301 // Disable warnings about Inline functions

//======================================================================
//! Structure representing a BDT entry in USB controller
typedef struct {
   union {
      struct {
         U8           :2;  
         U8 bdtkpid   :4;   //!< PID
         U8 data0_1   :1;   //!< data 0/1 toggle
         U8 own       :1;   //!< BDT ownership
      } a;
      struct {
         U8           :2;  
         U8 bdtstall  :1;  //!< Endpoint in stalled
         U8 dts       :1;  //!< ??
         U8           :2;  
         U8 data0_1   :1;  //!< data 0/1 toggle
         U8 own       :1;  //!< BDT ownership
      } b;
      U8 bits;
   } control;      //!< Control value
   U8 byteCount;   //!< Number of bytes in transfer
   U8 epAddr;      //!< Address of buffer offset by two bits 
} BDTEntry;

#define BDTEntry_OWN_MASK        (1<<7) //!< BDT Ownership
#define BDTEntry_DATA0_MASK      (0<<6) //!< DATA0 
#define BDTEntry_DATA1_MASK      (1<<6) //!< DATA1
#define BDTEntry_DTS_MASK        (1<<3) //!< DTS ??
#define BDTEntry_BDTSTALL_MASK   (1<<2) //!< Stall endpoint

//======================================================================
// USB RAM usage
//
#define USB_RAM_START (0x1860U)  //!< Location of RAM Endpoint buffer 
//! MAP a physical address to endpoint address as used BDT 
#define USB_MAP_ADDRESS(x)  ((((x) - USB_RAM_START)>>2)&0xFC)
//! Round address value to 16-byte value
#define ADDRESS_ROUND16(x)  ((x+0xF)&0xFFF0)

//======================================================================
// BDTs for the endpoints (in USB RAM)
//
static BDTEntry bdts[10]    @USB_RAM_START;
#define  ep0BDTIn    bdts[0] //!< Endpoint #0 
#define  ep0BDTOut   bdts[1] //!< Endpoint #1

//======================================================================
// Buffers for endpoint data packets (in USB RAM)

//! EP0 IN data buffer
#define EP0InDataBufferAddress ADDRESS_ROUND16(USB_RAM_START+0x20)
static U8 ep0InDataBuffer[ENDPT0MAXSIZE]  @EP0InDataBufferAddress;
//! EP0 OUT data buffer
#define EP0OutDataBufferAddress ADDRESS_ROUND16(EP0InDataBufferAddress+ENDPT0MAXSIZE)
static U8 ep0OutDataBuffer[ENDPT0MAXSIZE] @EP0OutDataBufferAddress;

//! All of the USB 2-port RAM
static volatile U8 usbRam[256] @USB_RAM_START;

//======================================================================
// Data packet odd/even indicator
enum {DATA0=0, //!< DATA0 indicator
      DATA1=1  //!< DATA1 indicator
     };

//======================================================================
// Descriptors
//
static const DeviceDescriptor deviceDescriptor = {
   sizeof(DeviceDescriptor),               // bLength
   DT_DEVICE,                              // bDescriptorType
   CONST_NATIVE_TO_LE16(0x0110),           // bcdUSB 			     = USB spec rel. No.[BCD = 1.10]
   0xFF,                                   // Class code             [none]
   0xFF,                                   // Sub Class code         [none]
   0xFF,                                   // Protocol               [none]
   ENDPT0MAXSIZE,                          // bMaxPacketSize0 	  = EndPt 0 max packet size
   CONST_NATIVE_TO_LE16(VendorID),         // idVendor 			  = Vendor ID
   CONST_NATIVE_TO_LE16(ProductID),        // idProduct 	      = Product ID
   CONST_NATIVE_TO_LE16(0x0100),           // bcdDevice 	      = Device Release [BCD = 1.00]
   0,                                      // iManufacturer 	  = String index of Manufacturer name
   0,                                      // iProduct 			  = String index of product desc.
   0,                                      // iSerialNumber 	  = String index desc. serial #
   1                                       // bNumConfigurations = Number of configurations
};

static const struct {
   ConfigurationDescriptor configDescriptor;
   InterfaceDescriptor     interfaceDescriptor;
} otherDescriptors = 
{
   { // configDescriptor
      sizeof(ConfigurationDescriptor),                // bLength
      DT_CONFIGURATION,                               // bDescriptorType
      CONST_NATIVE_TO_LE16(sizeof(otherDescriptors)), // wTotalLength
      1,                                              // bNumInterfaces
      1,                                              // bConfigurationValue
      0,                                              // iConfiguration
      0x80,                                           // bmAttributes        = Bus powered, no wakeup (yet?)
      USBMilliamps(200)                               // MaxPower
   },
   { // interfaceDescriptor
      sizeof(InterfaceDescriptor),  // bLength
      DT_INTERFACE,                 // bDescriptorType
      0,                            // bInterfaceNumber
      0,                            // bAlternateSetting
      0,                            // bNumEndpoints
      0xFF,                         // bInterfaceClass      = (Vendor specific)
      0xFF,                         // bInterfaceSubClass   = (Vendor specific)
      0xFF,                         // bInterfaceProtocol   = (Vendor specific)
      0                             // iInterface desc
   },
};
#ifdef BOOTLOADER
#define BOOT_USE_WDIC
#endif
#ifdef BOOT_USE_WDIC
static const MS_CompatibleIdFeatureDescriptor msCompatibleIdFeatureDescriptor = {
	/* lLength;             */  CONST_NATIVE_TO_LE32((uint32_t)sizeof(MS_CompatibleIdFeatureDescriptor)),
	/* wVersion;            */  CONST_NATIVE_TO_LE16(0x0100),
	/* wIndex;              */  CONST_NATIVE_TO_LE16(0x0004),
	/* bnumSections;        */  1,
	/* bReserved1[7];       */  {0},
	/* bInterfaceNum;       */  0,
	/* bReserved2;          */  1,
	/* bCompatibleId[8];    */  "WINUSB\0",
	/* bSubCompatibleId[8]; */  {0},
	/* bReserved3[6];       */  {0}  
};
#pragma MESSAGE DISABLE C3303 //  Implicit concatenation of strings

static const MS_PropertiesFeatureDescriptor msPropertiesFeatureDescriptor = {
	/* U32 lLength;         */ CONST_NATIVE_TO_LE32((uint32_t)sizeof(MS_PropertiesFeatureDescriptor)),
	/* U16 wVersion;        */ CONST_NATIVE_TO_LE16(0x0100),
	/* U16 wIndex;          */ CONST_NATIVE_TO_LE16(0x0005),
	/* U16 bnumSections;    */ CONST_NATIVE_TO_LE16(0x0001),
	/* U32 lPropertySize;   */ CONST_NATIVE_TO_LE32(132UL),
	/* U32 ldataType;       */ CONST_NATIVE_TO_LE32(1UL),
	/* U16 wNameLength;     */ CONST_NATIVE_TO_LE16(40),
	/* U8  bName[40];       */ "D\0e\0v\0i\0c\0e\0I\0n\0t\0e\0r\0f\0a\0c\0e\0G\0U\0I\0D\0\0",
	/* U32 wPropertyLength; */ CONST_NATIVE_TO_LE32(78UL),
	/* U8  bData[78];       */ "{\000"  
	                           "9\0003\000F\000E\000B\000D\0005\0001\000"
	                           "-\0006\0000\0000\0000\000"
	                           "-\0004\000E\0007\000E\000"
	                           "-\000A\0002\0000\000E\000"
	                           "-\000A\0008\0000\000F\000C\0007\0008\000C\0007\000E\000A\0001\000"
	                           "}\000"
};
#pragma MESSAGE DEFAULT C3303 //  Implicit concatenation of strings

#define VENDOR_CODE 0x30
static const U8 OS_StringDescriptor[] = {18, DT_STRING, 'M',0,'S',0,'F',0,'T',0,'1',0,'0',0,'0',0,VENDOR_CODE,0x00};

#endif
//===============================================================================
// Device Status
//       
typedef struct {
   int selfPowered  : 1;	//!< Device is self-powered
   int remoteWakeup : 1;    //!< Supports remote wakeup
   int portTest     : 1;    //!< Port test
   int res1         : 5;    //!< Reserved
   int res2         : 8;    //!< Reserved
} DeviceStatus; 

static const DeviceStatus deviceStatus = {0,0,0,0,0};

//! Device State
typedef struct {
   uint8_t           configuration;  //!< Configured state
   uint8_t           newUSBAddress;  //!< Used to hold new device address
} DeviceState;

//===============================================================================
// Endpoint Status
//       
typedef struct {
   int stall  : 1;   //!< Endpoint is stalled
   int res1   : 7;   //!< Reserved
   int res2   : 8;   //!< Reserved
} EPStatus;

// Endpoint state values
typedef enum { 
   EPIdle = 0,       //!< Idle (Tx complete)
   EPLastIn,         //!< Doing the last IN packet
   EPStatusIn,       //!< Doing an IN packet as a status handshake
   EPLastOut,        //!< Doing the last OUT packet
   EPStatusOut,      //!< Doing an OUT packet as a status handshake
} EPModes;

//! Endpoint information
typedef struct {
   EPModes   state:8;           //!< State of end-point
   U8       (*callback)(void);  //!< Call-back to execute at end of transfer 
} EPState;

static const EPState initialEPState = {EPIdle, NULL};

static SetupPacket ep0SetupBuffer;   //!< Buffer for EP0 Setup packet (copied from USB RAM)
static DeviceState deviceState;      //!< State of USB state machine
static EPState     ep0State;         //!< State of end-point \#0

// Flash Programming variables
static U8  dataLength;      //!< Length of data portion of USB pkt (also \#of bytes to Flash)
static U16 startAddress;    //!< Start address of area in Flash to program or verify
static U16 sourceAddress;   //!< Address in buffer for programming/verifying
static U8  flashCommand;    //!< Flash command code
static U8  flashData;       //!< Byte to program to Flash
static U8  commandStatus;   //!< Status of last command executed

/*! Copy a range of memory
 *
 * @param from  source ptr
 * @param to    destination ptr
 * @param size  number of bytes to copy
 */
static void myMemcpy(U8 *to, const U8 *from, U8 size) {
   while (size-->0) {
      *to++ = *from++;
   }
}

typedef struct {
   uint8_t mcgC1; 
   uint8_t mcgC3; 
} ClockFactors;

enum {clksel_4MHz, clksel_8MHz, clksel_12MHz};

static const ClockFactors clockFactors[] = {
      { // 4 MHz
        (1<<MCGC1_RDIV_BITNUM),                   // RDIV = 1 -> 4MHz/2=2 MHz
        MCGC3_PLLS_MASK | (6<<MCGC3_VDIV_BITNUM)  // VDIV = 6 -> multiply by 24 -> 2MHz * 24 = 48MHz
      },
      { // 8 MHz
        (2<<MCGC1_RDIV_BITNUM),                   // RDIV = 2 -> 8MHz/2=2 MHz
        MCGC3_PLLS_MASK | (6<<MCGC3_VDIV_BITNUM)  // VDIV = 6 -> multiply by 24 -> 2MHz * 24 = 48MHz
      },
      { // 12 MHz
        (3<<MCGC1_RDIV_BITNUM),                   // RDIV = 3 -> 12MHz/8=1.5 MHz
        MCGC3_PLLS_MASK | (8<<MCGC3_VDIV_BITNUM)  // VDIV = 8 -> multiply by 32 -> 1.5MHz * 32 = 48MHz
      }
};
/*!  Sets up the clock for USB operation
 *
 * MGCOUT = 48MHz, BUS_CLOCK = 24MHz, (PEE mode)
 *
 * Modes: FEI [FLL engaged internal] -> 
 *        FBE [FLL bypassed external] ->
 *        PBE [PLL bypassed external] ->
 *        PEE [PLL engaged external]
 *
 * Refer 12.5.2.1 of MC9S08JM60 Reference
 * 
 * @param clksel - parameters for external crystal used
 * 
 */
static void initCrystalClock(const ClockFactors clockFactors) {
   // Out of reset MCG is in FEI mode
   
   // 1. Switch from FEI (FLL engaged internal) to FBE (FLL bypassed external)
   // 1 a) Set up crystal 
   MCGC2 =                       // BDIV = 0, divide by 1
            MCGC2_HGO_MASK       // oscillator in high gain mode
          | MCGC2_EREFS_MASK     // because crystal is being used
          | MCGC2_RANGE_MASK     // 12 MHz is in high freq range
          | MCGC2_ERCLKEN_MASK;  // activate external reference clock
   // 1 c) Select clock mode
   MCGC1 =   (2<<MCGC1_CLKS_BITNUM) |  // CLKS = 10 -> External reference clock
			 clockFactors.mcgC1;       // RDIV = ?  -> xtal/? -> 2/1.5MHz
   
   // 1 b) Wait for crystal to start up, mode change & ext ref to MCGOUT        
   // 1 d) Wait for mode change
   // 1 e) Wait for MCGOUT indicating that the external reference to be fed to MCGOUT
    while ((MCGSC & (MCGSC_IREFST_MASK|MCGSC_OSCINIT_MASK|    MCGSC_CLKST_MASK)) != 
                    (        0        |MCGSC_OSCINIT_MASK|(2<<MCGSC_CLKST_BITNUM))) {
    }
   // 2. Switch FBE (FLL bypassed external) to PBE (PLL Bypassed External) mode
   // 2 b) Set RDIV for correct range
   // 2 c) Enable the PLL & set VDIV value
   MCGC3 = clockFactors.mcgC3;        // VDIV=? -> multiply by 24/32 -> 2/1.5MHz * 24 = 48MHz */

   // 2 e) Wait until PLLS clock source changes to the PLL
   while (MCGSC_PLLST == 0) {
   }
   // 2 f)  Wait for PLL to acquired lock
   while (MCGSC_LOCK == 0) {
   }
   // 3. PBE mode transitions into PEE mode:
   // 3 a) Set RDIV for correct range and select PLL.FLL clock
   MCGC1 = clockFactors.mcgC1;

   // 3 c)  Wait for clock stable
   while (MCGSC_CLKST != 3) {
   }
  /* Now MCGOUT=48MHz, BUS_CLOCK=24MHz */  
}

// Clock Trim values in Flash - dummy value overwritten by flash programmer
extern const volatile U8 NV_MCGTRM_INIT @0x0000FFAF = 0x00;  // MSB

#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_FRAME
#pragma NO_RETURN
static void autoInitClock( void ) {
#define MCGC2_VALUE  ((0<<MCGC2_BDIV_BITNUM)|MCGC2_ERCLKEN_MASK|MCGC2_EREFS_MASK|MCGC2_RANGE_MASK|MCGC2_HGO_MASK) 
#define MCGC1_VALUE  ((0<<MCGC1_CLKS_BITNUM)|MCGC1_IREFS_MASK) 
#define MCGSC_MASK   (MCGSC_IREFST_MASK|MCGSC_OSCINIT_MASK|MCGSC_LOCK_MASK|    MCGSC_CLKST_MASK)
#define MCGSC_VALUE  (MCGSC_IREFST_MASK|MCGSC_OSCINIT_MASK|MCGSC_LOCK_MASK|(0<<MCGSC_CLKST_BITNUM))
#define RTCSC_VALUE  ((1<<RTCSC_RTCLKS_BITNUM)|(9<<RTCSC_RTCPS_BITNUM))
#define RTCMOD_VALUE (255)
   asm {
#if CPU == JMxx
      // Trim clock for JMxx, JS16 is factory trimmed
      lda NV_MCGTRM_INIT //  Trim only if value has been programmed to Flash.
      beq dontTrim       
      sta MCGTRM
   dontTrim:
#endif	   
      // Set approx. 16 MHz bus clock assuming 31.25 MHz internal clock
      mov #MCGC2_VALUE,MCGC2 // BDIV=0,RANGE=1,HGO=1,LP=0,EREFS=1,ERCLKEN=1,EREFSTEN=0
      mov #MCGC1_VALUE,MCGC1 // CLKS=00,RDIV=0,IREFS=1,IRCLKEN=0,IREFSTEN=0
      
      // Wait for clock mode change
   clkLoop:
      lda MCGSC
      and #MCGSC_MASK
      cmp #MCGSC_VALUE
      bne clkLoop
      
      // RTC driven by xtal
      // RTC will roll-over as follows
      // xtal       Approx. time
      //  4 MHz      128 ms 
      //  8 MHz       64 ms  (91)
      // 12 MHz       43 ms  (52)
      mov #RTCSC_VALUE,RTCSC
      mov #RTCMOD_VALUE,RTCMOD

      // Count is in 1ms units
      clra 					// A = count in ms
   oloop:
      // ~1 ms
      clrh
      clrx
   loop:
      aix    #1             // 2 cy
      cphx   #2000          // 3 cy
      bne    loop           // 3 cy => 8 cy @16MHz => 500 ns per iteration
      inca                  // Count ms
      brclr  7,RTCSC,oloop  // Check RTC roll-over
      clr    RTCSC          // Stop RTC

      // JS16 & JMxx can auto-detect 4MHz without trimming
      ldhx   clockFactors   // clksel_4MHz
      cmp    #93
      bhi    skip
#if CPU == JMxx
      ldx    NV_MCGTRM_INIT //  JMxx cannot reliably detect 8MHz if not trimmed
      beq    is12MHz        //  Assume 12MHz
#endif
      ldhx   clockFactors:2 // clksel_8MHz
      cmp    #52
      bhi    skip
   is12MHz:
      ldhx   clockFactors:4 // clksel_12MHz
   skip:
      jmp    initCrystalClock
   }
}

//======================================================================
//! Configure EP0 for an OUT transaction [Rx, device<-host, DATA0/1]
//!
//! @param bufSize - Size of data to transfer
//! @param data0_1 - DATA0/DATA1 toggle value
//!
static void ep0StartOutTransaction(U8 data0_1) {
   ep0State.state = EPLastOut;       // Assume one and only data pkt
                                     // This may be changed by caller
   // Set up to Rx packet
   ep0BDTOut.epAddr      = USB_MAP_ADDRESS(EP0OutDataBufferAddress);
   ep0BDTOut.byteCount   = ENDPT0MAXSIZE;
   if (data0_1) { 
      ep0BDTOut.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
//      debugTx('X');
   }
   else {
      ep0BDTOut.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
//      debugTx('x');
   }
}

//======================================================================
//! Configure EP0 for a SETUP transaction [Rx, device<-host, DATA0]
//!
static void ep0StartSetupTransaction(void) {
   // DATA1 is used in case actually status OUT pkt (SETUP ignores toggle)
   ep0StartOutTransaction(DATA1);
   ep0State.state = EPIdle;         // Now Idle
}

//======================================================================
//! Configure EP0 for an IN transaction (Rx, device -> host, DATA1 [assuming single DATA pkt]
//!
static void ep0StartInTransaction( U8 dataSize, const U8 *dataPtr ) {
   // Assumes size < 8=bits!
   if (dataSize > ep0SetupBuffer.wLength.le.lo) {// Truncate if more bytes available than asked for
	  dataSize = ep0SetupBuffer.wLength.le.lo;
   }
   myMemcpy(ep0InDataBuffer, dataPtr, dataSize);

   // Set up to Tx packet
   ep0BDTIn.epAddr        = USB_MAP_ADDRESS(EP0InDataBufferAddress);
   ep0BDTIn.byteCount     = dataSize;
   // All IN pkts are DATA1
   ep0BDTIn.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   ep0State.state         = EPLastIn;     // The one and only data pkt
}

//======================================================================
// Configure EP0 for an Empty [Status] IN transaction [Tx, device->host, DATA1]
//
static void ep0StartStatusInTransaction(void) {
   ep0StartInTransaction( 0, NULL); // Do status Pkt transmission
   ep0State.state = EPStatusIn;     // Doing status handshake
}

//======================================================================
// Stall control for endpoints
//

//!==========================================
//! Stall ep0 Tx
//!
#pragma INLINE
static void ep0Stall(void) {
   ep0BDTIn.control.bits  = BDTEntry_OWN_MASK|BDTEntry_BDTSTALL_MASK;
}

//========================================================================================
//
#pragma INLINE
static void setUSBdefaultState( void ) {
   ADDR                       = 0;
   deviceState.configuration  = 0;
}

#pragma INLINE
static void setUSBaddressedState( U8 address ) {
   ADDR                       = address;
   deviceState.configuration  = 0;
}

#pragma INLINE
static void setUSBconfiguredState( U8 config ) {
   deviceState.configuration = config;
   GREEN_LED_OFF();
   if (config != 0) { 
	  // Configured
      GREEN_LED_ON();
   }
}

//==================================================================
// Handler for USB Bus reset
// 
static void handleUSBReset(void) {
   setUSBdefaultState();
   ep0State    = initialEPState;

   ERRSTAT = 0xFF;                 // Clear USB error flags

   // Clear all USB interrupt flags
   INTSTAT = INTSTAT_STALLF_MASK|INTSTAT_RESUMEF_MASK|INTSTAT_SLEEPF_MASK| 
             INTSTAT_TOKDNEF_MASK|INTSTAT_SOFTOKF_MASK|INTSTAT_ERRORF_MASK|INTSTAT_USBRSTF_MASK;

   // Set up to receive 1st SETUP packet
   ep0StartSetupTransaction();
 
   // Enable endpoint #0
   EPCTL0  = EPCTL0_EPRXEN_MASK|EPCTL0_EPTXEN_MASK|EPCTL0_EPHSHK_MASK;
}

//===============================================================================
// Get Descriptor - Device Req 0x06
//       
static void handleGetDescriptor( void ) {
   // Static variables used to reduce size (ZPAGE) 
   static U8          dataSize;
   static const char *dataPtr;

   switch (ep0SetupBuffer.wValue.le.hi) {
      case DT_DEVICE: // Get Device Desc. - 1
         dataPtr  = (U8 *) &deviceDescriptor;
         dataSize = (U8)deviceDescriptor.bLength;
         break;
      case DT_CONFIGURATION: // Get Configuration Desc. - 2
         dataPtr  = (U8 *) &otherDescriptors;
         dataSize = (U8)(otherDescriptors.configDescriptor.wTotalLength>>8);
         break;
#ifdef BOOT_USE_WDIC
      case DT_STRING: // Get String Desc.- 3
         dataPtr  = OS_StringDescriptor;
         dataSize = *dataPtr;
         break;
#endif
      default:        // shouldn't happen
         ep0Stall();
         return;
      } // switch
   ep0StartInTransaction( dataSize, dataPtr ); // Set up Tx
}

//===============================================================================
// Set device Address Callback
//       
static U8 setAddressCallback(void) {
   setUSBaddressedState(deviceState.newUSBAddress);
   return 0;
}

#pragma MESSAGE DISABLE C1404  // Disable warnings about Return expected
#pragma MESSAGE DISABLE C20001 // Disable warnings about stackpointer
#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
/*!   Low-level Flash programming code

      This code is copied to the stack [RAM] for execution as Flash memory
      cannot be used while being programmed.
      
      @note Global parameters 
       - dataLength       =>   Number of bytes to process (must be >0!)
       - startAddress     =>   Start address of area in Flash to process 
       - sourceAddress    =>   Data to use

      @return Final value of FSTAT
      
      @warning - If the size of this routine changes then the constant 
                  ONSTACK_SIZE must be corrected
*/
static U8 onStack(void) {
   asm {
   loop:
      ldhx  sourceAddress        // Get next byte to program
      mov   x+,flashData         //    & increment source address
      sthx  sourceAddress
      
   chkBusy:
      lda   FSTAT                // Loop if not ready to buffer next command
      bit   #FSTAT_FCBEF_MASK
      beq   chkBusy

      bit   #FSTAT_FACCERR_MASK|FSTAT_FPVIOL_MASK;   // Check for any errors
      bne   flashExit
      
   intoLoop:      
      ldhx  startAddress         // Write to flash, latch addr and data
      mov   flashData,x+         //    & increment Flash write address
      sthx  startAddress
      
      lda   flashCommand         // Write the flash command
      sta   FCMD 
      
      lda   #FSTAT_FCBEF_MASK    // Initiate command
      sta   FSTAT
      
      dbnz  dataLength,loop      // More bytes? - yes - loop
      
   chkDone: 
      lda   FSTAT                // Loop if command not complete
      bit   #FSTAT_FCCF_MASK
      beq   chkDone              
      
   flashExit:   
      rts
   }
}

#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
/*!   Low-level Flash programming code entry point

      Does the following:\n
      - Initialises sourceAddress 
      - Clears any current Flash errors
      - Sets Flash Clock divider
      - Copies the \ref onStack[] routine to the stack and executes it.
            
      @return \n
         - \ref ICP_RC_OK        - Success \n
         - \ref ICP_RC_FLASH_ERR - Programming or erasing failed.
*/
static U8 doOnStack(void) {
#define ONSTACK_SIZE (0x2C)  // #@doOnStack-@onStack 
   asm {
      lda   #FCDIV_PRDIV8_MASK|14  // Initialise Flash clock divider
      sta   FCDIV                  // Assumes 24MHz bus clock (req. for USB!)

      lda   #FSTAT_FACCERR_MASK|FSTAT_FPVIOL_MASK;   // Clear any errors
      sta   FSTAT
      
      ldhx  @dataBuffer            // Set up data source address
      sthx  sourceAddress

      // Copy routine onto stack (RAM)
      //
      ldhx  #@doOnStack       // End of range+1
   pshLoop:
      aix   #-1               // push byte on stack
      lda   ,x
      psha                    
      cphx  #@onStack         // c.f. Start of range
      bne   pshLoop
      
      tsx                     // Execute the routine on the stack
      jsr   ,x
      ais   #ONSTACK_SIZE     // Clean up stack
      
      bit   #FSTAT_FACCERR_MASK|FSTAT_FPVIOL_MASK;   // Check for any errors
      bne   errorExit

   okExit:      
      clra  // lda #ICP_RC_OK
      rts
      
   errorExit:
      lda   #ICP_RC_FLASH_ERR
      rts
   }
}

#pragma MESSAGE DEFAULT C1404  // Restore warnings about Return expected
#pragma MESSAGE DEFAULT C20001 // Restore warnings about stackpointer

/*!   Erase a Page in Flash Memory
   @note Global parameters 
   - startAddress     Any address within Flash Page to be erased
*/
static U8 erasePageCommand(void) {
   flashCommand = mPageErase;    // Page/sector erase command
   dataLength   = 1;             // A single command
   return doOnStack();
}

/*!   Program a Row in Flash memory
   @note Global parameters 
   - dataLength        =>  Number of bytes to program [must be >0]
   - startAddress      =>  Start address of area in Flash to program 
   - ep0OutDataBuffer  =>  Data to program
*/
static U8 programRowCommand(void) {
   flashCommand  = mBurstProg;   // Burst program command
   return doOnStack();
}

#pragma MESSAGE DISABLE C1404  // Restore warnings about Return expected
#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
/*!   Verify a Range of Flash memory

   @note Global parameters 
   - dataLength        =>  Number of bytes to verify [must be >0]
   - startAddress      =>  Start address of area in Flash
   - ep0OutDataBuffer  =>  Data to verify against
*/
static U8 verifyRowCommand(void) {
//    DEBUG_PIN     = 0;
//    debugTx('v'); // Verify complete

   asm {
      ldhx  #@dataBuffer
      sthx  sourceAddress
      
   Loop:
      ldhx  sourceAddress
      lda   ,x
      aix   #1
      sthx  sourceAddress
      ldhx  startAddress
      cbeq  ,x+,okByte
      lda   #ICP_RC_VERIFY_ERR
      rts

   okByte:
      sthx  startAddress
      
   IntoLoop:   
      dbnz  dataLength,Loop
      lda   #ICP_RC_OK
      rts
   }
}

//! Constant describing the Hardware and Software
static const uint8_t versionConstant[] = {
   BDM_RC_OK,                   //!< Success transaction
   0xFF, VERSION_HW,            //!< BDM code version unknown
   ICP_VERSION_SW, VERSION_HW   //!< ICP SW/HW version
};                                     

//==================================================================
// Handlers for Token Complete 
//

//================
// ep0 - IN
// 
static void handleEp0InToken(void) {

   switch (ep0State.state) {
      case EPLastIn:    
         // Just done the last IN packet
         ep0State.state = EPStatusOut;     // Receiving an OUT status pkt
         break;
         
      case EPStatusIn:  
         // Just done an IN packet as a status handshake for an OUT Data transfer
         if (ep0State.callback != NULL) {
            // Execute call-back function to process OUT data
            commandStatus = ep0State.callback();
         }
         ep0State.callback = NULL;
         ep0State.state    = EPIdle;
//         debugTx('0'+commandStatus);
         break;

      default:
         // We don't expect an IN token while in other states - discard
         break;
   }
}

//================
// ep0 - OUT
// 
static void handleEp0OutToken(void) {

   switch (ep0State.state) {
      case EPLastOut:                   
         // Done the last OUT packet
         myMemcpy(dataBuffer, ep0OutDataBuffer, dataLength); // Save data
         ep0StartSetupTransaction();      // Make ready for next SETUP pkt
         ep0StartStatusInTransaction();   // Do status Pkt transmission
         break;
      default:
         // Unexpected - discard
         ep0StartSetupTransaction();      // Make ready for next SETUP pkt
         break;
   }
}

//===============================================================================
//! Reboots the BDM
//!
//! - Detaches from the USB bus
//! - Waits around 1000 ms
//! - Resets the CPU
//!
#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_FRAME
#pragma NO_RETURN
U8 icpReset(void) {
#define USBCTL0_USBRESET_NUM (7)
#define ILLOP                (0x8D)
   asm {
	   // Reset USB interface - detach
       bset  USBCTL0_USBRESET_NUM,USBCTL0 

       // Wait a while for detach detection
       lda   #200   // 200 * 50us => 10 ms
   oloop:
       ldx   #200   // 200 * 250ns => 50 us  
   loop:
       dbnzx loop   // 4 cy @16MHz => 250 ns /iter
       dbnza oloop
       
	   // Reset the CPU
   reset:	   
	   dcb  ILLOP   // Illegal opcode - causes cpu reset!
   }
}

//===============================================================================
// Handles SETUP Packet
//       
static void handleSetupToken( void ) {
   
   // Save SETUP pkt
   myMemcpy((U8*)&ep0SetupBuffer, ep0OutDataBuffer, sizeof(ep0SetupBuffer));

   ep0State.callback = NULL;
   dataLength        = ep0SetupBuffer.wLength.le.lo;
   startAddress      = (ep0SetupBuffer.wValue.le.hi<<8)+ep0SetupBuffer.wValue.le.lo;
   ep0StartSetupTransaction();  // Default to expect another SETUP pkt
   
   if (IS_VENDOR_REQ(ep0SetupBuffer.bmRequestType)) {
      // Handle Vendor requests
#ifdef BOOT_USE_WDIC
      if (ep0SetupBuffer.bRequest == VENDOR_CODE) {
         if ((ep0SetupBuffer.wIndex.le.lo) == (0x0004)) { 
            ep0StartInTransaction( sizeof(msCompatibleIdFeatureDescriptor),  (uint8_t *)&msCompatibleIdFeatureDescriptor );
         }
         else if ((ep0SetupBuffer.wIndex.le.lo) == (0x0005)) { 
            ep0StartInTransaction( sizeof(msPropertiesFeatureDescriptor),  (uint8_t *)&msPropertiesFeatureDescriptor );
         }
         else {
             ep0Stall();
         }
      }
      else 
#endif
      if (ep0SetupBuffer.bRequest == ICP_PROGRAM_ROW) {
         ep0StartOutTransaction( DATA1 );         // Set up to get rest of command (data portion)
         ep0State.callback = programRowCommand;   // Set call-back to execute when data arrives
      } 
      else if (ep0SetupBuffer.bRequest == ICP_VERIFY_ROW) {
         ep0StartOutTransaction( DATA1 );         // Set up to get rest of command (data portion)
         ep0State.callback = verifyRowCommand;    // Set call-back to execute when data arrives
      } 
      else if (ep0SetupBuffer.bRequest == ICP_ERASE_PAGE) {
         commandStatus = erasePageCommand(); // Execute the command - no response
         ep0StartStatusInTransaction();      // Tx empty Status packet
      } 
      else if (ep0SetupBuffer.bRequest == ICP_GET_RESULT)  {
         ep0StartInTransaction( sizeof(commandStatus),  &commandStatus );
      } 
      else if (ep0SetupBuffer.bRequest == ICP_GET_VER)  {
         ep0StartInTransaction( sizeof(versionConstant),  versionConstant );
      } 
      else if (ep0SetupBuffer.bRequest == ICP_REBOOT)  {
         ep0State.callback          = icpReset;
         ep0StartStatusInTransaction();      // Tx empty Status packet
      } 
      else {
         ep0Stall();
      }
   }
   else  {
      // Handle standard USB requests
      if (ep0SetupBuffer.bRequest == GET_DESCRIPTOR) {
         handleGetDescriptor();
      }
      else if (ep0SetupBuffer.bRequest == GET_STATUS) {
         ep0StartInTransaction( sizeof(DeviceStatus), (U8 *) &deviceStatus );
      }
      else if (ep0SetupBuffer.bRequest == SET_ADDRESS) {
         // Save address for change after status transaction
         deviceState.newUSBAddress  = ep0SetupBuffer.wValue.le.lo; 
         ep0State.callback          = setAddressCallback;
         ep0StartStatusInTransaction(); // Tx empty Status packet
      }
      else if (ep0SetupBuffer.bRequest == GET_INTERFACE) {
         // No error checking!
         ep0StartInTransaction( 1, &otherDescriptors.interfaceDescriptor.bAlternateSetting);
      }
      else if (ep0SetupBuffer.bRequest == GET_CONFIGURATION) {
         ep0StartInTransaction( 1, &deviceState.configuration );
      }
      else if (ep0SetupBuffer.bRequest == SET_CONFIGURATION) {
         setUSBconfiguredState(ep0SetupBuffer.wValue.le.lo);
         ep0StartStatusInTransaction(); // Tx empty Status packet
      }
      else if (ep0SetupBuffer.bRequest == SET_INTERFACE) {
         ep0StartStatusInTransaction(); // Tx empty Status packet
      }
      else {
         ep0Stall();
      } 
   }
   // Allow transactions post SETUP pkt
   CTL_TSUSPEND = 0;
}

#ifdef ICP_DEBUG
static char buff[40];
#endif

//==================================================================
// Polling Loop for USB events
// 
// Determines source and dispatches to appropriate routine.
// Never returns
//
void startICP_USB( void ) {
//   static uint8_t 0intStatus;     
//   DEBUG_PIN_DDR = 1;

   //! Initialise the USB interface
   //!
   autoInitClock();
   
   // Clear USB RAM (includes BDTs)
   asm {
      ldhx   @usbRam
      clra   // 256 bytes
      loop:
      clr    ,x
      aix    #1
      dbnza  loop
   }
   // Reset USB   
   USBCTL0_USBRESET = 1;
   while (USBCTL0_USBRESET) {
   }
   // Enable USB module.
   CTL = CTL_USBEN_MASK;
   
   // Internal PUP, Internal 3V3 reg, Enable Txvr 
   USBCTL0 = USBCTL0_USBPU_MASK|USBCTL0_USBVREN_MASK|USBCTL0_USBPHYEN_MASK;
   
   // Disable error interrupts.   
   ERRENB = 0;
   
   // Disable all interrupts
   INTENB  = 0;
   
   // Set initial USB state
   handleUSBReset();
   
   for(;;) {
#ifdef ICP_DEBUG
      int idleCount;
      if ((ep0BDTOut.control.bits&BDTEntry_OWN_MASK) != 0) {
         idleCount=0;
         RED_LED_OFF();
      }
      else {
         if (idleCount++ > 10) {
            RED_LED_ON();
         }
      }
      int idleCount;
      DEBUG_PIN = 0;
      DEBUG_PIN = 1;
      DEBUG_PIN = 0;
      DEBUG_PIN = 1;
      if ((ep0BDTOut.control.bits&BDTEntry_OWN_MASK) != 0) {
         idleCount = 0;
      }
      else {
         if (idleCount<10) {
            debugTx('-');
            idleCount++;
         }
      }
      command = debugRx();
      if (command >= 0) {
         switch(command) {
         case 'R' :
         	CTL = 0x00;   // Disable USB
         	reset();      // Illegal opcode - causes cpu reset!
         	break;
         case 's' : ep0StartSetupTransaction();                   break;
         case 'c' : CTL_TSUSPEND = 0;                             break;
         case 't' : ep0ClearStall();                              break;
         case 'a' : CTL_TSUSPEND = 0; ep0StartSetupTransaction(); break;
         case 'i' :
         	(void)sprintf(buff, "\rep0BDTOut=0x%02X\r", ep0BDTOut.control.bits); debugPuts(buff);
         	(void)sprintf(buff,   "ep0BDTIn =0x%02X\r", ep0BDTIn.control.bits);  debugPuts(buff);
         	break;
         default: break;
         }
         continue;
      }
#endif
      if ((INTSTAT&INTSTAT_TOKDNEF_MASK) != 0) {
         uint8_t stat = STAT;
    	 INTSTAT = INTSTAT_TOKDNEF_MASK;
         if ((stat&STAT_ENDP_MASK) == (0<<STAT_ENDP_BITNUM)) {
            // EP # 0
            if (stat&STAT_IN_MASK) {
               // IN Transaction complete
               handleEp0InToken();
               // debugTx('I'); // IN Token
            }
            else if (ep0BDTOut.control.a.bdtkpid == SETUPToken) {
               // SETUP transaction complete
               handleSetupToken();
               //  debugTx('S'); // SETUP Token
            }
            else {
               // OUT Transaction complete
               handleEp0OutToken(); 
               // debugTx('O'); // OUT Token
            }
         }
      }
      else if ((INTSTAT&INTSTAT_USBRSTF_MASK) != 0) { 
         // USB Reset
         INTSTAT = INTSTAT_USBRSTF_MASK;
         handleUSBReset();
//         debugTx('R'); // Token complete
//         INTSTAT = INTSTAT_USBRSTF_MASK;
      }
      else if ((INTSTAT&INTSTAT_STALLF_MASK) != 0) {
         INTSTAT = INTSTAT_STALLF_MASK;
         ep0StartSetupTransaction();
//         debugTx('s'); // Unhandled stall
//         INTSTAT = INTSTAT_STALLF_MASK;
      }
//      INTSTAT = intStatus;
   }
}
