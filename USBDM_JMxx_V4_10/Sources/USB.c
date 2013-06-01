/*! \file
    \brief Simple USB Stack for JM60
    
   \verbatim
   JMxx USB Code
    
    Copyright (C) 2008-12  Peter O'Donoghue

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
Known Issues
The USB Specification says that the DATA0/1 toggle should be reset on configuration events.   
If I try to do this the stack breaks badly so:
  - either I'm not resetting it correctly, OR
  - Windows doesn't reset its DATA0/1 toggle
I need a USB analyser to find out which!

Change History
+==============================================================================================
| 16 Apr 2013 | Fixed buffer size in ep1SaveOutData()                             V4.10.5 - pgo 
| 26 Jul 2012 | Changed int. timing to avoid lockup on busy EP0 traffic (Win7)    V4.10   - pgo 
| 07 Nov 2010 | EP0 was not returning STALL when requested                        V4.2    - pgo 
| 29 Sep 2010 | Added CDC code & general cleanup                                  V4.2    - pgo 
|  8 Aug 2010 | Improved robustness of transaction handling (added reInit flag)   V3.5    - pgo 
| 17 Jul 2010 | Added ep1StartOutTransaction() to epClearStall(1)                       - pgo
| 15 Jul 2010 | Fixed configuration # bounds check                                      - pgo
| 30 Jun 2010 | Removed USB serial number (should be unit unique or not present!)       - pgo
|  4 Feb 2010 | Changed end-point sizes to ep0In/ep0Out/ep1out/ep2In = 32/32/64/64      - pgo
|  5 Sep 2009 | Moved SET_BOOT and GET_VERSION to USB module                            - pgo
-============================================================================================
|    Sep 2009 | Major changes for V2                                                    - pgo
-============================================================================================
| 30 Jul 2009 | Changed USB command/response structure - uses EP1/EP2                   - pgo
| 17 May 2009 | Tested with USBCV13.exe from USB.ORG - now passes                       - pgo
| 16 May 2009 | Increased validation on handleGetInterface[]                            - pgo
| 10 May 2009 | Changed String language to EN_AUS from GREEK!                           - pgo
|  7 May 2009 | Changed ep0ConfigureSetupTransaction[] & related                        - pgo
| 27 Jan 2009 | Changed SETUP pkt handling (I hate little-endian!)                      - pgo
| 27 Jan 2009 | Changed under-size IN transaction handling                              - pgo
| 24 Sep 2008 | Fixed possible ptr error in ep0SaveOutData                              - pgo
|  3 Mar 2008 | JM60 - USB code written from scratch                                    - pgo
+============================================================================================
   \endverbatim
*/
#include <string.h>
#include <hidef.h>
#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "BDM.h"
#include "BDMCommon.h"
#include "CmdProcessing.h"
#include "USBDefs.h"
#include "USB.h"
#include "ICP.h"
#include "SCI.h"

#if 1
//========================================================================
//! Swaps 16-bit value between \e big-endian and \e little-endian insitu
//! 
#define SWAP16(value)    \
   {                     \
   asm (ldx  value:0);   \
   asm (lda  value:1);   \
   asm (stx  value:1);   \
   asm (sta  value:0);   \
   }
         
//#define leToNative32(x) swap32(x)
#define leToNative16(x) SWAP16(x)
//#define nativeToLe32(x) swap32(x)
#define nativeToLe16(x) SWAP16(x)
#else
#define leToNative32(x) (x)
#define leToNative16(x) (x)
#define nativeToLe32(x) (x)
#define nativeToLe16(x) (x)
#endif

//======================================================================
// Data packet odd/even indicator
enum {DATA0=0, DATA1=1};

//======================================================================
// Structure representing a BDT entry
#pragma MESSAGE DISABLE C1106 // Disable warnings about Non-standard bitfield types
typedef struct {
   union {
      struct {
         U8           :2;  
         U8 bdtkpid   :4;
         U8 data0_1   :1;
         U8 own       :1;
      } a;
      struct {
         U8           :2;  
         U8 bdtstall  :1;  
         U8 dts       :1;
         U8           :2;  
         U8 data0_1   :1;
         U8 own       :1;
      } b;
      U8 bits;
   } control; //!< 
   U8 byteCount;
   U8 epAddr;  // Offset by two bits 
} BDTEntry;

#pragma MESSAGE DEFAULT C1106 // Restore warnings about Non-standard bitfield types

#define USB_INTMASKS (INTENB_STALL_MASK|INTENB_TOKDNE_MASK|\
                      INTENB_SOFTOK_MASK|INTENB_USBRST_MASK|\
                      INTENB_STALL_MASK)  //!< Mask for all USB interrupts

#define BDTEntry_OWN_MASK        (1<<7)	  //!< Mask for OWN bit in BDT
#define BDTEntry_DATA0_MASK      (0<<6)   //!< Mask for DATA1 bit in BDT (dummy)
#define BDTEntry_DATA1_MASK      (1<<6)   //!< Mask for DATA0 bit in BDT
#define BDTEntry_DTS_MASK        (1<<3)   //!< Mask for DTS bit in BDT
#define BDTEntry_BDTSTALL_MASK   (1<<2)   //!< Mask for STALL bit in BDT

//======================================================================
// Maximum packet sizes for each endpoint
//
#if (HW_CAPABILITY&CAP_CDC)
#define NUMBER_OF_EPS    (6)
                              //!< BDTs                                  32
#define ENDPT0MAXSIZE    (16) //!< USBDM - Control in/out   	    x2 = 32 
#define ENDPT1MAXSIZE    (64) //!< USBDM - BDM out                       64
#define ENDPT2MAXSIZE    (64) //!< USBDM - BDM in                        64
#define ENDPT3MAXSIZE    (16) //!< USBDM - CDC control                   16      
#define ENDPT4MAXSIZE    (16) //!< USBDM - CDC data out                  16
#define ENDPT5MAXSIZE    (16) //!< USBDM - CDC data in              x2 = 32
//                                                                    -------
//                                                                    <= 256 - each is rounded to 16 bytes
#else
#define NUMBER_OF_EPS    (3)  //!< Number of endpoints in use
#define ENDPT0MAXSIZE    (32) //!< USBDM - Control in/out    
#define ENDPT1MAXSIZE    (64) //!< USBDM - BDM out
#define ENDPT2MAXSIZE    (64) //!< USBDM - BDM in
#define ENDPT3MAXSIZE    (0)  //!< USBDM - CDC control (not used)
#define ENDPT4MAXSIZE    (0)  //!< USBDM - CDC data out (not used)
#define ENDPT5MAXSIZE    (0)  //!< USBDM - CDC data in (not used)
#endif
//======================================================================
// Descriptors
//
static const DeviceDescriptor deviceDescriptor = {
   sizeof(DeviceDescriptor),               // bLength
   DT_DEVICE,                              // bDescriptorType
   CONST_NATIVE_TO_LE16(0x0200),           // bcdUSB 			 = USB spec rel. No.      [BCD = 2.00]
#if (HW_CAPABILITY&CAP_CDC)
   0xEF,                                   // bDeviceClass 		 = Device Class code [Miscellaneous Device Class]
   0x02,                                   // bDeviceSubClass 	 = Sub Class code    [Common Class]
   0x01,                                   // bDeviceProtocol 	 = Protocol          [Interface Association Descriptor]
#else
   0xFF,                                   // Class code             [none]
   0xFF,                                   // Sub Class code         [none]
   0xFF,                                   // Protocol               [none]
#endif
   ENDPT0MAXSIZE,                          // bMaxPacketSize0 	 = EndPt 0 max packet size
   CONST_NATIVE_TO_LE16(VendorID),         // idVendor 			 = Vendor ID
   CONST_NATIVE_TO_LE16(ProductID),        // idProduct 		 = Product ID
   CONST_NATIVE_TO_LE16(0x0410),           // bcdDevice 		 = Device Release         [BCD = 4.10]
   1,                                      // iManufacturer 	 = String index of Manufacturer name
   2,                                      // iProduct 			 = String index of product desc.
   3,                                      // iSerialNumber 	 = String index desc. serial #
   1                                       // bNumConfigurations = Number of configurations
};

static const struct {
   ConfigurationDescriptor 					configDescriptor;
   InterfaceDescriptor                      interfaceDescriptor0;
   EndpointDescriptor                       endpointDescriptor1;
   EndpointDescriptor                       endpointDescriptor2;
#if (HW_CAPABILITY&CAP_CDC)
   InterfaceAssociationDescriptor           interfaceAssociationDescriptorCDC;
   InterfaceDescriptor     					interfaceDescriptor1;
   CDCHeaderFunctionalDescriptor    		headerFunctionalDescriptor;
   CDCCallManagementFunctionalDescriptor 	callManagementDescriptor;
   CDCAbstractControlManagementDescriptor	abstractControlManagementFunctionalDescriptor;
   CDCUnionFunctionalDescriptor             unionFunctionalDescriptor;
   EndpointDescriptor      					endpointDescriptor3;
   InterfaceDescriptor     					interfaceDescriptor2;
   EndpointDescriptor      					endpointDescriptor4;
   EndpointDescriptor      					endpointDescriptor5;
#endif   
} otherDescriptors =     
{
   { // configDescriptor
      sizeof(ConfigurationDescriptor),                // bLength
      DT_CONFIGURATION,                               // bDescriptorType
      CONST_NATIVE_TO_LE16(sizeof(otherDescriptors)), // wTotalLength
#if (HW_CAPABILITY&CAP_CDC)
      3,                                              // bNumInterfaces
#else   
      1,                                              // bNumInterfaces
#endif
      1,                                              // bConfigurationValue
      0,                                              // iConfiguration
      0x80,                                           // bmAttributes        = Bus powered, no wakeup (yet?)
      USBMilliamps(500)                               // MaxPower
   },
   { // interfaceDescriptor0
      sizeof(InterfaceDescriptor),  // bLength
      DT_INTERFACE,                 // bDescriptorType
      0,                            // bInterfaceNumber
      0,                            // bAlternateSetting
      2,                            // bNumEndpoints
      0xFF,                         // bInterfaceClass      = (Vendor specific)
      0xFF,                         // bInterfaceSubClass   = (Vendor specific)
      0xFF,                         // bInterfaceProtocol   = (Vendor specific)
      5                             // iInterface desc
   },
   { // endpointDescriptor1 - #01,OUT,Bulk
     sizeof(EndpointDescriptor),          // bLength
     DT_ENDPOINT,                         // bDescriptorType
     EP_OUT|1,                            // bEndpointAddress
     ATTR_BULK,                           // bmAttributes
     CONST_NATIVE_TO_LE16(ENDPT1MAXSIZE), // wMaxPacketSize
     1                                    // bInterval         = -
   },
   { // endpointDescriptor2 - #82,IN, BULK
     sizeof(EndpointDescriptor),          // bLength
     DT_ENDPOINT,                         // bDescriptorType
     EP_IN|2,                             // bEndpointAddress
     ATTR_BULK,                           // bmAttributes
     CONST_NATIVE_TO_LE16(ENDPT2MAXSIZE), // wMaxPacketSize
     0                                    // bInterval         = -
   },
#if (HW_CAPABILITY&CAP_CDC)
   { // interfaceAssociationDescriptorCDC
       sizeof(InterfaceAssociationDescriptor), // bLength
       DT_INTERFACEASSOCIATION,                // bDescriptorType
       1,                                      // bFirstInterface
       2,                                      // bInterfaceCount
   	   0x02,                                   // bFunctionClass    = bInterfaceClass    = CDC Control
   	   0x02,                                   // bFunctionSubClass = bInterfaceSubClass = Abstract Control Model
   	   0x01,                                   // bFunctionProtocol = bDeviceProtocol    = AT CommandL V.250
   	   6,                                      // iFunction = ""
   },
   { // interfaceDescriptor1
      sizeof(InterfaceDescriptor),  // bLength
      DT_INTERFACE,                 // bDescriptorType
      1,                            // bInterfaceNumber
      0,                            // bAlternateSetting
      1,                            // bNumEndpoints
      0x02,                         // bInterfaceClass      = CDC Control
      0x02,                         // bInterfaceSubClass   = Abstract Control Model
      0x01,                         // bInterfaceProtocol   = AT CommandL V.250
      7                             // iInterface desc
   },
   { // headerFunctionalDescriptor
      sizeof(CDCHeaderFunctionalDescriptor),  // bFunctionalLength
      CS_INTERFACE,                           // bDescriptorType
      DST_HEADER,                             // bDescriptorSubtype
      CONST_NATIVE_TO_LE16(0x0110),           // bcdCDC
   },
   { // callManagementDescriptor
	  sizeof(CDCCallManagementFunctionalDescriptor), // bFunctionalLength
	  CS_INTERFACE,                           		 // bDescriptorType
	  DST_CALL_MANAGEMENT,                     		 // bDescriptorSubtype
	  1,                                             // bmCapabilities
	  1,                                             // bDataInterface
   },
   { // abstractControlManagementFunctionalDescriptor
	  sizeof(CDCAbstractControlManagementDescriptor), // bFunctionalLength
	  CS_INTERFACE,                           		  // bDescriptorType
	  DST_ABSTRACT_CONTROL_MANAGEMENT,         		  // bDescriptorSubtype
	  0x06,                                           // bmCapabilities
   },
   { // unionFunctionalDescriptor
	  sizeof(CDCUnionFunctionalDescriptor),           // bFunctionalLength
	  CS_INTERFACE,                           		  // bDescriptorType
	  DST_UNION_MANAGEMENT,         		          // bDescriptorSubtype
	  1,                                              // bmControlInterface
	  2,                                              // bSubordinateInterface0
   },
   { // endpointDescriptor3 - #83,IN,interrupt
     sizeof(EndpointDescriptor),          // bLength
     DT_ENDPOINT,                         // bDescriptorType
     EP_IN|3,                             // bEndpointAddress
     ATTR_INTERRUPT,                      // bmAttributes
     CONST_NATIVE_TO_LE16(ENDPT3MAXSIZE), // wMaxPacketSize
     USBMilliseconds(255)                 // bInterval
   },
   { // interfaceDescriptor2
      sizeof(InterfaceDescriptor),  // bLength
      DT_INTERFACE,                 // bDescriptorType
      2,                            // bInterfaceNumber
      0,                            // bAlternateSetting
      2,                            // bNumEndpoints
      0x0A,                         // bInterfaceClass      = CDC DATA
      0x00,                         // bInterfaceSubClass   = - 
      0x00,                         // bInterfaceProtocol   = -
      8                             // iInterface desc
   },
   { // endpointDescriptor4 - #1,OUT,bulk
     sizeof(EndpointDescriptor),          // bLength
     DT_ENDPOINT,                         // bDescriptorType
     EP_OUT|4,                            // bEndpointAddress
     ATTR_BULK,                           // bmAttributes
     CONST_NATIVE_TO_LE16(ENDPT4MAXSIZE), // wMaxPacketSize
     0                                    // bInterval         = -
   },
   { // endpointDescriptor5 - #82,IN,bulk
     sizeof(EndpointDescriptor),            // bLength
     DT_ENDPOINT,                           // bDescriptorType
     EP_IN|5,                               // bEndpointAddress
     ATTR_BULK,                             // bmAttributes
     CONST_NATIVE_TO_LE16(2*ENDPT5MAXSIZE), // wMaxPacketSize (x2 so all pkts are terminating (short))
     0                                      // bInterval         = -
   },
#endif   
};

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
	// DeviceGUID = "{93FEBD51-6000-4E7E-A20E-A80FC78C7EA1}"
	/* U32 lLength;         */ CONST_NATIVE_TO_LE32((uint32_t)sizeof(MS_PropertiesFeatureDescriptor)),
	/* U16 wVersion;        */ CONST_NATIVE_TO_LE16(0x0100),
	/* U16 wIndex;          */ CONST_NATIVE_TO_LE16(0x0005),
	/* U16 bnumSections;    */ CONST_NATIVE_TO_LE16(0x0001),
	/* U32 lPropertySize;   */ CONST_NATIVE_TO_LE32(132UL),
	/* U32 ldataType;       */ CONST_NATIVE_TO_LE32(1UL),
	/* U16 wNameLength;     */ CONST_NATIVE_TO_LE16(40),
	/* U8  bName[40];       */ "D\0e\0v\0i\0c\0e\0I\0n\0t\0e\0r\0f\0a\0c\0e\0G\0U\0I\0D\0\0",
	/* U32 wPropertyLength; */ CONST_NATIVE_TO_LE32(78UL),
	// U8  bData[78];       {93FEBD51-6000-4E7E-A20E-A80FC78C7EA1}
	                           "{\000"  
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

static const U8 sd0[] = {4,  DT_STRING, 0x09, 0x0C};  // Language IDs
static const U8 sd1[] = "pgo";                        // Manufacturer
static const U8 sd2[] = ProductDescription;           // Product Description
//static const U8 sd3[] = "USBDM-Serial-0001";          // Serial Number
static const U8 sd4[] = "USBDM BDM Interface";        // Interface Association #1
static const U8 sd5[] = "Interface 0 - USBDM";        // Interface #0
static const U8 sd6[] = "USBDM CDC Interface";        // Interface Association #2
static const U8 sd7[] = "Interface 1 - CDC Control";  // Interface #1
static const U8 sd8[] = "Interface 2 - CDC Data";     // Interface #2

static const U8 *const stringDescriptors[] = {sd0, sd1, sd2, ICP_data.serialNumber, sd4, sd5, sd6, sd7, sd8};

//===============================================================================
// Device Status
//       
typedef struct {
   int selfPowered  : 1;    //!< Device is self-powered
   int remoteWakeup : 1;    //!< Supports remote wakeup
   int portTest     : 1;    //!< Port test
   int res1         : 5;    //!< Reserved
   int res2         : 8;    //!< Reserved
} DeviceStatus; 

//! USB device states
typedef enum { USBpowered, USBattached, USBdefault, USBaddressed,
               USBconfigured, USBsuspended } USBstateType;

#pragma MESSAGE DISABLE C1106 // Disable warnings about non-standard bitfield
#pragma DATA_SEG __SHORT_SEG Z_PAGE
struct {
   USBstateType   state:8;
   U8             configuration;
   U8             interfaceAltSetting;
   DeviceStatus   status;
   U8             newUSBAddress;
} deviceState = {USBattached, 0, 0, {0,0,0,0,0}, 0};  //!< USB device state information 
#pragma MESSAGE DEFAULT C1106 // Disable warnings about non-standard bitfield
#pragma DATA_SEG DEFAULT

//===============================================================================
//! Endpoint Status
//!       
typedef struct {
   int stall  : 1;   //!< Endpoint is stalled
   int res1   : 7;   //!< Reserved
   int res2   : 8;   //!< Reserved
} EPStatus; //!< Endpoint status un USB format

// Endpoint state values
typedef enum { 
   EPIdle = 0,       //!< Idle (Tx complete)
   EPDataIn,         //!< Doing a sequence of IN packets (until data count <= EPSIZE)
   EPDataOut,        //!< Doing a sequence of OUT packets (until data count == 0)
   EPLastIn,         //!< Doing the last IN packet
   EPStatusIn,       //!< Doing an IN packet as a status handshake
   EPStatusOut,      //!< Doing an OUT packet as a status handshake
   EPThrottle,       //!< Doing OUT packets but no buffers available (NAKed)
   EPStall,          //!< Endpoint is stalled
   EPComplete,       //!< Used for command protocol - new command available
} EPModes;

//! Endpoint information
typedef struct {
   U8*      dataPtr;               //!< Pointer to data buffer 
   U8       dataRemaining;         //!< Count of remaining bytes to Rx/Tx
   U8       dataCount;             //!< Count of bytes Rx/Tx so far
   int      shortInTransaction:1;  //!< Indicates that the IN transaction is under-sized 
   void     (*callback)( void );   //!< Call-back used on completion of packet reception
} EPState;

#pragma MESSAGE DISABLE C1106 // Non-standard bit-field type
//! Endpoint hardware state
typedef struct {
   int        data0_1:1;  //!< Data 0/1 toggle state
   int        odd:1;      //!< Odd/Even buffer
   EPModes    state:5;    //!< End-point state
} EPHardwareState;
#pragma MESSAGE DEFAULT C1106 // Non-standard bit-field type

// Used to flag USB system configuration change etc.
static volatile U8 reInit;

#pragma DATA_SEG __SHORT_SEG Z_PAGE
static volatile EPState ep0State = {NULL, 0, 0, 0, NULL};
static volatile EPState ep1State = {NULL, 0, 0, 0, NULL};
static volatile EPState ep2State = {NULL, 0, 0, 0, NULL};
#pragma DATA_SEG DEFAULT

static volatile EPHardwareState epHardwareState[NUMBER_OF_EPS] = {0};

//! Structure representing USB activity
typedef union {
   U8  byte;  //!< Allows access as byte (zero=>no activity, non-zero=>activity)
   struct {
      int bdmActive:1;			//!< Any activity
      int serialOutActive:1;	//!< Serial out active 
      int serialInActive:1;	//!< Serial in active
   } flags;                   //!< Overall flags
} ActivityType ;
static ActivityType  usbActivityFlag;

#pragma DATA_SEG DEFAULT

#pragma DATA_SEG __SHORT_SEG Z_PAGE
//======================================================================
// Buffer for EP0 Setup packet (copied from USB RAM)
volatile SetupPacket ep0SetupBuffer; //!< EP0 contents when receiving a Setup pkt

#pragma DATA_SEG DEFAULT
//static U16 frameNum        = 0;

//======================================================================
// USB RAM usage
//
#define USB_RAM_START (0x1860U) //!< Fixed hardware address of USB interface buffer

#define USB_MAP_ADDRESS(x)  ((((U16)(x) - USB_RAM_START)>>2)&0xFC) //!< Maps a linear address to USB buffer address
#define ADDRESS_ROUND16(x)  ((x+0xF)&0xFFF0)			           //!< Rounds address to USB buffer alignment

//======================================================================
// Addresses of buffers for endpoint data packets (in USB RAM).
// These are done this way so that the addresses are known at compile time
// to avoid run-time calculations
//
// Endpoint buffer address must be aligned multiple of 16 starting after bdts
#define EP0InDataBufferAddress   ADDRESS_ROUND16(USB_RAM_START+0x20) 					   //!< EP0 In buffer address
#define EP0OutDataBufferAddress  ADDRESS_ROUND16(EP0InDataBufferAddress    +ENDPT0MAXSIZE) //!< EP0 Out buffer address
#define EP1DataBufferAddress     ADDRESS_ROUND16(EP0OutDataBufferAddress   +ENDPT0MAXSIZE) //!< EP1 buffer address
#define EP2DataBufferAddress     ADDRESS_ROUND16(EP1DataBufferAddress      +ENDPT1MAXSIZE) //!< EP2 buffer address
#define EP3DataBufferAddress     ADDRESS_ROUND16(EP2DataBufferAddress      +ENDPT2MAXSIZE) //!< EP3 buffer address
#define EP4DataBufferAddress     ADDRESS_ROUND16(EP3DataBufferAddress      +ENDPT3MAXSIZE) //!< EP4 buffer address
#define EP5DataBuffer0Address    ADDRESS_ROUND16(EP4DataBufferAddress      +ENDPT4MAXSIZE) //!< EP5 buffer address
#define EP5DataBuffer1Address    ADDRESS_ROUND16(EP5DataBuffer0Address     +ENDPT5MAXSIZE) //!< EP5 buffer address
#if (EP5DataBuffer1Address+ENDPT5MAXSIZE) > USB_RAM_START+256
#error USB endpoint buffers too large
#endif

#pragma DATA_SEG USB_RAM
// Structure representing the USB Controller RAM area
//
struct {
   BDTEntry bdts[10];
   U8 reserved[2];
   U8 ep0InDataBuffer[ENDPT0MAXSIZE];
   U8 ep0OutDataBuffer[ENDPT0MAXSIZE];
   U8 ep1DataBuffer[ENDPT1MAXSIZE];
   U8 ep2DataBuffer[ENDPT2MAXSIZE];
#if (HW_CAPABILITY&CAP_CDC)   
   U8 ep3DataBuffer[ENDPT3MAXSIZE];
   U8 ep4DataBuffer[ENDPT4MAXSIZE];
   U8 ep5DataBuffer0[ENDPT5MAXSIZE];
   U8 ep5DataBuffer1[ENDPT5MAXSIZE];
#endif
} usbRamArea;
#pragma DATA_SEG DEFAULT

#define  ep0BDTIn    usbRamArea.bdts[0] //!< EP0 In BDT
#define  ep0BDTOut   usbRamArea.bdts[1] //!< EP0 Out BDT
#define  ep1BDT      usbRamArea.bdts[2] //!< EP1 BDT
#define  ep2BDT      usbRamArea.bdts[3] //!< EP2 BDT
#define  ep3BDT      usbRamArea.bdts[4] //!< EP3 BDT
#define  ep4BDT      usbRamArea.bdts[5] //!< EP4 BDT
#define  ep5BDTEven  usbRamArea.bdts[6] //!< EP5 Even BDT
#define  ep5BDTOdd   usbRamArea.bdts[7] //!< EP5 Odd BDT
#define  ep6BDTEven  usbRamArea.bdts[8] //!< EP6 Even BDT
#define  ep6BDTOdd   usbRamArea.bdts[9] //!< EP6 Odd BDT

#define ep0InDataBuffer    (usbRamArea.ep0InDataBuffer )
#define ep0OutDataBuffer   (usbRamArea.ep0OutDataBuffer)
#define ep1DataBuffer      (usbRamArea.ep1DataBuffer   )
#define ep2DataBuffer      (usbRamArea.ep2DataBuffer   )
#if (HW_CAPABILITY&CAP_CDC)
#define ep3DataBuffer      (usbRamArea.ep3DataBuffer )
#define ep4DataBuffer      (usbRamArea.ep4DataBuffer )
#define ep5DataBuffer0     (usbRamArea.ep5DataBuffer0)
#define ep5DataBuffer1     (usbRamArea.ep5DataBuffer1)
#endif

//======================================================================
//! Initialise the endpoint buffer pointers once only
//!
static void initEndpointBuffers(void) {
   ep0BDTIn.epAddr     = USB_MAP_ADDRESS(EP0InDataBufferAddress);
   ep0BDTOut.epAddr    = USB_MAP_ADDRESS(EP0OutDataBufferAddress);
   ep1BDT.epAddr       = USB_MAP_ADDRESS(EP1DataBufferAddress);
   ep2BDT.epAddr       = USB_MAP_ADDRESS(EP2DataBufferAddress);
#if (HW_CAPABILITY&CAP_CDC)
   ep3BDT.epAddr       = USB_MAP_ADDRESS(EP3DataBufferAddress);
   ep4BDT.epAddr       = USB_MAP_ADDRESS(EP4DataBufferAddress);
   ep5BDTEven.epAddr   = USB_MAP_ADDRESS(EP5DataBuffer0Address);
   ep5BDTOdd.epAddr    = USB_MAP_ADDRESS(EP5DataBuffer1Address);
   (void)setRxBuffer(ep5DataBuffer0);
#endif
}
static uint8_t doneEp0OutInit = FALSE;
//======================================================================
//! Configure the BDT for EP0 Out [Rx, device <- host, DATA0/1]
//!
//! @param data0_1 - value for USB Data toggle
//!
static void ep0InitialiseBDTOut( U8 data0_1 ) {
   // Set up to Rx packet
   ep0BDTOut.byteCount = ENDPT0MAXSIZE; // Always use ENDPT0MAXSIZE so can accept SETUP pkt
   if (data0_1) {
      ep0BDTOut.control.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      ep0BDTOut.control.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
   doneEp0OutInit = TRUE;
}

//=========================================================================
//! Save the data from an EP0 OUT pkt and advance ptrs etc.
//!
static U8 ep0SaveOutData( void ) {
U8 size = ep0BDTOut.byteCount;

   if (size > 0) {
	  // Check if more data than requested - discard excess
      if (size > ep0State.dataRemaining)
         size = ep0State.dataRemaining;
      // Check if external buffer in use
      if (ep0State.dataPtr != NULL) {
         // Copy the data from the Rx buffer to external buffer
         ( void )memcpy(ep0State.dataPtr, ep0OutDataBuffer, size);
         ep0State.dataPtr    += size;   // Advance external buffer ptr
      }
      ep0State.dataRemaining -= size;   // Count down bytes to go
      ep0State.dataCount     += size;   // Count bytes so far
      }
   return size;
}

#if (HW_CAPABILITY&CAP_CDC)		   
//======================================================================
/*! Configure EP0 for an OUT transaction [Rx, device <- host, DATA0/1]
 *
 * @param bufSize - Size of data to transfer
 * @param bufPtr  - Buffer for data, may be NULL - Useful when:
 *                - The transfer is < ENDPT0MAXSIZE
 *                - Data may be used directly from ep0OutDataBuffer
 *                - So no additional buffer is needed
 * @param data0_1 - Initial DATA0/DATA1 toggle value
 */
static void ep0StartOutTransaction( U8 bufSize, U8 *bufPtr, U8 data0_1 ) {

   ep0State.dataRemaining     = bufSize; // Total bytes for Rx
   ep0State.dataCount         = 0;       // Reset count of bytes so far
   ep0State.dataPtr           = bufPtr;  // Where to (eventually) place data
   epHardwareState[0].data0_1 = data0_1; // Initial data toggle
   
   if (bufSize == 0) 
      epHardwareState[0].state = EPStatusOut;  // Assume status handshake
   else
      epHardwareState[0].state = EPDataOut;    // Assume first of several data pkts

   ep0InitialiseBDTOut(data0_1); // Configure the BDT for transfer
}
#endif

//================================================================================
// Configure EP0-out for a SETUP transaction [Rx, device<-host, DATA0]
// Endpoint state is changed to EPIdle
//
static void ep0ConfigureSetupTransaction( void ) {
    // Set up EP0-OUT to Rx SETUP packets
    ep0InitialiseBDTOut(DATA1);          // v4.7 changed to DATA1
    epHardwareState[0].state = EPIdle;
}

#if 0
//================================================================================
// Configure EP0-out for a SETUP transaction [Rx, device<-host, DATA0]
// Only done if endpoint is not already configured for some other OUT transaction
// Endpoint state unchanged.
//
static void ep0EnsureReadyForSetupTransaction( void ) {
	uint8_t currentEp0State = epHardwareState[0].state;
#if 0
	if ((ep0BDTOut.control.bits&BDTEntry_OWN_MASK)==0) {
        ep0InitialiseBDTOut(DATA1);          // v4.9
    }
#else
   switch (currentEp0State) {
      case EPDataOut:        // Doing a sequence of OUT packets (until data count <= EPSIZE)
      case EPStatusOut:      // Doing an OUT packet as a status handshake
         // EP0-OUT is already set up for an OUT pkt
         break;

      case EPStall:	        // Stalled
      case EPIdle:           // Idle
      case EPDataIn:         // Doing a sequence of IN packets
      case EPLastIn:         // Doing the last IN packet
      case EPStatusIn:       // Doing an IN packet as a status handshake for an OUT Data transfer
      default:
         // Set up EP0-OUT to Rx SETUP packets
         ep0ConfigureSetupTransaction();
         epHardwareState[0].state = currentEp0State;
         break;
   }
#endif
}
#endif

//======================================================================
// Configure the BDT for EP0 In [Tx, device -> host]
//
static void ep0InitialiseBDTIn( void ) {
   U16 size;

   size = ep0State.dataRemaining;
   if (size > ENDPT0MAXSIZE)
      size = ENDPT0MAXSIZE;
   
   // Copy the Tx data to EP buffer
   ( void )memcpy(ep0InDataBuffer, ep0State.dataPtr, size);
      
   ep0State.dataPtr         += size;  // Ptr to _next_ data
   ep0State.dataRemaining   -= size;  // Count of remaining bytes
   ep0State.dataCount       += size;  // Count of bytes so far
   
   // Set up to Tx packet
   ep0BDTIn.byteCount     = (U8)size;
   if (epHardwareState[0].data0_1) 
      ep0BDTIn.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   else
      ep0BDTIn.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
}

//======================================================================
//! Configure EP0 for an IN transaction [Tx, device -> host, DATA0/1]
//!
static void ep0StartInTransaction( U8 bufSize, const U8 *bufPtr, U8 data0_1 ) {

   if (bufSize > ep0SetupBuffer.wLength.word) // More data than requested - truncate
      bufSize = (U8)ep0SetupBuffer.wLength.word;

   ep0State.dataPtr            = (U8*)bufPtr;   // Ptr to _next_ data
   ep0State.dataRemaining      = bufSize;       // Count of remaining bytes
   ep0State.dataCount          = 0;             // Reset count of bytes so far
   epHardwareState[0].data0_1  = data0_1;       // Initial data toggle
   ep0State.shortInTransaction = bufSize < ep0SetupBuffer.wLength.word; // Short transaction?

   if (bufSize == 0)
      epHardwareState[0].state = EPStatusIn;   // Assume status handshake
   else if ((bufSize < ENDPT0MAXSIZE) ||       // Undersize pkt OR
            ((bufSize == ENDPT0MAXSIZE) &&     // Full size AND
             !ep0State.shortInTransaction))    //   Don't need to flag undersize transaction 
      epHardwareState[0].state = EPLastIn;     // Sending one and only pkt
   else 
      epHardwareState[0].state = EPDataIn;     // Sending first of several pkts
      
   ep0InitialiseBDTIn(); // Configure the BDT for transfer
}

//======================================================================
// Configure the BDT for EP1 Out [Rx, device <- host, DATA0/1]
//
static void ep1InitialiseBDTOut( void ) {
   // Set up to Rx packet
   ep1BDT.byteCount   = ENDPT1MAXSIZE;
   if (epHardwareState[1].data0_1) {
      ep1BDT.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      ep1BDT.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
}

//=========================================================================
// Save the data from an EP1 OUT pkt and advance ptrs etc.
//
static U8 ep1SaveOutData( void ) {
U8 size = ep1BDT.byteCount;

   if (size > 0) {
	  // Check if more data than requested - discard excess
      if (size > ep1State.dataRemaining)
         size = ep1State.dataRemaining;
      // Check if external buffer in use
      if (ep1State.dataPtr != NULL) {
          // Copy the data from the Rx buffer to external buffer
         ( void )memcpy(ep1State.dataPtr, ep1DataBuffer, size);
         ep1State.dataPtr    += size;   // Advance buffer ptr
      }
      ep1State.dataRemaining -= size;   // Count down bytes to go
      ep1State.dataCount     += size;   // Count bytes so far
      }
   return size;
}

//#pragma INLINE
#pragma MESSAGE DISABLE C1404 // Disable warnings about no return statement
//! Inline for setting int mask and returning current value
U8 saveAndDisableInterrupts(void) {
	asm {
		tpa
		sei
	}
}

//#pragma INLINE
//! Inline for setting interrupt mask
//!
//! @param mask - interrupt mask to use
//!
#pragma MESSAGE DISABLE C5703 // Ignore unused parameter
void setInterrupts(U8 mask) {
   asm {
	   sei
	   bit #0x08 // Check if I is to be set
	   bne IisSet
	   cli
   IisSet:
   }
}
#pragma MESSAGE DEFAULT C5703

//======================================================================
/*! Configure EP1-out for an OUT transaction [Rx, device <- host, DATA0/1]
 *
 * @param bufSize - Size of data to transfer
 * @param bufPtr  - Buffer for data
 */
static void ep1StartOutTransaction( U8 bufSize, U8 *bufPtr ) {
   ep1State.dataRemaining  = bufSize; // Total bytes to Rx
   ep1State.dataCount      = 0;       // Reset count of bytes so far
   ep1State.dataPtr        = bufPtr;  // Where to (eventually) place data
   
   epHardwareState[1].state = EPDataOut;    // Assume first of several data pkts

   ep1InitialiseBDTOut(); // Configure the BDT for transfer
}

//======================================================================
// Configure the BDT for EP2 In [Tx, device -> host]
//
static void ep2InitialiseBDTIn( void ) {
U16 size;

   size = ep2State.dataRemaining;
   if (size > ENDPT2MAXSIZE)
      size = ENDPT2MAXSIZE;
   
   // Copy the Tx data to EP buffer
   (void) memcpy(ep2DataBuffer, ep2State.dataPtr, size);
      
   ep2State.dataPtr         += size;  // Ptr to _next_ data
   ep2State.dataRemaining   -= size;  // Count of remaining bytes
   ep2State.dataCount       += size;  // Count of bytes so far
   
   // Set up to Tx packet
   ep2BDT.byteCount     = (U8)size;
   if (epHardwareState[2].data0_1) 
      ep2BDT.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   else
      ep2BDT.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
}

static const U8 busyResponse[] = {BDM_RC_BUSY,1,2,3};	
static U8 commandBusyFlag = FALSE;

//======================================================================
// Configure EP2 for an IN transaction [Tx, device -> host, DATA0/1]
//
static void ep2StartInTransaction( U8 bufSize, const U8 *bufPtr ) {

   ep2State.dataPtr            = (U8*)bufPtr;   // Ptr to _next_ data
   ep2State.dataRemaining      = bufSize;       // Count of remaining bytes
   ep2State.dataCount          = 0;             // Reset count of bytes so far
   
   // Note - Always terminates transfers with a truncated/zero pkt
   if (bufSize < ENDPT2MAXSIZE)    // Undersize pkt OR
      epHardwareState[2].state = EPLastIn;    // Sending one and only pkt
   else 
      epHardwareState[2].state = EPDataIn;    // Sending first of several pkts
      
   ep2InitialiseBDTIn(); // Configure the BDT for transfer
}

//! Set BDM busy flag
//!
void setBDMBusy(void) {
	disableInterrupts();
	commandBusyFlag = TRUE;
	if (epHardwareState[2].state == EPIdle) {
        ep2StartInTransaction(sizeof(busyResponse), busyResponse);
	}
	enableInterrupts();
}

#if (HW_CAPABILITY&CAP_CDC)
//======================================================================
// Configure EP3 for an IN transaction [Tx, device -> host, DATA0/1]
//
static void ep3StartInTransaction( void ) {
   const CDCNotification cdcNotification= {CDC_NOTIFICATION, SERIAL_STATE, 0, RT_INTERFACE, CONST_NATIVE_TO_LE16(2)};
   uint8_t status = getSerialState();
   
   if ((status & SERIAL_STATE_CHANGE) == 0) {
      epHardwareState[3].state = EPIdle; // Not busy
      return;
   }

   // Copy the Tx data to Tx buffer
   ( void )memcpy(ep3DataBuffer, &cdcNotification, sizeof(cdcNotification));
   ep3DataBuffer[sizeof(cdcNotification)+0] = status&~SERIAL_STATE_CHANGE;
   ep3DataBuffer[sizeof(cdcNotification)+1] = 0;
   
   // Set up to Tx packet
   ep3BDT.byteCount     = sizeof(cdcNotification)+2;
   if (epHardwareState[3].data0_1) 
	  ep3BDT.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   else
	  ep3BDT.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   epHardwareState[3].state = EPLastIn;    // Sending one and only pkt
}

//======================================================================
// Configure the BDT for EP4 Out [Rx, device <- host, DATA0/1]
// CDC - OUT
static void ep4InitialiseBDTOut( void ) {

   // Set up to Rx packet
   ep4BDT.byteCount   = (U8)ENDPT4MAXSIZE;
   if (epHardwareState[4].data0_1) 
      ep4BDT.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   else
      ep4BDT.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
}

//=========================================================================
// Save the data from an EP4 OUT pkt
// CDC - OUT
static void ep4SaveOutData( void ) {
U8 size; 

   size = ep4BDT.byteCount;
   (void)putTxBuffer((uint8_t*)ep4DataBuffer, size);
   
   // Toggle on successful reception
   epHardwareState[4].data0_1 = !epHardwareState[4].data0_1;
}

//======================================================================
// Configure the BDT for EP5 In [Tx, device -> host]
// CDC - IN
static void ep5InitialiseBDTIn(void) {
//	uint8_t controlValue;
//	if (epHardwareState[5].data0_1 == DATA0) {
//		controlValue = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
//	}
//	else {
//		controlValue = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
//	}
   if (epHardwareState[5].odd) {
	   // Set to write to other buffer & get count in current buffer
	   ep5BDTOdd.byteCount     = setRxBuffer((uint8_t*)ep5DataBuffer0);
//	   ep5DataBuffer1[0]       = '|';
	   ep5BDTOdd.control.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK;
   }
   else {
	   // Set to write to other buffer & get count in current buffer
	   ep5BDTEven.byteCount    = setRxBuffer((uint8_t*)ep5DataBuffer1);
//	   ep5DataBuffer0[0]       = '^';
	   ep5BDTEven.control.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK;
   }
//   epHardwareState[5].data0_1 = !epHardwareState[5].data0_1; // Toggle data0/1
   epHardwareState[5].odd     = !epHardwareState[5].odd;
}

static U8 serialDelayCount = 0;

// This value controls how long the serial interface will wait before
// sending a buffered character. (count of SOFs ~ ms)
#define SERIAL_THRESHOLD (0) // ms

//======================================================================
// Configure the BDT for EP5 In [Tx, device -> host]
// CDC - IN
static void ep5StartInTransactionIfIdle() {
#if 0
	if ((epHardwareState[5].state == EPIdle) && (rxBufferItems()>0)) {
		ep5InitialiseBDTIn();
		epHardwareState[5].state = EPLastIn;
		serialDelayCount = 0;
	}
	else if ((epHardwareState[5].state == EPLastIn) && (rxBufferItems()==16)) {
		ep5InitialiseBDTIn();
		epHardwareState[5].state = EPDataIn;
		serialDelayCount = 0;
	}
#else
	if ((epHardwareState[5].state == EPIdle) && (rxBufferItems()>0)) {
#if (DEBUG&USB_PING_DEBUG)
		DEBUG_PIN = 0;
#endif
		ep5InitialiseBDTIn();
		epHardwareState[5].state = EPDataIn;
		serialDelayCount = 0;
	}
#endif
}

void checkUsbCdcRxData(void) {
	ep5StartInTransactionIfIdle();
}
#endif

//======================================================================
//! Receive a command over EP1
//!
//! @param maxSize  = max # of bytes to receive
//! @param buffer   = ptr to buffer for bytes received
//!
//! @note : Doesn't return until command has been received.
//! @note : Format 
//! 	- [0]    = size of command (N)
//! 	- [1]    = command
//! 	- [2..N] = parameters
//!
//! =======================================================
//! Format - a command is made up of up to 2 pkts
//! The size of the 1st pkt indicates if subsequent pkts 
//! are used.
//!
//!  1st pkt
//! +--------------------------+
//! |  Size of entire command  |  0 - size includes 2nd pkt
//! +--------------------------+
//! |  Command byte            |  1
//! +--------------------------+
//! |                          |  2... up to ENDPT1MAXSIZE-2
//! | //// DATA ////////////// |
//! |                          |
//! +--------------------------+
//!  2nd pkt (optional)
//! +--------------------------+
//! |  0                       |  0 - Ensures pkt can't be mistaken as 1st pkt 
//! +--------------------------+
//! |                          |  1... up to ENDPT1MAXSIZE-1
//! | //// DATA ////////////// |  
//! |                          |
//! +--------------------------+
void receiveUSBCommand(U8 maxSize, U8 *buffer) {
U8 size;
	enableInterrupts();
   // Size of first (command) transaction
   do {
      size = ENDPT1MAXSIZE;   
      if (size > maxSize)
         size = maxSize;
	   
	  // Get 1st/only pkt of command
      reInit = FALSE;
	  ep1StartOutTransaction( size, buffer );
      while ((epHardwareState[1].state != EPComplete) && !reInit) {
         wait();
      }
      if (reInit) {
         continue;
      }
      // Size for entire command from 1st pkt 
      size = buffer[0];  
      if (size > maxSize) {
         size = maxSize;
      }
      if (size == 0) {
    	  // Invalid pkt - try again
    	  // 0 indicates this is not an initial command pkt
    	  // but part of a longer command
    	  reInit = TRUE;
    	  continue;
      }
      // Receive rest of data if present (only possibly 2 transactions total)
      if (size > ep1State.dataCount) {
    	 // Save last byte of 1st pkt as overwritten by
    	 // second pkt (to save moving 2nd pkt when size is discarded)
     	 U8 saveByteOffset = ep1State.dataCount-1;   
    	 U8 saveByte       = buffer[saveByteOffset];
         ep1StartOutTransaction( size-saveByteOffset, buffer+saveByteOffset );
         while ((epHardwareState[1].state != EPComplete) && !reInit) {
            wait();
         }
         // Check if second pkt has correct marker
         if (buffer[saveByteOffset] != 0) {
        	// packet corrupt
        	reInit = TRUE;
         }
         // Restore saved byte
    	 buffer[saveByteOffset] = saveByte;
      }
   } while (reInit);
}

//======================================================================
//! Set a command response over EP2
//!
//! @param size   = # of bytes to send
//! @param buffer = ptr to bytes to send
//!
//! @note : Returns before the command has been sent.
//!
//! @note : Format 
//! 	- [0]    = response
//! 	- [1..N] = parameters
//!
void sendUSBResponse( U8 size, const U8 *buffer) {
   disableInterrupts();
   commandBusyFlag = FALSE;
   while (epHardwareState[2].state != EPIdle) {
      wait();
   }
   enableInterrupts();
   ep2StartInTransaction(size, buffer);
}

//======================================================================
// Stall control for endpoints
//
//! Arrangement of EP control registers
#if (CPU==JS16)
EPCTL0STR EPCTL[7] @0x0000006D;
#else
EPCTL0STR EPCTL[7] @0x0000009D;
#endif;

//! Stall endpoint
//!
//! @param epNum - endpoint number
//!
static void epStall(U8 epNum) {
   if (epNum == 0) {
      // EP0 just stalls the IN direction
      ep0BDTIn.control.bits  = BDTEntry_OWN_MASK|BDTEntry_BDTSTALL_MASK|BDTEntry_DTS_MASK;
   }
   else {
      // EPn stall entire endpoint
      EPCTL[epNum].Bits.EPSTALL         = 1;
   }
   epHardwareState[epNum].state         = EPStall;
   epHardwareState[epNum].data0_1       = DATA0;
}

//! Clear Stall on endpoint
//!
//! @param epNum - endpoint number
//!
static void epClearStall(U8 epNum) {
   EPCTL[epNum].Bits.EPSTALL            = 0; 
   if (epNum == 0) {								// v4.7
      ep0BDTIn.control.bits  = 0;                   // v4.7
   }                                                // v4.7
   if (epNum == 2) {                 // v4.10.3
	   ep2BDT.control.bits = 0;
   }
   epHardwareState[epNum].state    = EPIdle;
   epHardwareState[epNum].data0_1  = DATA0;
#if (HW_CAPABILITY&CAP_CDC)
   if (epNum == 5) {
	   CTL_ODDRST = 1; // reset ping-pong buffer
	   CTL_ODDRST = 0;
	   epHardwareState[5].odd      = 0;
   }
#endif
}

//========================================================================================
//
static void setUSBdefaultState( void ) {
   GREEN_LED_OFF();
   deviceState.state                = USBdefault;
   ADDR                             = 0;
   deviceState.configuration        = 0;
   deviceState.interfaceAltSetting  = 0; 
}

static void setUSBaddressedState( U8 address ) {
   if (address == 0) // Unaddress??
      setUSBdefaultState();
   else {
      GREEN_LED_OFF();
      deviceState.state                = USBaddressed;
      ADDR                             = address;
      deviceState.configuration        = 0;
      deviceState.interfaceAltSetting  = 0; 
   }
}

static void setUSBconfiguredState( U8 config ){
   if (config == 0) // unconfigure
      setUSBaddressedState(ADDR);
   else {
      GREEN_LED_ON();
      deviceState.state                = USBconfigured;
      deviceState.configuration        = config;
      deviceState.interfaceAltSetting  = otherDescriptors.interfaceDescriptor0.bAlternateSetting; // ToDo check
   }
}

//======================================================================
// (re)Initialises end-points other than EP0
//
static void initialiseEndpoints(void) {
	
   (void)memset(&EPCTL[1], 0, sizeof(EPCTL)-sizeof(EPCTL[0])); // Disable endpoints
   (void)memset(&usbRamArea.bdts[2], 0, sizeof(usbRamArea.bdts)-2*sizeof(usbRamArea.bdts[0])); // Clear BDTS so no pkts accepted

   initEndpointBuffers();
   
   epClearStall(1);  reInit = TRUE;

   epClearStall(2);

#if (HW_CAPABILITY&CAP_CDC)
   epClearStall(3);
   epClearStall(4);
   epClearStall(5);   
#endif

#if (HW_CAPABILITY&CAP_CDC)
   //ToDo - check is sensible on re-init????
   ep3StartInTransaction();		  // Interrupt pipe IN - status
   ep4InitialiseBDTOut();         // Tx pipe OUT
   CTL_ODDRST = 1;
   CTL_ODDRST = 0;
   epHardwareState[5].odd = 0;
   ep5StartInTransactionIfIdle(); // Rx pipe IN
#endif
   
   EPCTL1  = EPCTL1_EPRXEN_MASK|                   EPCTL1_EPHSHK_MASK; // OUT
   EPCTL2  =                    EPCTL2_EPTXEN_MASK|EPCTL2_EPHSHK_MASK; // IN
#if (HW_CAPABILITY&CAP_CDC)   
   EPCTL3  =                    EPCTL3_EPTXEN_MASK|EPCTL3_EPHSHK_MASK; // IN
   EPCTL4  = EPCTL4_EPRXEN_MASK|                   EPCTL4_EPHSHK_MASK; // OUT
   EPCTL5  =                    EPCTL5_EPTXEN_MASK|EPCTL5_EPHSHK_MASK; // IN
#endif
}

//==================================================================
// Handler for USB Bus reset
// 
static void handleUSBReset( void ) {
   ERRSTAT = 0xFF;                 // Clear USB error flags
   
   // Clear all USB interrupt flags
   INTSTAT = INTSTAT_STALLF_MASK|INTSTAT_RESUMEF_MASK|INTSTAT_SLEEPF_MASK| 
             INTSTAT_TOKDNEF_MASK|INTSTAT_SOFTOKF_MASK|INTSTAT_ERRORF_MASK|
             INTSTAT_USBRSTF_MASK;

   ERRENB  = 0x00;                 // Disable all USB error interrupt sources

   // Enable various interrupts
   INTENB  = USB_INTMASKS;

   setUSBdefaultState();

   epClearStall(0);   

   // Set up to receive setup packet
   ep0ConfigureSetupTransaction(); // re-initialise EP0 OUT // v4.7
}


//======================================================================
//! Initialise the USB interface
//!
void initUSB( void ) {

	   
#if (DEBUG&USB_PING_DEBUG)
   DEBUG_PIN_DDR = 1;
   DEBUG_PIN     = 0;
#endif

   // Clear USB RAM (includes BDTs)
   (void)memset(usbRamArea.bdts, 0x00, sizeof(usbRamArea));
   
   // Reset USB   
   USBCTL0_USBRESET = 1;
   while (USBCTL0_USBRESET) {
   }
   // Enable USB module.
   CTL = CTL_USBEN_MASK;
   
   // Internal PUP, Internal 3V3 reg, Enable Txvr 
   USBCTL0 = USBCTL0_USBPU_MASK|USBCTL0_USBVREN_MASK|USBCTL0_USBPHYEN_MASK;

   // Clear all pending interrupts except reset.
   INTSTAT = (INTSTAT_USBRSTF_MASK^0xFF);
   
   // Enable endpoints
   EPCTL0  = EPCTL0_EPRXEN_MASK|EPCTL0_EPTXEN_MASK|EPCTL0_EPHSHK_MASK; // IN/OUT/SEUP
   EPCTL1  = EPCTL1_EPRXEN_MASK|                   EPCTL1_EPHSHK_MASK; // OUT
   EPCTL2  =                    EPCTL2_EPTXEN_MASK|EPCTL2_EPHSHK_MASK; // IN
#if (HW_CAPABILITY&CAP_CDC)   
   EPCTL3  =                    EPCTL3_EPTXEN_MASK|EPCTL3_EPHSHK_MASK; // IN
   EPCTL4  = EPCTL4_EPRXEN_MASK|                   EPCTL4_EPHSHK_MASK; // OUT
   EPCTL5  =                    EPCTL5_EPTXEN_MASK|EPCTL5_EPHSHK_MASK; // IN
#endif
   
   // Enable usb reset interrupt
   INTENB_USBRST=1;    // check
  
   handleUSBReset();   
   
   initialiseEndpoints();
}

//===============================================================================
// Get Status - Device Req 0x00
//       
static void handleGetStatus( void ) {
static const U8 zeroReturn[] = {0,0};
static const EPStatus epStatusStalled = {1,0,0};
static const EPStatus epStatusOK      = {0,0,0};

const U8 *dataPtr = NULL;
U8 size;

   //dprint("hGS()");
   switch(ep0SetupBuffer.bmRequestType) {
      case (EP_IN|RT_DEVICE) : // Device Status
    	 dataPtr = (U8 *) &deviceState.status;
         size    = sizeof(deviceState.status);
         break;
      case (EP_IN|RT_INTERFACE) : // Interface Status - reserved
    	 dataPtr = zeroReturn;
         size = sizeof(zeroReturn);
         break;
      case (EP_IN|RT_ENDPOINT) : { // Endpoint Status
    	 U8 epNum = ep0SetupBuffer.wIndex.word&0x07;
    	 if (epNum <= NUMBER_OF_EPS) {
            if (EPCTL[epNum].Bits.EPSTALL)
               dataPtr = (U8*)&epStatusStalled;
            else
               dataPtr = (U8*)&epStatusOK;
//    	    dataPtr = (U8 *) &epHardwareState[epNum].status;
            size = sizeof(EPStatus);
    	    }
         }
    	 break;
      }
   if (dataPtr != NULL)
      ep0StartInTransaction( size, dataPtr, DATA1 );
   else
      epStall(0);
}    

//===============================================================================
// Clear Feature - Device Req 0x01
//       
static void handleClearFeature( void ) {
int okStatus = 0;

   switch(ep0SetupBuffer.bmRequestType) {
      case (RT_DEVICE) : // Device Feature
         if ((ep0SetupBuffer.wValue.word != DEVICE_REMOTE_WAKEUP) || // Device remote wakeup
             (ep0SetupBuffer.wIndex.word != 0))   {                  // Device index must be 0
            break;
            }
         deviceState.status.remoteWakeup = 0;
         okStatus                        = 1;
         break;

      case (RT_INTERFACE) : // Interface Feature
         break;

      case (RT_ENDPOINT) : { // Endpoint Feature ( Out,Standard,Endpoint )
      	 U8 epNum = ep0SetupBuffer.wIndex.word&0x07;
         if ((ep0SetupBuffer.wValue.word != ENDPOINT_HALT) || // Not Endpoint Stall ?
             (epNum >= NUMBER_OF_EPS))                        // or illegal EP# (ignores direction)
            break;
         epClearStall(epNum);
//         if (epNum == 1)
//        	 reInit = TRUE;
         okStatus = 1;
         }
         break;

      default : // Illegal
         break;
      }

   if (okStatus)
      ep0StartInTransaction( 0, NULL, DATA1 ); // Tx empty Status packet
   else
      epStall(0);
}

//===============================================================================
// Set Feature - Device Req 0x03
//       
static void handleSetFeature( void ) {
int okStatus = 0;

   switch(ep0SetupBuffer.bmRequestType) {
      case (RT_DEVICE) : // Device Feature
         if ((ep0SetupBuffer.wValue.word != DEVICE_REMOTE_WAKEUP) || // Device remote wakeup
             (ep0SetupBuffer.wIndex.word != 0)) {                    // device wIndex must be 0
            break;
            }
         deviceState.status.remoteWakeup = 1;
         okStatus                        = 1;
         break;

      case (RT_INTERFACE) : // Interface Feature
         break;

      case (RT_ENDPOINT) : { // Endpoint Feature ( Out,Standard,Endpoint )
      	 U8 epNum = ep0SetupBuffer.wIndex.word&0x07;
         if ((ep0SetupBuffer.wValue.word != ENDPOINT_HALT) || // Not Endpoint Stall ?
             (epNum >= NUMBER_OF_EPS))                        // or illegal EP# (ignores direction)
            break;
         epStall(epNum);
         okStatus = 1;
         }
         break;

      default : // Illegal
         break;
      }
      
   if (okStatus)
      ep0StartInTransaction( 0, NULL, DATA1 ); // Tx empty Status packet
   else
      epStall(0);
}

//===============================================================================
//! Creates a valid string descriptor in UTF-16-LE from a limited UTF-8 string
//!
//! @param source - Zero terminated UTF-8 C string
//!
//! @param dest   - Where to place descriptor
//!
//! @note Only handles UTF-16 characters that fit in a single UTF-16 'character'.
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

//===============================================================================
// Get Descriptor - Device Req 0x06
//       
static void handleGetDescriptor( void ) {
U8          descriptorIndex = ep0SetupBuffer.wValue.be.lo;
int         dataSize = 0;
const U8   *dataPtr = NULL;

   if (ep0SetupBuffer.bmRequestType != (EP_IN|RT_DEVICE)) {// In,Standard,Device
      epStall(0);
      return;
   }
   switch (ep0SetupBuffer.wValue.be.hi) {

      case DT_DEVICE: // Get Device Desc. - 1
         dataPtr  = (U8 *) &deviceDescriptor;
         dataSize = sizeof(DeviceDescriptor);
         break;
      case DT_CONFIGURATION: // Get Configuration Desc. - 2
         //dprint("hGDconf()\r\n");
         if (ep0SetupBuffer.wValue.be.lo != 0) {
            epStall(0);
            return;
         }
         dataPtr  = (U8 *) &otherDescriptors;
         dataSize = sizeof(otherDescriptors);
         break;
      case DT_DEVICEQUALIFIER: // Get Device Qualifier Descr.
         epStall(0);
         return;
      case DT_STRING: // Get String Desc.- 3
#ifdef MS_COMPATIBLE_ID_FEATURE    	  
    	 if (descriptorIndex == 0xEE) {
    		 dataPtr  = OS_StringDescriptor;
    	 }
    	 else
#endif    		 
         if (descriptorIndex >= sizeof(stringDescriptors)/sizeof(stringDescriptors[0])) {
            epStall(0); // Illegal string index - stall
            return;
         }
         if (descriptorIndex == 0) { // Language bytes
            dataPtr  = stringDescriptors[0];
         } 
         else { // Strings are stored in limited UTF-8 and need conversion
            dataPtr = commandBuffer;
            utf8ToStringDescriptor(stringDescriptors[descriptorIndex], commandBuffer);
         }
         dataSize = *dataPtr;
         break;
      default: // shouldn't happen
         epStall(0);
         return;
      } // switch
   ep0StartInTransaction( (U8)dataSize, dataPtr, DATA1 ); // Set up Tx
}

//===============================================================================
// resetDevice Callback to execute after status transaction
//       
static void resetDeviceCallback( void ) {
   forceICPReset();
}

//===============================================================================
// SetDeviceAddress Callback to execute after status transaction
//       
static void setAddressCallback( void ) {
   //dprint("setACB()");
   if ((deviceState.state == USBdefault) && (deviceState.newUSBAddress != 0)) {
      setUSBaddressedState(deviceState.newUSBAddress);
   }
   else if ((deviceState.state == USBaddressed) && (deviceState.newUSBAddress == 0)) {
      setUSBdefaultState();
   }
}

//===============================================================================
// Set device Address - Device Req 0x05
//       
static void handleSetAddress( void ) {

   if (ep0SetupBuffer.bmRequestType != (EP_OUT|RT_DEVICE)) {// Out,Standard,Device
      //dprint("hSA():inv. bmR");
      epStall(0); // Illegal format - stall ep0
      return;
   }
   // Save address for change after status transaction
   deviceState.newUSBAddress  = ep0SetupBuffer.wValue.be.lo; 
   ep0State.callback          = setAddressCallback;
   
   ep0StartInTransaction( 0, NULL, DATA1 ); // Tx empty Status packet
}

//===============================================================================
// Get Configuration - Device Req 0x08
//       
static void handleGetConfiguration( void ) {

   ep0StartInTransaction( 1, (U8 *) &deviceState.configuration, DATA1 );
}

//===============================================================================
// Set Configuration - Device Req 0x09
//       
static void handleSetConfiguration( void ) {

   //dprint("hSC()\r\n");
   if ((ep0SetupBuffer.bmRequestType != (EP_OUT|RT_DEVICE)) || // Out,Standard,Device
       ((ep0SetupBuffer.wValue.be.lo != 0) &&       // Only supports 0=> unconfigure, 1=> only valid configuration
        (ep0SetupBuffer.wValue.be.lo != otherDescriptors.configDescriptor.bConfigurationValue))) { 
      epStall(0);
      return;
      }
   setUSBconfiguredState(ep0SetupBuffer.wValue.be.lo);

   ep1InitialiseBDTOut();
//   initialiseEndpoints();

   ep0StartInTransaction( 0, NULL, DATA1 ); // Tx empty Status packet
}

//===============================================================================
// Get interface - Device Req 0x0A
//       
static void handleGetInterface( void ) {
U8 interfaceBuffer;

   if ((ep0SetupBuffer.bmRequestType != (EP_IN|RT_INTERFACE)) || // NOT In,Standard,Interface
       (ep0SetupBuffer.wLength.word != 1) ||                     // NOT correct length
       (deviceState.state != USBaddressed)) {                    // NOT in addressed state
      epStall(0); // Error
      return;
   }
   // Only support one interface
   if (ep0SetupBuffer.wValue.word != otherDescriptors.interfaceDescriptor0.bInterfaceNumber) {
      epStall(0); // Error
      return;
   }
   interfaceBuffer = (U8)deviceState.interfaceAltSetting;  // set up buffer for transfer
   ep0StartInTransaction( sizeof(interfaceBuffer), (U8 *) &interfaceBuffer, DATA1 ); // Send packet
}

#if (HW_CAPABILITY&CAP_CDC)
//static void handleGetEncapsulatedCommand() {
//   static const char dummy[] = "Hello there";
//   
//   ep0StartInTransaction( sizeof(dummy), dummy, DATA1 ); // Send packet
//}
//
//static void handleSendEncapsulatedCommand() {
//
//   ep0StartInTransaction( 0, NULL, DATA1 ); // Tx empty Status packet
//}

static void handleGetLineCoding() {

   ep0StartInTransaction( sizeof(LineCodingStructure), (const U8*)sciGetLineCoding(), DATA1 ); // Send packet
}

static void setLineCodingCallback( void ) {
   sciSetLineCoding((LineCodingStructure * const)&ep0OutDataBuffer);
}

static void handleSetLineCoding() {

   ep0State.callback          = setLineCodingCallback;
   // Don't use buffer - this requires sizeof(LineCodingStructure) < ENDPT0MAXSIZE
   ep0StartOutTransaction(sizeof(LineCodingStructure), NULL, DATA1); 
}

static void handleSetControlLineState() {

   sciSetControlLineState(ep0SetupBuffer.wValue.le.lo);
   ep0StartInTransaction( 0, NULL, DATA1 ); // Tx empty Status packet
}

static void handleSendBreak() {

   sciSendBreak(ep0SetupBuffer.wValue.word);  // time in milliseconds, 0xFFFF => continuous
   ep0StartInTransaction( 0, NULL, DATA1 );   // Tx empty Status packet
}
#endif

//=================================================
// Illegal request in SETUP pkt
//
static void handleUnexpected( void ) {
   epStall(0);
}
  
//===============================================================================
// Handles SETUP Packet
//       
static void handleSetupToken( void ) {
   // Save data from SETUP pkt
   ep0SetupBuffer = *(SetupPacket *)ep0OutDataBuffer;

   epHardwareState[0].state    = EPIdle;
   ep0State.callback           = NULL;
   
   // Convert SETUP values to big-endian
   SWAP16(ep0SetupBuffer.wLength);
   SWAP16(ep0SetupBuffer.wValue);
   SWAP16(ep0SetupBuffer.wIndex);

   switch(REQ_TYPE(ep0SetupBuffer.bmRequestType)) {
	   case REQ_TYPE_STANDARD :
		   // Standard device requests
		   switch (ep0SetupBuffer.bRequest) {
				case GET_STATUS :          handleGetStatus();			break;
				case CLEAR_FEATURE :       handleClearFeature();		break;
				case SET_FEATURE :         handleSetFeature();			break;
				case SET_ADDRESS :         handleSetAddress();			break;
				case GET_DESCRIPTOR :      handleGetDescriptor();		break;
				case GET_CONFIGURATION :   handleGetConfiguration(); 	break;
				case SET_CONFIGURATION :   handleSetConfiguration();	break;
				case GET_INTERFACE :       handleGetInterface();		break;
				case SET_DESCRIPTOR :
				case SET_INTERFACE :
				case SYNCH_FRAME :
				default :                  handleUnexpected();			break;
		   }
		   break;

	   case REQ_TYPE_CLASS :
		   // Class requests
		   switch (ep0SetupBuffer.bRequest) {
#if (HW_CAPABILITY&CAP_CDC)		   
//				case SEND_ENCAPSULATED_COMMAND : handleSendEncapsulatedCommand();	break;
//				case GET_ENCAPSULATED_COMMAND :  handleGetEncapsulatedCommand();	break;
				case SET_LINE_CODING :		 	 handleSetLineCoding();		      	break;
				case GET_LINE_CODING :		 	 handleGetLineCoding();		  		break;
				case SET_CONTROL_LINE_STATE: 	 handleSetControlLineState(); 		break;
				case SEND_BREAK:             	 handleSendBreak();           		break;
#endif		   
				default :					 	 handleUnexpected();		  		break;
		   }
		   break;

	   case REQ_TYPE_VENDOR :
		   // Handle special commands here
		   reInit = TRUE;  // tell command handler to re-init
		   switch (ep0SetupBuffer.bRequest) {
	          case ICP_GET_VER : {
			     U8 versionResponse[5];
				 versionResponse[0] = BDM_RC_OK; 
				 versionResponse[1] = VERSION_SW;      // BDM SW/HW version
				 versionResponse[2] = VERSION_HW;
				 versionResponse[3] = ICP_Version_SW;  // ICP SW/HW version
				 versionResponse[4] = ICP_Version_HW;
				 ep0StartInTransaction( sizeof(versionResponse),  versionResponse, DATA1 );
				 }
				 break;
	          case VENDOR_CODE:	        	  
                 // ToDo: The commented code should be used but seems to prevent the WCID process from completing!
                 //       Needs investigation & debugging to find reason
//	        	 if (REQ_RECIPIENT(ep0SetupBuffer.bmRequestType) != REQ_RECIPIENT_DEVICE) {
//		                handleUnexpected();
//		             }
//                 else 
                 if ((ep0SetupBuffer.wIndex.word) == (0x0004)) { 
					ep0StartInTransaction( sizeof(msCompatibleIdFeatureDescriptor),  (uint8_t *)&msCompatibleIdFeatureDescriptor, DATA1 );
				 }
				 else {//if ((ep0SetupBuffer.wIndex.word) == (0x0005)) { 
					ep0StartInTransaction( sizeof(msPropertiesFeatureDescriptor),  (uint8_t *)&msPropertiesFeatureDescriptor, DATA1 );
				 }
//	             else {
//	                handleUnexpected();
//	             }
	        	 break;
	          case CMD_USBDM_ICP_BOOT :
	        	  // Reboots to ICP mode
                  ep0State.callback = resetDeviceCallback;
                  ep0StartInTransaction( 0, NULL, DATA1 ); // Tx empty Status packet
	        	  break;   
			  default :                 
				  handleUnexpected();   
				  break;
		   }
		   break;

	   case REQ_TYPE_OTHER :
		   handleUnexpected();
		   break;
   }
   // Allow transactions post SETUP
   CTL = CTL_USBEN_MASK;
}

//==================================================================
// Handlers for Token Complete USB interrupt
//

//================
// ep0 - IN
// 
static void ep0HandleInToken( void ) {

   epHardwareState[0].data0_1 = !epHardwareState[0].data0_1;   // Toggle DATA0/1
	
   switch (epHardwareState[0].state) {
      case EPDataIn:    // Doing a sequence of IN packets (until data count <= EPSIZE)
         if ((ep0State.dataRemaining < ENDPT0MAXSIZE) ||   // Undersize pkt OR
             ((ep0State.dataRemaining == ENDPT0MAXSIZE) && // Full size AND
               !ep0State.shortInTransaction))              // Don't need to flag undersize transaction 
            epHardwareState[0].state = EPLastIn;    // Sending last pkt
         else 
            epHardwareState[0].state = EPDataIn;    // Sending full pkt
         ep0InitialiseBDTIn(); // Set up next IN pkt
         break;
         
      case EPLastIn:    
	     // Just done the last IN packet
//         ep0StartOutTransaction(0, NULL, DATA1);   // Do status Pkt reception //XXX
//         ep0ConfigureSetupTransaction();
         epHardwareState[0].state = EPStatusOut;   // Receiving an OUT status pkt
         break;
         
      case EPStatusIn:  
	     // Just done an IN packet as a status handshake for an OUT Data transfer
         epHardwareState[0].state = EPIdle;           // Now Idle
         if (ep0State.callback != NULL)
            ep0State.callback(); // Execute callback function to process OUT data
         ep0State.callback = NULL;
         break;

      // We don't expect an IN token while in the following states
      case EPIdle:           // Idle (Tx complete)
      case EPDataOut:        // Doing a sequence of OUT packets (until data count <= EPSIZE)
      case EPStatusOut:      // Doing an OUT packet as a status handshake
      default:
         break;
   }
}

//================
// ep0 - OUT
// 
static void ep0HandleOutToken( void ) {
U8 transferSize;

   epHardwareState[0].data0_1 = !epHardwareState[0].data0_1; // Toggle DATA0/1

   switch (epHardwareState[0].state) {
      case EPDataOut:        // Receiving a sequence of OUT packets
         transferSize = ep0SaveOutData();          // Save the data from the Rx buffer
         // Check if completed an under-size pkt or expected number of bytes
         if ((transferSize < ENDPT0MAXSIZE) || (ep0State.dataRemaining == 0)) { // Last pkt?
//            ep0ConfigureSetupTransaction(); //v4.10
            epHardwareState[0].state = EPIdle;
            ep0StartInTransaction(0, NULL, DATA1); // Do status Pkt transmission
            }
         else {
            ep0InitialiseBDTOut(epHardwareState[0].data0_1); // Set up next OUT pkt
            }
         break;

      case EPStatusOut:       // Done an OUT packet as a status handshake
//         ep0ConfigureSetupTransaction(); //v4.10
//         epHardwareState[0].state = EPIdle;
         break;
        
      // We don't expect an OUT token while in the following states
      case EPLastIn:          // Just done the last IN packet
      case EPDataIn:          // Doing a sequence of IN packets (until data count <= EPSIZE)
      case EPStatusIn:        // Just done an IN packet as a status handshake
      case EPIdle:            // Idle (Tx complete)
      default:
//         ep0ConfigureSetupTransaction(); //v4.10
         break;
   }
//   ep0EnsureReadyForSetupTransaction();  // Make ready for a SETUP pkt
}

//=================================================
// ep0 - STALL completed - re-enable ep0 for SETUP
// 
static void ep0HandleStallComplete( void ) {
   epClearStall(0);
   ep0ConfigureSetupTransaction(); // re-initialise EP0 OUT // v4.7
}

//===========================
// ep1 - OUT (host->device)
// 
static void ep1HandleOutToken( void ) {
U8 transferSize;

   epHardwareState[1].data0_1 = !epHardwareState[1].data0_1;   // Toggle DATA0/1
   
   switch (epHardwareState[1].state) {
      case EPDataOut:        // Doing a sequence of OUT packets making up a command
         transferSize = ep1SaveOutData();          // Save the data from the Rx buffer
         // Completed transfer on undersize pkt or received expected number of bytes
         if ((transferSize < ENDPT1MAXSIZE) || (ep1State.dataRemaining == 0)) { // Last pkt?
            epHardwareState[1].state = EPComplete;
         }
         else
            ep1InitialiseBDTOut(); // Set up next OUT pkt
         break;
       
         // We don't expect an OUT token while in the following states
      case EPIdle:           // Idle (Rx complete)
      case EPComplete:       // Command reception complete (Rx complete)
      case EPLastIn:         // Just done the last IN packet
      case EPDataIn:         // Doing a sequence of IN packets (until data count <= EPSIZE)
      case EPStatusIn:       // Just done an IN packet as a status handshake
      case EPStatusOut:      // Done an OUT packet as a status handshake
         break;
   }
}

//================
// ep2 - IN
// 
static void ep2HandleInToken( void ) {

   epHardwareState[2].data0_1 = !epHardwareState[2].data0_1;   // Toggle DATA0/1 for next pkt
   
   switch (epHardwareState[2].state) {
      case EPDataIn:    // Doing a sequence of IN packets (until data count <= EPSIZE)
         if (ep2State.dataRemaining < ENDPT2MAXSIZE) 
            epHardwareState[2].state = EPLastIn;    // Sending last pkt (may be empty)
         else 
            epHardwareState[2].state = EPDataIn;    // Sending full pkt
         ep2InitialiseBDTIn(); // Set up next IN pkt
         break;
         
      case EPLastIn:    // Just done the last IN packet
    	 if (commandBusyFlag)
            ep2StartInTransaction(sizeof(busyResponse), busyResponse);
    	 else
            epHardwareState[2].state = EPIdle;  // No more transactions expected
         break;
         
      // We don't expect an IN token while in the following states
      case EPIdle:           // Idle (Tx complete)
      case EPDataOut:        // Doing a sequence of OUT packets (until data count <= EPSIZE)
      case EPStatusIn:       // Just done an IN packet as a status handshake
      case EPStatusOut:      // Doing an OUT packet as a status handshake
      default:
         break;
   }
}

//U8 rtcCount = 0;
//#pragma TRAP_PROC
//void rtcHandler(void) {
//   RTCSC = RTCSC_RTIF_MASK|(2<<RTCSC_RTCLKS_BITNUM)|RTCSC_RTIE_MASK|(1<<RTCSC_RTCPS_BITNUM);
//   if (rtcCount++ >1) {
//      ep5StartInTransactionIfIdle();
//      rtcCount = 0;
//#if (DEBUG&USB_PING_DEBUG)
//   DEBUG_PIN = 0;
//#endif          
//   }
//}

//==================================================================
// Handler for Token Complete USB interrupt
//
// Handles ep0 [SETUP, IN & OUT]
// Handles ep1 [Out]
// Handles ep2 [In]
// Handles ep3 [In]
// Handles ep4 [Out]
// Handles ep5 [In]
// 
static void handleTokenComplete(U8 status) {
   
   U8 endPoint      = ((U8)status)>>4;             // Endpoint number
   U8 directionIsIn = status&STAT_IN_MASK;         // Direction of T/F 0=>OUT, (!=0)=>IN
   U8 isOddBuffer   = status&STAT_ODD_MASK;        // Odd/even buffer

   switch (endPoint) {
	   case 0: // Control - Accept IN, OUT or SETUP token
		  if (directionIsIn) { // IN Transaction complete
			 ep0HandleInToken();
		  }
		  else {
             doneEp0OutInit = FALSE;
             if (ep0BDTOut.control.a.bdtkpid == SETUPToken) { // SETUP transaction complete
                handleSetupToken();
             }
             else { // OUT Transaction
                ep0HandleOutToken();
             }
             if (!doneEp0OutInit) {
                ep0InitialiseBDTOut(DATA1);          // v4.10
             }
		  }
		  return;
	   case 1: // USBDM BDM - Accept OUT token
          usbActivityFlag.flags.bdmActive = 1;
          ep1HandleOutToken();
		  return;
	   case 2: // USBDM BDM - Accept IN token
          usbActivityFlag.flags.bdmActive = 1;
          ep2HandleInToken();
		  return;
#if (HW_CAPABILITY&CAP_CDC)
	   case 3: // USBDM CDC Control - Accept IN token
          epHardwareState[3].data0_1 = !epHardwareState[3].data0_1; // Toggle data0/1
          ep3StartInTransaction();
		  return;
	   case 4: // USBDM CDC Data - Accept OUT token
//		  usbActivityFlag.flags.serialOutActive = 1;
          if (sciTxBufferFree()) {
             ep4SaveOutData();
             ep4InitialiseBDTOut();
          }
          else {
             //! Throttle endpoint - send NAKs
             epHardwareState[4].state = EPThrottle;
          }
		  return;
	   case 5:  // USBD CDC Data - Accept IN token		   
//           usbActivityFlag.flags.serialInActive = 1;
#if 0
          if (epHardwareState[5].state == EPDataIn) {
             epHardwareState[5].state = EPLastIn;
          }
          else {
        	  epHardwareState[5].state = EPIdle;
          }
#else
#if (DEBUG&USB_PING_DEBUG)
          DEBUG_PIN = 1;
#endif          
          epHardwareState[5].state = EPIdle;
//          RTCMOD = 0;
//          rtcCount = 0;
#endif
//          ep5StartInTransactionIfIdle();
          return;
#endif
   }
}

#pragma MESSAGE DISABLE C4003
//==================================================================
// Handler for Start of Frame Token interrupt (~1ms interval)
//
static void handleSOFToken( void ) {
   // Green LED
   // Off                     - no USB activity, not connected
   // On                      - no USB activity, connected
   // Off, flash briefly on   - USB activity, not connected
   // On,  flash briefly off  - USB activity, connected
   if (FRMNUML==0) { // Every ~256 ms
      switch (FRMNUMH&0x03) {
         case 0:
            if (deviceState.state == USBconfigured) {
            	// Green LED on when USB connection established
            	GREEN_LED_ON(); 
            }
            else {
            	// Green LED off when no USB connection
            	GREEN_LED_OFF(); 
            }
            break;
         case 1:
         case 2:
            break;
         case 3:
         default :
            if (usbActivityFlag.flags.bdmActive) { 
               // Green LED flashes on USB activity
               GREEN_LED_TOGGLE();
               usbActivityFlag.byte = 0;
            }
            break;            
         }
   }
#if (HW_CAPABILITY&CAP_CDC)
   // Check if need to restart EP5 (CDC IN)
   ep5StartInTransactionIfIdle();
//   if (serialDelayCount++>SERIAL_THRESHOLD) {
//     ep5StartInTransactionIfIdle();
//   }
#endif
#if (HW_CAPABILITY&CAP_CDC) && 0
   // Check for need to restart idle EP3
   ep3StartInTransactionIfIdle();
#endif   
}
#pragma MESSAGE DEFAULT C4003
#if (HW_CAPABILITY&CAP_CDC)
void checkUsbCdcTxData(void) {
   // Check if we need to unThrottle EP4
   if ((epHardwareState[4].state == EPThrottle) && sciTxBufferFree()) {
	  ep4SaveOutData();        // Save data from last transfer
	  ep4InitialiseBDTOut();   // Set up next transfer
	  epHardwareState[4].state = EPDataOut;
   }
}
#endif
//==================================================================
// Handler for USB Suspend
//
// * Enables the USB module to wakeup the CPU 
// * Stops the CPU
// On wakeup
// * Re-checks the USB after a small delay to avoid wakeups by noise
//
static void handleUSBSuspend( void ) {

   INTSTAT_SLEEPF    = 1;   // Clear the sleep int flag                    
   // Need to disable BDM interface & everything else to reduce power
   bdm_suspend();
   deviceState.state = USBsuspended;

   // A re-check loop is used here to ensure USB bus noise doesn't wakeup the CPU
   for(;;) {
      INTSTAT = INTSTAT_RESUMEF_MASK;  // Clear resume int flag
      INTENB_RESUME     = 1;           // Enable resume detection interrupts on the USB bus
      USBCTL0_USBRESMEN = 1;           // Allow the USB module to wakeup the CPU
     
      asm ("stop");                    // Processor stop for low power
      // The CPU has woken up!

      INTSTAT = INTSTAT_RESUMEF_MASK;  // Clear resume int flag
      asm { // Wait 3 us
         ldhx     #(3*BUS_FREQ)/(4*1000)
      Loop:
         dbnzx    Loop  ; [4]
      }
      // We should have another resume int by now
      if (INTSTAT&INTSTAT_RESUMEF_MASK)
         break;
   }
   USBCTL0_USBRESMEN = 0;  

   return;
}

//==================================================================
// Handler for USB Resume
// 
// Disables further USB module wakeups
static void handleUSBResume( void ) {
   INTENB_RESUME     = 0;              // Mask further resume ints
   CTL               = CTL_USBEN_MASK; // Enable the transmit or receive of packets
   deviceState.state = USBconfigured;

   // Set up to receive setup packet
   ep0ConfigureSetupTransaction(); // re-initialise EP0 OUT // v4.7
   // power up BDM interface?
}

//==================================================================
// Handler for USB interrupt
// 
// Determines source and dispatches to appropriate routine.
//
//interrupt //VectorNumber_Vusb 
#pragma TRAP_PROC
//! Handler for USB interrupts
static uint8_t stat; // Capture Token status 
void USBInterruptHandler( void ) {
//U8 interruptFlags;

   while (INTSTAT != 0) {
      if ((INTSTAT&INTSTAT_TOKDNEF_MASK) != 0) { // Token complete int?
    	 stat = STAT; // Capture Token status 
         INTSTAT = INTSTAT_TOKDNEF_MASK; // Clear source
         handleTokenComplete(stat);
      }
      else if ((USBCTL0_LPRESF) && (deviceState.state==USBsuspended)) {
         USBCTL0_USBRESMEN = 0;
      }
      else if ((INTSTAT&INTSTAT_RESUMEF_MASK) != 0) { // Resume signalled on Bus?
         handleUSBResume();
         INTSTAT = INTSTAT_RESUMEF_MASK; // Clear source
      }
      else if ((INTSTAT&INTSTAT_USBRSTF_MASK) != 0) {
         handleUSBReset();
         INTSTAT = INTSTAT_USBRSTF_MASK; // Clear source
      }
      else if ((INTSTAT&INTSTAT_STALLF_MASK) != 0) { // Stall sent?
         ep0HandleStallComplete();
         INTSTAT = INTSTAT_STALLF_MASK; // Clear source
      }
      else if ((INTSTAT&INTSTAT_SOFTOKF_MASK) != 0) { // SOF Token?
         handleSOFToken();
         INTSTAT = INTSTAT_SOFTOKF_MASK; // Clear source
      }
      else if ((INTSTAT&INTSTAT_SLEEPF_MASK) != 0) { // Bus Idle 3ms? => sleep
         handleUSBSuspend();
         INTSTAT = INTSTAT_SLEEPF_MASK; // Clear source
      }

//      else  {
//         // unexpected int
//         INTSTAT = INTSTAT; // Clear & ignore
//      }
   }
}
