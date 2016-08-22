/*! \file
    \brief Simple USB Stack for JM60

   \verbatim
    Kinetis USB Code

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
Change History
+=================================================================================
| 15 Jul 2013 | Changed serial number to use chip UID
| 04 Sep 2011 | Ported to MK20DX128
+=================================================================================
\endverbatim
*/
#include <string.h>
#include <stdio.h>
#include "derivative.h" /* include peripheral declarations */
#include "Common.h"
#include "Configure.h"
#include "Commands.h"
//#include "BDM.h"
//#include "BDMCommon.h"
//#include "CmdProcessing.h"
#include "USBDefs.h"
#include "USB.h"
#include "ICP.h"
#include "CDC.h"

#if (HW_CAPABILITY&CAP_CDC)
static void ep3StartTxTransaction( void );
static void ep4InitialiseBdtRx( void );
static void ep5StartTxTransactionIfIdle();
#endif

#if CPU == MK20D5
   #define enableUSBIrq()   NVIC_EnableIRQ(USB0_IRQn)
   #define disableUSBIrq()  NVIC_DisableIRQ(USB0_IRQn)
#elif CPU == MKL25Z4
   #define enableUSBIrq()   NVIC_ISER = NVIC_ISER_SETENA(1<<((INT_USB0-16)%32));
   #define disableUSBIrq()  NVIC_ICER = NVIC_ICER_CLRENA(1<<((INT_USB0-16)%32));
#else
   #error "CPU not set"
#endif

uint8_t commandBuffer[300];

#ifdef __BYTE_ORDER__
   #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
      __inline
      static uint32_t swap32(uint32_t data) {
         return ((data<<24)&0xFF000000)|((data<<8)&0x00FF0000)|
               ((data>>24)&0x000000FF)|((data>>8)&0x0000FF00);
      }
      __inline
      static uint16_t swap16(uint16_t data) {
         return ((data<<16)&0xFF00)|((data>>8)&0xFF);
      }
      #define leToNative32(x) swap32(x)
      #define leToNative16(x) swap16(x)
      #define nativeToLe32(x) swap32(x)
      #define nativeToLe16(x) swap16(x)
   #elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      __inline
      static uint32_t noChange32(uint32_t data) {
         return data;
      }
      __inline
      static uint16_t noChange16(uint16_t data) {
         return data;
      }
      #define leToNative32(x) noChange32(x)
      #define leToNative16(x) noChange16(x)
      #define nativeToLe32(x) noChange32(x)
      #define nativeToLe16(x) noChange16(x)
   #endif
#else
#error "Please define __BIG_ENDIAN__ or __LITTLE_ENDIAN__"
#endif

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
// Data packet odd/even indicator
enum {DATA0=0, //!< DATA0 indicator
      DATA1=1  //!< DATA1 indicator
     };

//======================================================================
//! Structure representing a BDT entry in USB controller
//!
#pragma pack(1)
#if 1
// Little-endian on Kinetis
typedef struct {
   union {
      volatile uint8_t bits:8;   // Access a bit masks
      volatile struct {          // BDT setup access
         uint8_t :2;
         uint8_t bdt_stall:1;
         uint8_t dts:1;
         uint8_t ninc:1;
         uint8_t keep:1;
         uint8_t data0_1:1;
         uint8_t own:1;
      } setup;
      volatile struct {          // BDT result access
         uint8_t :2;
         uint8_t tok_pid:4;
         uint8_t data0_1:1;
         uint8_t own:1;
      } result;
   } u;
   volatile uint8_t  :8;
   volatile uint16_t bc;          // Byte count
   volatile uint32_t addr;        // buffer address
} BdtEntry ;
#else
// Big-endian (Used on Coldfire)
typedef struct {
   union {
      struct {
         uint8_t rsvd:2;
         uint8_t bdt_stall:1;
         uint8_t dts:1;
         uint8_t ninc:1;
         uint8_t keep:1;
         uint8_t data0_1:1;
         uint8_t own:1;
      } setup;           // BDT setup access
      struct {
         uint8_t rsvd:2;
         uint8_t tok_pid:4;
         uint8_t data0_1:1;
         uint8_t own:1;
      } result;          // BDT result access
      uint8_t bits;      // Access as bit masks
   } u;
   uint8_t  rsvd2:8;
   uint16_t bc:16;       // Byte count
   uint32_t addr;        // Buffer address
} BdtEntry ;
#endif
#pragma pack(0)

#define USB_INTMASKS \
   (USB_INTEN_STALLEN_MASK|USB_INTEN_TOKDNEEN_MASK|\
    USB_INTEN_SOFTOKEN_MASK|USB_INTEN_USBRSTEN_MASK|\
    USB_INTEN_SLEEPEN_SHIFT)  //!< Mask for all USB interrupts

typedef struct {
   BdtEntry rxEven;
   BdtEntry rxOdd;
   BdtEntry txEven;
   BdtEntry txOdd;
} EndpointBdtEntry;

// Bit masks bits field in above
#define BDTEntry_OWN_MASK        (1<<7)   //!< Mask for OWN bit in BDT
#define BDTEntry_DATA0_MASK      (0<<6)   //!< Mask for DATA1 bit in BDT (dummy)
#define BDTEntry_DATA1_MASK      (1<<6)   //!< Mask for DATA0 bit in BDT
#define BDTEntry_KEEP_MASK       (1<<5)   //!< KEEP
#define BDTEntry_NINC_MASK       (1<<4)   //!< NINC
#define BDTEntry_DTS_MASK        (1<<3)   //!< Mask for DTS bit in BDT
#define BDTEntry_STALL_MASK      (1<<2)   //!< Stall endpoint

//======================================================================
// Buffers for endpoint data packets (in USB RAM)

uint8_t ep0InDataBuffer[ENDPT0MAXSIZE];
uint8_t ep0OutDataBuffer[ENDPT0MAXSIZE];
uint8_t ep1DataBuffer[ENDPT1MAXSIZE];     // #01,OUT,Bulk   - BDM Command & data out
uint8_t ep2DataBuffer[ENDPT2MAXSIZE];     // #82,IN, BULK   - BDM data in
#if (HW_CAPABILITY&CAP_CDC)
uint8_t ep3DataBuffer[ENDPT3MAXSIZE];     // #83,IN,interrupt CDC status
uint8_t ep4DataBuffer[ENDPT4MAXSIZE];     // #04,OUT,bulk     CDC Tx
uint8_t ep5DataBuffer0[ENDPT5MAXSIZE];    // #85,IN,bulk      CDC Rx
uint8_t ep5DataBuffer1[ENDPT5MAXSIZE];
#endif

//__attribute__ ((section(".user_data2")))
static EndpointBdtEntry endPointBdts[16] __attribute__ ((aligned (512)));
// Raw BDTS (0 - 4*#endpoints)
#define BDTS(n) (*((&endPointBdts[0].rxEven)+(n)))

//======================================================================
// Descriptors
//
static const DeviceDescriptor deviceDescriptor = {
   sizeof(DeviceDescriptor),               // bLength
   DT_DEVICE,                              // bDescriptorType
   CONST_NATIVE_TO_LE16(0x0200),           // bcdUSB              = USB spec rel. No.      [BCD = 2.00]
#if (HW_CAPABILITY&CAP_CDC)
   0xEF,                                   // bDeviceClass        = Device Class code [Miscellaneous Device Class]
   0x02,                                   // bDeviceSubClass     = Sub Class code    [Common Class]
   0x01,                                   // bDeviceProtocol     = Protocol          [Interface Association Descriptor]
#else
   0xFF,                                   // Class code             [none]
   0xFF,                                   // Sub Class code         [none]
   0xFF,                                   // Protocol               [none]
#endif
   ENDPT0MAXSIZE,                          // bMaxPacketSize0    = EndPt 0 max packet size
   CONST_NATIVE_TO_LE16(VendorID),         // idVendor           = Vendor ID
#if (HW_CAPABILITY&CAP_CDC)
   CONST_NATIVE_TO_LE16(ProductID_CDC),    // idProduct          = Product ID for Composite device
#else
   CONST_NATIVE_TO_LE16(ProductID),        // idProduct          = Product ID
#endif
   CONST_NATIVE_TO_LE16(VersionID),        // bcdDevice          = Device Release         [BCD = 4.10]
   1,                                      // iManufacturer      = String index of Manufacturer name
   2,                                      // iProduct           = String index of product desc.
   3,                                      // iSerialNumber      = String index desc. serial #
   1                                       // bNumConfigurations = Number of configurations
};

static const struct {
   ConfigurationDescriptor                  configDescriptor;
   InterfaceDescriptor                      interfaceDescriptor0;
   EndpointDescriptor                       endpointDescriptor1;
   EndpointDescriptor                       endpointDescriptor2;
#if (HW_CAPABILITY&CAP_CDC)
   InterfaceAssociationDescriptor           interfaceAssociationDescriptorCDC;
   InterfaceDescriptor                      interfaceDescriptor1;
   CDCHeaderFunctionalDescriptor            headerFunctionalDescriptor;
   CDCCallManagementFunctionalDescriptor    callManagementDescriptor;
   CDCAbstractControlManagementDescriptor   abstractControlManagementFunctionalDescriptor;
   CDCUnionFunctionalDescriptor             unionFunctionalDescriptor;
   EndpointDescriptor                       endpointDescriptor3;
   InterfaceDescriptor                      interfaceDescriptor2;
   EndpointDescriptor                       endpointDescriptor4;
   EndpointDescriptor                       endpointDescriptor5;
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
     USBMilliseconds(1)                   // bInterval         = -
   },
   { // endpointDescriptor2 - #82,IN, BULK
     sizeof(EndpointDescriptor),          // bLength
     DT_ENDPOINT,                         // bDescriptorType
     EP_IN|2,                             // bEndpointAddress
     ATTR_BULK,                           // bmAttributes
     CONST_NATIVE_TO_LE16(ENDPT2MAXSIZE), // wMaxPacketSize
     USBMilliseconds(1)                   // bInterval         = -
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
      CS_INTERFACE,                                  // bDescriptorType
      DST_CALL_MANAGEMENT,                           // bDescriptorSubtype
      1,                                             // bmCapabilities
      1,                                             // bDataInterface
   },
   { // abstractControlManagementFunctionalDescriptor
      sizeof(CDCAbstractControlManagementDescriptor), // bFunctionalLength
      CS_INTERFACE,                                   // bDescriptorType
      DST_ABSTRACT_CONTROL_MANAGEMENT,                // bDescriptorSubtype
      0x06,                                           // bmCapabilities
   },
   { // unionFunctionalDescriptor
      sizeof(CDCUnionFunctionalDescriptor),           // bFunctionalLength
      CS_INTERFACE,                                   // bDescriptorType
      DST_UNION_MANAGEMENT,                           // bDescriptorSubtype
      1,                                              // bmControlInterface
      {2},                                            // bSubordinateInterface0
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
   { // endpointDescriptor4 - #4,OUT,bulk
     sizeof(EndpointDescriptor),          // bLength
     DT_ENDPOINT,                         // bDescriptorType
     EP_OUT|4,                            // bEndpointAddress
     ATTR_BULK,                           // bmAttributes
     CONST_NATIVE_TO_LE16(ENDPT4MAXSIZE), // wMaxPacketSize
     USBMilliseconds(1)                   // bInterval         = -
   },
   { // endpointDescriptor5 - #85,IN,bulk
     sizeof(EndpointDescriptor),            // bLength
     DT_ENDPOINT,                           // bDescriptorType
     EP_IN|5,                               // bEndpointAddress
     ATTR_BULK,                             // bmAttributes
     CONST_NATIVE_TO_LE16(2*ENDPT5MAXSIZE), // wMaxPacketSize (x2 so all pkts are terminating (short))
     USBMilliseconds(1)                     // bInterval         = -
   },
#endif
};

#ifdef MS_COMPATIBLE_ID_FEATURE

#if WCHAR_MAX != 65535
#error "Wide chars must be 16-bits for this file (use -fshort-wchar option)"
#endif

#define DeviceInterfaceGUIDs L"DeviceInterfaceGUIDs"
#define DeviceGUID           L"{93FEBD51-6000-4E7E-A20E-A80FC78C7EA1}\0"
#define ICONS                L"Icons"
#define ICON_PATH            L"\%SystemRoot\%\\system32\\SHELL32.dll,-4\0"

typedef struct {
   uint32_t lLength;                 //!< Size of this Descriptor in Bytes
   uint16_t wVersion;                //!< Version
   uint16_t wIndex;                  //!< Index (must be 4)
   uint8_t  bnumSections;            //!< Number of sections
   uint8_t  bReserved1[7];           //!<
   //------------- Section ----------//
   uint8_t  bInterfaceNum;           //!<
   uint8_t  bReserved2;              //!<
   uint8_t  bCompatibleId[8];        //!<
   uint8_t  bSubCompatibleId[8];     //!<
   uint8_t  bReserved3[6];

} MS_CompatibleIdFeatureDescriptor;

// See https://github.com/pbatard/libwdi/wiki/WCID-Devices
//
static const MS_CompatibleIdFeatureDescriptor msCompatibleIdFeatureDescriptor = {
    /* lLength;             */  CONST_NATIVE_TO_LE32((uint32_t)sizeof(MS_CompatibleIdFeatureDescriptor)),
    /* wVersion;            */  CONST_NATIVE_TO_LE16(0x0100),
    /* wIndex;              */  CONST_NATIVE_TO_LE16(0x0004),
    /* bnumSections;        */  1,
    /*---------------------- Section 1 -----------------------------*/
    /* bReserved1[7];       */  {0},
    /* bInterfaceNum;       */  0,
    /* bReserved2;          */  1,
    /* bCompatibleId[8];    */  "WINUSB\0",
    /* bSubCompatibleId[8]; */  {0},
    /* bReserved3[6];       */  {0}
};

typedef struct {
   uint32_t lLength;                //!< Size of this Descriptor in Bytes
   uint16_t wVersion;               //!< Version
   uint16_t wIndex;                 //!< Index (must be 5)
   uint16_t bnumSections;           //!< Number of property sections
   /*---------------------- Section 1 -----------------------------*/
   uint32_t lPropertySize0;          //!< Size of property section
   uint32_t ldataType0;              //!< Data type (1 = Unicode REG_SZ etc
   uint16_t wNameLength0;            //!< Length of property name
   wchar_t  bName0[sizeof(DeviceInterfaceGUIDs)/sizeof(wchar_t)];
   uint32_t wPropertyLength0;        //!< Length of property data
   wchar_t  bData0[sizeof(DeviceGUID)/sizeof(wchar_t)];
   /*---------------------- Section 2 -----------------------------*/
   uint32_t lPropertySize1;          //!< Size of property section
   uint32_t ldataType1;              //!< Data type (1 = Unicode REG_SZ etc
   uint16_t wNameLength1;            //!< Length of property name
   wchar_t  bName1[sizeof(ICONS)/sizeof(wchar_t)];
   uint32_t wPropertyLength1;        //!< Length of property data
   wchar_t  bData1[sizeof(ICON_PATH)/sizeof(wchar_t)];
} MS_PropertiesFeatureDescriptor;

static const MS_PropertiesFeatureDescriptor msPropertiesFeatureDescriptor = {
    /* U32 lLength;         */ CONST_NATIVE_TO_LE32((uint32_t)sizeof(MS_PropertiesFeatureDescriptor)),
    /* U16 wVersion;        */ CONST_NATIVE_TO_LE16(0x0100),
    /* U16 wIndex;          */ CONST_NATIVE_TO_LE16(5),
    /* U16 bnumSections;    */ CONST_NATIVE_TO_LE16(2),
    /*---------------------- Section 1 -----------------------------*/
    /* U32 lPropertySize0;  */ CONST_NATIVE_TO_LE32(
          sizeof(msPropertiesFeatureDescriptor.lPropertySize0)+
          sizeof(msPropertiesFeatureDescriptor.ldataType0)+
          sizeof(msPropertiesFeatureDescriptor.wNameLength0)+
          sizeof(msPropertiesFeatureDescriptor.bName0)+
          sizeof(msPropertiesFeatureDescriptor.wPropertyLength0)+
          sizeof(msPropertiesFeatureDescriptor.bData0)
          ),
    /* U32 ldataType0;       */ CONST_NATIVE_TO_LE32(7UL), // 7 == REG_MULTI_SZ
    /* U16 wNameLength0;     */ CONST_NATIVE_TO_LE16(sizeof(msPropertiesFeatureDescriptor.bName0)),
    /* U8  bName0[42];       */ DeviceInterfaceGUIDs,
    /* U32 wPropertyLength0; */ CONST_NATIVE_TO_LE32(sizeof(msPropertiesFeatureDescriptor.bData0)),
    /* U8  bData0[78];       */ DeviceGUID,
    /*---------------------- Section 2 -----------------------------*/
    /* U32 lPropertySize1;   */ CONST_NATIVE_TO_LE32(
          sizeof(msPropertiesFeatureDescriptor.lPropertySize1)+
          sizeof(msPropertiesFeatureDescriptor.ldataType1)+
          sizeof(msPropertiesFeatureDescriptor.wNameLength1)+
          sizeof(msPropertiesFeatureDescriptor.bName1)+
          sizeof(msPropertiesFeatureDescriptor.wPropertyLength1)+
          sizeof(msPropertiesFeatureDescriptor.bData1)
          ),
    /* U32 ldataType1;       */ CONST_NATIVE_TO_LE32(7UL), // 7 == REG_MULTI_SZ
    /* U16 wNameLength1;     */ CONST_NATIVE_TO_LE16(sizeof(msPropertiesFeatureDescriptor.bName1)),
    /* U8  bName1[];         */ ICONS,
    /* U32 wPropertyLength1; */ CONST_NATIVE_TO_LE32(sizeof(msPropertiesFeatureDescriptor.bData1)),
    /* U8  bData1[];         */ ICON_PATH,
};

#define VENDOR_CODE 0x30
static const uint8_t OS_StringDescriptor[] = {18, DT_STRING, 'M',0,'S',0,'F',0,'T',0,'1',0,'0',0,'0',0,VENDOR_CODE,0x00};

#endif

static const uint8_t sd0[] = {4,  DT_STRING, 0x09, 0x0C};  // Language IDs
static const uint8_t sd1[] = "pgo";                        // Manufacturer
static const uint8_t sd2[] = ProductDescription;           // Product Description
static const uint8_t sd3[] = SERIAL_NO;                    // Serial Number
static const uint8_t sd4[] = "USBDM BDM Interface";        // Interface Association #1
static const uint8_t sd5[] = "Interface 0 - USBDM";        // Interface #0
static const uint8_t sd6[] = "USBDM CDC Interface";        // Interface Association #2
static const uint8_t sd7[] = "Interface 1 - CDC Control";  // Interface #1
static const uint8_t sd8[] = "Interface 2 - CDC Data";     // Interface #2

static const uint8_t *const stringDescriptors[] = {sd0, sd1, sd2, sd3, sd4, sd5, sd6, sd7, sd8};

//===============================================================================
// Device Status
//
typedef struct {
   int selfPowered  : 1;    //!< Device is self-powered
   int remoteWakeup : 1;    //!< Supports remote wake-up
   int portTest     : 1;    //!< Port test
   int res1         : 5;    //!< Reserved
   int res2         : 8;    //!< Reserved
} DeviceStatus;

//! USB device states
typedef enum { USBpowered, USBattached, USBdefault, USBaddressed,
               USBconfigured, USBsuspended } USBstateType;

struct {
   USBstateType   state:8;
   uint8_t        configuration;
   uint8_t        interfaceAltSetting;
   DeviceStatus   status;
   uint8_t        newUSBAddress;
} deviceState = {USBattached, 0, 0, {0,0,0,0,0}, 0};  //!< USB device state information

//===============================================================================
//! Endpoint Status
//!
typedef struct {
   int stall  : 1;   //!< Endpoint is stalled
   int res1   : 7;   //!< Reserved
   int res2   : 8;   //!< Reserved
} EPStatus; //!< Endpoint status in USB format

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
   uint8_t*  volatile       dataPtr;               //!< Pointer to data buffer
   uint16_t  volatile       dataRemaining;         //!< Count of remaining bytes to Rx/Tx
   uint16_t  volatile       dataCount;             //!< Count of bytes Rx/Tx so far
   uint8_t   volatile       shortInTransaction:1;  //!< Indicates that the IN transaction is undersized
   void   (*volatile callback)( void );            //!< Callback used on completion of packet reception
} EPState;

//! Endpoint hardware state
typedef volatile struct {
   uint8_t   data0_1:1;   //!< Data 0/1 toggle state
   uint8_t   txOdd:1;     //!< Odd/Even tx buffer
   uint8_t   rxOdd:1;     //!< Odd/Even rx buffer
   EPModes    state:5;    //!< End-point state
} EPHardwareState;

// Used to flag USB system configuration change etc.
static volatile uint8_t reInit;

static volatile EPState ep0State = {NULL, 0, 0, 0, NULL};
static volatile EPState ep1State = {NULL, 0, 0, 0, NULL};
static volatile EPState ep2State = {NULL, 0, 0, 0, NULL};

static EPHardwareState epHardwareState[NUMBER_OF_EPS] = {{0}};

//! Structure representing USB activity
typedef union {
   volatile uint8_t  byte;    //!< Allows access as byte (zero=>no activity, non-zero=>activity)
   volatile struct {
      int bdmActive:1;            //!< Any activity
      int serialOutActive:1;      //!< Serial out active
      int serialInActive:1;       //!< Serial in active
   } flags;                 //!< Overall flags
} ActivityType ;
static ActivityType  usbActivityFlag;

/*
 * =================================================================
 * Used for debugging suspend/resume
 */
//#define SUSPEND_RESUME_TESTING
#ifdef SUSPEND_RESUME_TESTING
static void resLow() {
   RESET_OUT_PCOR =  RESET_OUT_MASK;
}

static void resHigh() {
   RESET_OUT_PSOR =  RESET_OUT_MASK;
}

static void resInit() {
   resLow();
   RESET_OUT_PCR   = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK;
   RESET_OUT_PDDR |= RESET_OUT_MASK;
}

static void delay(void) {
   int i;
   for(i=0; i<1000; i++) {
      __asm__("nop");
   }
}
#endif

/*
 * =================================================================
 * Buffer for EP0 Setup packet (copied from USB RAM)
 */
static SetupPacket ep0SetupBuffer;   //!< Buffer for EP0 Setup packet (copied from USB RAM)

/*
 * =================================================================
 * initialise the endpoint buffer pointers once only
 */
static void initEndpointBuffers(void) {
   // EP0 Control I/O
   endPointBdts[0].txEven.addr = nativeToLe32((uint32_t)ep0InDataBuffer);
   endPointBdts[0].txOdd.addr  = nativeToLe32((uint32_t)ep0InDataBuffer);
   endPointBdts[0].rxEven.addr = nativeToLe32((uint32_t)ep0OutDataBuffer);
   endPointBdts[0].rxOdd.addr  = nativeToLe32((uint32_t)ep0OutDataBuffer);
   // EP1 USBDM out
   endPointBdts[1].rxEven.addr = nativeToLe32((uint32_t)ep1DataBuffer);
   endPointBdts[1].rxOdd.addr  = nativeToLe32((uint32_t)ep1DataBuffer);
   // EP2 USBDM in
   endPointBdts[2].txEven.addr = nativeToLe32((uint32_t)ep2DataBuffer);
   endPointBdts[2].txOdd.addr  = nativeToLe32((uint32_t)ep2DataBuffer);

#if (HW_CAPABILITY&CAP_CDC)
   // EP3 CDC Control
   endPointBdts[3].rxEven.addr = nativeToLe32((uint32_t)ep3DataBuffer);
   endPointBdts[3].rxOdd.addr  = nativeToLe32((uint32_t)ep3DataBuffer);
   // EP4 CDC Rx (out)
   endPointBdts[4].rxEven.addr = nativeToLe32((uint32_t)ep4DataBuffer);
   endPointBdts[4].rxOdd.addr  = nativeToLe32((uint32_t)ep4DataBuffer);
   // EP5 CDC Tx (in)
   endPointBdts[5].txEven.addr = nativeToLe32((uint32_t)ep5DataBuffer0);
   endPointBdts[5].txOdd.addr  = nativeToLe32((uint32_t)ep5DataBuffer1);
   (void)cdc_setRxBuffer((char *)ep5DataBuffer0);
#endif
}

static void epClearStall(uint8_t epNum);

//======================================================================
//! Configure the BDT for EP0 Out [Rx, device <- host, DATA0/1]
//!
//! @param data0_1 - value for USB Data toggle
//!
static void ep0InitialiseBdtRx( uint8_t data0_1 ) {
   // Set up to Rx packet
   BdtEntry *bdt = epHardwareState[0].rxOdd?&endPointBdts[0].rxOdd:&endPointBdts[0].rxEven;
   bdt->bc = ENDPT0MAXSIZE; // Always use ENDPT0MAXSIZE so can accept SETUP pkt
   if (data0_1) {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
      PUTS("ep0BdtRx.1");
   }
   else {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
      PUTS("ep0BdtRx.0");
   }
}

//=========================================================================
//! Save the data from an EP0 OUT pkt and advance ptrs etc.
//!
static uint8_t ep0SaveRxData( void ) {
   // Set up to Rx packet
   BdtEntry *bdt = (!epHardwareState[0].rxOdd)?&endPointBdts[0].rxOdd:&endPointBdts[0].rxEven;
   uint16_t size = bdt->bc;
   if (size > 0) {
      // Check if more data than requested - discard excess
      if (size > ep0State.dataRemaining) {
         size = ep0State.dataRemaining;
      }
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
static void ep0StartRxTransaction( uint8_t bufSize, uint8_t *bufPtr ) {

   ep0State.dataRemaining     = bufSize; // Total bytes for Rx
   ep0State.dataCount         = 0;       // Reset count of bytes so far
   ep0State.dataPtr           = bufPtr;  // Where to (eventually) place data
   epHardwareState[0].data0_1 = DATA1;   // Initial data toggle

   if (bufSize == 0) {
      epHardwareState[0].state = EPStatusOut;  // Assume status handshake
   }
   else {
      epHardwareState[0].state = EPDataOut;    // Assume first of several data pkts
   }
   ep0InitialiseBdtRx(epHardwareState[0].data0_1); // Configure the BDT for transfer
}
#endif

//================================================================================
// Configure EP0-out for a SETUP transaction [Rx, device<-host, DATA0]
// Endpoint state is changed to EPIdle
//
static void ep0ConfigureSetupTransaction( void ) {
   PUTS("ep0ConfigSetupTr - ");
   // Set up EP0-RX to Rx SETUP packets
   ep0InitialiseBdtRx(DATA0);
   epHardwareState[0].state = EPIdle;
}

//================================================================================
// Configure EP0-RX for a SETUP transaction [Rx, device<-host, DATA0]
// Only done if endpoint is not already configured for some other OUT transaction
// Endpoint state unchanged.
//
static void ep0EnsureReadyForSetupTransaction( void ) {
   EPModes currentEp0State = epHardwareState[0].state;

   switch (currentEp0State) {
      case EPDataOut:        // Doing a sequence of OUT packets (until data count <= EPSIZE)
      case EPStatusOut:      // Doing an OUT packet as a status handshake
         // EP0-OUT is already set up for an OUT pkt
         break;

      case EPStall:          // Stalled
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
}

//======================================================================
// Configure the BDT for EP0 In [Tx, device -> host]
//
static void ep0InitialiseBdtTx( void ) {
   uint16_t size = ep0State.dataRemaining;
   if (size > ENDPT0MAXSIZE) {
      size = ENDPT0MAXSIZE;
   }
   // Copy the Tx data to EP buffer
   ( void )memcpy(ep0InDataBuffer, ep0State.dataPtr, size);

   ep0State.dataPtr         += size;  // Ptr to _next_ data
   ep0State.dataRemaining   -= size;  // Count of remaining bytes
   ep0State.dataCount       += size;  // Count of bytes so far

   // Set up to Tx packet
   BdtEntry *bdt = epHardwareState[0].txOdd?&endPointBdts[0].txOdd:&endPointBdts[0].txEven;
   bdt->bc = size;
   if (epHardwareState[0].data0_1) {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
      PRINTF("ep0BdtTx %d.1\n", size);
   }
   else {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
      PRINTF("ep0BdtTx %d.0\n", size);
   }
}

//======================================================================
//! Configure EP0 for an IN transaction [Tx, device -> host, DATA0/1]
//!
static void ep0StartTxTransaction( uint16_t bufSize, const uint8_t *bufPtr ) {

   if (bufSize > ep0SetupBuffer.wLength.word) { // More data than requested - truncate
      bufSize = (uint8_t)ep0SetupBuffer.wLength.word;
   }
   ep0State.dataPtr            = (uint8_t*)bufPtr;    // Pointer to _next_ data
   ep0State.dataRemaining      = bufSize;             // Count of remaining bytes
   ep0State.dataCount          = 0;                   // Reset count of bytes so far
   epHardwareState[0].data0_1  = DATA1;               // Initial data toggle
   ep0State.shortInTransaction = bufSize < ep0SetupBuffer.wLength.word; // Short transaction?

   if (bufSize == 0) {
      epHardwareState[0].state = EPStatusIn;   // Assume status handshake
   }
   else if ((bufSize < ENDPT0MAXSIZE) ||       // Undersize packet OR
            ((bufSize == ENDPT0MAXSIZE) &&     // Full size AND
             !ep0State.shortInTransaction)) {  //   Don't need to flag undersize transaction
      epHardwareState[0].state = EPLastIn;     // Sending one and only packet
   }
   else {
      epHardwareState[0].state = EPDataIn;     // Sending first of several packets
   }
   ep0InitialiseBdtTx(); // Configure the BDT for transfer
}

//======================================================================
// Configure the BDT for EP1 Out [Rx, device <- host, DATA0/1]
//
static void ep1InitialiseBdtRx( void ) {
   // Set up to Rx packet
   BdtEntry *bdt = epHardwareState[1].rxOdd?&endPointBdts[1].rxOdd:&endPointBdts[1].rxEven;

   // Set up to Rx packet
   bdt->bc = ENDPT1MAXSIZE;
   if (epHardwareState[1].data0_1) {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
}

//=========================================================================
// Save the data from an EP1 OUT pkt and advance ptrs etc.
//
static uint8_t ep1SaveRxData( void ) {
   // Get BDT
   BdtEntry *bdt = (!epHardwareState[1].rxOdd)?&endPointBdts[1].rxOdd:&endPointBdts[1].rxEven;
   uint8_t size = bdt->bc;

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

//======================================================================
/*! Configure EP1-out for an OUT transaction [Rx, device <- host, DATA0/1]
 *
 * @param bufSize - Size of data to transfer
 * @param bufPtr  - Buffer for data
 */
static void ep1StartRxTransaction( uint8_t bufSize, uint8_t *bufPtr ) {
   ep1State.dataRemaining  = bufSize; // Total bytes to Rx
   ep1State.dataCount      = 0;       // Reset count of bytes so far
   ep1State.dataPtr        = bufPtr;  // Where to (eventually) place data

   epHardwareState[1].state = EPDataOut;    // Assume first of several data pkts

   ep1InitialiseBdtRx(); // Configure the BDT for transfer
}

//======================================================================
// Configure the BDT for EP2 In [Tx, device -> host]
//
static void ep2InitialiseBdtTx( void ) {
   // Set up to Rx packet
   BdtEntry *bdt = epHardwareState[2].txOdd?&endPointBdts[2].txOdd:&endPointBdts[2].txEven;

   uint16_t size = ep2State.dataRemaining;
   if (size > ENDPT2MAXSIZE) {
      size = ENDPT2MAXSIZE;
   }
   // Copy the Tx data to EP buffer
   (void) memcpy(ep2DataBuffer, ep2State.dataPtr, size);

   ep2State.dataPtr         += size;  // Ptr to _next_ data
   ep2State.dataRemaining   -= size;  // Count of remaining bytes
   ep2State.dataCount       += size;  // Count of bytes so far

   // Set up to Tx packet
   bdt->bc     = (uint8_t)size;
   if (epHardwareState[2].data0_1) {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
}

static const uint8_t busyResponse[] = {BDM_RC_BUSY,1,2,3};
static uint8_t commandBusyFlag = false;

//======================================================================
// Configure EP2 for an IN transaction [Tx, device -> host, DATA0/1]
//
static void ep2StartTxTransaction( uint8_t bufSize, const uint8_t *bufPtr ) {

   ep2State.dataPtr            = (uint8_t*)bufPtr;    // Pointer to _next_ data
   ep2State.dataRemaining      = bufSize;             // Count of remaining bytes
   ep2State.dataCount          = 0;                   // Reset count of bytes so far

   // Note - Always terminate transfers with a truncated/zero packet
   if (bufSize < ENDPT2MAXSIZE) {             // Undersize packet OR
      epHardwareState[2].state = EPLastIn;    // Sending one and only packet
   }
   else {
      epHardwareState[2].state = EPDataIn;    // Sending first of several packets
   }
   ep2InitialiseBdtTx(); // Configure the BDT for transfer
}


//======================================================================
// (re)Initialises end-points
//
static void initialiseEndpoints(void) {

   PUTS("initialiseEndpoints()");

   // Clear USB BDTS
   memset((uint8_t*)endPointBdts, 0, sizeof(endPointBdts));

   epHardwareState[0].rxOdd = 0;
   epHardwareState[0].txOdd = 0;
   epHardwareState[1].rxOdd = 0;
   epHardwareState[1].txOdd = 0;
   epHardwareState[2].rxOdd = 0;
   epHardwareState[2].txOdd = 0;
#if (HW_CAPABILITY&CAP_CDC)
   epHardwareState[3].rxOdd = 0;
   epHardwareState[3].txOdd = 0;
   epHardwareState[4].rxOdd = 0;
   epHardwareState[4].txOdd = 0;
   epHardwareState[5].rxOdd = 0;
   epHardwareState[5].txOdd = 0;
#endif

   initEndpointBuffers();
   epClearStall(0);
   epClearStall(1);  reInit = true;
   epClearStall(2);

#if (HW_CAPABILITY&CAP_CDC)
   epClearStall(3);
   epClearStall(4);
   epClearStall(5);
#endif

#if (HW_CAPABILITY&CAP_CDC)
   ep3StartTxTransaction();       // Interrupt pipe IN - status
   ep4InitialiseBdtRx();          // Tx pipe OUT
   ep5StartTxTransactionIfIdle(); // Rx pipe IN
#endif

   // Clear odd/even bits & Enable Rx/Tx
   USB0->CTL = USB_CTL_USBENSOFEN_MASK|USB_CTL_ODDRST_MASK;
   USB0->CTL = USB_CTL_USBENSOFEN_MASK;

   // Enable endpoints
   USB0->ENDPOINT[0].ENDPT  = USB_ENDPT_EPRXEN_MASK|USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK; // Rx/Tx/SEUP
   USB0->ENDPOINT[1].ENDPT  = USB_ENDPT_EPRXEN_MASK|                      USB_ENDPT_EPHSHK_MASK; // Rx
   USB0->ENDPOINT[2].ENDPT  =                       USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK; // Tx
#if (HW_CAPABILITY&CAP_CDC)
   USB0->ENDPOINT[3].ENDPT  =                       USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK; // Tx
   USB0->ENDPOINT[4].ENDPT  = USB_ENDPT_EPRXEN_MASK|                      USB_ENDPT_EPHSHK_MASK; // Rx
   USB0->ENDPOINT[5].ENDPT  =                       USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK; // Tx
#else
   USB0->ENDPOINT[3].ENDPT = 0;
   USB0->ENDPOINT[4].ENDPT = 0;
   USB0->ENDPOINT[5].ENDPT = 0;
#endif
}

//! Set BDM busy flag
//!
void setBDMBusy(void) {
//    disableUSBIrq();
    commandBusyFlag = true;
    if (epHardwareState[2].state == EPIdle) {
       ep2StartTxTransaction(sizeof(busyResponse), busyResponse);
    }
//    enableUSBIrq();
}

#if (HW_CAPABILITY&CAP_CDC)
//======================================================================
// Configure EP3 for an IN transaction [Tx, device -> host, DATA0/1]
//
static void ep3StartTxTransaction( void ) {
   const CDCNotification cdcNotification= {CDC_NOTIFICATION, SERIAL_STATE, 0, RT_INTERFACE, CONST_NATIVE_TO_LE16(2)};
   uint8_t status = cdc_getSerialState();

   if ((status & SERIAL_STATE_CHANGE) == 0) {
      epHardwareState[3].state = EPIdle; // Not busy
      return;
   }
   // Copy the Tx data to Tx buffer
   (void)memcpy(ep3DataBuffer, &cdcNotification, sizeof(cdcNotification));
   ep3DataBuffer[sizeof(cdcNotification)+0] = status&~SERIAL_STATE_CHANGE;
   ep3DataBuffer[sizeof(cdcNotification)+1] = 0;

   // Set up to Tx packet
   BdtEntry *bdt = epHardwareState[3].txOdd?&endPointBdts[3].txOdd:&endPointBdts[3].txEven;
   bdt->bc = sizeof(cdcNotification)+2;
   if (epHardwareState[3].data0_1) {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
   epHardwareState[3].state = EPLastIn;    // Sending one and only pkt
}

//======================================================================
// Configure the BDT for EP4 Out [Rx, device <- host, DATA0/1]
// CDC - OUT
static void ep4InitialiseBdtRx( void ) {
   BdtEntry *bdt = epHardwareState[4].rxOdd?&endPointBdts[4].rxOdd:&endPointBdts[4].rxEven;

   // Set up to Rx packet
   bdt->bc = ENDPT4MAXSIZE;
   if (epHardwareState[4].data0_1) {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
}

//=========================================================================
// Save the data from an EP4 OUT pkt
// CDC - OUT
static void ep4SaveRxData( void ) {
   // Get BDT
   BdtEntry *bdt = (!epHardwareState[4].rxOdd)?&endPointBdts[4].rxOdd:&endPointBdts[4].rxEven;
   uint8_t size = bdt->bc;
   (void)cdc_putTxBuffer((char*)ep4DataBuffer, size);

   // Toggle on successful reception
   epHardwareState[4].data0_1 = !epHardwareState[4].data0_1;
}

//======================================================================
// Configure the BDT for EP5 In [Tx, device -> host]
// CDC - IN
static void ep5InitialiseBdtTx(void) {
   // Set up to Tx packet
   if (epHardwareState[5].txOdd) {
      // Set to write to other buffer & get count in current buffer
      endPointBdts[5].txOdd.bc     = cdc_setRxBuffer((char*)ep5DataBuffer0);
//       ep5DataBuffer1[0]       = '|';
      endPointBdts[5].txOdd.u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK;
   }
   else {
      // Set to write to other buffer & get count in current buffer
      endPointBdts[5].txEven.bc    = cdc_setRxBuffer((char*)ep5DataBuffer1);
//       ep5DataBuffer0[0]       = '^';
      endPointBdts[5].txEven.u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK;
   }
//   epHardwareState[5].data0_1 = !epHardwareState[5].data0_1; // Toggle data0/1
   epHardwareState[5].txOdd     = !epHardwareState[5].txOdd;
}

static uint8_t serialDelayCount = 0;

// This value controls how long the serial interface will wait before
// sending a buffered character. (count of SOFs ~ ms)
#define SERIAL_THRESHOLD (0) // ms

//======================================================================
// Configure the BDT for EP5 In [Tx, device -> host]
// CDC - IN
static void ep5StartTxTransactionIfIdle() {
#if 0
   if ((epHardwareState[5].state == EPIdle) && (cdc_rxBufferItemCount()>0)) {
      ep5InitialiseBdtTx();
      epHardwareState[5].state = EPLastIn;
      serialDelayCount = 0;
   }
   else if ((epHardwareState[5].state == EPLastIn) && (cdc_rxBufferItemCount()==16)) {
      ep5InitialiseBdtTx();
      epHardwareState[5].state = EPDataIn;
      serialDelayCount = 0;
   }
#else
   if ((epHardwareState[5].state == EPIdle) && (cdc_rxBufferItemCount()>0)) {
#if (DEBUG&USB_PING_DEBUG)
      DEBUG_PIN_PCOR = DEBUG_PIN_MASK;
#endif
      ep5InitialiseBdtTx();
      epHardwareState[5].state = EPDataIn;
      serialDelayCount = 0;
   }
#endif
}

void checkUsbCdcRxData(void) {
    ep5StartTxTransactionIfIdle();
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
//!     - [0]    = size of command (N)
//!     - [1]    = command
//!     - [2..N] = parameters
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
void receiveUSBCommand(uint8_t maxSize, uint8_t *buffer) {
   uint8_t size;

   // Wait for USB connection
   while(deviceState.state != USBconfigured) {
      __WFI();
   }

   enableUSBIrq();
   // Size of first (command) transaction
   do {
      size = ENDPT1MAXSIZE;
      if (size > maxSize) {
         size = maxSize;
      }
      // Get 1st/only pkt of command
      reInit = false;
      ep1StartRxTransaction( size, buffer );
      while ((epHardwareState[1].state != EPComplete) && !reInit) {
          __WFI();
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
          reInit = true;
          continue;
      }
      // Receive rest of data if present (only possibly 2 transactions total)
      if (size > ep1State.dataCount) {
         // Save last byte of 1st pkt as overwritten by
         // second pkt (to save moving 2nd pkt when size is discarded)
         uint8_t saveByteOffset = ep1State.dataCount-1;
         uint8_t saveByte       = buffer[saveByteOffset];
         ep1StartRxTransaction( size-saveByteOffset, buffer+saveByteOffset );
         while ((epHardwareState[1].state != EPComplete) && !reInit) {
             __WFI();
         }
         // Check if second pkt has correct marker
         if (buffer[saveByteOffset] != 0) {
            // packet corrupt
            reInit = true;
         }
         // Restore saved byte
         buffer[saveByteOffset] = saveByte;
      }
   } while (reInit);
   PUTS(epHardwareState[1].data0_1?"receiveUSBCommand-1\n":"receiveUSBCommand-0\n");
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
//!     - [0]    = response
//!     - [1..N] = parameters
//!
void sendUSBResponse( uint8_t size, const uint8_t *buffer) {
   commandBusyFlag = false;
//   enableUSBIrq();
//   while (epHardwareState[2].state != EPIdle) {
//   }
   PUTS(epHardwareState[2].data0_1?"sendUSBResponse-1\n":"sendUSBResponse-0\n");
   ep2StartTxTransaction(size, buffer);
}

//======================================================================
// Stall control for endpoints
//

//! Stall endpoint
//!
//! @param epNum - endpoint number
//!
static void epStall(uint8_t epNum) {
   if (epNum == 0) {
      // Stall Tx only
      PUTS("epStall.ep0");
      BdtEntry *bdt = epHardwareState[0].txOdd?&endPointBdts[0].txOdd:&endPointBdts[0].txEven;
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_STALL_MASK|BDTEntry_DTS_MASK;
   }
   else {
      if (epNum==1) {
         PUTS(epHardwareState[1].data0_1?"epStall-C-SS-1":"epStall-C-SS-0");
      }
      else if (epNum==2) {
         PUTS(epHardwareState[1].data0_1?"epStall-R-SS-1":"epStall-R-SS-0");
      }
      epHardwareState[epNum].state    = EPStall;
      epHardwareState[epNum].data0_1  = DATA0;
      USB0->ENDPOINT[epNum].ENDPT    |= USB_ENDPT_EPSTALL_MASK;
   }
}

//! Clear Stall on endpoint
//!
//! @param epNum - endpoint number
//!
static void epClearStall(uint8_t epNum) {
   USB0->ENDPOINT[epNum].ENDPT          &= ~USB_ENDPT_EPSTALL_MASK;
   epHardwareState[epNum].state          = EPIdle;
   epHardwareState[epNum].data0_1        = DATA0;
   if (epNum==1) {
      PUTS("epClearStall1-0");
   }
   else if (epNum==2) {
      PUTS("epClearStall2-0");
   }
}

//========================================================================================
//
static void setUSBdefaultState( void ) {
   greenLedOff();
   deviceState.state                = USBdefault;
   USB0->ADDR                       = 0;
   deviceState.configuration        = 0;
   deviceState.interfaceAltSetting  = 0;
}

static void setUSBaddressedState( uint8_t address ) {
   if (address == 0) {
      // Unaddress??
      setUSBdefaultState();
   }
   else {
      greenLedOff();
      deviceState.state                = USBaddressed;
      USB0->ADDR                        = address;
      deviceState.configuration        = 0;
      deviceState.interfaceAltSetting  = 0;
   }
}

static void setUSBconfiguredState( uint8_t config ){
   if (config == 0) {
      // unconfigure
      setUSBaddressedState(USB0->ADDR);
   }
   else {
      greenLedOn();
      deviceState.state                = USBconfigured;
      deviceState.configuration        = config;
      deviceState.interfaceAltSetting  = 0;
   }
}

//==================================================================
// Handler for USB Bus reset
//
static void handleUSBReset(void) {
#ifdef SUSPEND_RESUME_TESTING
   resInit();
   resLow();
   delay();
   resHigh();
   delay();
   delay();
   resLow();
   delay();
   resHigh();
#endif
   PUTS("\nReset");

   // Disable all interrupts
   USB0->INTEN = 0x00;
   USB0->ERREN = 0x00;

   USB0->ERRSTAT = 0xFF;                 // Clear USB error flags

   // Clear all USB interrupt flags
   USB0->ISTAT = 0xFF;

   setUSBdefaultState();

   initialiseEndpoints();

   // Set up to receive 1st SETUP packet
   ep0ConfigureSetupTransaction(); // re-initialise EP0 OUT // v4.7

   // Enable various interrupts
   USB0->INTEN = USB_INTMASKS|USB_INTEN_ERROREN_MASK;
   USB0->ERREN = 0xFF;
}

//======================================================================
//! Initialise the USB interface
//!
//! @note Assumes clock set up for USB operation (48MHz)
//!
void initUSB() {
#ifdef SUSPEND_RESUME_TESTING
   resInit();
   resLow();
   delay();
   resHigh();
   delay();
   resLow();
   delay();
   resHigh();
#endif


   // Make sure no interrupt during setup
   disableUSBIrq();

   // Clear USB RAM (includes BDTs)
   memset((uint8_t*)endPointBdts, 0, sizeof(endPointBdts));

   // Enable clock to USB Module
   SIM->SCGC4 |= SIM_SCGC4_USBOTG_MASK;

   USB0->OTGISTAT = 0;
   USB0->OTGICR   = 0;
   USB0->OTGCTL   = 0;
   USB0->INTEN    = 0;
   USB0->ERRSTAT  = 0;
   USB0->ERREN    = 0;
   USB0->CTL      = 0;
   USB0->ADDR     = 0;
   for (unsigned i=0; i<(sizeof(USB0->ENDPOINT)/sizeof(USB0->ENDPOINT[0])); i++) {
      USB0->ENDPOINT[i].ENDPT = 0;
   }
   USB0->USBCTRL = 0;
   USB0->CONTROL = 0;
   USB0->USBTRC0 = 0;

#ifdef MPU_CESR_VLD_MASK
   // Disable MPU & clear errors
   MPU->CESR = MPU_CESR_SPERR_MASK;
#endif
   // Enable USB regulator
   SIM->SOPT1CFG  = SIM_SOPT1CFG_URWE_MASK;
   SIM->SOPT1    |= SIM_SOPT1_USBREGEN_MASK;
   // Enable in LP modes
   //SIM->SOPT1CFG  = SIM_SOPT1CFG_URWE_MASK;
   //SIM->SOPT1    &= ~(SIM_SOPT1_USBSSTBY_MASK|SIM_SOPT1_USBVSTBY_MASK);

   //   DEBUG_PIN_DDR = 1;


   // Removed due to errata e5928: USBOTG: USBx_USBTRC0[USBRESET] bit does not operate as expected in all cases
   // Reset USB H/W
//   USB0_USBTRC0 = 0x40|USB_USBTRC0_USBRESET_MASK;
//   while ((USB0_USBTRC0&USB_USBTRC0_USBRESET_MASK) != 0) {
//   }

   // This bit is undocumented but seems to be necessary
   USB0->USBTRC0 = 0x40;

   // Set initial USB state
   setUSBdefaultState();
//   ep0State    = initialEPState;

   // Point USB at BDT array
   USB0->BDTPAGE3 = (uint8_t) (((unsigned)endPointBdts)>>24);
   USB0->BDTPAGE2 = (uint8_t) (((unsigned)endPointBdts)>>16);
   USB0->BDTPAGE1 = (uint8_t) (((unsigned)endPointBdts)>>8);

//   for (int i=0; i<100000; i++) {
//      __asm__("nop");
//   }
   // Clear all pending interrupts
   USB0->ISTAT = 0xFF;

   // Enable usb reset interrupt
   USB0->INTEN = USB_INTEN_USBRSTEN_MASK|USB_INTEN_SLEEPEN_MASK;

   // Weak pull downs
   USB0->USBCTRL = USB_USBCTRL_PDE_MASK;

   // Enable Pull-up
   USB0->CONTROL = USB_CONTROL_DPPULLUPNONOTG_MASK;

   // Enable interface
   USB0->CTL = USB_CTL_USBENSOFEN_MASK;

   // Enable USB interrupts
   enableUSBIrq();
}

//===============================================================================
// Get Status - Device Req 0x00
//
static void handleGetStatus( void ) {
   static const uint8_t zeroReturn[] = {0,0};
   static const EPStatus epStatusStalled = {1,0,0};
   static const EPStatus epStatusOK      = {0,0,0};

   const uint8_t *dataPtr = NULL;
   uint8_t size;
   uint8_t epNum;

   PUTS("getStatus");

   //dprint("hGS()");
   switch(ep0SetupBuffer.bmRequestType) {
      case (EP_IN|RT_DEVICE) : // Device Status
         dataPtr = (uint8_t *) &deviceState.status;
         size    = sizeof(deviceState.status);
         break;
      case (EP_IN|RT_INTERFACE) : // Interface Status - reserved
         dataPtr = zeroReturn;
         size = sizeof(zeroReturn);
         break;
      case (EP_IN|RT_ENDPOINT) : // Endpoint Status
         epNum = ep0SetupBuffer.wIndex.word&0x07;
         if (epNum <= NUMBER_OF_EPS) {
            if (USB0->ENDPOINT[epNum].ENDPT&USB_ENDPT_EPSTALL_MASK) {
               dataPtr = (uint8_t*)&epStatusStalled;
            }
            else {
               dataPtr = (uint8_t*)&epStatusOK;
            }
//            dataPtr = (uint8_t *) &epHardwareState[epNum].status;
            size = sizeof(EPStatus);
         }
         break;
   }
   if (dataPtr != NULL)
      ep0StartTxTransaction( size, dataPtr );
   else
      epStall(0);
}

//===============================================================================
// Clear Feature - Device Req 0x01
//
static void handleClearFeature( void ) {
   int okStatus = 0;

   PUTS("clearFeature");

   switch(ep0SetupBuffer.bmRequestType) {
      case (RT_DEVICE) : // Device Feature
         if ((ep0SetupBuffer.wValue.word != DEVICE_REMOTE_WAKEUP) || // Device remote wake up
             (ep0SetupBuffer.wIndex.word != 0))   {                  // Device index must be 0
            break;
            }
         deviceState.status.remoteWakeup = 0;
         okStatus                        = 1;
         break;

      case (RT_INTERFACE) : // Interface Feature
         break;

      case (RT_ENDPOINT) : { // Endpoint Feature ( Out,Standard,Endpoint )
         uint8_t epNum = ep0SetupBuffer.wIndex.word&0x07;
         if ((ep0SetupBuffer.wValue.word != ENDPOINT_HALT) || // Not Endpoint Stall ?
             (epNum >= NUMBER_OF_EPS))                        // or illegal EP# (ignores direction)
            break;
         epClearStall(epNum);
         okStatus = 1;
         }
         break;

      default : // Illegal
         break;
      }

   if (okStatus)
      ep0StartTxTransaction( 0, NULL ); // Tx empty Status packet
   else
      epStall(0);
}

//===============================================================================
// Set Feature - Device Req 0x03
//
static void handleSetFeature( void ) {
   int okStatus = 0;

   PUTS("setFeature");

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
         uint8_t epNum = ep0SetupBuffer.wIndex.word&0x07;
         if ((ep0SetupBuffer.wValue.word != ENDPOINT_HALT) || // Not Endpoint Stall ?
             (epNum >= NUMBER_OF_EPS))  {                     // or illegal EP# (ignores direction)
            break;
         }
         epStall(epNum);
         okStatus = 1;
         }
         break;

      default : // Illegal
         break;
      }

   if (okStatus) {
      ep0StartTxTransaction( 0, NULL ); // Tx empty Status packet
   }
   else {
      epStall(0);
   }
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
static void utf8ToStringDescriptor(const uint8_t *source, uint8_t *dest) {
uint8_t *size = dest; // 1st byte is where to place descriptor size

    *dest++ = 2;         // 1st byte = descriptor size (2 bytes so far)
    *dest++ = DT_STRING; // 2nd byte = DT_STRING;

    while (*source != '\0') {
      uint16_t utf16Char;
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
   unsigned        descriptorIndex = (unsigned)ep0SetupBuffer.wValue.le.lo;
   int             dataSize = 0;
   const uint8_t  *dataPtr = NULL;

   if (ep0SetupBuffer.bmRequestType != (EP_IN|RT_DEVICE)) {// In,Standard,Device
      epStall(0);
      return;
   }
   switch (ep0SetupBuffer.wValue.le.hi) {

      case DT_DEVICE: // Get Device Descriptor - 1
         PUTS("getDescriptor-device - ");
         dataPtr  = (uint8_t *) &deviceDescriptor;
         dataSize = sizeof(DeviceDescriptor);
         break;
      case DT_CONFIGURATION: // Get Configuration Descriptor - 2
         //dprint("hGDconf()\r\n");
         PUTS("getDescriptor-config - ");
         if (ep0SetupBuffer.wValue.le.lo != 0) {
            epStall(0);
            return;
         }
         dataPtr  = (uint8_t *) &otherDescriptors;
         dataSize = sizeof(otherDescriptors);
         break;
      case DT_DEVICEQUALIFIER: // Get Device Qualifier Descriptor
         PUTS("getDescriptor-deviceQ - ");
         epStall(0);
         return;
      case DT_STRING: // Get String Desc.- 3
         PUTS("getDescriptor-string - ");
#ifdef MS_COMPATIBLE_ID_FEATURE
         if (descriptorIndex == 0xEE) {
            dataPtr  = OS_StringDescriptor;
            dataSize = *dataPtr;
            break;
         }
#endif
         if (descriptorIndex >= sizeof(stringDescriptors)/sizeof(stringDescriptors[0])) {
            // Illegal string index - stall
            epStall(0);
            return;
         }
         if (descriptorIndex == 0) {
            // Language bytes
            dataPtr  = stringDescriptors[0];
         }
#if defined(UNIQUE_ID)
         else if (descriptorIndex == 3) {
            // Generate Semi-unique Serial number
            uint32_t uid = SIM->UIDH^SIM->UIDMH^SIM->UIDML^SIM->UIDL;
            // Use part of commandBuffer as temporary (leave 60 free at start)
            snprintf((char*)commandBuffer+2*22+10, 22, SERIAL_NO, uid);
            dataPtr = commandBuffer;
            utf8ToStringDescriptor(commandBuffer+2*22+10, commandBuffer);
         }
#endif
         else {
            // Strings are stored in limited UTF-8 and need conversion
            dataPtr = commandBuffer;
            utf8ToStringDescriptor(stringDescriptors[descriptorIndex], commandBuffer);
         }
         dataSize = *dataPtr;
         break;
      default:
         PUTS("getDescriptor-default - ");
         // shouldn't happen
         epStall(0);
         return;
   } // switch
   ep0StartTxTransaction( (uint8_t)dataSize, dataPtr ); // Set up Tx
}

//===============================================================================
// resetDevice Callback to execute after status transaction
//
static void resetDeviceCallback( void ) {
//   forceICPReset();
}

//===============================================================================
// SetDeviceAddress Callback - executed after status transaction
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
   PUTS("setAddress - ");

   if (ep0SetupBuffer.bmRequestType != (EP_OUT|RT_DEVICE)) {// Out,Standard,Device
      //dprint("hSA():inv. bmR");
      epStall(0); // Illegal format - stall ep0
      return;
   }
   // Save address for change after status transaction
   deviceState.newUSBAddress  = ep0SetupBuffer.wValue.le.lo;
   ep0State.callback          = setAddressCallback;

   ep0StartTxTransaction( 0, NULL ); // Tx empty Status packet
}

//===============================================================================
// Get Configuration - Device Req 0x08
//
static void handleGetConfiguration( void ) {
   PUTS("getConfiguration");

   ep0StartTxTransaction( 1, (uint8_t *) &deviceState.configuration );
}

//===============================================================================
// Set Configuration - Device Req 0x09
// Treated as soft reset
//
static void handleSetConfiguration( void ) {
   PUTS("setConfiguration");

   reInit = true;  // tell command handler to re-init

   if ((ep0SetupBuffer.bmRequestType != (EP_OUT|RT_DEVICE)) || // Out,Standard,Device
       ((ep0SetupBuffer.wValue.le.lo != 0) &&       // Only supports 0=> un-configure, 1=> only valid configuration
        (ep0SetupBuffer.wValue.le.lo != otherDescriptors.configDescriptor.bConfigurationValue))) {
      epStall(0);
      return;
   }
   setUSBconfiguredState(ep0SetupBuffer.wValue.le.lo);

   initialiseEndpoints();

   ep1InitialiseBdtRx();

   ep0StartTxTransaction( 0, NULL ); // Tx empty Status packet
}

//===============================================================================
// Set interface - Device Req 0x0B
// Not required to be implemented
static void handleSetInterface( void ) {
   PUTS("setInterface");

   if ((ep0SetupBuffer.bmRequestType != (EP_OUT|RT_INTERFACE)) || // NOT In,Standard,Interface
       (ep0SetupBuffer.wLength.word != 0) ||                      // NOT correct length
       (deviceState.state != USBconfigured)) {                    // NOT in addressed state
      epStall(0); // Error
      return;
   }
   // Only support one Alternate Setting == 0
   if (ep0SetupBuffer.wValue.word != 0) {
      epStall(0); // Error
      return;
   }
   ep0StartTxTransaction( 0, NULL ); // Tx empty Status packet
}

//===============================================================================
// Get interface - Device Req 0x0A
//
static void handleGetInterface( void ) {
   static const uint8_t interfaceAltSetting = 0;

   PUTS("getInterface");

   if ((ep0SetupBuffer.bmRequestType != (EP_IN|RT_INTERFACE)) || // NOT In,Standard,Interface
       (ep0SetupBuffer.wLength.word != 1)) {                     // NOT correct length
      epStall(0); // Error
      return;
   }
   ep0StartTxTransaction( sizeof(interfaceAltSetting), &interfaceAltSetting ); // Send packet
}

#if (HW_CAPABILITY&CAP_CDC)
//static void handleGetEncapsulatedCommand() {
//   static const char dummy[] = "Hello there";
//
//   ep0StartInTransaction( sizeof(dummy), dummy ); // Send packet
//}
//
//static void handleSendEncapsulatedCommand() {
//
//   ep0StartInTransaction( 0, NULL ); // Tx empty Status packet
//}

static void handleGetLineCoding() {
   PUTS("getLineCoding");

   ep0StartTxTransaction( sizeof(LineCodingStructure), (const uint8_t*)cdc_getLineCoding() ); // Send packet
}

static void setLineCodingCallback( void ) {
   cdc_setLineCoding((LineCodingStructure * const)&ep0OutDataBuffer);
}

static void handleSetLineCoding() {
   PUTS("setLineCoding");

   ep0State.callback          = setLineCodingCallback;
   // Don't use buffer - this requires sizeof(LineCodingStructure) < ENDPT0MAXSIZE
   ep0StartRxTransaction(sizeof(LineCodingStructure), NULL);
}

static void handleSetControlLineState() {
   PUTS("setControlLineState");

   cdc_setControlLineState(ep0SetupBuffer.wValue.le.lo);
   ep0StartTxTransaction( 0, NULL ); // Tx empty Status packet
}

static void handleSendBreak() {
   PUTS("sendBreak");

   cdc_sendBreak(ep0SetupBuffer.wValue.word);  // time in milliseconds, 0xFFFF => continuous
   ep0StartTxTransaction( 0, NULL );   // Tx empty Status packet
}
#endif

//=================================================
// Illegal request in SETUP pkt
//
static void handleUnexpected( void ) {
   PUTS("unexpected");
   epStall(0);
}

//===============================================================================
// Handles SETUP Packet
//
static void handleSetupToken( void ) {
   PUTS("\nep0SetupTok");

   // Save data from SETUP pkt
   ep0SetupBuffer = *(SetupPacket *)ep0OutDataBuffer;

   epHardwareState[0].data0_1  = DATA1;
   epHardwareState[0].state    = EPIdle;
   ep0State.callback           = NULL;

   // Convert SETUP values to big-endian
   ep0SetupBuffer.wLength.word = leToNative16(ep0SetupBuffer.wLength.word);
   ep0SetupBuffer.wValue.word  = leToNative16(ep0SetupBuffer.wValue.word);
   ep0SetupBuffer.wIndex.word  = leToNative16(ep0SetupBuffer.wIndex.word);

   switch(REQ_TYPE(ep0SetupBuffer.bmRequestType)) {
   case REQ_TYPE_STANDARD :
      // Standard device requests
      switch (ep0SetupBuffer.bRequest) {
      case GET_STATUS :          handleGetStatus();            break;
      case CLEAR_FEATURE :       handleClearFeature();         break;
      case SET_FEATURE :         handleSetFeature();           break;
      case SET_ADDRESS :         handleSetAddress();           break;
      case GET_DESCRIPTOR :      handleGetDescriptor();        break;
      case GET_CONFIGURATION :   handleGetConfiguration();     break;
      case SET_CONFIGURATION :   handleSetConfiguration();     break;
      case SET_INTERFACE :       handleSetInterface();         break;
      case GET_INTERFACE :       handleGetInterface();         break;
      case SET_DESCRIPTOR :
      case SYNCH_FRAME :
      default :                  handleUnexpected();           break;
      }
      break;

   case REQ_TYPE_CLASS :
      // Class requests
      switch (ep0SetupBuffer.bRequest) {
#if (HW_CAPABILITY&CAP_CDC)
      case SET_LINE_CODING :           handleSetLineCoding();              break;
      case GET_LINE_CODING :           handleGetLineCoding();              break;
      case SET_CONTROL_LINE_STATE:     handleSetControlLineState();        break;
      case SEND_BREAK:                 handleSendBreak();                  break;
#endif
      default :                        handleUnexpected();                 break;
      }
      break;

   case REQ_TYPE_VENDOR :
      PUTS("REQ_TYPE_VENDOR");
      // Handle special commands here
      switch (ep0SetupBuffer.bRequest) {
      case ICP_GET_VER : {
         uint8_t versionResponse[5];
         reInit = true;  // tell command handler to re-init
         versionResponse[0] = BDM_RC_OK;
         versionResponse[1] = VERSION_SW;      // BDM SW version
         versionResponse[2] = VERSION_HW;      // BDM HW version
         versionResponse[3] = 0;               // ICP_Version_SW;
         versionResponse[4] = VERSION_HW;      // ICP_Version_HW;
         ep0StartTxTransaction( sizeof(versionResponse),  versionResponse );
      }
      break;

#ifdef MS_COMPATIBLE_ID_FEATURE
      case VENDOR_CODE :
         if (ep0SetupBuffer.wIndex.word == 0x0004) {
            ep0StartTxTransaction(sizeof(msCompatibleIdFeatureDescriptor), (const uint8_t *)&msCompatibleIdFeatureDescriptor, DATA1);
         }
         else if (ep0SetupBuffer.wIndex.word == 0x0005) {
            ep0StartTxTransaction(sizeof(msPropertiesFeatureDescriptor), (const uint8_t *)&msPropertiesFeatureDescriptor, DATA1);
         }
         else {
            handleUnexpected();
         }
         break;
#endif

      case CMD_USBDM_ICP_BOOT :
         // Reboots to ICP mode
         ep0State.callback = resetDeviceCallback;
         ep0StartTxTransaction( 0, NULL ); // Tx empty Status packet
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

   ep0EnsureReadyForSetupTransaction();   // In case another SETUP pkt

   // Allow transactions post SETUP pkt (clear TXSUSPENDTOKENBUSY)
   USB0->CTL = USB_CTL_USBENSOFEN_MASK;
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
         PUTS("ep0InTok-EPDataIn - ");
         if ((ep0State.dataRemaining < ENDPT0MAXSIZE) ||   // Undersize pkt OR
             ((ep0State.dataRemaining == ENDPT0MAXSIZE) && // Full size AND
               !ep0State.shortInTransaction)) {
            // Don't need to flag undersize transaction
            epHardwareState[0].state = EPLastIn;    // Sending last pkt
         }
         else {
            epHardwareState[0].state = EPDataIn;    // Sending full pkt
         }
         ep0InitialiseBdtTx(); // Set up next IN pkt
         break;

      case EPLastIn:
         PUTS("ep0InTok-EPLastIn");
          // Just done the last IN packet
         epHardwareState[0].state = EPStatusOut;   // Receiving an OUT status pkt
         break;

      case EPStatusIn:
         // Just done an IN packet as a status handshake for an OUT Data transfer
         PUTS("ep0InTok-EPStatusIn");
         epHardwareState[0].state = EPIdle;           // Now Idle
         if (ep0State.callback != NULL) {
            // Execute callback function to process OUT data
            PUTS("callback");
            ep0State.callback();
         }
         ep0State.callback = NULL;
         break;

      // We don't expect an IN token while in the following states
      case EPIdle:           // Idle (Tx complete)
      case EPDataOut:        // Doing a sequence of OUT packets (until data count <= EPSIZE)
      case EPStatusOut:      // Doing an OUT packet as a status handshake
      default:
         PUTS("ep0InTok-default");
         break;
   }
}

//================
// ep0 - OUT
//
static void ep0HandleOutToken( void ) {
uint8_t transferSize;

   epHardwareState[0].data0_1 = !epHardwareState[0].data0_1; // Toggle DATA0/1

   switch (epHardwareState[0].state) {
      case EPDataOut:        // Receiving a sequence of OUT packets
         PUTS("ep0OutTok - EPDataOut");
         transferSize = ep0SaveRxData();          // Save the data from the Rx buffer
         // Check if completed an under-size pkt or expected number of bytes
         if ((transferSize < ENDPT0MAXSIZE) || (ep0State.dataRemaining == 0)) { // Last pkt?
            epHardwareState[0].state = EPIdle;
            ep0StartTxTransaction(0, NULL); // Do status Pkt transmission
            }
         else {
            ep0InitialiseBdtRx(epHardwareState[0].data0_1); // Set up next OUT pkt
            }
         break;

      case EPStatusOut:       // Done an OUT packet as a status handshake
         PUTS("ep0OutTok - EPStatusOut");
         epHardwareState[0].state = EPIdle;
         break;

      // We don't expect an OUT token while in the following states
      default:
      case EPLastIn:          // Just done the last IN packet
      case EPDataIn:          // Doing a sequence of IN packets (until data count <= EPSIZE)
      case EPStatusIn:        // Just done an IN packet as a status handshake
      case EPIdle:            // Idle (Tx complete)
         PUTS("ep0OutTok - default");
         break;
   }
   ep0EnsureReadyForSetupTransaction();  // Make ready for a SETUP pkt
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
uint8_t transferSize;

   epHardwareState[1].data0_1 = !epHardwareState[1].data0_1;   // Toggle DATA0/1
   PUTS(epHardwareState[1].data0_1?"ep1HandleOutToken-T-1\n":"ep1HandleOutToken-T-0\n");

   switch (epHardwareState[1].state) {
      case EPDataOut:        // Doing a sequence of OUT packets making up a command
         transferSize = ep1SaveRxData();          // Save the data from the Rx buffer
         // Completed transfer on undersize pkt or received expected number of bytes
         if ((transferSize < ENDPT1MAXSIZE) || (ep1State.dataRemaining == 0)) { // Last pkt?
            epHardwareState[1].state = EPComplete;
         }
         else {
            ep1InitialiseBdtRx(); // Set up next OUT pkt
         }
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
   PUTS(epHardwareState[1].data0_1?"ep2HandleInToken-T-1\n":"ep2HandleInToken-T-0\n");

   switch (epHardwareState[2].state) {
      case EPDataIn:    // Doing a sequence of IN packets (until data count <= EPSIZE)
         if (ep2State.dataRemaining < ENDPT2MAXSIZE) {
            epHardwareState[2].state = EPLastIn;    // Sending last pkt (may be empty)
         }
         else {
            epHardwareState[2].state = EPDataIn;    // Sending full pkt
         }
         ep2InitialiseBdtTx(); // Set up next IN pkt
         break;

      case EPLastIn:    // Just done the last IN packet
         if (commandBusyFlag) {
            ep2StartTxTransaction(sizeof(busyResponse), busyResponse);
         }
         else {
            epHardwareState[2].state = EPIdle;  // No more transactions expected
         }
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
static void handleTokenComplete(void) {
   uint8_t   usbStat  = USB0->STAT;
   uint8_t   endPoint = ((uint8_t)usbStat)>>4;        // Endpoint number
   uint8_t   isTx     = usbStat&USB_STAT_TX_MASK;     // Direction of T/F 0=>OUT, (!=0)=>IN
   uint8_t   isOdd    = usbStat&USB_STAT_ODD_MASK;    // Odd/even buffer
   BdtEntry *bdt      = &BDTS(usbStat>>2);
   if (isTx) {
      epHardwareState[endPoint].txOdd = !isOdd; // Buffer to use next
   }
   else {
      epHardwareState[endPoint].rxOdd = !isOdd; // Buffer to use next
   }
   switch (endPoint) {
       case 0: // Control - Accept IN, OUT or SETUP token
          if (isTx) { // IN Transaction complete
             ep0HandleInToken();
          }
          else if (bdt->u.result.tok_pid == SETUPToken) { // SETUP transaction complete
             handleSetupToken();
          }
          else { // OUT Transaction
             ep0HandleOutToken();
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
          ep3StartTxTransaction();
          return;
       case 4: // USBDM CDC Data - Accept OUT token
//          usbActivityFlag.flags.serialOutActive = 1;
          if (cdc_txBufferIsFree()) {
             ep4SaveRxData();
             ep4InitialiseBdtRx();
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
//#if (DEBUG&USB_PING_DEBUG)
//          DEBUG_PIN_PSOR = DEBUG_PIN_MASK;
//#endif
          epHardwareState[5].state = EPIdle;
//          RTCMOD = 0;
//          rtcCount = 0;
#endif
//          ep5StartInTransactionIfIdle();
          return;
#endif
   }
}

//#pragma MESSAGE DISABLE C4003
//==================================================================
// Handler for Start of Frame Token interrupt (~1ms interval)
//
static void handleSOFToken( void ) {
   // Activity LED
   // Off                     - no USB activity, not connected
   // On                      - no USB activity, connected
   // Off, flash briefly on   - USB activity, not connected
   // On,  flash briefly off  - USB activity, connected
   if (USB0->FRMNUML==0) { // Every ~256 ms
      switch (USB0->FRMNUMH&0x03) {
         case 0:
            if (deviceState.state == USBconfigured) {
                // Green LED on when USB connection established
                greenLedOn();
            }
            else {
                // Green LED off when no USB connection
                greenLedOff();
            }
            break;
         case 1:
         case 2:
            break;
         case 3:
         default :
            if (usbActivityFlag.flags.bdmActive) {
               // Green LED flashes on USB activity
               greenLedToggle();
               usbActivityFlag.byte = 0;
            }
            break;
         }
   }
#if (HW_CAPABILITY&CAP_CDC)
   // Check if need to restart EP5 (CDC IN)
   ep5StartTxTransactionIfIdle();
#endif
}
//#pragma MESSAGE DEFAULT C4003
#if (HW_CAPABILITY&CAP_CDC)
void checkUsbCdcTxData(void) {
   // Check if we need to unThrottle EP4
   if ((epHardwareState[4].state == EPThrottle) && cdc_txBufferIsFree()) {
      ep4SaveRxData();        // Save data from last transfer
      ep4InitialiseBdtRx();   // Set up next transfer
      epHardwareState[4].state = EPDataOut;
   }
}
#endif

/*==========================================================================
 * Handler for USB Suspend
 *   - Enables the USB module to wake-up the CPU
 *   - Stops the CPU
 * On wake-up
 *   - Re-checks the USB after a small delay to avoid wake-ups by noise
 *   - The RESUME interrupt is left pending so the resume handler can execute
 */
static void handleUSBSuspend( void ) {
#ifdef SUSPEND_RESUME_TESTING
   resHigh();
   delay();
   resLow();
   delay();
   delay();
   resHigh();
   delay();
   delay();
   delay();
   resLow();
   delay();
   resHigh();
#endif

   if (deviceState.state != USBconfigured) {
      // Ignore if not configured
      return;
   }
   // Need to disable BDM interface & everything else to reduce power
   VDD_OFF();

   // Clear the sleep and resume interrupt flags
   USB0->ISTAT    = USB_ISTAT_SLEEP_MASK;

   // Asynchronous Resume Interrupt Enable (USB->CPU)
   // Only enable is transceiver is disabled
//   USB0->USBTRC0  |= USB_USBTRC0_USBRESMEN_MASK;

   // Enable resume detection or reset interrupts from the USB
   USB0->INTEN   |= (USB_ISTAT_RESUME_MASK|USB_ISTAT_USBRST_MASK);
   deviceState.state = USBsuspended;
   greenLedOff();

#if 0
   // Wait for resume or reset
#ifdef SUSPEND_RESUME_TESTING
   resLow();
#endif
   do {
      __WFI();  // Processor stop for low power
   } while ((USB0->ISTAT&(USB_ISTAT_RESUME_MASK|USB_INTMASKS)) == 0);
#ifdef SUSPEND_RESUME_TESTING
   resHigh();
#endif

#else
   // A re-check loop is used here to ensure USB bus noise doesn't wake-up the CPU
   do {
      USB0->ISTAT = USB_ISTAT_RESUME_MASK;       // Clear resume interrupt flag

      __WFI();  // Processor stop for low power

      // The CPU has woken up!

      if ((USB0->ISTAT&(USB_ISTAT_RESUME_MASK|USB_ISTAT_USBRST_MASK)) == 0) {
         // Ignore if not resume or reset from USB module
         continue;
      }
      USB0->ISTAT = USB_ISTAT_RESUME_MASK;  // Clear resume interrupt flag

      // Wait for a second resume (or existing reset) ~5 ms
      for(int delay=0; delay<24000; delay++) {
         // We should have another resume interrupt by end of loop
         if ((USB0->ISTAT&(USB_ISTAT_RESUME_MASK|USB_ISTAT_USBRST_MASK)) != 0) {
            break;
         }
      }
   } while ((USB0->ISTAT&(USB_ISTAT_RESUME_MASK|USB_ISTAT_USBRST_MASK)) == 0);
#endif
   return;
}

//==================================================================
// Handler for USB Resume
//
// Disables further USB module wakeups
static void handleUSBResume( void ) {
#ifdef SUSPEND_RESUME_TESTING
   resHigh();
   delay();
   resLow();
   delay();
   delay();
   delay();
   delay();
   resHigh();
   delay();
   resLow();
   delay();
   resHigh();
#endif
   PUTS("Resume");
   // Mask further resume interrupts
   USB0->INTEN   &= ~USB_INTEN_RESUMEEN_MASK;

   if (deviceState.state != USBsuspended) {
      // Ignore if not suspended
      return;
   }
//   USB0->USBTRC0 &= ~USB_USBTRC0_USBRESMEN_MASK;

   // Clear the sleep and resume interrupt flags
   USB0->ISTAT = USB_ISTAT_SLEEP_MASK|USB_ISTAT_RESUME_MASK;

   deviceState.state = USBconfigured;

   // Set up to receive setup packet
   ep0ConfigureSetupTransaction();        // re-initialise EP0 OUT // v4.7
   USB0->CTL = USB_CTL_USBENSOFEN_MASK;   // Enable the transmit or receive of packets
   // power up BDM interface?
}

//==================================================================
// Handler for USB interrupt
//
// Determines source and dispatches to appropriate routine.
//
//! Handler for USB interrupts
void USB0_IRQHandler( void ) {

   // Get active interrupt flags
   uint8_t activeInterruptFlags = USB0->ISTAT;

   // Get active and enabled interrupt flags
   uint8_t interruptFlags = activeInterruptFlags & USB0->INTEN;

   if ((activeInterruptFlags&USB_ISTAT_TOKDNE_MASK) != 0) {
      // Token complete interrupt?
      handleTokenComplete();
      USB0->ISTAT = USB_ISTAT_TOKDNE_MASK; // Clear source
   }
   else if ((activeInterruptFlags&USB_ISTAT_RESUME_MASK) != 0) {
      // Resume signaled on Bus?
      handleUSBResume();
      USB0->ISTAT = USB_ISTAT_RESUME_MASK; // Clear source
   }
   else if ((activeInterruptFlags&USB_ISTAT_USBRST_MASK) != 0) {
      // Reset signaled on Bus
      handleUSBReset();
      USB0->ISTAT = USB_ISTAT_USBRST_MASK; // Clear source
   }
   else if ((activeInterruptFlags&USB_ISTAT_STALL_MASK) != 0) {
      // Stall sent?
      PUTS("StallCom");
      ep0HandleStallComplete();
      USB0->ISTAT = USB_ISTAT_STALL_MASK; // Clear source
   }
   else if ((activeInterruptFlags&USB_ISTAT_SOFTOK_MASK) != 0) {
      // SOF Token?
      handleSOFToken();
      USB0->ISTAT = USB_ISTAT_SOFTOK_MASK; // Clear source
   }
   else if ((activeInterruptFlags&USB_ISTAT_SLEEP_MASK) != 0) {
      // Bus Idle 3ms? => sleep
      PUTS("Suspend");
      handleUSBSuspend();
      USB0->ISTAT = USB_ISTAT_SLEEP_MASK; // Clear source
   }
   else if ((activeInterruptFlags&USB_ISTAT_ERROR_MASK) != 0) {
      // Any Error
      PRINTF("Error s=0x%2X\n", USB0->ERRSTAT);
      USB0->ISTAT = USB_ISTAT_ERROR_MASK; // Clear source
   }
   else  { // unexpected int
      PUTS("Unexpected");
      USB0->ISTAT = interruptFlags; // Clear & ignore
   }
}
