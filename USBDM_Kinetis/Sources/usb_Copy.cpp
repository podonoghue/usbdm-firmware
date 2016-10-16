/*! \file
    \brief Simple USB Stack for Kinetis

   \verbatim
    Kinetis USB Code

    Copyright (C) 2008-16  Peter O'Donoghue

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
#include "derivative.h"
#include "common.h"
#include "configure.h"
#include "commands.h"
#include "usb_defs.h"
#include "usb.h"
#include "cdc.h"
#include "utilities.h"

//#define MS_COMPATIBLE_ID_FEATURE

namespace USBDM {

/*
 * Forward declarations
 */
static void epClearStall(uint8_t epNum);

#if (HW_CAPABILITY&CAP_CDC)
static void ep3StartTxTransaction();
static void ep4InitialiseBdtRx();
static void ep5StartTxTransactionIfIdle();
#endif

uint8_t commandBuffer[300];

//======================================================================
// Maximum packet sizes for each endpoint
//
#if (HW_CAPABILITY&CAP_CDC)
constexpr uint  CONTROL_EP_MAXSIZE      = 16; //!< USBDM - Control in/out   	         16
constexpr uint  BDM_OUT_EP_MAXSIZE      = 64; //!< USBDM - BDM out                       64
constexpr uint  BDM_IN_EP_MAXSIZE       = 64; //!< USBDM - BDM in                        64
constexpr uint  CDC_CONTROL_EP_MAXSIZE  = 16; //!< USBDM - CDC control                   16
constexpr uint  CDC_DATA_OUT_EP_MAXSIZE = 16; //!< USBDM - CDC data out                  16
constexpr uint  CDC_DATA_IN_EP_MAXSIZE  = 16; //!< USBDM - CDC data in              x2 = 32
//                                                                    -------
//                                                                 Each is rounded up to 16 bytes
#else
constexpr uint  CONTROL_EP_MAXSIZE      = 32; //!< USBDM - Control in/out
constexpr uint  BDM_OUT_EP_MAXSIZE      = 64; //!< USBDM - BDM out
constexpr uint  BDM_IN_EP_MAXSIZE       = 64; //!< USBDM - BDM in
constexpr uint  CDC_CONTROL_EP_MAXSIZE  = 0;  //!< USBDM - CDC control (not used)
constexpr uint  CDC_DATA_OUT_EP_MAXSIZE = 0;  //!< USBDM - CDC data out (not used)
constexpr uint  CDC_DATA_IN_EP_MAXSIZE  = 0;  //!< USBDM - CDC data in (not used)
#endif

//======================================================================
// Data packet odd/even indicator
enum {
   DATA0=0, //!< DATA0 indicator
   DATA1=1  //!< DATA1 indicator
};

/**
 * Structure representing a BDT entry in USB controller
 */
#pragma pack(1)
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
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
   volatile uint32_t addr;        // Buffer address
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

//! Mask for all USB interrupts
constexpr uint8_t USB_INTMASKS =
      USB_INTEN_STALLEN_MASK|USB_INTEN_TOKDNEEN_MASK|
      USB_INTEN_SOFTOKEN_MASK|USB_INTEN_USBRSTEN_MASK|
      USB_INTEN_SLEEPEN_MASK;

typedef struct {
   BdtEntry rxEven;
   BdtEntry rxOdd;
   BdtEntry txEven;
   BdtEntry txOdd;
} EndpointBdtEntry;

// Bit masks for fields in BdtEntry
constexpr uint8_t BDTEntry_OWN_MASK   = (1<<7);   //!< Mask for OWN bit in BDT
constexpr uint8_t BDTEntry_DATA0_MASK = (0<<6);   //!< Mask for DATA1 bit in BDT (dummy)
constexpr uint8_t BDTEntry_DATA1_MASK = (1<<6);   //!< Mask for DATA0 bit in BDT
constexpr uint8_t BDTEntry_KEEP_MASK  = (1<<5);   //!< KEEP
constexpr uint8_t BDTEntry_NINC_MASK  = (1<<4);   //!< NINC
constexpr uint8_t BDTEntry_DTS_MASK   = (1<<3);   //!< Mask for DTS bit in BDT
constexpr uint8_t BDTEntry_STALL_MASK = (1<<2);   //!< Stall endpoint

//======================================================================
// Buffers for endpoint data packets (in USB RAM)
//
uint8_t controlDataBuffer[CONTROL_EP_MAXSIZE];

uint8_t bdmOutDataBuffer[BDM_OUT_EP_MAXSIZE];          // #01,OUT,Bulk   - BDM Command & data out
uint8_t bdmInDataBuffer[BDM_IN_EP_MAXSIZE];            // #82,IN, BULK   - BDM data in

#if (HW_CAPABILITY&CAP_CDC)
uint8_t cdcControlDataBuffer[CDC_CONTROL_EP_MAXSIZE];  // #83,IN,interrupt CDC status
uint8_t cdcOutDataBuffer[CDC_DATA_OUT_EP_MAXSIZE];     // #04,OUT,bulk     CDC Tx
uint8_t cdcInDataBuffer0[CDC_DATA_IN_EP_MAXSIZE];      // #85,IN,bulk      CDC Rx ping-pong
uint8_t cdcInDataBuffer1[CDC_DATA_IN_EP_MAXSIZE];
#endif

/** BDTs organised by endpoint, odd/even, tx/rx */
static EndpointBdtEntry endPointBdts[16] __attribute__ ((aligned (512)));

/** BDTs as simple array */
constexpr BdtEntry * bdts = (BdtEntry *)endPointBdts;

/*
 * String descriptors
 */
static const uint8_t s_language[]        = {4, DT_STRING, 0x09, 0x0C};   // Language IDs
static const uint8_t s_manufacturer[]    = "pgo";                        // Manufacturer
static const uint8_t s_product[]         = ProductDescription;           // Product Description
static const uint8_t s_serial[]          = SERIAL_NO;                    // Serial Number
static const uint8_t s_interface_assoc[] = "USBDM BDM Interface";        // Interface Association #1
static const uint8_t s_bdm_interface[]   = "Interface 0 - USBDM";        // Interface #0
static const uint8_t s_cdc_interface[]   = "USBDM CDC Interface";        // Interface Association #2
static const uint8_t s_cdc_control[]     = "CDC Control Interface";      // Interface
static const uint8_t s_cdc_data[]        = "CDC Data Interface";         // Interface

/**
 * String descriptor table
 */
static const uint8_t *const stringDescriptors[] = {
      s_language, s_manufacturer, s_product, s_serial, s_interface_assoc, s_bdm_interface, s_cdc_interface, s_cdc_control, s_cdc_data
};

/**
 * String indexes
 *
 * Must agree with stringDescriptors[] order
 */
enum {
   s_language_index=0,    // Must be zero
   s_manufacturer_index,
   s_product_index,
   s_serial_index,
   s_interface_assoc_index,
   s_bdm_interface_index,
   s_cdc_interface_index,
   s_cdc_control_index,
   s_cdc_data_index,
   s_last_string_descriptor_index
};

/**
 * Device Descriptor
 */
static const DeviceDescriptor deviceDescriptor = {
      /* bLength             */ sizeof(DeviceDescriptor),
      /* bDescriptorType     */ DT_DEVICE,
      /* bcdUSB              */ nativeToLe16(0x0200),    // USB specification release No. [BCD = 2.00]
      /* bDeviceClass        */ 0x02,                    // Device Class code [CDC]
      /* bDeviceSubClass     */ 0x00,                    // Sub Class code    [none]
      /* bDeviceProtocol     */ 0x00,                    // Protocol          [none]
      /* bMaxPacketSize0     */ CONTROL_EP_MAXSIZE,      // EndPt 0 max packet size
      /* idVendor            */ nativeToLe16(VendorID),  // Vendor ID
      /* idProduct           */ nativeToLe16(ProductID), // Product ID for Composite device
      /* bcdDevice           */ nativeToLe16(VersionID), // Device Release    [BCD = 4.10]
      /* iManufacturer       */ s_manufacturer_index,    // String index of Manufacturer name
      /* iProduct            */ s_product_index,         // String index of product description
      /* iSerialNumber       */ s_serial_index,          // String index of serial number
      /* bNumConfigurations  */ 1                        // Number of configurations
};

enum {
   /** Interface number for CDC Control channel */
   CDC_COMM_INTF_ID,
   /** Interface number for CDC Data channel */
   CDC_DATA_INTF_ID,
   /** Total number of interfaces */
   NUMBER_OF_INTERFACES,
};

enum {
   /** USB Control endpoint number - must be zero */
   CONTROL_ENDPOINT  = 0,

   /** BDM Control and Data out endpoint number */
   BDM_OUT_ENDPOINT,
   /** BDM Data in endpoint number */
   BDM_IN_ENDPOINT,

   /** CDC Control endpoint number */
   CDC_CONTROL_ENDPOINT,
   /** CDC Data out endpoint number */
   CDC_DATA_OUT_ENDPOINT,
   /** CDC Data in endpoint number */
   CDC_DATA_IN_ENDPOINT,
   /** Total number of end-points */
   NUMBER_OF_ENDPOINTS,
};
/**
 * All other descriptors
 */
static const struct {
   ConfigurationDescriptor                  configDescriptor;

   InterfaceDescriptor                      cdc_CCI_Interface;
   CDCHeaderFunctionalDescriptor            cdc_Functional_Header;
   CDCCallManagementFunctionalDescriptor    cdc_CallManagement;
   CDCAbstractControlManagementDescriptor   cdc_Functional_ACM;
   CDCUnionFunctionalDescriptor             cdc_Functional_Union;
   EndpointDescriptor                       cdc_notification_Endpoint;

   InterfaceDescriptor                      cdc_DCI_Interface;
   EndpointDescriptor                       cdc_dataOut_Endpoint;
   EndpointDescriptor                       cdc_dataIn_Endpoint;

} otherDescriptors =
{
      { // configDescriptor
            /* bLength             */ sizeof(ConfigurationDescriptor),
            /* bDescriptorType     */ DT_CONFIGURATION,
            /* wTotalLength        */ nativeToLe16(sizeof(otherDescriptors)),
            /* bNumInterfaces      */ NUMBER_OF_INTERFACES,
            /* bConfigurationValue */ 1,
            /* iConfiguration      */ 0,
            /* bmAttributes        */ 0x80,                //  = Bus powered, no wakeup (yet?)
            /* bMaxPower           */ USBMilliamps(500)
      },
#if (HW_CAPABILITY&CAP_CDC)
      /**
       * CDC Control/Communication Interface, 1 end-point
       */
      { // cdc_CCI_Interface
            /* bLength                 */ sizeof(InterfaceDescriptor),
            /* bDescriptorType         */ DT_INTERFACE,
            /* bInterfaceNumber        */ CDC_COMM_INTF_ID,
            /* bAlternateSetting       */ 0,
            /* bNumEndpoints           */ 1,
            /* bInterfaceClass         */ 0x02,                         //  CDC Communication
            /* bInterfaceSubClass      */ 0x02,                         //  Abstract Control Model
            /* bInterfaceProtocol      */ 0x01,                         //  V.25ter, AT Command V.250
            /* iInterface description  */ s_cdc_control_index
      },
      { // cdc_Functional_Header
            /* bFunctionalLength       */ sizeof(CDCHeaderFunctionalDescriptor),
            /* bDescriptorType         */ CS_INTERFACE,
            /* bDescriptorSubtype      */ DST_HEADER,
            /* bcdCDC                  */ nativeToLe16(0x0110),
      },
      { // cdc_CallManagement
            /* bFunctionalLength       */ sizeof(CDCCallManagementFunctionalDescriptor),
            /* bDescriptorType         */ CS_INTERFACE,
            /* bDescriptorSubtype      */ DST_CALL_MANAGEMENT,
            /* bmCapabilities          */ 1,
            /* bDataInterface          */ CDC_DATA_INTF_ID,
      },
      { // cdc_Functional_ACM
            /* bFunctionalLength       */ sizeof(CDCAbstractControlManagementDescriptor),
            /* bDescriptorType         */ CS_INTERFACE,
            /* bDescriptorSubtype      */ DST_ABSTRACT_CONTROL_MANAGEMENT,
            /* bmCapabilities          */ 0x06,
      },
      { // cdc_Functional_Union
            /* bFunctionalLength       */ sizeof(CDCUnionFunctionalDescriptor),
            /* bDescriptorType         */ CS_INTERFACE,
            /* bDescriptorSubtype      */ DST_UNION_MANAGEMENT,
            /* bmControlInterface      */ CDC_COMM_INTF_ID,
            /* bSubordinateInterface0  */ {CDC_DATA_INTF_ID},
      },
      { // cdc_notification_Endpoint - IN,interrupt
            /* bLength                 */ sizeof(EndpointDescriptor),
            /* bDescriptorType         */ DT_ENDPOINT,
            /* bEndpointAddress        */ EP_IN|CDC_CONTROL_ENDPOINT,
            /* bmAttributes            */ ATTR_INTERRUPT,
            /* wMaxPacketSize          */ nativeToLe16(CDC_CONTROL_EP_MAXSIZE),
            /* bInterval               */ USBMilliseconds(255)
      },
      /**
       * CDC Data Interface, 2 end-points
       */
      { // cdc_DCI_Interface
            /* bLength                 */ sizeof(InterfaceDescriptor),
            /* bDescriptorType         */ DT_INTERFACE,
            /* bInterfaceNumber        */ CDC_DATA_INTF_ID,
            /* bAlternateSetting       */ 0,
            /* bNumEndpoints           */ 2,
            /* bInterfaceClass         */ 0x0A,                         //  CDC DATA
            /* bInterfaceSubClass      */ 0x00,                         //  -
            /* bInterfaceProtocol      */ 0x00,                         //  -
            /* iInterface description  */ s_cdc_data_index
      },
      { // cdc_dataOut_Endpoint - OUT,bulk
            /* bLength                 */ sizeof(EndpointDescriptor),
            /* bDescriptorType         */ DT_ENDPOINT,
            /* bEndpointAddress        */ EP_OUT|CDC_DATA_OUT_ENDPOINT,
            /* bmAttributes            */ ATTR_BULK,
            /* wMaxPacketSize          */ nativeToLe16(CDC_DATA_OUT_EP_MAXSIZE),
            /* bInterval               */ USBMilliseconds(1)
      },
      { // cdc_dataIn_Endpoint - IN,bulk
            /*  bLength                */ sizeof(EndpointDescriptor),
            /*  bDescriptorType        */ DT_ENDPOINT,
            /*  bEndpointAddress       */ EP_IN|CDC_DATA_IN_ENDPOINT,
            /*  bmAttributes           */ ATTR_BULK,
            /*  wMaxPacketSize         */ nativeToLe16(2*CDC_DATA_IN_EP_MAXSIZE), // x2 so all packets are terminating (short))
            /*  bInterval              */ USBMilliseconds(1)
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
      /* wVersion;            */  nativeToLe16(0x0100),
      /* wIndex;              */  nativeToLe16(0x0004),
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
      /* U16 wVersion;        */ nativeToLe16(0x0100),
      /* U16 wIndex;          */ nativeToLe16(5),
      /* U16 bnumSections;    */ nativeToLe16(2),
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
      /* U16 wNameLength0;     */ nativeToLe16(sizeof(msPropertiesFeatureDescriptor.bName0)),
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
      /* U16 wNameLength1;     */ nativeToLe16(sizeof(msPropertiesFeatureDescriptor.bName1)),
      /* U8  bName1[];         */ ICONS,
      /* U32 wPropertyLength1; */ CONST_NATIVE_TO_LE32(sizeof(msPropertiesFeatureDescriptor.bData1)),
      /* U8  bData1[];         */ ICON_PATH,
};

#define VENDOR_CODE 0x30
static const uint8_t OS_StringDescriptor[] = {18, DT_STRING, 'M',0,'S',0,'F',0,'T',0,'1',0,'0',0,'0',0,VENDOR_CODE,0x00};

#endif

/**
 * Device Status
 */
typedef struct {
   int selfPowered  : 1;    //!< Device is self-powered
   int remoteWakeup : 1;    //!< Supports remote wake-up
   int portTest     : 1;    //!< Port test
   int res1         : 5;    //!< Reserved
   int res2         : 8;    //!< Reserved
} DeviceStatus;

/**
 * USB device states
 */
typedef enum {
   USBpowered,
   USBattached,
   USBdefault,
   USBaddressed,
   USBconfigured,
   USBsuspended
} DeviceState;

struct {
   DeviceState    state:8;
   uint8_t        configuration;
   uint8_t        interfaceAltSetting;
   DeviceStatus   status;
   uint8_t        newUSBAddress;
} deviceState = {USBattached, 0, 0, {0,0,0,0,0}, 0};  //!< USB device state information

/**
 *  Endpoint Status
 */
typedef struct {
   int stall  : 1;   //!< Endpoint is stalled
   int res1   : 7;   //!< Reserved
   int res2   : 8;   //!< Reserved
} EndpointStatus; //!< Endpoint status in USB format

/**
 * Endpoint state values
 */
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
} EndpointState;

/**
 * Endpoint information
 */
typedef struct {
   uint8_t*  volatile       dataPtr;               //!< Pointer to data buffer for Rx/Tx
   uint16_t  volatile       dataRemaining;         //!< Count of remaining bytes to Rx/Tx
   uint16_t  volatile       dataCount;             //!< Count of bytes Rx/Tx so far
   uint8_t   volatile       shortInTransaction:1;  //!< Indicates that the IN transaction is undersized
   void   (*volatile callback)( void );            //!< Callback used on completion of packet reception
} EP0State;

/**
 * Endpoint hardware state
 */
typedef volatile struct {
   uint8_t        data0_1:1;   //!< Data 0/1 toggle state
   uint8_t        txOdd:1;     //!< Odd/Even tx buffer
   uint8_t        rxOdd:1;     //!< Odd/Even rx buffer
   EndpointState  state:5;     //!< End-point state
} EPHardwareState;

// Used to flag USB system configuration change etc.
static volatile uint8_t reInit;

static volatile EP0State ep0State = {nullptr, 0, 0, 0, nullptr};
static volatile EP0State ep1State = {nullptr, 0, 0, 0, nullptr};
static volatile EP0State ep2State = {nullptr, 0, 0, 0, nullptr};

static EPHardwareState epHardwareState[NUMBER_OF_ENDPOINTS] = {{0,0,0,EPIdle}};

/**
 * Structure representing USB activity
 */
typedef union {
   volatile uint8_t  byte;    //!< Allows access as byte (zero=>no activity, non-zero=>activity)
   volatile struct {
      int bdmActive:1;        //!< Any activity
      int serialOutActive:1;  //!< Serial out active
      int serialInActive:1;   //!< Serial in active
   } flags;                   //!< Overall flags
} ActivityType ;

/**
 *
 */
static ActivityType  usbActivityFlag;

/**
 * Buffer for EP0 Setup packet (copied from USB RAM)
 */
static SetupPacket ep0SetupBuffer;   //!< Buffer for EP0 Setup packet (copied from USB RAM)

static const char *getTokenName(int token) {
   switch (token) {
   default         : return "Unknown";
   case SOFToken   : return "SOFToken";
   case SETUPToken : return "SETUPToken";
   case OUTToken   : return "OUTToken";
   case INToken    : return "INToken";
   case DATA0Token : return "DATA0Token";
   case DATA1Token : return "DATA1Token";
   case DATA2Token : return "DATA2Token";
   case MDATAToken : return "MDATAToken";
   case ACKToken   : return "ACKToken";
   case NAKToken   : return "NAKToken";
   case NYETToken  : return "NYETToken";
   case STALLToken : return "STALLToken";
   case PREToken   : return "PREToken";
   }
}

//static const char * getStateName(EndpointState state){
//   switch (state) {
//   default         : return "Unknown";
//   case EPIdle     : return "EPIdle";
//   case EPDataIn   : return "EPDataIn";
//   case EPDataOut  : return "EPDataOut,";
//   case EPLastIn   : return "EPLastIn";
//   case EPStatusIn : return "EPStatusIn";
//   case EPStatusOut: return "EPStatusOut";
//   case EPThrottle : return "EPThrottle";
//   case EPStall    : return "EPStall";
//   case EPComplete : return "EPComplete";
//   }
//}

//__attribute__((unused))
//void reportBdt(const char *name, BdtEntry *bdt) {
//   (void)name;
//   (void)bdt;
//   if (bdt->u.setup.own) {
//      PRINTF("%s addr=0x%08lX, bc=%d, %s, %s, %s\n",
//            name,
//            bdt->addr, bdt->bc,
//            bdt->u.setup.data0_1?"DATA1":"DATA0",
//            bdt->u.setup.bdt_stall?"STALL":"OK",
//            "USB"
//            );
//   }
//   else {
//      PRINTF("%s addr=0x%08lX, bc=%d, %s, %s\n",
//            name,
//            bdt->addr, bdt->bc,
//            getTokenName(bdt->u.result.tok_pid),
//            "PROC"
//            );
//   }
//}
//
//__attribute__((unused))
//static void reportLineCoding(const LineCodingStructure *lineCodingStructure) {
//   (void)lineCodingStructure;
//   PRINTF("rate   = %ld bps\n", lineCodingStructure->dwDTERate);
//   PRINTF("format = %d\n", lineCodingStructure->bCharFormat);
//   PRINTF("parity = %d\n", lineCodingStructure->bParityType);
//   PRINTF("bits   = %d\n", lineCodingStructure->bDataBits);
//}
//
//__attribute__((unused))
//static void reportLineState(uint8_t value) {
//   (void)value;
//   PRINTF("Line state: RTS=%d, DTR=%d\n", (value&(1<<1))?1:0, (value&(1<<0))?1:0);
//}

/**
 * Initialise the endpoint buffer pointers once only
 */
static void initEndpointBuffers(void) {
   // EP0 Control I/O
   endPointBdts[CONTROL_ENDPOINT].txEven.addr = nativeToLe32((uint32_t)controlDataBuffer);
   endPointBdts[CONTROL_ENDPOINT].txOdd.addr  = nativeToLe32((uint32_t)controlDataBuffer);
   endPointBdts[CONTROL_ENDPOINT].rxEven.addr = nativeToLe32((uint32_t)controlDataBuffer);
   endPointBdts[CONTROL_ENDPOINT].rxOdd.addr  = nativeToLe32((uint32_t)controlDataBuffer);
   // EP1 USBDM out
   endPointBdts[BDM_OUT_ENDPOINT].rxEven.addr = nativeToLe32((uint32_t)bdmOutDataBuffer);
   endPointBdts[BDM_OUT_ENDPOINT].rxOdd.addr  = nativeToLe32((uint32_t)bdmOutDataBuffer);
   // EP2 USBDM in
   endPointBdts[BDM_IN_ENDPOINT].txEven.addr = nativeToLe32((uint32_t)bdmInDataBuffer);
   endPointBdts[BDM_IN_ENDPOINT].txOdd.addr  = nativeToLe32((uint32_t)bdmInDataBuffer);

#if (HW_CAPABILITY&CAP_CDC)
   // CDC Control
   endPointBdts[CDC_CONTROL_ENDPOINT].rxEven.addr = nativeToLe32((uint32_t)cdcControlDataBuffer);
   endPointBdts[CDC_CONTROL_ENDPOINT].rxOdd.addr  = nativeToLe32((uint32_t)cdcControlDataBuffer);
   // CDC Rx (out)
   endPointBdts[CDC_DATA_OUT_ENDPOINT].rxEven.addr = nativeToLe32((uint32_t)cdcOutDataBuffer);
   endPointBdts[CDC_DATA_OUT_ENDPOINT].rxOdd.addr  = nativeToLe32((uint32_t)cdcOutDataBuffer);
   // CDC Tx (in - ping-pong)
   endPointBdts[CDC_DATA_IN_ENDPOINT].txEven.addr = nativeToLe32((uint32_t)cdcInDataBuffer0);
   endPointBdts[CDC_DATA_IN_ENDPOINT].txOdd.addr  = nativeToLe32((uint32_t)cdcInDataBuffer1);
   (void)cdc_setRxBuffer((char *)cdcInDataBuffer0);
#endif
}

/**
 *  Configure the BDT for EP0 Out [Rx, device <- host, DATA0/1]
 *
 *  @param data0_1 - value for USB Data toggle
 */
static void ep0InitialiseBdtRx( uint8_t data0_1 ) {
   // Set up to Rx packet
   BdtEntry *bdt = epHardwareState[CONTROL_ENDPOINT].rxOdd?&endPointBdts[CONTROL_ENDPOINT].rxOdd:&endPointBdts[CONTROL_ENDPOINT].rxEven;
   bdt->bc = CONTROL_EP_MAXSIZE; // Always use CONTROL_EP_MAXSIZE so can accept SETUP packet
   if (data0_1) {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
      //      PRINTF("ep0InitialiseBdt.Rx.DATA1.%s\n", epHardwareState[CONTROL_ENDPOINT].rxOdd?"Odd":"Even");
   }
   else {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
      //      PRINTF("ep0InitialiseBdt.Rx.DATA0.%s\n", epHardwareState[CONTROL_ENDPOINT].rxOdd?"Odd":"Even");
   }
   //   reportBdt("ODD: ", &endPointBdts[CONTROL_ENDPOINT].rxOdd);
   //   reportBdt("EVEN:", &endPointBdts[CONTROL_ENDPOINT].rxEven);
}

/**
 * Save the data from an EP0 OUT packet and advance pointers etc.
 */
static uint8_t ep0SaveRxData( void ) {
   // Set up to Rx packet
   BdtEntry *bdt = (!epHardwareState[CONTROL_ENDPOINT].rxOdd)?&endPointBdts[CONTROL_ENDPOINT].rxOdd:&endPointBdts[CONTROL_ENDPOINT].rxEven;
   uint16_t size = bdt->bc;
   if (size > 0) {
      // Check if more data than requested - discard excess
      if (size > ep0State.dataRemaining) {
         size = ep0State.dataRemaining;
      }
      // Check if external buffer in use
      if (ep0State.dataPtr != nullptr) {
         // Copy the data from the Rx buffer to external buffer
         ( void )memcpy(ep0State.dataPtr, controlDataBuffer, size);
         ep0State.dataPtr    += size;   // Advance external buffer ptr
      }
      ep0State.dataRemaining -= size;   // Count down bytes to go
      ep0State.dataCount     += size;   // Count bytes so far
   }
   return size;
}

#if (HW_CAPABILITY&CAP_CDC)
/**
 *  Configure EP0 for an OUT transaction [Rx, device <- host, DATA0/1]
 *
 * @param bufSize - Size of data to transfer
 * @param bufPtr  - Buffer for data, may be nullptr - Useful when:
 *                - The transfer is < CONTROL_EP_MAXSIZE
 *                - Data may be used directly from controlDataBuffer
 *                - So no additional buffer is needed
 * @param data0_1 - Initial DATA0/DATA1 toggle value
 */
static void ep0StartRxTransaction( uint8_t bufSize, uint8_t *bufPtr ) {

   ep0State.dataRemaining     = bufSize; // Total bytes for Rx
   ep0State.dataCount         = 0;       // Reset count of bytes so far
   ep0State.dataPtr         = bufPtr;  // Where to (eventually) place data
   epHardwareState[CONTROL_ENDPOINT].data0_1 = DATA1;   // Initial data toggle

   if (bufSize == 0) {
      epHardwareState[CONTROL_ENDPOINT].state = EPStatusOut;  // Assume status handshake
   }
   else {
      epHardwareState[CONTROL_ENDPOINT].state = EPDataOut;    // Assume first of several data pkts
   }
   ep0InitialiseBdtRx(epHardwareState[CONTROL_ENDPOINT].data0_1); // Configure the BDT for transfer
}
#endif

/**
 * Configure EP0-out for a SETUP transaction [Rx, device<-host, DATA0]
 * Endpoint state is changed to EPIdle
 */
static void ep0ConfigureSetupTransaction( void ) {
   //   PUTS("ep0ConfigureSetupTransaction() - ");

   // Set up EP0-RX to Rx SETUP packets
   ep0InitialiseBdtRx(DATA0);
   epHardwareState[CONTROL_ENDPOINT].state = EPIdle;
}

/**
 *  Configure EP0-RX for a SETUP transaction [Rx, device<-host, DATA0]
 *  Only done if endpoint is not already configured for some other OUT transaction
 *  Endpoint state unchanged.
 */
static void ep0EnsureReadyForSetupTransaction( void ) {

   switch (epHardwareState[CONTROL_ENDPOINT].state) {
   case EPDataOut:        // Doing a sequence of OUT packets (until data count <= EPSIZE)
   case EPStatusOut:      // Doing an OUT packet as a status handshake
      // EP0-OUT is already set up for an OUT packet for data
      break;

   case EPDataIn:         // Doing a sequence of IN packets
   case EPLastIn:         // Doing the last IN packet
      // Set up EP0-OUT to Rx DATA1 Status packet
      //      ep0InitialiseBdtRx(DATA1);
      break;

   case EPStall:          // Stalled
   case EPIdle:           // Idle
   case EPStatusIn:       // Doing an IN packet as a status handshake for an OUT Data transfer
   default:
      // Set up EP0-OUT to Rx SETUP packets
      ep0InitialiseBdtRx(DATA0);
      break;
   }
}

/**
 *  Configure the BDT for EP0 In [Tx, device -> host]\n
 *  This would be part of a series of IN transactions set up by ep0StartTxTransaction()
 */
static void ep0InitialiseBdtTx( void ) {
   uint16_t size = ep0State.dataRemaining;
   if (size > CONTROL_EP_MAXSIZE) {
      size = CONTROL_EP_MAXSIZE;
   }
   // Copy the Tx data to EP buffer if not NULL
   // The controlDataBuffer may be used directly for small transactions
   if (ep0State.dataPtr != nullptr) {
      (void)memcpy(controlDataBuffer, ep0State.dataPtr, size);
   }
   ep0State.dataPtr         += size;  // Ptr to _next_ data
   ep0State.dataRemaining   -= size;  // Count of remaining bytes
   ep0State.dataCount       += size;  // Count of bytes so far

   // Set up to Tx packet
   BdtEntry *bdt = epHardwareState[CONTROL_ENDPOINT].txOdd?&endPointBdts[CONTROL_ENDPOINT].txOdd:&endPointBdts[CONTROL_ENDPOINT].txEven;
   bdt->bc = size;
   if (epHardwareState[CONTROL_ENDPOINT].data0_1) {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
      //      PRINTF("ep0InitialiseBdt.Tx.DATA1(%d bytes).%s\n", size, epHardwareState[CONTROL_ENDPOINT].txOdd?"Odd":"Even");
   }
   else {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
      //      PRINTF("ep0InitialiseBdt.Tx.DATA0(%d bytes).%s\n", size, epHardwareState[CONTROL_ENDPOINT].txOdd?"Odd":"Even");
   }
}

/**
 * Configure EP0 for a series of IN transactions [Tx, device -> host, DATA0/1]\n
 * This will be in response to a SETUP transaction\n
 * The data may be split into multiple DATA0/DATA1 packets
 *
 * @param bufSize Size of buffer to send
 * @param bufPtr  Pointer to buffer (may be NULL to indicate controlDataBuffer is being used directly)
 */
static void ep0StartTxTransaction( uint16_t bufSize, const uint8_t *bufPtr ) {

   if (bufSize > ep0SetupBuffer.wLength) {
      // More data than requested in SETUP request - truncate
      bufSize = (uint8_t)ep0SetupBuffer.wLength;
   }
   ep0State.dataPtr            = (uint8_t*)bufPtr;    // Pointer to _next_ data
   ep0State.dataRemaining      = bufSize;             // Count of remaining bytes
   ep0State.dataCount          = 0;                   // Reset count of bytes so far
   epHardwareState[CONTROL_ENDPOINT].data0_1  = DATA1;               // Initial data toggle
   ep0State.shortInTransaction = bufSize < ep0SetupBuffer.wLength; // Short transaction?

   if (bufSize == 0) {
      epHardwareState[CONTROL_ENDPOINT].state = EPStatusIn;   // Assume status handshake
   }
   else if ((bufSize < CONTROL_EP_MAXSIZE) ||    // Undersize packet OR
         ((bufSize == CONTROL_EP_MAXSIZE) &&     // Full size AND
               !ep0State.shortInTransaction)) {  //   Don't need to flag undersize transaction
      epHardwareState[CONTROL_ENDPOINT].state = EPLastIn;       // Sending one and only packet
   }
   else {
      epHardwareState[CONTROL_ENDPOINT].state = EPDataIn;       // Sending first of several packets
   }
   ep0InitialiseBdtTx(); // Configure the BDT for transfer
}

/**
 * Configure the BDT for EP1 Out [Rx, device <- host, DATA0/1]
 */
static void ep1InitialiseBdtRx( void ) {
   // Set up to Rx packet
   BdtEntry *bdt = epHardwareState[BDM_OUT_ENDPOINT].rxOdd?&endPointBdts[BDM_OUT_ENDPOINT].rxOdd:&endPointBdts[BDM_OUT_ENDPOINT].rxEven;

   // Set up to Rx packet
   bdt->bc = BDM_OUT_EP_MAXSIZE;
   if (epHardwareState[BDM_OUT_ENDPOINT].data0_1) {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
}

/**
 *  Save the data from an EP1 OUT packet and advance ptrs etc.
 */
static uint8_t ep1SaveRxData( void ) {
   // Get BDT
   BdtEntry *bdt = (!epHardwareState[BDM_OUT_ENDPOINT].rxOdd)?&endPointBdts[BDM_OUT_ENDPOINT].rxOdd:&endPointBdts[BDM_OUT_ENDPOINT].rxEven;
   uint8_t size = bdt->bc;

   if (size > 0) {
      // Check if more data than requested - discard excess
      if (size > ep1State.dataRemaining)
         size = ep1State.dataRemaining;
      // Check if external buffer in use
      if (ep1State.dataPtr != nullptr) {
         // Copy the data from the Rx buffer to external buffer
         ( void )memcpy(ep1State.dataPtr, bdmOutDataBuffer, size);
         ep1State.dataPtr    += size;   // Advance buffer ptr
      }
      ep1State.dataRemaining -= size;   // Count down bytes to go
      ep1State.dataCount     += size;   // Count bytes so far
   }
   return size;
}

/**
 *  Configure EP1-out for an OUT transaction [Rx, device <- host, DATA0/1]
 *
 *   @param bufSize - Size of data to transfer
 *   @param bufPtr  - Buffer for data
 */
static void ep1StartRxTransaction( uint8_t bufSize, uint8_t *bufPtr ) {
   ep1State.dataRemaining  = bufSize; // Total bytes to Rx
   ep1State.dataCount      = 0;       // Reset count of bytes so far
   ep1State.dataPtr      = bufPtr;  // Where to (eventually) place data

   epHardwareState[BDM_OUT_ENDPOINT].state = EPDataOut;    // Assume first of several data pkts

   ep1InitialiseBdtRx(); // Configure the BDT for transfer
}

/**
 * Configure the BDT for EP2 In [Tx, device -> host]
 */
static void ep2InitialiseBdtTx( void ) {
   // Set up to Rx packet
   BdtEntry *bdt = epHardwareState[BDM_IN_ENDPOINT].txOdd?&endPointBdts[BDM_IN_ENDPOINT].txOdd:&endPointBdts[BDM_IN_ENDPOINT].txEven;

   uint16_t size = ep2State.dataRemaining;
   if (size > BDM_IN_EP_MAXSIZE) {
      size = BDM_IN_EP_MAXSIZE;
   }
   // Copy the Tx data to EP buffer
   (void) memcpy(bdmInDataBuffer, ep2State.dataPtr, size);

   ep2State.dataPtr       += size;  // Ptr to _next_ data
   ep2State.dataRemaining   -= size;  // Count of remaining bytes
   ep2State.dataCount       += size;  // Count of bytes so far

   // Set up to Tx packet
   bdt->bc     = (uint8_t)size;
   if (epHardwareState[BDM_IN_ENDPOINT].data0_1) {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
}

static const uint8_t busyResponse[] = {BDM_RC_BUSY,1,2,3};
static uint8_t commandBusyFlag = false;

/**
 * Configure EP2 for an IN transaction [Tx, device -> host, DATA0/1]
 */
static void ep2StartTxTransaction( uint8_t bufSize, const uint8_t *bufPtr ) {

   ep2State.dataPtr          = (uint8_t*)bufPtr;    // Pointer to _next_ data
   ep2State.dataRemaining      = bufSize;             // Count of remaining bytes
   ep2State.dataCount          = 0;                   // Reset count of bytes so far

   // Note - Always terminate transfers with a truncated/zero packet
   if (bufSize < BDM_IN_EP_MAXSIZE) {         // Undersize packet
      epHardwareState[BDM_IN_ENDPOINT].state = EPLastIn;    // Sending one and only packet
   }
   else {
      epHardwareState[BDM_IN_ENDPOINT].state = EPDataIn;    // Sending first of several packets
   }
   ep2InitialiseBdtTx(); // Configure the BDT for transfer
}


/**
 * (re)Initialises end-points
 */
static void initialiseEndpoints(void) {

   //   PUTS("initialiseEndpoints()");

   // Clear USB BDTs
   memset((uint8_t*)endPointBdts, 0, sizeof(endPointBdts));

   // Clear hardware state
   memset((uint8_t*)epHardwareState, 0, sizeof(epHardwareState));

   initEndpointBuffers();
   epClearStall(CONTROL_ENDPOINT);
   epClearStall(BDM_OUT_ENDPOINT);  reInit = true;
   epClearStall(BDM_IN_ENDPOINT);

#if (HW_CAPABILITY&CAP_CDC)
   epClearStall(CDC_CONTROL_ENDPOINT);
   epClearStall(CDC_DATA_OUT_ENDPOINT);
   epClearStall(CDC_DATA_IN_ENDPOINT);

   ep3StartTxTransaction();       // Interrupt pipe IN - status
   ep4InitialiseBdtRx();          // Tx pipe OUT
   ep5StartTxTransactionIfIdle(); // Rx pipe IN
#endif

   // Clear odd/even bits & Enable Rx/Tx
   USB0->CTL = USB_CTL_USBENSOFEN_MASK|USB_CTL_ODDRST_MASK;
   USB0->CTL = USB_CTL_USBENSOFEN_MASK;

   // Enable end-points
   USB0->ENDPOINT[CONTROL_ENDPOINT].ENDPT      = USB_ENDPT_EPRXEN_MASK|USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK; // Rx/Tx/SETUP
   USB0->ENDPOINT[BDM_OUT_ENDPOINT].ENDPT      = USB_ENDPT_EPRXEN_MASK|                      USB_ENDPT_EPHSHK_MASK; // Rx
   USB0->ENDPOINT[BDM_IN_ENDPOINT].ENDPT       =                       USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK; // Tx
#if (HW_CAPABILITY&CAP_CDC)
   USB0->ENDPOINT[CDC_CONTROL_ENDPOINT].ENDPT  =                       USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK; // Tx
   USB0->ENDPOINT[CDC_DATA_OUT_ENDPOINT].ENDPT = USB_ENDPT_EPRXEN_MASK|                      USB_ENDPT_EPHSHK_MASK; // Rx
   USB0->ENDPOINT[CDC_DATA_IN_ENDPOINT].ENDPT  =                       USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK; // Tx
#endif
}

#if (HW_CAPABILITY&CAP_CDC)
//======================================================================
// Configure EP3 for an IN transaction [Tx, device -> host, DATA0/1]
//
static void ep3StartTxTransaction( void ) {
   const CDCNotification cdcNotification= {CDC_NOTIFICATION, SERIAL_STATE, 0, RT_INTERFACE, nativeToLe16(2)};
   uint8_t status = cdc_getSerialState();

   if ((status & SERIAL_STATE_CHANGE) == 0) {
      epHardwareState[CDC_CONTROL_ENDPOINT].state = EPIdle; // Not busy
      return;
   }
   // Copy the Tx data to Tx buffer
   (void)memcpy(cdcControlDataBuffer, &cdcNotification, sizeof(cdcNotification));
   cdcControlDataBuffer[sizeof(cdcNotification)+0] = status&~SERIAL_STATE_CHANGE;
   cdcControlDataBuffer[sizeof(cdcNotification)+1] = 0;

   // Set up to Tx packet
   BdtEntry *bdt = epHardwareState[CDC_CONTROL_ENDPOINT].txOdd?&endPointBdts[CDC_CONTROL_ENDPOINT].txOdd:&endPointBdts[CDC_CONTROL_ENDPOINT].txEven;
   bdt->bc = sizeof(cdcNotification)+2;
   if (epHardwareState[CDC_CONTROL_ENDPOINT].data0_1) {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
   epHardwareState[CDC_CONTROL_ENDPOINT].state = EPLastIn;    // Sending one and only packet
}

/**
 *  Configure the BDT for EP4 Out [Rx, device <- host, DATA0/1]
 *  CDC - OUT
 */
static void ep4InitialiseBdtRx( void ) {
   BdtEntry *bdt = epHardwareState[CDC_DATA_OUT_ENDPOINT].rxOdd?&endPointBdts[CDC_DATA_OUT_ENDPOINT].rxOdd:&endPointBdts[CDC_DATA_OUT_ENDPOINT].rxEven;

   // Set up to Rx packet
   bdt->bc = CDC_DATA_OUT_EP_MAXSIZE;
   if (epHardwareState[CDC_DATA_OUT_ENDPOINT].data0_1) {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
}

/**
 * Save the data from an EP4 OUT packet
 * CDC - OUT
 */
static void ep4SaveRxData( void ) {
   // Get BDT
   BdtEntry *bdt = (!epHardwareState[CDC_DATA_OUT_ENDPOINT].rxOdd)?&endPointBdts[CDC_DATA_OUT_ENDPOINT].rxOdd:&endPointBdts[CDC_DATA_OUT_ENDPOINT].rxEven;
   uint8_t size = bdt->bc;
   (void)cdc_putTxBuffer((char*)cdcOutDataBuffer, size);

   // Toggle on successful reception
   epHardwareState[CDC_DATA_OUT_ENDPOINT].data0_1 = !epHardwareState[CDC_DATA_OUT_ENDPOINT].data0_1;
}

/**
 * Configure the BDT for EP5 In [Tx, device -> host]
 * CDC - IN
 */
static void ep5InitialiseBdtTx(void) {
   // Set up to Tx packet
   if (epHardwareState[CDC_DATA_IN_ENDPOINT].txOdd) {
      // Set to write to other buffer & get count in current buffer
      endPointBdts[CDC_DATA_IN_ENDPOINT].txOdd.bc     = cdc_setRxBuffer((char*)cdcInDataBuffer0);
      //       cdcInDataBuffer1[0]       = '|';
      endPointBdts[CDC_DATA_IN_ENDPOINT].txOdd.u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK;
   }
   else {
      // Set to write to other buffer & get count in current buffer
      endPointBdts[CDC_DATA_IN_ENDPOINT].txEven.bc    = cdc_setRxBuffer((char*)cdcInDataBuffer1);
      //       cdcInDataBuffer0[0]       = '^';
      endPointBdts[CDC_DATA_IN_ENDPOINT].txEven.u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK;
   }
   //   epHardwareState[CDC_DATA_IN_ENDPOINT].data0_1 = !epHardwareState[CDC_DATA_IN_ENDPOINT].data0_1; // Toggle data0/1
   epHardwareState[CDC_DATA_IN_ENDPOINT].txOdd     = !epHardwareState[CDC_DATA_IN_ENDPOINT].txOdd;
}

static uint8_t serialDelayCount = 0;

// This value controls how long the serial interface will wait before
// sending a buffered character. (count of SOFs ~ ms)
constexpr uint SERIAL_THRESHOLD = 0; // ms

/**
 * Configure the BDT for EP5 In [Tx, device -> host]
 * CDC - IN
 */
static void ep5StartTxTransactionIfIdle() {
#if 0
   if ((epHardwareState[CDC_DATA_IN_ENDPOINT].state == EPIdle) && (cdc_rxBufferItemCount()>0)) {
      ep5InitialiseBdtTx();
      epHardwareState[CDC_DATA_IN_ENDPOINT].state = EPLastIn;
      serialDelayCount = 0;
   }
   else if ((epHardwareState[CDC_DATA_IN_ENDPOINT].state == EPLastIn) && (cdc_rxBufferItemCount()==16)) {
      ep5InitialiseBdtTx();
      epHardwareState[CDC_DATA_IN_ENDPOINT].state = EPDataIn;
      serialDelayCount = 0;
   }
#else
   if ((epHardwareState[CDC_DATA_IN_ENDPOINT].state == EPIdle) && (cdc_rxBufferItemCount()>0)) {
      ep5InitialiseBdtTx();
      epHardwareState[CDC_DATA_IN_ENDPOINT].state = EPDataIn;
      serialDelayCount = 0;
   }
#endif
}

void checkUsbCdcRxData(void) {
   ep5StartTxTransactionIfIdle();
}
#endif

/**
 *   Receive a command over EP1
 *
 *   @param maxSize  = max # of bytes to receive
 *   @param buffer   = ptr to buffer for bytes received
 *
 *   @note : Doesn't return until command has been received.
 *   @note : Format
 *       - [0]    = size of command (N)
 *       - [1]    = command
 *       - [2..N] = parameters
 *
 *   =======================================================
 *   Format - a command is made up of up to 2 pkts
 *   The size of the 1st packet indicates if subsequent pkts
 *   are used.
 *
 *    1st packet
 *   +--------------------------+
 *   |  Size of entire command  |  0 - size includes 2nd packet
 *   +--------------------------+
 *   |  Command byte            |  1
 *   +--------------------------+
 *   |                          |  2... up to BDM_OUT_EP_MAXSIZE-2
 *   | //// DATA ////////////// |
 *   |                          |
 *   +--------------------------+
 *    2nd packet (optional)
 *   +--------------------------+
 *   |  0                       |  0 - Ensures packet can't be mistaken as 1st packet
 *   +--------------------------+
 *   |                          |  1... up to BDM_OUT_EP_MAXSIZE-1
 *   | //// DATA ////////////// |
 *   |                          |
 *   +--------------------------+
 *
 */
void Usb0::receiveUSBCommand(uint8_t maxSize, uint8_t buffer[]) {
   uint8_t size;

   // Wait for USB connection
   while(deviceState.state != USBconfigured) {
      __WFI();
   }

   USBDM::Usb0::enableNvicInterrupts();
   // Size of first (command) transaction
   do {
      size = BDM_OUT_EP_MAXSIZE;
      if (size > maxSize) {
         size = maxSize;
      }
      // Get 1st/only packet of command
      reInit = false;
      ep1StartRxTransaction( size, buffer );
      while ((epHardwareState[BDM_OUT_ENDPOINT].state != EPComplete) && !reInit) {
         __WFI();
      }
      if (reInit) {
         continue;
      }
      // Size for entire command from 1st packet
      size = buffer[0];
      if (size > maxSize) {
         size = maxSize;
      }
      if (size == 0) {
         // Invalid packet - try again
         // 0 indicates this is not an initial command packet
         // but part of a longer command
         reInit = true;
         continue;
      }
      // Receive rest of data if present (only possibly 2 transactions total)
      if (size > ep1State.dataCount) {
         // Save last byte of 1st packet as overwritten by
         // second packet (to save moving 2nd packet when size is discarded)
         uint8_t saveByteOffset = ep1State.dataCount-1;
         uint8_t saveByte       = buffer[saveByteOffset];
         ep1StartRxTransaction( size-saveByteOffset, buffer+saveByteOffset );
         while ((epHardwareState[BDM_OUT_ENDPOINT].state != EPComplete) && !reInit) {
            __WFI();
         }
         // Check if second packet has correct marker
         if (buffer[saveByteOffset] != 0) {
            // packet corrupt
            reInit = true;
         }
         // Restore saved byte
         buffer[saveByteOffset] = saveByte;
      }
   } while (reInit);
   //   PUTS(epHardwareState[BDM_OUT_ENDPOINT].data0_1?"receiveUSBCommand-1\n":"receiveUSBCommand-0\n");
}

/**
 *  Set a command response over EP2
 *
 *  @param size   = # of bytes to send
 *  @param buffer = ptr to bytes to send
 *
 *  @note : Returns before the command has been sent.
 *
 *  @note : Format
 *      - [0]    = response
 *      - [1..N] = parameters
 */
void Usb0::sendUSBResponse( uint8_t size, const uint8_t buffer[]) {
   commandBusyFlag = false;
   //   enableUSBIrq();
   //   while (epHardwareState[BDM_IN_ENDPOINT].state != EPIdle) {
   //   }
   //   PUTS(epHardwareState[BDM_IN_ENDPOINT].data0_1?"sendUSBResponse-1\n":"sendUSBResponse-0\n");
   ep2StartTxTransaction(size, buffer);
}

//======================================================================
// Stall control for end-points
//

/**
 * Stall endpoint
 *
 *  @param epNum - endpoint number
 */
static void epStall(uint8_t epNum) {
   if (epNum == CONTROL_ENDPOINT) {
      // Stall Tx only
      BdtEntry *bdt = epHardwareState[CONTROL_ENDPOINT].txOdd?&endPointBdts[CONTROL_ENDPOINT].txOdd:&endPointBdts[CONTROL_ENDPOINT].txEven;
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_STALL_MASK|BDTEntry_DTS_MASK;
   }
   else {
      epHardwareState[epNum].state    = EPStall;
      epHardwareState[epNum].data0_1  = DATA0;
      USB0->ENDPOINT[epNum].ENDPT    |= USB_ENDPT_EPSTALL_MASK;
   }
}

/*
 * Clear Stall on endpoint
 *
 *  @param epNum - endpoint number
 */
static void epClearStall(uint8_t epNum) {
   USB0->ENDPOINT[epNum].ENDPT          &= ~USB_ENDPT_EPSTALL_MASK;
   epHardwareState[epNum].state          = EPIdle;
   epHardwareState[epNum].data0_1        = DATA0;
}

//========================================================================================
//
static void setUSBdefaultState( void ) {
   ActivityLed::off();
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
      ActivityLed::off();
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
      ActivityLed::on();
      deviceState.state                = USBconfigured;
      deviceState.configuration        = config;
      deviceState.interfaceAltSetting  = 0;
   }
}

/**
 * Handler for USB Bus reset
 */
template<>
void UsbBase_T<Usb0Info>::handleUSBReset() {
   // Disable all interrupts
   usb->INTEN = 0x00;
   usb->ERREN = 0x00;

   // Clear USB error flags
   usb->ERRSTAT = 0xFF;

   // Clear all USB interrupt flags
   usb->ISTAT = 0xFF;

   setUSBdefaultState();

   initialiseEndpoints();

   // Set up to receive 1st SETUP packet
   ep0ConfigureSetupTransaction();

   // Enable various interrupts
   usb->INTEN = USB_INTMASKS|USB_INTEN_ERROREN_MASK;
   usb->ERREN = 0xFF;
}

/**
 * Initialise the USB interface
 *
 *  @note Assumes clock set up for USB operation (48MHz)
 */
template<>
void UsbBase_T<Usb0Info>::initialise() {

   assert(USBDM::SimInfo::getUsbClock() == 48000000);

   enable();

   // Make sure no interrupt during setup
   enableNvicInterrupts(false);

   // Clear USB RAM (includes BDTs)
   memset((uint8_t*)endPointBdts, 0, sizeof(endPointBdts));

   usb->OTGISTAT = 0;
   usb->OTGICR   = 0;
   usb->OTGCTL   = 0;
   usb->INTEN    = 0;
   usb->ERRSTAT  = 0;
   usb->ERREN    = 0;
   usb->CTL      = 0;
   usb->ADDR     = 0;
   for (unsigned i=0; i<(sizeof(usb->ENDPOINT)/sizeof(usb->ENDPOINT[CONTROL_ENDPOINT])); i++) {
      usb->ENDPOINT[i].ENDPT = 0;
   }
   usb->USBCTRL = 0;
   usb->CONTROL = 0;
   usb->USBTRC0 = 0;

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

   // Removed due to errata e5928: USBOTG: USBx_USBTRC0[USBRESET] bit does not operate as expected in all cases
   // Reset USB H/W
   //   USB0_USBTRC0 = 0x40|USB_USBTRC0_USBRESET_MASK;
   //   while ((USB0_USBTRC0&USB_USBTRC0_USBRESET_MASK) != 0) {
   //   }

   // This bit is undocumented but seems to be necessary
   usb->USBTRC0 = 0x40;

   // Set initial USB state
   setUSBdefaultState();

   // Point USB at BDT array
   usb->BDTPAGE3 = (uint8_t) (((unsigned)endPointBdts)>>24);
   usb->BDTPAGE2 = (uint8_t) (((unsigned)endPointBdts)>>16);
   usb->BDTPAGE1 = (uint8_t) (((unsigned)endPointBdts)>>8);

   // Clear all pending interrupts
   usb->ISTAT = 0xFF;

   // Enable usb reset interrupt
   usb->INTEN = USB_INTEN_USBRSTEN_MASK|USB_INTEN_SLEEPEN_MASK;

   // Weak pull downs
   usb->USBCTRL = USB_USBCTRL_PDE_MASK;

   // Enable Pull-up
   usb->CONTROL = USB_CONTROL_DPPULLUPNONOTG_MASK;

   // Enable interface
   usb->CTL = USB_CTL_USBENSOFEN_MASK;

   // Enable USB interrupts
   enableNvicInterrupts();
}

/**
 * Get Status - Device Req 0x00
 */
static void handleGetStatus( void ) {
   static const uint8_t        zeroReturn[]    = {0,0};
   static const EndpointStatus epStatusStalled = {1,0,0};
   static const EndpointStatus epStatusOK      = {0,0,0};

   const uint8_t *dataPtr = nullptr;
   uint8_t size;
   uint8_t epNum;

   switch(ep0SetupBuffer.bmRequestType) {
   case EP_IN|RT_DEVICE : // Device Status
      dataPtr = (uint8_t *) &deviceState.status;
      size    = sizeof(deviceState.status);
      break;

   case EP_IN|RT_INTERFACE : // Interface Status - reserved
      dataPtr = zeroReturn;
      size = sizeof(zeroReturn);
      break;

   case EP_IN|RT_ENDPOINT : // Endpoint Status
      epNum = ep0SetupBuffer.wIndex&0x07;
      if (epNum <= NUMBER_OF_ENDPOINTS) {
         if (USB0->ENDPOINT[epNum].ENDPT&USB_ENDPT_EPSTALL_MASK) {
            dataPtr = (uint8_t*)&epStatusStalled;
         }
         else {
            dataPtr = (uint8_t*)&epStatusOK;
         }
         size = sizeof(EndpointStatus);
      }
      break;
   }
   if (dataPtr != nullptr)
      ep0StartTxTransaction( size, dataPtr );
   else
      epStall(0);
}

/**
 * Clear Feature - Device Req 0x01
 */
static void handleClearFeature( void ) {
   bool okResponse = false;

   switch(ep0SetupBuffer.bmRequestType) {
   case RT_DEVICE : // Device Feature
      if ((ep0SetupBuffer.wValue != DEVICE_REMOTE_WAKEUP) || // Device remote wake up
            (ep0SetupBuffer.wIndex != 0))   {                  // Device index must be 0
         break;
      }
      deviceState.status.remoteWakeup = 0;
      okResponse                      = true;
      break;

   case RT_INTERFACE : // Interface Feature
      break;

   case RT_ENDPOINT : { // Endpoint Feature ( Out,Standard,Endpoint )
      uint8_t epNum = ep0SetupBuffer.wIndex&0x07;
      if ((ep0SetupBuffer.wValue != ENDPOINT_HALT) || // Not Endpoint Stall ?
            (epNum >= NUMBER_OF_ENDPOINTS))                        // or illegal EP# (ignores direction)
         break;
      epClearStall(epNum);
      okResponse = true;
   }
   break;

   default : // Illegal
      break;
   }

   if (okResponse)
      ep0StartTxTransaction( 0, nullptr ); // Tx empty Status packet
   else
      epStall(0);
}

/**
 * Set Feature - Device Req 0x03
 */
static void handleSetFeature( void ) {
   bool okResponse = false;

   switch(ep0SetupBuffer.bmRequestType) {
   case RT_DEVICE : // Device Feature
      if ((ep0SetupBuffer.wValue != DEVICE_REMOTE_WAKEUP) || // Device remote wakeup
            (ep0SetupBuffer.wIndex != 0)) {                  // device wIndex must be 0
         break;
      }
      deviceState.status.remoteWakeup = 1;
      okResponse                      = true;
      break;

   case RT_INTERFACE : // Interface Feature
      break;

   case RT_ENDPOINT : { // Endpoint Feature ( Out,Standard,Endpoint )
      uint8_t epNum = ep0SetupBuffer.wIndex&0x07;
      if ((ep0SetupBuffer.wValue != ENDPOINT_HALT) || // Not Endpoint Stall ?
            (epNum >= NUMBER_OF_ENDPOINTS))  {                   // or illegal EP# (ignores direction)
         break;
      }
      epStall(epNum);
      okResponse = true;
   }
   break;

   default : // Illegal
      break;
   }

   if (okResponse) {
      ep0StartTxTransaction( 0, nullptr ); // Tx empty Status packet
   }
   else {
      epStall(0);
   }
}

/**
 *  Creates a valid string descriptor in UTF-16-LE from a limited UTF-8 string
 *
 *  @param source - Zero terminated UTF-8 C string
 *
 *  @param dest   - Where to place descriptor
 *
 *  @note Only handles UTF-16 characters that fit in a single UTF-16 'character'.
 */
static void utf8ToStringDescriptor(const uint8_t *source, uint8_t *dest) {
   uint8_t *size = dest; // 1st byte is where to place descriptor size

   *dest++ = 2;         // 1st byte = descriptor size (2 bytes so far)
   *dest++ = DT_STRING; // 2nd byte = DT_STRING;

   while (*source != '\0') {
      uint16_t utf16Char=0;
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

/**
 * Get Descriptor - Device Req 0x06
 */
static void handleGetDescriptor( void ) {
   unsigned        descriptorIndex = (unsigned)ep0SetupBuffer.wValue.lo();
   int             dataSize = 0;
   const uint8_t  *dataPtr = nullptr;

   if (ep0SetupBuffer.bmRequestType != (EP_IN|RT_DEVICE)) {
      // Must be In,Standard,Device
      epStall(0);
      return;
   }
   switch (ep0SetupBuffer.wValue.hi()) {

   case DT_DEVICE: // Get Device Descriptor - 1
      //      PUTS("getDescriptor-device - ");
      dataPtr  = (uint8_t *) &deviceDescriptor;
      dataSize = sizeof(DeviceDescriptor);
      break;
   case DT_CONFIGURATION: // Get Configuration Descriptor - 2
      //      PUTS("getDescriptor-config - ");
      if (ep0SetupBuffer.wValue.lo() != 0) {
         epStall(0);
         return;
      }
      dataPtr  = (uint8_t *) &otherDescriptors;
      dataSize = sizeof(otherDescriptors);
      break;
   case DT_DEVICEQUALIFIER: // Get Device Qualifier Descriptor
      //      PUTS("getDescriptor-deviceQ - ");
      epStall(0);
      return;
   case DT_STRING: // Get String Desc.- 3
      //      PRINTF("getDescriptor-string - %d\n", descriptorIndex);
#ifdef MS_COMPATIBLE_ID_FEATURE
      if (descriptorIndex == 0xEE) {
         //         PUTS("getDescriptor-string - MS_COMPATIBLE_ID_FEATURE");
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
      if (descriptorIndex == s_language_index) {
         // Language bytes (unchanged)
         dataPtr  = (uint8_t *)stringDescriptors[s_language_index];
      }
#if defined(UNIQUE_ID)
      else if (descriptorIndex == s_serial_index) {
         uint8_t utf8Buff[sizeof(SERIAL_NO)+10];

         // Generate Semi-unique Serial number
         uint32_t uid = SIM->UIDH^SIM->UIDMH^SIM->UIDML^SIM->UIDL;
         snprintf((char *)utf8Buff, sizeof(utf8Buff), SERIAL_NO, uid);

         dataPtr = commandBuffer;
         utf8ToStringDescriptor(utf8Buff, commandBuffer);
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
      // shouldn't happen
      epStall(0);
      return;
   } // switch
   ep0StartTxTransaction( (uint8_t)dataSize, dataPtr ); // Set up Tx
}

/**
 * resetDevice Callback to execute after status transaction
 */
static void resetDeviceCallback( void ) {
   //   forceICPReset();
}

/**
 * SetDeviceAddress Callback - executed after status transaction
 */
static void setAddressCallback( void ) {
   if ((deviceState.state == USBdefault) && (deviceState.newUSBAddress != 0)) {
      setUSBaddressedState(deviceState.newUSBAddress);
   }
   else if ((deviceState.state == USBaddressed) && (deviceState.newUSBAddress == 0)) {
      setUSBdefaultState();
   }
}

/**
 * Set device Address - Device Req 0x05
 */
static void handleSetAddress( void ) {
   //   PUTS("setAddress - ");

   if (ep0SetupBuffer.bmRequestType != (EP_OUT|RT_DEVICE)) {// Out,Standard,Device
      epStall(0); // Illegal format - stall ep0
      return;
   }
   // Save address for change after status transaction
   deviceState.newUSBAddress  = ep0SetupBuffer.wValue.lo();
   ep0State.callback          = setAddressCallback;

   ep0StartTxTransaction( 0, nullptr ); // Tx empty Status packet
}

/**
 * Get Configuration - Device Req 0x08
 */
static void handleGetConfiguration( void ) {
   //   PUTS("getConfiguration");

   ep0StartTxTransaction( 1, (uint8_t *) &deviceState.configuration );
}

/**
 * Set Configuration - Device Req 0x09
 * Treated as soft reset
 */
static void handleSetConfiguration( void ) {
   //   PUTS("setConfiguration");

   reInit = true;  // tell command handler to re-init

   if ((ep0SetupBuffer.bmRequestType != (EP_OUT|RT_DEVICE)) || // Out,Standard,Device
         ((ep0SetupBuffer.wValue.lo() != 0) &&       // Only supports 0=> un-configure, 1=> only valid configuration
               (ep0SetupBuffer.wValue.lo() != otherDescriptors.configDescriptor.bConfigurationValue))) {
      epStall(0);
      return;
   }
   setUSBconfiguredState(ep0SetupBuffer.wValue.lo());

   initialiseEndpoints();

   ep1InitialiseBdtRx();

   ep0StartTxTransaction( 0, nullptr ); // Tx empty Status packet
}

/**
 * Set interface - Device Req 0x0B
 * Not required to be implemented
 */
static void handleSetInterface( void ) {
   //   PUTS("setInterface");

   if ((ep0SetupBuffer.bmRequestType != (EP_OUT|RT_INTERFACE)) || // NOT In,Standard,Interface
         (ep0SetupBuffer.wLength != 0) ||                    // NOT correct length
         (deviceState.state != USBconfigured)) {                  // NOT in addressed state
      epStall(0); // Error
      return;
   }
   // Only support one Alternate Setting == 0
   if (ep0SetupBuffer.wValue != 0) {
      epStall(0); // Error
      return;
   }
   // Tx empty Status packet
   ep0StartTxTransaction( 0, nullptr );
}

/**
 * Get interface - Device Req 0x0A
 */
static void handleGetInterface( void ) {
   static const uint8_t interfaceAltSetting = 0;

   //   PUTS("getInterface");

   if ((ep0SetupBuffer.bmRequestType != (EP_IN|RT_INTERFACE)) || // NOT In,Standard,Interface
         (ep0SetupBuffer.wLength != 1)) {                   // NOT correct length
      epStall(0); // Error
      return;
   }
   // Send packet
   ep0StartTxTransaction( sizeof(interfaceAltSetting), &interfaceAltSetting );
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
//   ep0StartInTransaction( 0, nullptr ); // Tx empty Status packet
//}

static void handleGetLineCoding() {
   //   PUTS("getLineCoding");
   const LineCodingStructure *lc = cdc_getLineCoding();
   //   reportLineCoding(lc);
   // Send packet
   ep0StartTxTransaction( sizeof(LineCodingStructure), (const uint8_t*)lc);
}

static void setLineCodingCallback( void ) {
   cdc_setLineCoding((LineCodingStructure * const)&controlDataBuffer);
   //   reportLineCoding((LineCodingStructure * const)&controlDataBuffer);
}

static void handleSetLineCoding() {
   //   PUTS("setLineCoding");

   ep0State.callback = setLineCodingCallback;

   // Don't use buffer - this requires sizeof(LineCodingStructure) < CONTROL_EP_MAXSIZE
   ep0StartRxTransaction(sizeof(LineCodingStructure), nullptr);
}

static void handleSetControlLineState() {
   //   PUTS("setControlLineState");
   //   reportLineState(ep0SetupBuffer.wValue.lo());
   cdc_setControlLineState(ep0SetupBuffer.wValue.lo());
   ep0StartTxTransaction( 0, nullptr ); // Tx empty Status packet
}

static void handleSendBreak() {
   //   PUTS("sendBreak");

   cdc_sendBreak(ep0SetupBuffer.wValue);  // time in milliseconds, 0xFFFF => continuous
   ep0StartTxTransaction( 0, nullptr );   // Tx empty Status packet
}
#endif

/**
 * Illegal request in SETUP packet
 */
static void handleUnexpected( void ) {
   //   PUTS("unexpected");
   epStall(0);
}

/**
 * Handles SETUP Packet
 */
static void handleSetupToken( void ) {
   //   PUTS("ep0SetupTok");

   // Save data from SETUP packet
   memcpy(&ep0SetupBuffer, controlDataBuffer, sizeof(ep0SetupBuffer));

   epHardwareState[CONTROL_ENDPOINT].data0_1  = DATA1;
   epHardwareState[CONTROL_ENDPOINT].state    = EPIdle;

   // Call backs only persist during a SETUP transaction
   ep0State.callback = nullptr;

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
            //            PUTS("REQ_TYPE_VENDOR");
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
               //               PUTS("REQ_TYPE_VENDOR - VENDOR_CODE");
               if (ep0SetupBuffer.wIndex == 0x0004) {
                  //                  PUTS("REQ_TYPE_VENDOR - msCompatibleIdFeatureDescriptor");
                  ep0StartTxTransaction(sizeof(msCompatibleIdFeatureDescriptor), (const uint8_t *)&msCompatibleIdFeatureDescriptor);
               }
               else if (ep0SetupBuffer.wIndex == 0x0005) {
                  //                  PUTS("REQ_TYPE_VENDOR - msPropertiesFeatureDescriptor");
                  ep0StartTxTransaction(sizeof(msPropertiesFeatureDescriptor), (const uint8_t *)&msPropertiesFeatureDescriptor);
               }
               else {
                  handleUnexpected();
               }
               break;
#endif

            case CMD_USBDM_ICP_BOOT :
               // Reboots to ICP mode
               ep0State.callback = resetDeviceCallback;
               ep0StartTxTransaction( 0, nullptr ); // Tx empty Status packet
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

   ep0EnsureReadyForSetupTransaction();   // In case another SETUP packet

   // Allow transactions post SETUP packet (clear TXSUSPENDTOKENBUSY)
   USB0->CTL = USB_CTL_USBENSOFEN_MASK;
}

/**
 * ep0 - IN
 */
static void ep0HandleInToken( void ) {

   epHardwareState[CONTROL_ENDPOINT].data0_1 = !epHardwareState[CONTROL_ENDPOINT].data0_1;   // Toggle DATA0/1

   switch (epHardwareState[CONTROL_ENDPOINT].state) {
   case EPDataIn:    // Doing a sequence of IN packets (until data count <= EPSIZE)
      //   {
      //      PRINTF("ep0InTok-EPDataIn - ");
      //      uint8_t *p = (uint8_t*)(bdts[USB0->STAT>>2].addr);
      //      for(int i=0; i<bdts[USB0->STAT>>2].bc; i++) {
      //         PRINTF("%2.2X ", *p++);
      //      }
      //      PUTS("");
      //   }
      if ((ep0State.dataRemaining < CONTROL_EP_MAXSIZE) ||   // Undersize packet OR
            ((ep0State.dataRemaining == CONTROL_EP_MAXSIZE) && // Full size AND
                  !ep0State.shortInTransaction)) {
         // Don't need to flag undersize transaction
         epHardwareState[CONTROL_ENDPOINT].state = EPLastIn;    // Sending last packet
      }
      else {
         epHardwareState[CONTROL_ENDPOINT].state = EPDataIn;    // Sending full packet
      }
      ep0InitialiseBdtTx(); // Set up next IN packet
      break;

   case EPLastIn:
      //      {
      //         PRINTF("ep0InTok-EPLastIn - ");
      //         uint8_t *p = (uint8_t*)(bdts[USB0->STAT>>2].addr);
      //         for(int i=0; i<bdts[USB0->STAT>>2].bc; i++) {
      //            PRINTF("%2.2X ", *p++);
      //         }
      //         PUTS("");
      //      }
      // Just done the last IN packet
      epHardwareState[CONTROL_ENDPOINT].state = EPStatusOut;   // Receiving an OUT status packet
      ep0InitialiseBdtRx(DATA1);
      break;

   case EPStatusIn:
      // Just done an IN packet as a status handshake for an OUT Data transfer
      //      PUTS("ep0InTok-EPStatusIn");
      epHardwareState[CONTROL_ENDPOINT].state = EPIdle;           // Now Idle
      if (ep0State.callback != nullptr) {
         // Execute callback function to process OUT data
         //         PUTS("callback");
         ep0State.callback();
      }
      ep0State.callback = nullptr;
      break;

      // We don't expect an IN token while in the following states
   case EPIdle:           // Idle (Tx complete)
   case EPDataOut:        // Doing a sequence of OUT packets (until data count <= EPSIZE)
   case EPStatusOut:      // Doing an OUT packet as a status handshake
   case EPStall:
   case EPComplete:
   case EPThrottle:
      PRINTF("ep0InTok- Unexpected state=%d\n", epHardwareState[CONTROL_ENDPOINT].state);
      break;
   }
}

/**
 * ep0 - OUT
 */
static void ep0HandleOutToken( void ) {
   uint8_t transferSize;

   epHardwareState[CONTROL_ENDPOINT].data0_1 = !epHardwareState[CONTROL_ENDPOINT].data0_1; // Toggle DATA0/1

   switch (epHardwareState[CONTROL_ENDPOINT].state) {
   case EPDataOut:        // Receiving a sequence of OUT packets
      //      PUTS("ep0HandleOutToken - EPDataOut");
      transferSize = ep0SaveRxData();          // Save the data from the Rx buffer
      // Check if completed an under-size packet or expected number of bytes
      if ((transferSize < CONTROL_EP_MAXSIZE) || (ep0State.dataRemaining == 0)) { // Last packet?
         epHardwareState[CONTROL_ENDPOINT].state = EPIdle;
         ep0StartTxTransaction(0, nullptr); // Do status packet transmission
      }
      else {
         ep0InitialiseBdtRx(epHardwareState[CONTROL_ENDPOINT].data0_1); // Set up next OUT packet
      }
      break;

   case EPStatusOut:       // Done an OUT packet as a status handshake
      //      PUTS("ep0HandleOutToken - EPStatusOut");
      epHardwareState[CONTROL_ENDPOINT].state = EPIdle;
      break;

      // We don't expect an OUT token while in the following states
   case EPLastIn:          // Just done the last IN packet
   case EPDataIn:          // Doing a sequence of IN packets (until data count <= EPSIZE)
   case EPStatusIn:        // Just done an IN packet as a status handshake
   case EPIdle:            // Idle (Tx complete)
   case EPComplete:
   case EPStall:
   case EPThrottle:
      PRINTF("ep0HandleOutToken - unexpected, s = %d\n", epHardwareState[CONTROL_ENDPOINT].state);
      epHardwareState[CONTROL_ENDPOINT].state = EPIdle;
      //      ep0StartTxTransaction(0, nullptr); // Do status packet transmission
      break;
   }
   ep0EnsureReadyForSetupTransaction();  // Make ready for a SETUP packet
}

/**
 * ep1 - OUT (host->device)
 */
static void ep1HandleOutToken( void ) {
   uint8_t transferSize;

   epHardwareState[BDM_OUT_ENDPOINT].data0_1 = !epHardwareState[BDM_OUT_ENDPOINT].data0_1;   // Toggle DATA0/1
   //   PUTS(epHardwareState[BDM_OUT_ENDPOINT].data0_1?"ep1HandleOutToken-T-1\n":"ep1HandleOutToken-T-0\n");

   switch (epHardwareState[BDM_OUT_ENDPOINT].state) {
   case EPDataOut:        // Doing a sequence of OUT packets making up a command
      transferSize = ep1SaveRxData();          // Save the data from the Rx buffer
      // Completed transfer on undersize packet or received expected number of bytes
      if ((transferSize < BDM_OUT_EP_MAXSIZE) || (ep1State.dataRemaining == 0)) { // Last packet?
         epHardwareState[BDM_OUT_ENDPOINT].state = EPComplete;
      }
      else {
         ep1InitialiseBdtRx(); // Set up next OUT packet
      }
      break;

      // We don't expect an OUT token while in the following states
   case EPIdle:           // Idle (Rx complete)
   case EPComplete:       // Command reception complete (Rx complete)
   case EPLastIn:         // Just done the last IN packet
   case EPDataIn:         // Doing a sequence of IN packets (until data count <= EPSIZE)
   case EPStatusIn:       // Just done an IN packet as a status handshake
   case EPStatusOut:      // Done an OUT packet as a status handshake
   case EPThrottle:       // Not used
   case EPStall:          // Not used
      break;
   }
}

/**
 * ep2 - IN
 */
static void ep2HandleInToken( void ) {

   epHardwareState[BDM_IN_ENDPOINT].data0_1 = !epHardwareState[BDM_IN_ENDPOINT].data0_1;   // Toggle DATA0/1 for next packet
   //   PUTS(epHardwareState[BDM_OUT_ENDPOINT].data0_1?"ep2HandleInToken-T-1\n":"ep2HandleInToken-T-0\n");

   switch (epHardwareState[BDM_IN_ENDPOINT].state) {
   case EPDataIn:    // Doing a sequence of IN packets (until data count <= EPSIZE)
      if (ep2State.dataRemaining < BDM_IN_EP_MAXSIZE) {
         epHardwareState[BDM_IN_ENDPOINT].state = EPLastIn;    // Sending last packet (may be empty)
      }
      else {
         epHardwareState[BDM_IN_ENDPOINT].state = EPDataIn;    // Sending full packet
      }
      ep2InitialiseBdtTx(); // Set up next IN packet
      break;

   case EPLastIn:    // Just done the last IN packet
      if (commandBusyFlag) {
         ep2StartTxTransaction(sizeof(busyResponse), busyResponse);
      }
      else {
         epHardwareState[BDM_IN_ENDPOINT].state = EPIdle;  // No more transactions expected
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


/**
 * Handler for Token Complete USB interrupt
 *
 * Handles ep0 [SETUP, IN & OUT]
 * Handles ep1 [Out]
 * Handles ep2 [In]
 * Handles ep3 [In]
 * Handles ep4 [Out]
 * Handles ep5 [In]
 */
template<>
void UsbBase_T<Usb0Info>::handleTokenComplete(void) {
   uint8_t   usbStat  = usb->STAT;
   uint8_t   endPoint = ((uint8_t)usbStat)>>4;        // Endpoint number
   uint8_t   isTx     = usbStat&USB_STAT_TX_MASK;     // Direction of T/F 0=>OUT, (!=0)=>IN
   uint8_t   isOdd    = usbStat&USB_STAT_ODD_MASK;    // Odd/even buffer
   BdtEntry *bdt      = &bdts[usbStat>>2];

   if (endPoint == 0) {
      //      if (bdt->u.result.tok_pid == SETUPToken) {
      //         PUTS("\n=====");
      //      }
      //      PRINTF("\nTOKEN=%s, STATE=%s, size=%d, %s, %s, %s\n",
      //            getTokenName(bdt->u.result.tok_pid),
      //            getStateName(epHardwareState[CONTROL_ENDPOINT].state),
      //            bdt->bc,
      //            isTx?"Tx":"Rx",
      //                  bdt->u.result.data0_1?"DATA1":"DATA0",
      //                        isOdd?"Odd":"Even");
   }
   if (isTx) {
      // Flip Transmit buffer
      epHardwareState[endPoint].txOdd = !isOdd;
   }
   else {
      // Flip Receive buffer
      epHardwareState[endPoint].rxOdd = !isOdd;
   }
   switch (endPoint) {
      case CONTROL_ENDPOINT: // Control - Accept IN, OUT or SETUP token
         switch (bdt->u.result.tok_pid) {
         case SETUPToken:
            handleSetupToken();
            break;
         case INToken:
            ep0HandleInToken();
            break;
         case OUTToken:
            ep0HandleOutToken();
            break;
         default:
            PRINTF("Unexpected token on EP0 = %s\n", getTokenName(bdt->u.result.tok_pid));
            break;
         }
         return;
      case BDM_OUT_ENDPOINT: // USBDM BDM - Accept OUT token
         usbActivityFlag.flags.bdmActive = 1;
         ep1HandleOutToken();
         return;
      case BDM_IN_ENDPOINT: // USBDM BDM - Accept IN token
         usbActivityFlag.flags.bdmActive = 1;
         ep2HandleInToken();
         return;
#if (HW_CAPABILITY&CAP_CDC)
      case CDC_CONTROL_ENDPOINT: // USBDM CDC Control - Accept IN token
         //         PRINTF("EP3\n");
         epHardwareState[CDC_CONTROL_ENDPOINT].data0_1 = !epHardwareState[CDC_CONTROL_ENDPOINT].data0_1; // Toggle data0/1
         ep3StartTxTransaction();
         return;
      case CDC_DATA_OUT_ENDPOINT: // USBDM CDC Data - Accept OUT token
         //          usbActivityFlag.flags.serialOutActive = 1;
         //         PRINTF("EP4\n");
         if (cdc_txBufferIsFree()) {
            ep4SaveRxData();
            ep4InitialiseBdtRx();
         }
         else {
            //! Throttle endpoint - send NAKs
            epHardwareState[CDC_DATA_OUT_ENDPOINT].state = EPThrottle;
         }
         return;
      case CDC_DATA_IN_ENDPOINT:  // USBD CDC Data - Accept IN token
         epHardwareState[CDC_DATA_IN_ENDPOINT].state = EPIdle;
         //          ep5StartInTransactionIfIdle();
         return;
#endif
   }
}

/**
 * STALL completed - re-enable ep0 for SETUP
 */
template<>
void UsbBase_T<Usb0Info>::handleStallComplete( void ) {
   //   PUTS("StallComplete");
   epClearStall(0);
   // Re-initialise EP0 OUT
   //      ep0ConfigureSetupTransaction();
}

/**
 * Handler for Start of Frame Token interrupt (~1ms interval)
 */
template<>
void UsbBase_T<Usb0Info>::handleSOFToken( void ) {
   // Activity LED
   // Off                     - no USB activity, not connected
   // On                      - no USB activity, connected
   // Off, flash briefly on   - USB activity, not connected
   // On,  flash briefly off  - USB activity, connected
   if (usb->FRMNUML==0) { // Every ~256 ms
      switch (usb->FRMNUMH&0x03) {
      case 0:
         if (deviceState.state == USBconfigured) {
            // Activity LED on when USB connection established
            ActivityLed::on();
         }
         else {
            // Activity LED off when no USB connection
            ActivityLed::off();
         }
         break;
      case 1:
      case 2:
         break;
      case 3:
      default :
         if (usbActivityFlag.flags.bdmActive) {
            // Activity LED flashes on BDM activity
            ActivityLed::toggle();
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

#if (HW_CAPABILITY&CAP_CDC)
void checkUsbCdcTxData(void) {
   // Check if we need to unThrottle EP4
   if ((epHardwareState[CDC_DATA_OUT_ENDPOINT].state == EPThrottle) && cdc_txBufferIsFree()) {
      ep4SaveRxData();        // Save data from last transfer
      ep4InitialiseBdtRx();   // Set up next transfer
      epHardwareState[CDC_DATA_OUT_ENDPOINT].state = EPDataOut;
   }
}
#endif

/*
 * Handler for USB Suspend
 *   - Enables the USB module to wake-up the CPU
 *   - Stops the CPU
 * On wake-up
 *   - Re-checks the USB after a small delay to avoid wake-ups by noise
 *   - The RESUME interrupt is left pending so the resume handler can execute
 */
template<>
void UsbBase_T<Usb0Info>::handleUSBSuspend( void ) {
   //   PUTS("Suspend");
   if (deviceState.state != USBconfigured) {
      // Ignore if not configured
      return;
   }
   // Need to disable BDM interface & everything else to reduce power
   TargetVdd::off();

   // Clear the sleep and resume interrupt flags
   usb->ISTAT    = USB_ISTAT_SLEEP_MASK;

   // Asynchronous Resume Interrupt Enable (USB->CPU)
   // Only enable if transceiver is disabled
   //   usb->USBTRC0  |= USB_USBTRC0_USBRESMEN_MASK;

   // Enable resume detection or reset interrupts from the USB
   usb->INTEN   |= (USB_ISTAT_RESUME_MASK|USB_ISTAT_USBRST_MASK);
   deviceState.state = USBsuspended;
   ActivityLed::off();

   // A re-check loop is used here to ensure USB bus noise doesn't wake-up the CPU
   do {
      usb->ISTAT = USB_ISTAT_RESUME_MASK;       // Clear resume interrupt flag

      __WFI();  // Processor stop for low power

      // The CPU has woken up!

      if ((usb->ISTAT&(USB_ISTAT_RESUME_MASK|USB_ISTAT_USBRST_MASK)) == 0) {
         // Ignore if not resume or reset from USB module
         continue;
      }
      usb->ISTAT = USB_ISTAT_RESUME_MASK;  // Clear resume interrupt flag

      // Wait for a second resume (or existing reset) ~5 ms
      for(int delay=0; delay<24000; delay++) {
         // We should have another resume interrupt by end of loop
         if ((usb->ISTAT&(USB_ISTAT_RESUME_MASK|USB_ISTAT_USBRST_MASK)) != 0) {
            break;
         }
      }
   } while ((usb->ISTAT&(USB_ISTAT_RESUME_MASK|USB_ISTAT_USBRST_MASK)) == 0);

   // Disable resume interrupts
   usb->INTEN   &= ~USB_ISTAT_RESUME_MASK;
   return;
}

/**
 * Handler for USB Resume
 *
 * Disables further USB module wakeups
 */
template<>
void UsbBase_T<Usb0Info>::handleUSBResume( void ) {
   //   PUTS("Resume");
   // Mask further resume interrupts
   usb->INTEN   &= ~USB_INTEN_RESUMEEN_MASK;

   if (deviceState.state != USBsuspended) {
      // Ignore if not suspended
      return;
   }
   // Mask further resume interrupts
   //   usb->USBTRC0 &= ~USB_USBTRC0_USBRESMEN_MASK;

   // Clear the sleep and resume interrupt flags
   usb->ISTAT = USB_ISTAT_SLEEP_MASK|USB_ISTAT_RESUME_MASK;

   deviceState.state = USBconfigured;

   // Set up to receive setup packet
   ep0ConfigureSetupTransaction();        // re-initialise EP0 OUT // v4.7
   usb->CTL = USB_CTL_USBENSOFEN_MASK;   // Enable the transmit or receive of packets
   // power up BDM interface?
}

/**
 * Handler for USB interrupt
 *
 * Determines source and dispatches to appropriate routine.
 */
template<>
void UsbBase_T<Usb0Info>::irqHandler(void) {
   // All active flags
   uint8_t interruptFlags = usb->ISTAT;

   // Get active and enabled interrupt flags
   uint8_t enabledInterruptFlags = interruptFlags & usb->INTEN;

   if ((enabledInterruptFlags&USB_ISTAT_TOKDNE_MASK) != 0) {
      // Token complete interrupt?
      handleTokenComplete();
      usb->ISTAT = USB_ISTAT_TOKDNE_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_RESUME_MASK) != 0) {
      // Resume signaled on Bus?
      handleUSBResume();
      usb->ISTAT = USB_ISTAT_RESUME_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_USBRST_MASK) != 0) {
      // Reset signaled on Bus
      handleUSBReset();
      usb->ISTAT = USB_ISTAT_USBRST_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_STALL_MASK) != 0) {
      // Stall sent?
      handleStallComplete();
      usb->ISTAT = USB_ISTAT_STALL_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_SOFTOK_MASK) != 0) {
      // SOF Token?
      handleSOFToken();
      usb->ISTAT = USB_ISTAT_SOFTOK_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_SLEEP_MASK) != 0) {
      // Bus Idle 3ms? => sleep
      //      PUTS("Suspend");
      handleUSBSuspend();
      usb->ISTAT = USB_ISTAT_SLEEP_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_ERROR_MASK) != 0) {
      // Any Error
      PRINTF("Error s=0x%02X\n", usb->ERRSTAT);
      usb->ERRSTAT = 0xFF;
      usb->ISTAT   = USB_ISTAT_ERROR_MASK; // Clear source
   }
   else  {
      // Unexpected interrupt
      // Clear & ignore
      PRINTF("Unexpected interrupt, flags=0x%02X\n", interruptFlags);
      usb->ISTAT = interruptFlags; // Clear & ignore
   }
}

} // end namespace USBDM

