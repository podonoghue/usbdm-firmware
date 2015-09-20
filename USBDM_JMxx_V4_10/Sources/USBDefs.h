#ifndef _USBDefs_H_
#define _USBDefs_H_

#include "Common.h"

//! Device Descriptor 
typedef struct {
   U8           bLength;             //!<  Size of this Descriptor in Bytes
   U8           bDescriptorType;     //!<  Descriptor Type (=1)
   U16          bcdUSB;              //!<  USB Spec Release Number in BCD
   U8           bDeviceClass;        //!<  Device Class Code
   U8           bDeviceSubClass;     //!<  Device Subclass Code      
   U8           bDeviceProtocol;     //!<  Device Protocol Code
   U8           bMaxPacketSize0;     //!<  Maximum Packet Size for EP0
   U16          idVendor;            //!<  Vendor ID 
   U16          idProduct;           //!<  Product ID
   U16          bcdDevice;           //!<  Device Release Number in BCD
   U8           iManufacturer;       //!<  Index of String Desc for Manufacturer
   U8           iProduct;            //!<  Index of String Desc for Product
   U8           iSerialNumber;       //!<  Index of String Desc for SerNo
   U8           bNumConfigurations;  //!<  Number of possible Configurations
} DeviceDescriptor;                

//! USB Configuration Descriptor
typedef struct {
   U8           bLength;             //!<  Size of this Descriptor in Bytes
   U8           bDescriptorType;     //!<  Descriptor Type (=2)
   U16          wTotalLength;        //!<  Total Length of Data for this Configuration 
   U8           bNumInterfaces;      //!<  No of Interfaces supported by this Configuration 
   U8           bConfigurationValue; //!<  Designator Value for this Configuration 
   U8           iConfiguration;      //!<  Index of String Desc for this Configuration 
   U8           bmAttributes;        //!<  Configuration Characteristics 
   U8           bMaxPower;           //!<  Max. Power Consumption in this Configuration (in 2mA steps)
} ConfigurationDescriptor;

//! USB Interface Descriptor
typedef struct {
   U8           bLength;             //!<  Size of this Descriptor in Bytes
   U8           bDescriptorType;     //!<  Descriptor Type (=4)            
   U8           bInterfaceNumber;    //!<  Number of this Interface (0..)  
   U8           bAlternateSetting;   //!<  Alternative for this Interface (if any)
   U8           bNumEndpoints;       //!<  No of EPs used by this IF (excl. EP0)
   U8           bInterfaceClass;     //!<  Interface Class Code
   U8           bInterfaceSubClass;  //!<  Interface Subclass Code
   U8           bInterfaceProtocol;  //!<  Interface Protocol Code
   U8           iInterface;          //!<  Index of String Desc for this Interface
} InterfaceDescriptor;

//! Endpoint Descriptor
typedef struct {
   U8           bLength;             //!<  Size of this Descriptor in Bytes
   U8           bDescriptorType;     //!<  Descriptor Type (=5)
   U8           bEndpointAddress;    //!<  Endpoint Address (Number + Direction)
   U8           bmAttributes;        //!<  Endpoint Attributes (Transfer Type)
   U16          wMaxPacketSize;      //!<  Max. Endpoint Packet Size
   U8           bInterval;           //!<  Polling Interval (Interrupt) in ms
} EndpointDescriptor;

//! Structure of Setup Packet sent during SETUP Stage of Standard Device Requests
//! @note Shuffled fields for MCFJM128 
typedef struct {
   U8           bmRequestType;       //!<  Characteristics (Direction,Type,Recipient)
   U8           bRequest;            //!<  Standard Request Code
   U16u         wValue;              //!<  Value Field
   U16u         wIndex;              //!<  Index or Offset Field
   U16u         wLength;             //!<  Number of Bytes to transfer (Data Stage)
} SetupPacket;

//! Structure of Device Qualifier Descriptor
typedef struct {
   U8           bLength;             //!<  Size of this Descriptor in Bytes
   U8           bDescriptorType;     //!<  Descriptor Type (=6)
   U16          bcdUSB;              //!<  USB Spec Release Number in BCD
   U8           bDeviceClass;        //!<  Device Class Code
   U8           bDeviceSubClass;     //!<  Device Subclass Code      
   U8           bDeviceProtocol;     //!<  Device Protocol Code
   U8           bMaxPacketSize0;     //!<  Maximum Packet Size for EP0
   U8           bNumConfigurations;  //!<  Number of possible Configurations
   U8           bReserved;           //!<  Reserved
} DeviceQualifierDescriptor;

//! Endpoint direction masks
enum {EP_OUT=0x00, //!< Endpoint is OUT (host -> node)
	  EP_IN=0x80   //!< Endpoint is IN (node -> host)
	  };

/*----------------------------------------------------------------------------
** USB Status Codes
*/
#define US_ATTACHED             0x00
#define US_POWERED              0x01
#define US_DEFAULT              0x02
#define US_ADDRESSED            0x03
#define US_CONFIGURED           0x04
#define US_SUSPENDED            0x80

/*----------------------------------------------------------------------------
** USB Request Type (bmRequestType)
*/
#define REQ_TYPE(x)  	      ((3<<5) & (x))
#define REQ_TYPE_STANDARD      (0<<5)
#define REQ_TYPE_CLASS         (1<<5)
#define REQ_TYPE_VENDOR        (2<<5)
#define REQ_TYPE_OTHER         (3<<5)
#define IS_VENDOR_REQ(x)       (REQ_TYPE(x) == REQ_TYPE_VENDOR)

/*----------------------------------------------------------------------------
** USB Request Type (bmRequestType)
*/
#define REQ_RECIPIENT(x)  	      ((0x1F<<0) & (x))
#define REQ_RECIPIENT_DEVICE      (0<<0)
#define REQ_RECIPIENT_INTERFACE   (1<<0)
#define REQ_RECIPIENT_ENDPOINT    (2<<0)
#define REQ_RECIPIENT_OTHER       (3<<0)

/*----------------------------------------------------------------------------
** USB Standard Device Request Codes (bRequest)
*/
#define GET_STATUS              0x00
#define CLEAR_FEATURE           0x01
#define SET_FEATURE             0x03
#define SET_ADDRESS             0x05
#define GET_DESCRIPTOR          0x06
#define SET_DESCRIPTOR          0x07
#define GET_CONFIGURATION       0x08
#define SET_CONFIGURATION       0x09
#define GET_INTERFACE           0x0a
#define SET_INTERFACE           0x0b
#define SYNCH_FRAME             0x0c

/*----------------------------------------------------------------------------
** Descriptor Types
*/
#define DT_DEVICE                     1
#define DT_CONFIGURATION              2
#define DT_STRING                     3
#define DT_INTERFACE                  4
#define DT_ENDPOINT                   5
#define DT_DEVICEQUALIFIER            6
#define DT_OTHERSPEEDCONFIGURATION    7
#define DT_INTERFACEPOWER             8
#define DT_INTERFACEASSOCIATION     0xB

/*----------------------------------------------------------------------------
** USB Tokens
*/
#define SOFToken     (0x5) //!< Start of Frame token
#define SETUPToken   (0xD) //!< Setup token
#define OUTToken     (0x1) //!< Out token
#define INToken      (0x9) //!< In token
#define DATA0Token   (0x3) //!< Data 0
#define DATA1Token   (0xB) //!< Data 1
#define DATA2Token   (0x7) //!< Data 2
#define MDATAToken   (0xF) //!< M data
#define ACKToken     (0x2) //!< Acknowledge
#define NAKToken     (0xA) //!< Negative Acknowledge
#define NYETToken    (0x6) //!< No Response Yet
#define STALLToken   (0xE) //!< Stall
#define PREToken     (0xC) //!< Preamble

/*----------------------------------------------------------------------------
** Feature selector values (for Clear/Set feature)
*/
#define ENDPOINT_HALT         (0x00)
#define DEVICE_REMOTE_WAKEUP  (0x01)
#define TEST_MODE             (0x02)

/*----------------------------------------------------------------------------
** bmRequest types
*/
#define RT_DEVICE          (0x00)
#define RT_INTERFACE       (0x01)
#define RT_ENDPOINT        (0x02)

/*----------------------------------------------------------------------------
** bmAttributes types
*/
#define ATTR_CONTROL		(0x00)
#define ATTR_ISOCHRONOUS 	(0x01)
#define ATTR_BULK  			(0x02)
#define ATTR_INTERRUPT  	(0x03)

#define USBMilliamps(x)     ((x)/2)
#define USBMilliseconds(x)  (x)

//============================================================================
// IAD Stuff (Composite devices)
//============================================================================
typedef struct {
   U8 bLength;                //!< Size of this Descriptor in Bytes
   U8 bDescriptorType;        //!< Descriptor Type (=0B)
   U8 bFirstInterface;        //!< First interface #
   U8 bInterfaceCount;		   //!< Number of interfaces
	U8 bFunctionClass;         //!< bInterfaceClass;
	U8 bFunctionSubClass;      //!< bInterfaceSubClass;
	U8 bFunctionProtocol;      //!< Protocol
	U8 iFunction;     		   //!< Function
} InterfaceAssociationDescriptor;

//============================================================================
// CDC Stuff
//============================================================================

// bDescriptorType
#define CS_INTERFACE  (0x24)
#define CS_ENDPOINT   (0x25)

// 
#define ST_HEADER     (0x00)
#define ST_INTERFACE  (0x01)

// bDescriptorSubtype
#define DST_HEADER                      (0x00)
#define DST_CALL_MANAGEMENT             (0x01)
#define DST_ABSTRACT_CONTROL_MANAGEMENT (0x02)
#define DST_UNION_MANAGEMENT            (0x06)

//! USB CDC Header Functional Descriptor
typedef struct {
	U8  bFunctionLength;		//!< Size
	U8  bDescriptorType;		//!< Type
	U8  bDescriptorSubtype;		//!< Sub-type
	U16 bcdCDC;					//!< ??
} CDCHeaderFunctionalDescriptor;

//! USB CDC Call Management Functional Descriptor
typedef struct {
	U8  bFunctionLength;		//!< Length
	U8  bDescriptorType;		//!< Type
	U8  bDescriptorSubtype;		//!< Sub-type
    U8  bmCapabilities;			//!< Capabilities
    U8  bDataInterface;			//!< Data interface
} CDCCallManagementFunctionalDescriptor;

//! USB CDC Abstract Control Management Descriptor
typedef struct {
	U8  bFunctionLength;		//!< Length
	U8  bDescriptorType;		//!< Type
	U8  bDescriptorSubtype;		//!< Sub-type
	U8  bmCapabilities;			//!< Capabilities
} CDCAbstractControlManagementDescriptor;

//! USB CDC Union Functional Descriptor
typedef struct {
	U8  bFunctionLength;		//!< Length
	U8  bDescriptorType;		//!< Type
	U8  bDescriptorSubtype;		//!< Sub-type
	U8  bMasterInterface;		//!< Interface
	U8  bSlaveInterface[1];		//!< Slave interface
} CDCUnionFunctionalDescriptor;

/*----------------------------------------------------------------------------
** CDC Class requests
*/
#define SEND_ENCAPSULATED_COMMAND  (0x00)
#define GET_ENCAPSULATED_COMMAND   (0x01)
#define SET_LINE_CODING            (0x20)
#define GET_LINE_CODING            (0x21)
#define SET_CONTROL_LINE_STATE     (0x22)
#define SEND_BREAK                 (0x23)

//! USB CDC Notification
typedef struct {
	U8  bmRequestType;	//!< Request type
	U8  bNotification;	//!< Notification
	U16 wValue;			//!< Value
	U16 wIndex;			//!< Index
	U16 wLength;		//!< Length
} CDCNotification;

#define CDC_NOTIFICATION   (0xA1)
#define RESPONSE_AVAILABLE (0x01)
#define SERIAL_STATE       (0x20)

// Bit masks for SetControlLineState value
#define DTR_MASK (1<<0)
#define RTS_MASK (1<<1)

typedef struct {
   U32 lLength;                //!< Size of this Descriptor in Bytes
   U16 wVersion;               //!< Version
   U16 wIndex;                 //!< Index (must be 4)
   U8  bnumSections;           //!< Number of sections
   U8  bReserved1[7];	        //!< 
   //------------- Section ----------//
	U8  bInterfaceNum;           //!< 
	U8  bReserved2;              //!<
	U8  bCompatibleId[8];        //!<
	U8  bSubCompatibleId[8];	 //!<
	U8  bReserved3[6];
	
} MS_CompatibleIdFeatureDescriptor;

typedef struct {
   U32 lLength;                //!< Size of this Descriptor in Bytes
   U16 wVersion;               //!< Version
   U16 wIndex;                 //!< Index (must be 5)
   U16 bnumSections;           //!< Number of property sections
   //-------------------- Section --------------//
   U32 lPropertySize;          //!< Size of property section
   U32 ldataType;              //!< Data type (1 = Unicode REG_SZ etc
   U16 wNameLength;            //!< Length of property name
   U8  bName[40];
   U32 wPropertyLength;        //!< Length of property data
   U8  bData[78];
} MS_PropertiesFeatureDescriptor;

#endif /* _USBDefs_H_  */
