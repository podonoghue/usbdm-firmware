#ifndef _USBDefs_H_
#define _USBDefs_H_

#pragma pack(push)
#pragma pack(1)
//! Device Descriptor 
typedef struct {
   uint8_t           bLength;             //!<  Size of this Descriptor in Bytes
   uint8_t           bDescriptorType;     //!<  Descriptor Type (=1)
   uint16_t          bcdUSB;              //!<  USB Spec Release Number in BCD
   uint8_t           bDeviceClass;        //!<  Device Class Code
   uint8_t           bDeviceSubClass;     //!<  Device Subclass Code      
   uint8_t           bDeviceProtocol;     //!<  Device Protocol Code
   uint8_t           bMaxPacketSize0;     //!<  Maximum Packet Size for EP0
   uint16_t          idVendor;            //!<  Vendor ID 
   uint16_t          idProduct;           //!<  Product ID
   uint16_t          bcdDevice;           //!<  Device Release Number in BCD
   uint8_t           iManufacturer;       //!<  Index of String Desc for Manufacturer
   uint8_t           iProduct;            //!<  Index of String Desc for Product
   uint8_t           iSerialNumber;       //!<  Index of String Desc for SerNo
   uint8_t           bNumConfigurations;  //!<  Number of possible Configurations
} DeviceDescriptor;                

//! USB Configuration Descriptor
typedef struct {
   uint8_t           bLength;             //!<  Size of this Descriptor in Bytes
   uint8_t           bDescriptorType;     //!<  Descriptor Type (=2)
   uint16_t          wTotalLength;        //!<  Total Length of Data for this Configuration 
   uint8_t           bNumInterfaces;      //!<  No of Interfaces supported by this Configuration 
   uint8_t           bConfigurationValue; //!<  Designator Value for this Configuration 
   uint8_t           iConfiguration;      //!<  Index of String Desc for this Configuration 
   uint8_t           bmAttributes;        //!<  Configuration Characteristics 
   uint8_t           bMaxPower;           //!<  Max. Power Consumption in this Configuration (in 2mA steps)
} ConfigurationDescriptor;

//! USB Interface Descriptor
typedef struct {
   uint8_t           bLength;             //!<  Size of this Descriptor in Bytes
   uint8_t           bDescriptorType;     //!<  Descriptor Type (=4)            
   uint8_t           bInterfaceNumber;    //!<  Number of this Interface (0..)  
   uint8_t           bAlternateSetting;   //!<  Alternative for this Interface (if any)
   uint8_t           bNumEndpoints;       //!<  No of EPs used by this IF (excl. EP0)
   uint8_t           bInterfaceClass;     //!<  Interface Class Code
   uint8_t           bInterfaceSubClass;  //!<  Interface Subclass Code
   uint8_t           bInterfaceProtocol;  //!<  Interface Protocol Code
   uint8_t           iInterface;          //!<  Index of String Desc for this Interface
} InterfaceDescriptor;

//! Endpoint Descriptor
typedef struct {
   uint8_t           bLength;             //!<  Size of this Descriptor in Bytes
   uint8_t           bDescriptorType;     //!<  Descriptor Type (=5)
   uint8_t           bEndpointAddress;    //!<  Endpoint Address (Number + Direction)
   uint8_t           bmAttributes;        //!<  Endpoint Attributes (Transfer Type)
   uint16_t          wMaxPacketSize;      //!<  Max. Endpoint Packet Size
   uint8_t           bInterval;           //!<  Polling Interval (Interrupt) in ms
} EndpointDescriptor;

//! Structure of Setup Packet sent during SETUP Stage of Standard Device Requests
//! @note Shuffled fields for MCFJM128 
typedef struct {
   uint8_t      bmRequestType;       //!<  Characteristics (Direction,Type,Recipient)
   uint8_t      bRequest;            //!<  Standard Request Code
   U16u         wValue;              //!<  Value Field
   U16u         wIndex;              //!<  Index or Offset Field
   U16u         wLength;             //!<  Number of Bytes to transfer (Data Stage)
} SetupPacket;
//
//typedef struct {
//   uint16_t     wValue;              //!<  Value Field
//   uint8_t      bRequest;            //!<  Standard Request Code
//   uint8_t      bmRequestType;       //!<  Characteristics (Direction,Type,Recipient)
//   uint16_t     wLength;             //!<  Number of Bytes to transfer (Data Stage)
//   U16u         wIndex;              //!<  Index or Offset Field
//} SetupPacket;
//
//! Structure of Device Qualifier Descriptor
typedef struct {
   uint8_t           bLength;             //!<  Size of this Descriptor in Bytes
   uint8_t           bDescriptorType;     //!<  Descriptor Type (=6)
   uint16_t          bcdUSB;              //!<  USB Spec Release Number in BCD
   uint8_t           bDeviceClass;        //!<  Device Class Code
   uint8_t           bDeviceSubClass;     //!<  Device Subclass Code      
   uint8_t           bDeviceProtocol;     //!<  Device Protocol Code
   uint8_t           bMaxPacketSize0;     //!<  Maximum Packet Size for EP0
   uint8_t           bNumConfigurations;  //!<  Number of possible Configurations
   uint8_t           bReserved;           //!<  Reserved
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
#define ATTR_CONTROL	      (0x00)
#define ATTR_ISOCHRONOUS 	(0x01)
#define ATTR_BULK  			(0x02)
#define ATTR_INTERRUPT  	(0x03)

#define USBMilliamps(x)     ((x)/2)
#define USBMilliseconds(x)  (x)

//============================================================================
// IAD Stuff (Composite devices)
//============================================================================
typedef struct {
   uint8_t bLength;                //!< Size of this Descriptor in Bytes
   uint8_t bDescriptorType;        //!< Descriptor Type (=0B)
   uint8_t bFirstInterface;        //!< First interface #
   uint8_t bInterfaceCount;		  //!< Number of interfaces
	uint8_t bFunctionClass;         //!< bInterfaceClass;
	uint8_t bFunctionSubClass;      //!< bInterfaceSubClass;
	uint8_t bFunctionProtocol;      //!< Protocol
	uint8_t iFunction;			     //!< Function
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
	uint8_t  bFunctionLength;		//!< Size
	uint8_t  bDescriptorType;		//!< Type
	uint8_t  bDescriptorSubtype;	//!< Sub-type
	uint16_t bcdCDC;					//!< ??
} CDCHeaderFunctionalDescriptor;

//! USB CDC Call Management Functional Descriptor
typedef struct {
	uint8_t  bFunctionLength;		//!< Length
	uint8_t  bDescriptorType;		//!< Type
	uint8_t  bDescriptorSubtype;	//!< Sub-type
   uint8_t  bmCapabilities;		//!< Capabilities
   uint8_t  bDataInterface;		//!< Data interface
} CDCCallManagementFunctionalDescriptor;

//! USB CDC Abstract Control Management Descriptor
typedef struct {
	uint8_t  bFunctionLength;		//!< Length
	uint8_t  bDescriptorType;		//!< Type
	uint8_t  bDescriptorSubtype;	//!< Sub-type
	uint8_t  bmCapabilities;		//!< Capabilities
} CDCAbstractControlManagementDescriptor;

//! USB CDC Union Functional Descriptor
typedef struct {
	uint8_t  bFunctionLength;     //!< Length
	uint8_t  bDescriptorType;	   //!< Type
	uint8_t  bDescriptorSubtype;  //!< Sub-type
	uint8_t  bMasterInterface;		//!< Interface
	uint8_t  bSlaveInterface[1];	//!< Slave interface
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
	uint8_t  bmRequestType;	//!< Request type
	uint8_t  bNotification;	//!< Notification
	uint16_t wValue;			//!< Value
	uint16_t wIndex;			//!< Index
	uint16_t wLength;		   //!< Length
} CDCNotification;

#define CDC_NOTIFICATION   (0xA1)
#define RESPONSE_AVAILABLE (0x01)
#define SERIAL_STATE       (0x20)

// Bit masks for SetControlLineState value
#define DTR_MASK (1<<0)
#define RTS_MASK (1<<1)

typedef struct {
   uint32_t lLength;                 //!< Size of this Descriptor in Bytes
   uint16_t wVersion;                //!< Version
   uint16_t wIndex;                  //!< Index (must be 4)
   uint8_t  bnumSections;            //!< Number of sections
   uint8_t  bReserved1[7];	          //!< 
   //------------- Section ----------//
	uint8_t  bInterfaceNum;           //!< 
	uint8_t  bReserved2;              //!<
	uint8_t  bCompatibleId[8];        //!<
	uint8_t  bSubCompatibleId[8];     //!<
	uint8_t  bReserved3[6];
	
} MS_CompatibleIdFeatureDescriptor;

typedef struct {
   uint32_t lLength;                //!< Size of this Descriptor in Bytes
   uint16_t wVersion;               //!< Version
   uint16_t wIndex;                 //!< Index (must be 5)
   uint16_t bnumSections;           //!< Number of property sections
   //-------------------- Section --------------//
   uint32_t lPropertySize;          //!< Size of property section
   uint32_t ldataType;              //!< Data type (1 = Unicode REG_SZ etc
   uint16_t wNameLength;            //!< Length of property name
   uint8_t  bName[40];
   uint32_t wPropertyLength;        //!< Length of property data
   uint8_t  bData[78];
} MS_PropertiesFeatureDescriptor;

#pragma pack(pop)
#endif /* _USBDefs_H_  */
