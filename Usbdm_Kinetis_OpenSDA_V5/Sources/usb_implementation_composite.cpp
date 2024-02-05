/**
 * @file     usb_implementation_composite.cpp
 * @brief    USB Kinetis implementation
 *
 * @version  V4.12.1.150
 * @date     13 Nov 2016
 *
 *  This file provides the implementation specific code for the USB interface.
 *  It will need to be modified to suit an application.
 */
#include <string.h>

#include "usb.h"
#include "usb_cdc_uart.h"
#include "interface.h"
#include "commands.h"

namespace USBDM {

/**
 * Interface numbers for USB descriptors
 */
enum InterfaceNumbers {
   /** Interface number for BDM channel */
   BULK_INTF_ID,

   /** Interface number for CDC Control channel */
   CDC_COMM_INTF_ID,

   /** Interface number for CDC Data channel */
   CDC_DATA_INTF_ID,
   /** Total number of interfaces */
   NUMBER_OF_INTERFACES,
};

/** Force command handler to exit and restart */
bool Usb0::forceCommandHandlerInitialise = false;

/** Set to discard Rx characters when garbage is expected e.g. when programming target */
bool Usb0::discardCharacters = false;

/*
 * String descriptors
 */
static const uint8_t s_language[]        = {4, DT_STRING, 0x09, 0x0C};  //!< Language IDs
static const uint8_t s_manufacturer[]    = MANUFACTURER;                //!< Manufacturer
static const uint8_t s_product[]         = PRODUCT_DESCRIPTION;         //!< Product Description
static const uint8_t s_serial[]          = SERIAL_NO;                   //!< Serial Number
static const uint8_t s_config[]          = "Default configuration";     //!< Configuration name

static const uint8_t s_bulk_interface[]  = "Bulk Interface";            //!< Bulk Interface

static const uint8_t s_cdc_interface[]   = "CDC Interface";             //!< Interface Association #2
static const uint8_t s_cdc_control[]     = "CDC Control Interface";     //!< CDC Control Interface
static const uint8_t s_cdc_data[]        = "CDC Data Interface";        //!< CDC Data Interface
/*
 * Add additional String descriptors here
 */

/**
 * String descriptor table
 */
const uint8_t *const Usb0::stringDescriptors[] = {
      s_language,
      s_manufacturer,
      s_product,
      s_serial,
      s_config,

      s_bulk_interface,

      s_cdc_interface,
      s_cdc_control,
      s_cdc_data
      /*
       * Add additional String descriptors here
       */
};

/**
 * Device Descriptor
 */
const DeviceDescriptor Usb0::deviceDescriptor = {
      /* bLength             */ (uint8_t) sizeof(DeviceDescriptor),
      /* bDescriptorType     */ (uint8_t) DT_DEVICE,
      /* bcdUSB              */ (uint8_t) nativeToLe16(0x0200),           // USB specification release No. [BCD = 2.00]
      /* bDeviceClass        */ (uint8_t) 0xEF,                           // Device Class code [Miscellaneous Device Class]
      /* bDeviceSubClass     */ (uint8_t) 0x02,                           // Sub Class code    [Common Class]
      /* bDeviceProtocol     */ (uint8_t) 0x01,                           // Protocol          [Interface Association Descriptor]
      /* bMaxPacketSize0     */ (uint8_t) CONTROL_EP_MAXSIZE,             // EndPt 0 max packet size
      /* idVendor            */ (uint16_t)nativeToLe16(VENDOR_ID),        // Vendor ID
      /* idProduct           */ (uint16_t)nativeToLe16(PRODUCT_ID),       // Product ID
      /* bcdDevice           */ (uint16_t)nativeToLe16(VERSION_ID),       // Device Release
      /* iManufacturer       */ (uint8_t) s_manufacturer_index,           // String index of Manufacturer name
      /* iProduct            */ (uint8_t) s_product_index,                // String index of product description
      /* iSerialNumber       */ (uint8_t) s_serial_index,                 // String index of serial number
      /* bNumConfigurations  */ (uint8_t) NUMBER_OF_CONFIGURATIONS        // Number of configurations
};

/**
 * All other descriptors
 */
const Usb0::Descriptors Usb0::otherDescriptors = {
      { // configDescriptor
            /* bLength                 */ (uint8_t) sizeof(ConfigurationDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_CONFIGURATION,
            /* wTotalLength            */ (uint16_t)nativeToLe16(sizeof(otherDescriptors)),
            /* bNumInterfaces          */ (uint8_t) NUMBER_OF_INTERFACES,
            /* bConfigurationValue     */ (uint8_t) CONFIGURATION_NUM,
            /* iConfiguration          */ (uint8_t) s_config_index,
            /* bmAttributes            */ (uint8_t) 0x80,     //  = Bus powered, no wake-up
            /* bMaxPower               */ (uint8_t) USBMilliamps(500)
      },
      /**
       * Bulk interface, 2 endpoints
       */
      { // bulk_interface
            /* bLength                 */ (uint8_t) sizeof(InterfaceDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_INTERFACE,
            /* bInterfaceNumber        */ (uint8_t) BULK_INTF_ID,
            /* bAlternateSetting       */ (uint8_t) 0,
            /* bNumEndpoints           */ (uint8_t) 2,
            /* bInterfaceClass         */ (uint8_t) 0xFF,                         // (Vendor specific)
            /* bInterfaceSubClass      */ (uint8_t) 0xFF,                         // (Vendor specific)
            /* bInterfaceProtocol      */ (uint8_t) 0xFF,                         // (Vendor specific)
            /* iInterface desc         */ (uint8_t) s_bulk_interface_index,
      },
      { // bulk_out_endpoint - OUT, Bulk
            /* bLength                 */ (uint8_t) sizeof(EndpointDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_ENDPOINT,
            /* bEndpointAddress        */ (uint8_t) EP_OUT|BULK_OUT_ENDPOINT,
            /* bmAttributes            */ (uint8_t) ATTR_BULK,
            /* wMaxPacketSize          */ (uint16_t)nativeToLe16(BULK_OUT_EP_MAXSIZE),
            /* bInterval               */ (uint8_t) USBMilliseconds(1)
      },
      { // bulk_in_endpoint - IN, Bulk
            /* bLength                 */ (uint8_t) sizeof(EndpointDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_ENDPOINT,
            /* bEndpointAddress        */ (uint8_t) EP_IN|BULK_IN_ENDPOINT,
            /* bmAttributes            */ (uint8_t) ATTR_BULK,
            /* wMaxPacketSize          */ (uint16_t)nativeToLe16(BULK_IN_EP_MAXSIZE),
            /* bInterval               */ (uint8_t) USBMilliseconds(1)
      },
      { // interfaceAssociationDescriptorCDC
            /* bLength                 */ (uint8_t) sizeof(InterfaceAssociationDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_INTERFACEASSOCIATION,
            /* bFirstInterface         */ (uint8_t) CDC_COMM_INTF_ID,
            /* bInterfaceCount         */ (uint8_t) 2,
            /* bFunctionClass          */ (uint8_t) 0x02,                      //  CDC Control
            /* bFunctionSubClass       */ (uint8_t) 0x02,                      //  Abstract Control Model
            /* bFunctionProtocol       */ (uint8_t) 0x01,                      //  AT CommandL V.250
            /* iFunction = ""          */ (uint8_t) s_cdc_interface_index,
      },
      /**
       * CDC Control/Communication Interface, 1 endpoint
       */
      { // cdc_CCI_Interface
            /* bLength                 */ (uint8_t) sizeof(InterfaceDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_INTERFACE,
            /* bInterfaceNumber        */ (uint8_t) CDC_COMM_INTF_ID,
            /* bAlternateSetting       */ (uint8_t) 0,
            /* bNumEndpoints           */ (uint8_t) 1,
            /* bInterfaceClass         */ (uint8_t) 0x02,      //  CDC Communication
            /* bInterfaceSubClass      */ (uint8_t) 0x02,      //  Abstract Control Model
            /* bInterfaceProtocol      */ (uint8_t) 0x01,      //  V.25ter, AT Command V.250
            /* iInterface description  */ (uint8_t) s_cdc_control_interface_index
      },
      { // cdc_Functional_Header
            /* bFunctionalLength       */ (uint8_t) sizeof(CDCHeaderFunctionalDescriptor),
            /* bDescriptorType         */ (uint8_t) CS_INTERFACE,
            /* bDescriptorSubtype      */ (uint8_t) DST_HEADER,
            /* bcdCDC                  */ (uint8_t) nativeToLe16(0x0110),
      },
      { // cdc_CallManagement
            /* bFunctionalLength       */ (uint8_t) sizeof(CDCCallManagementFunctionalDescriptor),
            /* bDescriptorType         */ (uint8_t) CS_INTERFACE,
            /* bDescriptorSubtype      */ (uint8_t) DST_CALL_MANAGEMENT,
            /* bmCapabilities          */ (uint8_t) 1,
            /* bDataInterface          */ (uint8_t) CDC_DATA_INTF_ID,
      },
      { // cdc_Functional_ACM
            /* bFunctionalLength       */ (uint8_t) sizeof(CDCAbstractControlManagementDescriptor),
            /* bDescriptorType         */ (uint8_t) CS_INTERFACE,
            /* bDescriptorSubtype      */ (uint8_t) DST_ABSTRACT_CONTROL_MANAGEMENT,
            /* bmCapabilities          */ (uint8_t) 0x06,
      },
      { // cdc_Functional_Union
            /* bFunctionalLength       */ (uint8_t) sizeof(CDCUnionFunctionalDescriptor),
            /* bDescriptorType         */ (uint8_t) CS_INTERFACE,
            /* bDescriptorSubtype      */ (uint8_t) DST_UNION_MANAGEMENT,
            /* bmControlInterface      */ (uint8_t) CDC_COMM_INTF_ID,
            /* bSubordinateInterface0  */ (uint8_t) {CDC_DATA_INTF_ID},
      },
      { // cdc_notification_Endpoint - IN,interrupt
            /* bLength                 */ (uint8_t) sizeof(EndpointDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_ENDPOINT,
            /* bEndpointAddress        */ (uint8_t) EP_IN|CDC_NOTIFICATION_ENDPOINT,
            /* bmAttributes            */ (uint8_t) ATTR_INTERRUPT,
            /* wMaxPacketSize          */ (uint16_t)nativeToLe16(CDC_NOTIFICATION_EP_MAXSIZE),
            /* bInterval               */ (uint8_t) USBMilliseconds(255)
      },
      /**
       * CDC Data Interface, 2 endpoints
       */
      { // cdc_DCI_Interface
            /* bLength                 */ (uint8_t) sizeof(InterfaceDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_INTERFACE,
            /* bInterfaceNumber        */ (uint8_t) CDC_DATA_INTF_ID,
            /* bAlternateSetting       */ (uint8_t) 0,
            /* bNumEndpoints           */ (uint8_t) 2,
            /* bInterfaceClass         */ (uint8_t) 0x0A,                         //  CDC DATA
            /* bInterfaceSubClass      */ (uint8_t) 0x00,                         //  -
            /* bInterfaceProtocol      */ (uint8_t) 0x00,                         //  -
            /* iInterface description  */ (uint8_t) s_cdc_data_Interface_index
      },
      { // cdc_dataOut_Endpoint - OUT, Bulk
            /* bLength                 */ (uint8_t) sizeof(EndpointDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_ENDPOINT,
            /* bEndpointAddress        */ (uint8_t) EP_OUT|CDC_DATA_OUT_ENDPOINT,
            /* bmAttributes            */ (uint8_t) ATTR_BULK,
            /* wMaxPacketSize          */ (uint16_t)nativeToLe16(CDC_DATA_OUT_EP_MAXSIZE),
            /* bInterval               */ (uint8_t) USBMilliseconds(1)
      },
      { // cdc_dataIn_Endpoint - IN, Bulk
            /* bLength                 */ (uint8_t) sizeof(EndpointDescriptor),
            /* bDescriptorType         */ (uint8_t) DT_ENDPOINT,
            /* bEndpointAddress        */ (uint8_t) EP_IN|CDC_DATA_IN_ENDPOINT,
            /* bmAttributes            */ (uint8_t) ATTR_BULK,
            /* wMaxPacketSize          */ (uint16_t)nativeToLe16(CDC_DATA_IN_EP_MAXSIZE),
            /* bInterval               */ (uint8_t) USBMilliseconds(1)
      },
      /*
       * TODO Add additional Descriptors here
       */
};

/** Out endpoint for BULK data out */
OutEndpoint <Usb0Info, Usb0::BULK_OUT_ENDPOINT,         BULK_OUT_EP_MAXSIZE>          Usb0::epBulkOut(EndPointType_Bulk);

/** In endpoint for BULK data in */
InEndpoint  <Usb0Info, Usb0::BULK_IN_ENDPOINT,          BULK_IN_EP_MAXSIZE>           Usb0::epBulkIn(EndPointType_Bulk);

/** In endpoint for CDC notifications */
InEndpoint  <Usb0Info, Usb0::CDC_NOTIFICATION_ENDPOINT, CDC_NOTIFICATION_EP_MAXSIZE>  Usb0::epCdcNotification(EndPointType_Bulk);

/** Out endpoint for CDC data out */
OutEndpoint <Usb0Info, Usb0::CDC_DATA_OUT_ENDPOINT,     CDC_DATA_OUT_EP_MAXSIZE>      Usb0::epCdcDataOut(EndPointType_Interrupt);

/** In endpoint for CDC data in */
InEndpoint  <Usb0Info, Usb0::CDC_DATA_IN_ENDPOINT,      CDC_DATA_IN_EP_MAXSIZE>       Usb0::epCdcDataIn(EndPointType_Interrupt);
/*
 * TODO Add additional endpoints here
 */

/**
 * Handler for Start of Frame Token interrupt (~1ms interval)
 *
 * @param frameNumber Frame number from SOF token
 *
 * @return  E_NO_ERROR on success
 */
ErrorCode Usb0::sofCallback(uint16_t frameNumber) {
   // Activity LED
   // Off                     - no USB activity, not connected
   // On                      - no USB activity, connected
   // Off, flash briefly on   - USB activity, not connected
   // On,  flash briefly off  - USB activity, connected
   if ((frameNumber&0xFF)==0) {
      // Every ~256 ms
      switch ((frameNumber>>8)&0x3) {
         case 0:
            // LED on if configured, off if not
            UsbLed::write(fConnectionState == USBconfigured);
            break;
         case 1:
         case 2:
            break;
         case 3:
         default :
            if (fActivityFlag) {
               // Flash LED to indicate activity
               UsbLed::toggle();
               fActivityFlag = false;
            }
            break;
      }
   }
   // Notify any changes in CDC status
   epCdcSendNotification();

   if ((epCdcDataOut.getState() == EPBusy) &&
       (Uart::getRemaingCapacity()>epCdcDataOut.BUFFER_SIZE)) {
      // Now there is sufficient space for another CDC out transfer
      // Set up for next transfer
      console.writeln("Restart");
      epCdcDataOut.startRxTransfer(EPDataOut, epCdcDataOut.BUFFER_SIZE);
   }

   return E_NO_ERROR;
}

ErrorCode userCallbackFunction(const Usb0::UserEvent event) {
   switch(event) {
      case Usb0::UserEvent_Suspend:
      case Usb0::UserEvent_Reset:
         UsbLed::off();
         break;

      case Usb0::UserEvent_Resume:
      case Usb0::UserEvent_Configure:
         break;
   }
   return E_NO_ERROR;
}

/**
 * Check for change in CDC status
 * Notify as needed (IN status transaction [Tx, device -> host, DATA0/1])
 */
void Usb0::epCdcSendNotification() {
//   if (fConnectionState != USBconfigured) {
//      // Only send notifications if configured.
//      return;
//   }
   static const struct CDCNotificationData {
      CDCNotification notification;
      uint8_t         data[2];
   } cdcNotification = {
      { CDC_NOTIFICATION, SERIAL_STATE, 0, RT_INTERFACE, nativeToLe16(2) },
      {0,0}};

   uint8_t status = Uart::getSerialState().bits;

   if ((status & Uart::CDC_STATE_CHANGE_MASK) == 0) {
      // No change
      return;
   }
   if (epCdcNotification.getState() != EPIdle) {
      // Busy with previous
      return;
   }
   static_assert(epCdcNotification.BUFFER_SIZE>=sizeof(cdcNotification), "Buffer size insufficient");

   // Copy the data to Tx buffer
   CDCNotificationData *buff = (CDCNotificationData*)epCdcNotification.getTxBuffer();
   Endpoint::safeCopy(buff, &cdcNotification, sizeof(cdcNotification));
   buff->data[0] = status&~Uart::CDC_STATE_CHANGE_MASK;
   buff->data[1] = 0;

   // Set up to Tx packet
//   console.write("epCdcSendNotification() 0x").writeln(epCdcNotification.getBuffer()[sizeof(cdcNotification)+0], USBDM::Radix_16);
   epCdcNotification.startTxTransfer(EPDataIn, sizeof(cdcNotification));
}

/**
 * Call-back handling CDC-OUT transaction complete\n
 * Data received is passed to the cdcInterface
 *
 * @param[in] state Current endpoint state (always EPDataOut)
 *
 * @return The endpoint state to set after call-back (EPDataOut)
 */
EndpointState Usb0::cdcOutTransactionCallback(EndpointState state) {
   //   console.WRITELN("cdc_out");
   (void)state;
   usbdm_assert(state == EPDataOut, "Incorrect endpoint state");

   volatile const uint8_t *buff = epCdcDataOut.getRxBuffer();
   unsigned size = epCdcDataOut.getDataTransferredSize();
   for (int i=size; i>0; i--) {
      if (!Uart::putChar(*buff++)) {
         // Discard further data from this transfer - should not happen!
         break;
      }
   }
   if (Uart::getRemaingCapacity()>=epCdcDataOut.BUFFER_SIZE) {
      // Sufficient space for another CDC out transfer
      // Set up for next transfer
      console.writeln("OK - ", size);
      epCdcDataOut.startRxTransfer(EPDataOut, epCdcDataOut.BUFFER_SIZE);
      return EPDataOut;
   }
   else {
      // Insufficient space for another CDC out transfer
      console.writeln("Busy - ", size);
      return EPBusy;
   }
}

/* Input queue HOST <- UART IN transfers */
static UartQueue<uint8_t, 2*CDC_DATA_IN_EP_MAXSIZE> inQueue;

/**
 * Call-back handling CDC-IN transaction complete\n
 * Checks for data and schedules transfer as necessary\n
 * Each transfer will have a ZLP as necessary.
 *
 * @param[in] state Current endpoint state (always EPDataIn)
 *
 * @return The endpoint state to set after call-back (EPIdle/EPDataIn)
 */
EndpointState Usb0::cdcInTransactionCallback(EndpointState state) {

   usbdm_assert(state == EPDataIn, "Incorrect endpoint state");
   (void)state;

   unsigned charCount     = 0;
   volatile uint8_t *buff = epCdcDataIn.getTxBuffer();

   // Copy characters from UART to endpoint buffer
   while(!inQueue.isEmpty()) {
      if (charCount>=epCdcDataIn.BUFFER_SIZE) {
         // Buffer full. Leave rest for next transfer.
         break;
      }
      *buff++ = inQueue.deQueue();
      charCount++;
   }
   if (charCount>0) {
      // Schedules transfer if data available
      epCdcDataIn.startTxTransfer(EPDataIn, charCount);
      return EPDataIn;
   }
   return EPIdle;
}

/**
 * Add character to CDC OUT buffer.
 *
 * @param[in] ch Character to send
 *
 * @return true  Character accepted or discarded (see discardCharacters)
 * @return false Overrun, character not accepted
 */
bool Usb0::putCdcChar(uint8_t ch) {

   // Assume character accepted
   bool rc = true;
   do {
      if (discardCharacters) {
         continue;
      }
      if (inQueue.isFull()) {
         // Overrun
         rc = false;
         continue;
      }
      inQueue.enQueue(ch);
   } while (false);
   // A char has been queued (or buffer already had chars)

   // Always check if configured
   CriticalSection cr;
   if (epCdcDataIn.getState() == EPIdle) {
      // Restart IN transactions
      cdcInTransactionCallback(EPDataIn);
   }
   return rc;
}

//_______ Bulk Call-backs ________________________________________________________________

/**
 * Call-back handling BULK-OUT transaction complete
 *
 * @param[in] state Current endpoint state
 *
 * @return The endpoint state to set after call-back (EPIdle)
 */
EndpointState Usb0::bulkOutTransactionCallback(EndpointState state) {
   (void)state;
   // No actions - End-point is polled
   return EPIdle;
}

/**
 * Call-back handling BULK-IN transaction complete
 *
 * @param[in] state Current endpoint state
 *
 * @return The endpoint state to set after call-back (EPIdle)
 */
EndpointState Usb0::bulkInTransactionCallback(EndpointState state) {
   (void)state;
   // No actions - End-point is polled
   return EPIdle;
}

/**
 * Initialise the USB0 interface
 *
 *  @note Assumes clock set up for USB operation (48MHz)
 */
void Usb0::initialise() {

   forceCommandHandlerInitialise = false;

   // Add extra handling of CDC requests directed to EP0
   setUnhandledSetupCallback(handleUserEp0SetupRequests);

   // Set SOF handler
   setSOFCallback(sofCallback);

   setUserCallback(userCallbackFunction);

   Uart::setInCallback(putCdcChar);
   Uart::initialise();

   UsbBase_T::initialise();
}

//_______ Bulk Transmission ________________________________________________________________

/**
 *  Blocking reception of data over bulk OUT endpoint
 *
 *   @param[in] maxSize  Maximum # of bytes to receive
 *   @param[in] buffer   Pointer to buffer for bytes received
 *
 *   @return Number of bytes received
 *
 *   @note Doesn't return until command has been received.
 */
int Usb0::receiveBulkData(uint8_t maxSize, uint8_t *buffer) {
   epBulkOut.startRxTransfer(EPDataOut, maxSize, buffer);
   while(epBulkOut.getState() != EPIdle) {
      __enable_irq();
      Smc::enterWaitMode();
   }
   setActive();
   return epBulkOut.getDataTransferredSize();
}

/**
 *  Blocking transmission of data over bulk IN endpoint
 *
 *  @param[in] size   Number of bytes to send
 *  @param[in] buffer Pointer to bytes to send
 *
 *   @note : Waits for idle BEFORE transmission but\n
 *   returns before data has been transmitted
 *
 */
void Usb0::sendBulkData(uint8_t size, const uint8_t *buffer) {
//   commandBusyFlag = false;
   //   enableUSBIrq();
   while (epBulkIn.getState() != EPIdle) {
      Smc::enterWaitMode();
   }
   epBulkIn.startTxTransfer(EPDataIn, size, buffer);
}

/**
 * CDC Set line coding handler
 */
void Usb0::handleSetLineCoding() {
//   console.WRITELN("handleSetLineCoding()");

   // Call-back to do after transaction complete
   static auto callback = [](EndpointState) {
      // The controlEndpoint buffer will contain the LineCodingStructure data at call-back time
      Uart::setLineCoding((LineCodingStructure *)fControlEndpoint.getRxBuffer());
      fControlEndpoint.setCallback(nullptr);
      return EPIdle;
   };
   fControlEndpoint.setCallback(callback);

   // Don't use external buffer - this requires response to fit in internal EP buffer
   static_assert(sizeof(LineCodingStructure) < fControlEndpoint.BUFFER_SIZE, "Buffer insufficient size");
   fControlEndpoint.startRxTransfer(EPDataOut, sizeof(LineCodingStructure));
}

/**
 * CDC Get line coding handler
 */
void Usb0::handleGetLineCoding() {
//   console.WRITELN("handleGetLineCoding()");
   // Send packet
   ep0StartTxStage( sizeof(LineCodingStructure), (const uint8_t*)Uart::getLineCoding());
}

/**
 * CDC Set line state handler
 */
void Usb0::handleSetControlLineState() {
//   console.write("handleSetControlLineState() ").writeln(fEp0SetupBuffer.wValue.lo(), USBDM::Radix_16);
   Uart::setControlLineState(fEp0SetupBuffer.wValue.lo());
   // Tx empty Status packet
   ep0StartTxStage( 0, nullptr );
}

/**
 * CDC Send break handler
 */
void Usb0::handleSendBreak() {
//   console.WRITELN("handleSendBreak()");
   Uart::sendBreak(fEp0SetupBuffer.wValue);
   // Tx empty Status packet
   ep0StartTxStage( 0, nullptr );
}

/**
 * Handle SETUP requests not handled by base handler
 *
 * @param[in] setup SETUP packet received from host
 *
 * @note Provides CDC extensions
 * @note Provides BDM extensions
 */
ErrorCode Usb0::handleUserEp0SetupRequests(const SetupPacket &setup) {
   //console.WRITE("handleUserEp0SetupRequests(").WRITE(setup.bRequest).WRITELN(")");
   switch(REQ_TYPE(setup.bmRequestType)) {
      case UsbRequestType_CLASS :
         // Class requests
         switch (setup.bRequest) {
            case SET_LINE_CODING :       handleSetLineCoding();       break;
            case GET_LINE_CODING :       handleGetLineCoding();       break;
            case SET_CONTROL_LINE_STATE: handleSetControlLineState(); break;
            case SEND_BREAK:             handleSendBreak();           break;
            default :                    fControlEndpoint.stall();    break;
         }
         break;
      case UsbRequestType_VENDOR :
//         console.WRITELN("REQ_TYPE_VENDOR");
         switch (setup.bRequest) {
            case ICP_GET_VER : {
               // Tell command handler to re-initialise
               forceCommandHandlerInitialise = true;

               // Version response
               static const uint8_t versionResponse[5] = {
                     BDM_RC_OK,
                     VERSION_SW,      // BDM SW version
                     VERSION_HW,      // BDM HW version
                     0,               // ICP_Version_SW;
                     VERSION_HW,      // ICP_Version_HW;
               };
               ep0StartTxStage( sizeof(versionResponse),  versionResponse );
               }
               break;
            default :
               fControlEndpoint.stall();
               break;
         }
         break;
      default:
         fControlEndpoint.stall();
         break;
   }
   return E_NO_ERROR;
}

} // End namespace USBDM

