/**
 * @file     usb_implementation_composite.h
 * @brief    USB Composite device implementation
 *
 * @version  V4.12.1.170
 * @date     2 April 2017
 *
 *  This file provides the implementation specific code for the USB interface.
 *  It will need to be modified to suit an application.
 */
#ifndef PROJECT_HEADERS_USB_IMPLEMENTATION_COMPOSITE_H_
#define PROJECT_HEADERS_USB_IMPLEMENTATION_COMPOSITE_H_

#include <stdint.h>

/*
 * Under Windows 8, or 10 there is no need to install a driver for
 * the bulk end-points if the MS_COMPATIBLE_ID_FEATURE is enabled.
 * winusb.sys driver will be automatically loaded.
 *
 * Under Windows 10 the usbser.sys driver will be loaded automatically
 * for the CDC (serial) interface
 *
 * Under Linux drivers for bulk and CDC are automatically loaded
 */

#define MS_COMPATIBLE_ID_FEATURE
#include "usb_cdc_uart.h"

/** Causes a semi-unique serial number to be generated for each USB device */
// Done on command line where needed
//#define UNIQUE_ID 1

#include "configure.h"

#include "queue.h"

namespace USBDM {

//======================================================================
// Customise for each USB device
//

#ifndef SERIAL_NO
#ifdef UNIQUE_ID
#define SERIAL_NO           "USBDM-%lu"
#else
#define SERIAL_NO           "USBDM-0001"
#endif
#endif
#ifndef PRODUCT_DESCRIPTION
#define PRODUCT_DESCRIPTION "USB ARM"
#endif
#ifndef MANUFACTURER
#define MANUFACTURER        "pgo"
#endif

#ifndef VENDOR_ID
#define VENDOR_ID             (0x16D0)    // Vendor (actually MCS)
#endif
#ifndef PRODUCT_ID
#define PRODUCT_ID            (0xFFFF)    // Product ID
#endif
#ifndef VERSION_ID
#define VERSION_ID            (0x0200)
#endif

//======================================================================
// Maximum packet sizes for each endpoint
//
static constexpr unsigned  CONTROL_EP_MAXSIZE           = 64; //!< Control in/out
/*
 *  TODO Define additional end-point sizes
 */
static constexpr unsigned  BULK_OUT_EP_MAXSIZE          = 64; //!< Bulk out
static constexpr unsigned  BULK_IN_EP_MAXSIZE           = 64; //!< Bulk in

static constexpr unsigned  CDC_NOTIFICATION_EP_MAXSIZE  = 16; //!< CDC notification
static constexpr unsigned  CDC_DATA_OUT_EP_MAXSIZE      = 16; //!< CDC data out
static constexpr unsigned  CDC_DATA_IN_EP_MAXSIZE       = 16; //!< CDC data in

#ifdef USBDM_USB0_IS_DEFINED
/**
 * Class representing USB0
 */
class Usb0 : public UsbBase_T<Usb0Info, CONTROL_EP_MAXSIZE> {

   // Select UART to use
   using Uart = CdcUart<Uart1Info>;

public:

   /**
    * String indexes
    *
    * Must agree with stringDescriptors[] order
    */
   enum StringIds {
      /** Language information for string descriptors */
      s_language_index=0,    // Must be zero
      /** Manufacturer */
      s_manufacturer_index,
      /** Product Description */
      s_product_index,
      /** Serial Number */
      s_serial_index,
      /** Configuration Index */
      s_config_index,

      /** Name of Bulk interface */
      s_bulk_interface_index,

      /** Name of CDC interface */
      s_cdc_interface_index,
      /** CDC Control Interface */
      s_cdc_control_interface_index,
      /** CDC Data Interface */
      s_cdc_data_Interface_index,

      /** Marks last entry */
      s_number_of_string_descriptors
   };

   /**
    * Endpoint numbers\n
    * Must be consecutive
    */
   enum EndpointNumbers {
      /** USB Control endpoint number - must be zero */
      CONTROL_ENDPOINT  = 0,

      /* end-points are assumed consecutive */

      /** Bulk out endpoint number */
      BULK_OUT_ENDPOINT,
      /** Bulk in endpoint number */
      BULK_IN_ENDPOINT,

      /** CDC Control endpoint number */
      CDC_NOTIFICATION_ENDPOINT,
      /** CDC Data out endpoint number */
      CDC_DATA_OUT_ENDPOINT,
      /** CDC Data in endpoint number */
      CDC_DATA_IN_ENDPOINT,

      /** Total number of end-points */
      NUMBER_OF_ENDPOINTS,
   };

   /**
    * Configuration numbers, consecutive from 1
    */
   enum Configurations {
      CONFIGURATION_NUM = 1,
      /*
       * Assumes single configuration
       */
      /** Total number of configurations */
      NUMBER_OF_CONFIGURATIONS = CONFIGURATION_NUM,
   };

   /**
    * String descriptor table
    */
   static const uint8_t *const stringDescriptors[];

   /**
    * Device Descriptor
    */
   static const DeviceDescriptor deviceDescriptor;

   /**
    * Other descriptors type
    */
   struct Descriptors {
      ConfigurationDescriptor                  configDescriptor;

      InterfaceDescriptor                      bulk_interface;
      EndpointDescriptor                       bulk_out_endpoint;
      EndpointDescriptor                       bulk_in_endpoint;

      InterfaceAssociationDescriptor           interfaceAssociationDescriptorCDC;
      InterfaceDescriptor                      cdc_CCI_Interface;
      CDCHeaderFunctionalDescriptor            cdc_Functional_Header;
      CDCCallManagementFunctionalDescriptor    cdc_CallManagement;
      CDCAbstractControlManagementDescriptor   cdc_Functional_ACM;
      CDCUnionFunctionalDescriptor             cdc_Functional_Union;
      EndpointDescriptor                       cdc_notification_Endpoint;

      InterfaceDescriptor                      cdc_DCI_Interface;
      EndpointDescriptor                       cdc_dataOut_Endpoint;
      EndpointDescriptor                       cdc_dataIn_Endpoint;
   };

   /**
    * All other descriptors
    */
   static const Descriptors otherDescriptors;

   /**
    * Handler for Token Complete USB interrupts for
    * end-points other than EP0
    *
    * @param usbStat USB Status value from USB hardware
    */
   static void handleTokenComplete(UsbStat usbStat);
   /**
    * Clear value reflecting selected hardware based ping-pong buffer.
    * This would normally only be called when resetting the USB hardware or using
    * USBx_CTL_ODDRST.
    */
   static void clearPinPongToggle() {
      epBulkOut.clearPinPongToggle();
      epBulkIn.clearPinPongToggle();
      epCdcNotification.clearPinPongToggle();
      epCdcDataOut.clearPinPongToggle();
      epCdcDataIn.clearPinPongToggle();
   }

   /**
    * Initialises all end-points
    */
   static void initialiseEndpoints(void) {
      epBulkOut.initialise();
      addEndpoint(&epBulkOut);
      epBulkOut.setCallback(bulkOutTransactionCallback);

      epBulkIn.initialise();
      addEndpoint(&epBulkIn);
      epBulkIn.setCallback(bulkInTransactionCallback);

      epCdcNotification.initialise();
      addEndpoint(&epCdcNotification);

      epCdcDataOut.initialise();
      addEndpoint(&epCdcDataOut);
      epCdcDataOut.setCallback(cdcOutTransactionCallback);

      // Make sure epCdcDataOut is ready for polling (interrupt OUT)
      epCdcDataOut.startRxStage(EPDataOut, epCdcDataOut.getMaximumTransferSize());

      epCdcDataIn.initialise();
      addEndpoint(&epCdcDataIn);
      epCdcDataIn.setCallback(cdcInTransactionCallback);

      // Start CDC status transmission
      epCdcSendNotification();
   }

   /**
    *  Blocking transmission of data over bulk IN end-point
    *
    *  @param[in] size   Number of bytes to send
    *  @param[in] buffer Pointer to bytes to send
    *
    *  @note : Waits for idle BEFORE transmission but\n
    *          returns before data has been transmitted
    */
   static void sendBulkData(const uint8_t size, const uint8_t *buffer);

   /**
    *  Blocking reception of data over bulk OUT end-point
    *
    *   @param[in] maxSize Maximum number of bytes to receive
    *   @param[in] buffer  Pointer to buffer for bytes received
    *
    *   @return Number of bytes received
    *
    *   @note Doesn't return until command has been received.
    */
   static int receiveBulkData(uint8_t maxSize, uint8_t *buffer);

   /**
    * Initialise the USB0 interface
    *
    *  @note Assumes clock set up for USB operation (48MHz)
    */
   static void initialise();

protected:
   /* end-points */

   /** Out end-point for Bulk */
   static OutEndpoint <Usb0Info, Usb0::BULK_OUT_ENDPOINT, BULK_OUT_EP_MAXSIZE> epBulkOut;

   /** In end-point for Bulk */
   static InEndpoint  <Usb0Info, Usb0::BULK_IN_ENDPOINT,  BULK_IN_EP_MAXSIZE>  epBulkIn;

   /** In end-point for CDC notifications */
   static InEndpoint  <Usb0Info, Usb0::CDC_NOTIFICATION_ENDPOINT, CDC_NOTIFICATION_EP_MAXSIZE>  epCdcNotification;

   /** Out end-point for CDC data out */
   static OutEndpoint <Usb0Info, Usb0::CDC_DATA_OUT_ENDPOINT,     CDC_DATA_OUT_EP_MAXSIZE>      epCdcDataOut;

   /** In end-point for CDC data in */
   static InEndpoint  <Usb0Info, Usb0::CDC_DATA_IN_ENDPOINT,      CDC_DATA_IN_EP_MAXSIZE>       epCdcDataIn;
   /*
    * TODO Add additional End-points here
    */
    
   static bool forceCommandHandlerInitialise;

   /**
    * CDC Transmit
    *
    * @param[in] data Pointer to data to transmit
    * @param[in] size Number of bytes to transmit
    */
   static void sendCdcData(const uint8_t *data, unsigned size);

   /**
    * CDC Receive
    *
    * @param[in] data    Pointer to data to receive
    * @param[in] maxSize Maximum number of bytes to receive
    *
    * @return Number of bytes received
    */
   static int receiveCdcData(uint8_t *data, unsigned maxSize);

   static bool putCdcChar(uint8_t ch);

protected:

   /**
    * Callback for SOF tokens
    *
    * @param frameNumber Frame number from SOF token
    *
    * @return  Error code
    */
   static ErrorCode sofCallback(uint16_t frameNumber);

   /**
    * Call-back handling BULK-OUT transaction complete
    *
    * @param[in] state Current end-point state (always EPDataOut)
    *
    * @return The endpoint state to set after call-back (EPDataOut)
    */
   static EndpointState bulkOutTransactionCallback(EndpointState state);

   /**
    * Call-back handling BULK-IN transaction complete
    *
    * @param[in] state Current end-point state (always EPDataIn)
    *
    * @return The endpoint state to set after call-back (EPIdle/EPDataIn)
    */
   static EndpointState bulkInTransactionCallback(EndpointState state);

   /**
    * Call-back handling CDC-IN transaction complete\n
    * Checks for data and schedules transfer as necessary\n
    * Each transfer will have a ZLP as necessary.
    *
    * @param[in] state Current end-point state
    *
    * @return The endpoint state to set after call-back (EPIdle)
    */
   static EndpointState cdcInTransactionCallback(EndpointState state);

   /**
    * Call-back handling CDC-OUT transaction complete\n
    * Data received is passed to the cdcInterface
    *
    * @param[in] state Current end-point state
    *
    * @return The endpoint state to set after call-back (EPIdle)
    */
   static EndpointState cdcOutTransactionCallback(EndpointState state);

   /**
    * Configure epCdcNotification for an IN transaction [Tx, device -> host, DATA0/1]
    */
   static void epCdcSendNotification();

   /**
    * Handle SETUP requests not handled by base handler
    *
    * @param[in] setup SETUP packet received from host
    *
    * @note Provides CDC extensions
    * @note Provides BDM extensions
    */
   static ErrorCode handleUserEp0SetupRequests(const SetupPacket &setup);

   /**
    * CDC Set line coding handler
    */
   static void handleSetLineCoding();

   /**
    * CDC Get line coding handler
    */
   static void handleGetLineCoding();

   /**
    * CDC Set line state handler
    */
   static void handleSetControlLineState();

   /**
    * CDC Send break handler
    */
   static void handleSendBreak();
};

using UsbImplementation = Usb0;

#endif // USBDM_USB0_IS_DEFINED

} // End namespace USBDM

#endif /* PROJECT_HEADERS_USB_IMPLEMENTATION_COMPOSITE_H_ */
