/**
 * @file     usb_implementation_bulk.h
 * @brief    USB Bulk device implementation
 *
 * This module provides an implementation of a USB Bulk interface
 * including the following end points:
 *  - EP0 Standard control
 *  - EP1 Bulk OUT
 *  - EP2 Bulk IN
 *
 * @version  V4.12.1.170
 * @date     2 April 2017
 *
 *  This file provides the implementation specific code for the USB interface.
 *  It will need to be modified to suit an application.
 */
#ifndef PROJECT_HEADERS_USB_IMPLEMENTATION_BULK_H_
#define PROJECT_HEADERS_USB_IMPLEMENTATION_BULK_H_

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

#define UNIQUE_ID
//#include "configure.h"

namespace USBDM {

//======================================================================
// Customise for each USB device
//

/** Causes a semi-unique serial number to be generated for each USB device */
#define UNIQUE_ID

#ifdef UNIQUE_ID
#define SERIAL_NO           "USBDM-"
#else
#define SERIAL_NO           "USBDM-0001"
#endif
#define PRODUCT_DESCRIPTION "Kinetis"
#define MANUFACTURER        "pgo"

static constexpr uint16_t  VENDOR_ID   = 0x16D0;    // Vendor ID (actually MCS)
static constexpr uint16_t  PRODUCT_ID  = 0x4325;    // Product ID
static constexpr uint16_t  VERSION_ID  = 0x0100;    // Version ID

//======================================================================
// Maximum packet sizes for each endpoint
//
static constexpr unsigned  CONTROL_EP_MAXSIZE           = 64; //!< Control in/out
/*
 *  TODO Define additional end-point sizes
 */
static constexpr unsigned  BULK_OUT_EP_MAXSIZE          = 64; //!< Bulk out
static constexpr unsigned  BULK_IN_EP_MAXSIZE           = 64; //!< Bulk in

#ifdef USBDM_USB0_IS_DEFINED
/**
 * Class representing USB0
 */
class Usb0 : public UsbBase_T<Usb0Info, CONTROL_EP_MAXSIZE> {

   // Allow superclass to access handleTokenComplete(void);
   friend UsbBase_T<Usb0Info, CONTROL_EP_MAXSIZE>;

public:

   using UsbBase_T<Usb0Info, CONTROL_EP_MAXSIZE>::setUserCallback;

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

      /*
       * TODO Add additional String indexes
       */

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


      /*
       * TODO Add additional Endpoint numbers here
       */
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

protected:
   /* end-points */

   /** Out end-point for Bulk */
   static OutEndpoint <Usb0Info, Usb0::BULK_OUT_ENDPOINT, BULK_OUT_EP_MAXSIZE> epBulkOut;

   /** In end-point for Bulk */
   static InEndpoint  <Usb0Info, Usb0::BULK_IN_ENDPOINT,  BULK_IN_EP_MAXSIZE>  epBulkIn;

   /*
    * TODO Add additional End-points here
    */

public:

   /**
    * Initialise the USB0 interface
    *
    *  @note Assumes clock set up for USB operation (48MHz)
    */
   static void initialise();

   /**
    *  Blocking transmission of data over bulk IN end-point
    *
    *  @param[IN] size    Number of bytes to send
    *  @param[IN] buffer  Pointer to bytes to send
    *  @param[IN] timeout Maximum time to wait for packet
    *
    *  @note : Waits for idle BEFORE transmission but\n
    *          returns before data has been transmitted
    */
   static ErrorCode sendBulkData(const uint16_t size, const uint8_t *buffer, uint32_t timeout);

   /**
    *  Blocking reception of data over bulk OUT end-point
    *
    *   @param[IN/OUT] size    Max Number of bytes to receive/Actual number received
    *   @param[IN]     buffer  Pointer to buffer for bytes received
    *
    *   @return Error code
    *
    *   @note Doesn't return until some bytes have been received
    */
   static ErrorCode receiveBulkData(uint16_t &size, uint8_t *buffer);

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

      /*
       * TODO Add additional Descriptors here
       */
   };

   /**
    * All other descriptors
    */
   static const Descriptors otherDescriptors;

protected:
   /**
    * Clear value reflecting selected hardware based ping-pong buffer.
    * This would normally only be called when resetting the USB hardware or using
    * USBx_CTL_ODDRST.
    */
   static void clearPinPongToggle() {
      epBulkOut.clearPinPongToggle();
      epBulkIn.clearPinPongToggle();
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

      /*
       * TODO Initialise additional End-points here
       */
   }

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
    * Handler for Token Complete USB interrupts for
    * end-points other than EP0
    *
    * @param usbStat USB Status value from USB hardware
    */
   static void handleTokenComplete(UsbStat usbStat);

};

using UsbImplementation = Usb0;

#endif // USBDM_USB0_IS_DEFINED

} // End namespace USBDM

#endif /* PROJECT_HEADERS_USB_IMPLEMENTATION_BULK_H_ */
