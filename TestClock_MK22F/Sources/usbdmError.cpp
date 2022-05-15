/**
 * @file     usbdmError.cpp (180.ARM_Peripherals/Sources/usbdmError.cpp)
 * @brief    Error handling
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */
#include "pin_mapping.h"

namespace USBDM {

/** Last error set by USBDM code */
volatile ErrorCode errorCode = E_NO_ERROR;

#if USE_CONSOLE

/** Table of error messages indexed by error code */
static const char *messages[] {
      "No error",
      "General error",
      "Too small",
      "Too large",
      "Illegal parameter",
      "Call-back not installed",
      "Flash initialisation failed",
      "ADC Calibration failed",
      "Illegal processor run-mode transition",
      "Failed communication",
      "I2C No acknowledge",
      "I2C Lost arbitration for bus",
      "Program has terminated",
      "Clock initialisation failed",
      "Callback already installed",
      "Failed resource allocation",
};
#endif

/**
 * Get error message from error code or last error if not provided
 *
 * @param[in]   err Error code
 *
 * @return Pointer to static string
 */
const char *getErrorMessage(ErrorCode err) {
#ifdef __CMSIS_RTOS
   // Check for CMSIS error codes
   if (err & E_CMSIS_ERR_OFFSET) {
      return "CMSIS error";
   }
#endif
#if USE_CONSOLE
   if (err>(sizeof(messages)/sizeof(messages[0]))) {
      return "Unknown error";
   }
   return messages[err];
#else
   (void) err;
   return "";
#endif
}

#ifdef DEBUG_BUILD
void abort(const char *msg __attribute__((unused))) {
#if USE_CONSOLE
   log_error(msg);
#endif
   while(true) {
      __BKPT();
   }
}

/**
 * Check for error code being set (drastically!)
 * This routine does not return if there is an error
 */
ErrorCode checkError() {
   while (errorCode != E_NO_ERROR) {
#if USE_CONSOLE
      const char *msg = getErrorMessage();
      __attribute__((unused))
      int cmsisErrorCode = errorCode & ~E_CMSIS_ERR_OFFSET;
      log_error(msg);
#endif
      // If you arrive here then an error has been detected.
      // If a CMSIS error, check the 'cmsisErrorCode' above and refer to the CMSIS error codes
      __BKPT();
   }
   return errorCode;
}
#endif

/**
 * Enable and set priority of interrupts in NVIC.
 *
 * @param[in]  irqNum        Interrupt number
 * @param[in]  nvicPriority  Interrupt priority
 *
 * @note Any pending interrupts are cleared before enabling.
 */
void enableNvicInterrupt(IRQn_Type irqNum, uint32_t nvicPriority) {

   // Clear Pending interrupts
   NVIC_ClearPendingIRQ(irqNum);

   // Enable interrupts
   NVIC_EnableIRQ(irqNum);

   // Set priority level
   NVIC_SetPriority(irqNum, nvicPriority);
}
} // end namespace USBDM
