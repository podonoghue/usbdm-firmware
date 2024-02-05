/**
 * @file    ftfa.h (180.ARM_Peripherals/Project_Headers/ftfa.h)
 * @brief   Flash support for FTFA
 *
 *  Created on: 21 Sep 2016
 *      Author: podonoghue
 */

/* *************************************************************
 * NOTE - Can't use other objects here as initialisation of
 *        Flash is done very early (including writeln())
 ************************************************************* */

#ifndef SOURCES_FLASH_H_
#define SOURCES_FLASH_H_

#include "pin_mapping.h"

namespace USBDM {
/**
 * @addtogroup FTFA_Group FTFA, Flash Memory Module
 * @brief Abstraction for Flash Memory Module
 * @{
 */
// Error codes
enum FlashDriverError_t {
   FLASH_ERR_OK                = (0),
   FLASH_ERR_LOCKED            = (1),  // Flash is still locked
   FLASH_ERR_ILLEGAL_PARAMS    = (2),  // Parameters illegal
   FLASH_ERR_PROG_FAILED       = (3),  // STM - Programming operation failed - general
   FLASH_ERR_PROG_WPROT        = (4),  // STM - Programming operation failed - write protected
   FLASH_ERR_VERIFY_FAILED     = (5),  // Verify failed
   FLASH_ERR_ERASE_FAILED      = (6),  // Erase or Blank Check failed
   FLASH_ERR_TRAP              = (7),  // Program trapped (illegal instruction/location etc.)
   FLASH_ERR_PROG_ACCERR       = (8),  // Kinetis/CFVx - Programming operation failed - ACCERR
   FLASH_ERR_PROG_FPVIOL       = (9),  // Kinetis/CFVx - Programming operation failed - FPVIOL
   FLASH_ERR_PROG_MGSTAT0      = (10), // Kinetis - Programming operation failed - MGSTAT0
   FLASH_ERR_CLKDIV            = (11), // CFVx - Clock divider not set
   FLASH_ERR_ILLEGAL_SECURITY  = (12), // Kinetis/CFV1+ - Illegal value for security location
   FLASH_ERR_UNKNOWN           = (13), // Unspecified error
   FLASH_ERR_PROG_RDCOLERR     = (14), // Read Collision
   FLASH_ERR_NEW_EEPROM        = (15), // Indicates EEPROM has just been partitioned and needs initialisation
   FLASH_ERR_NOT_AVAILABLE     = (16), // Attempt to do flash operation when not available (e.g. while in VLPR mode)
};

/**
 * Class representing Flash interface.
 */
class Flash : public FtfaInfo {

public:
   // Sector size for program flash (minimum erase element)
   static constexpr unsigned programFlashSectorSize = 2048;

   // Phrase size for program flash (minimum programming element)
   static constexpr unsigned programFlashPhraseSize = 4;
   

protected:

   /**
    * Constructor.
    * Typically this method would be overridden in a derived class
    * to do the initialisation of the flash and non-volatile variables.
    * Alternatively, the startup code may call the static methods directly.
    */
   Flash() {
      static int singletonFlag __attribute__((unused)) = false;
      usbdm_assert (!singletonFlag, "Creating multiple instances of Flash");
      singletonFlag = true;
      waitForFlashReady();
   }

   /**
    * Launch & wait for Flash command to complete.
    */
   static void executeFlashCommand_ram();

   /**
    * Launch & wait for Flash command to complete.
    */
   static FlashDriverError_t executeFlashCommand();

   /**
    * Read Flash Resource (IFR etc).
    * This command reads 4 bytes from the selected flash resource
    *
    * @param[in]  resourceSelectCode 00 => IFR, 01 => Version ID
    * @param[in]  address            Address in IFR etc, A23=0 => Program flash, A23=1 => Data flash
    * @param[out] data               Buffer for data returned
    *
    * @return Error code, 0 => no error
    */
   static FlashDriverError_t readFlashResource(uint8_t resourceSelectCode, uint32_t address, uint8_t *data);

public:

   static void Command_irqHandler() {
   }
 
   static void ReadCollision_irqHandler() {
   }

   /**
    * Hardware instance pointer
    *
    * @return Reference to Flash hardware
    */
   static constexpr HardwarePtr<FTFA_Type> flashController = baseAddress;

   /**
    * Wait until flash is ready.
    * Any flash operations will have completed.
    *
    * @return true => OK, false => timeout
    */
   static bool waitForFlashReady() {
      for(int timeout=0; timeout<100000; timeout++) {
         if ((flashController->FSTAT&FTFA_FSTAT_CCIF_MASK) != 0) {
            return true;
         }
      }
      return false;
   }

   /**
    * Check if flash operations are available.
    * This will check if the processor is in the correct mode for flash operations.
    *
    * @return true  => OK
    * @return false => Processor not in correct mode
    */
   static bool isFlashAvailable() {
      return (SmcInfo::getStatus() == SmcStatus_RUN);
   }

   /**
    * Waits until the current flash operation is complete with run mode check.
    * This is used to wait until a FlexRAM write has completed.
    *
    * @return true  => Operation complete and FlexRAM idle
    * @return false => Timeout or flash not available
    */
   static bool waitUntilFlexIdle() {
      usbdm_assert(isFlashAvailable(), "Flash use in unsuitable run mode");
      return
            isFlashAvailable() &&
            waitForFlashReady();
   }

private:
   /**
    * Program phrase to Flash memory
    *
    * @param[in]  data       Location of data to program
    * @param[out] address    Memory address to program - must be phrase boundary
    *
    * @return Error code
    */
   static FlashDriverError_t programPhrase(const uint8_t *data, uint8_t *address);

   /**
    * Erase sector of Flash memory.
    *
    * @param[in]  address    Memory address to erase - must be sector boundary
    *
    * @return Error code
    */
   static FlashDriverError_t eraseSector(uint8_t *address);

public:
   /**
    * Program a range of bytes to Flash memory
    *
    * @param[in]  data       Location of data to program
    * @param[out] address    Memory address to program - must be phrase boundary
    * @param[in]  size       Size of range (in bytes) to program - must be multiple of phrase size
    *
    * @return Error code
    */
   static FlashDriverError_t programRange(const uint8_t *data, uint8_t *address, uint32_t size);

   /**
    * Erase a range of Flash memory.
    *
    * @param[in]  address    Memory address to start erasing - must be sector boundary
    * @param[in]  size       Size of range (in bytes) to erase - must be multiple of sector size
    *
    * @return Error code
    */
   static FlashDriverError_t eraseRange(uint8_t *address, uint32_t size);
   /**
    * Mass erase entire Flash memory.
    */
   static void eraseAll();
};

/**
 * @}
 */

} // namespace USBDM

#endif /* SOURCES_FLASH_H_ */
