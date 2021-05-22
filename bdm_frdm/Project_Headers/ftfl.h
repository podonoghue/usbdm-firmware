/**
 * @file    ftfl.h (180.ARM_Peripherals/Project_Headers/ftfl.h)
 * @brief   Flash support for FTFL
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

#include "derivative.h"
#include "hardware.h"
#include "delay.h"
#include "smc.h"

namespace USBDM {
/**
 * @addtogroup FTFL_Group FTFL, Flash Memory Module
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
   FLASH_ERR_NEW_EEPROM        = (15), // Indicates EEPROM has just bee partitioned and need initialisation
   FLASH_ERR_NOT_AVAILABLE     = (16), // Attempt to do flash operation when not available (e.g. while in VLPR mode)
};

/**
 * Class representing Flash interface.
 */
class Flash : public FtflInfo {

public:
    // Sector size for program flash (minimum erase element)
   static constexpr unsigned programFlashSectorSize = 1024;

   // Phrase size for program flash (minimum programming element)
   static constexpr unsigned programFlashPhraseSize = 4;

   // Sector size for data flash (minimum erase element)
   static constexpr unsigned dataFlashSectorSize = 1024;

   // Phrase size for data flash (minimum programming element)
   static constexpr unsigned dataFlashPhraseSize = 4;


   /** A23 == 0 => indicates PROGRAM flash */
   static constexpr uint32_t PROGRAM_ADDRESS_FLAG = (0<<23);

   /** A23 == 1 => indicates DATA flash */
   static constexpr uint32_t DATA_ADDRESS_FLAG    = (1<<23);

protected:

   /** Minimum ratio for EEPROM to Flash backing storage */
   static constexpr unsigned MINIMUM_BACKING_RATIO = 16;

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
   static void executeFlashCommand_asm();

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

   /**
    * Program EEPROM Data Size Code and FlexNVM Partition Code.
    *
    * See device reference manual for the meaning of the following parameters
    *
    * @param[in]  eeprom     EEPROM Data Size value
    * @param[in]  partition  FlexNVM Partition value
    *
    * @return Error code, 0 => no error
    */
   static FlashDriverError_t partitionFlash(uint8_t eeprom, uint8_t partition);

   /**
    * Initialise the EEPROM.
    *
    * This function should be called before the first access to variables located in the eeprom.
    *
    * @tparam eeprom     EEPROM Data Size choice
    * @tparam partition  FlexNVM Partition choice (defaults to all EEPROM backing store)
    * @tparam split      Split between A/B Flash portions (if supported by target)
    *
    * @return FLASH_ERR_OK         => EEPROM previous configured - no action required
    * @return FLASH_ERR_NEW_EEPROM => EEPROM has just been partitioned - contents are 0xFF, initialisation required
    *
    * @note This routine will only partition EEPROM when first executed after the device has been programmed.
    */
   template<EepromSel eeprom=eepromSel, PartitionSel partition=partitionSel, SplitSel split=partitionSplit>
   static FlashDriverError_t initialiseEeprom () {

//      console.
//      write("initialiseEeprom(eeprom=").write(eepromSizes[eeprom].size).write(" bytes, ").
//      write("eeprom backing=").write(eepromSizes[eeprom].size).write("K, ").
//      write("residual flash=").write(partitionInformation[partition].eeepromSize>>10).writeln("K)");

      if (isFlexRamConfigured()) {
//         console.write("flashController().FCNFG.FTFL_FCNFG_EEERDY = ").writeln((bool)(flashController().FCNFG&FTFL_FCNFG_EEERDY_MASK));
//         console.writeln("Flex RAM is already configured");
         return FLASH_ERR_OK;
      }
//      console.write("flashController().FCNFG.FTFL_FCNFG_EEERDY = ").writeln((bool)(flashController().FCNFG&FTFL_FCNFG_EEERDY_MASK));
      if ((eepromSizes[eeprom].size*MINIMUM_BACKING_RATIO)>(partitionInformation[partition].eeepromSize)) {
//         console.writeln("Backing ratio (Flash/EEPROM) is too small\n");
         USBDM::setErrorCode(E_FLASH_INIT_FAILED);
         return FLASH_ERR_ILLEGAL_PARAMS;
      }
#if defined(RELEASE_BUILD)
      // EEPROM only available in release build
      FlashDriverError_t rc = partitionFlash(eepromSizes[eeprom].value|split, partitionInformation[partition].value);
      if (rc != 0) {
//         console.writeln("Partitioning Flash failed\n");
         return rc;
      }
      // Indicate EEPROM needs initialisation - this is not an error
      return FLASH_ERR_NEW_EEPROM;
#else
      (void) eeprom;
      (void) partition;
      (void) split;

      // For debug, initialise FlexRam every time (no actual writes to flash)

      // Initialisation pretend EEPROM on every reset
      // This return code is not an error
      return FLASH_ERR_NEW_EEPROM;
#endif
   }

public:

   /**
    * Hardware instance pointer
    *
    * @return Reference to Flash hardware
    */
   __attribute__((always_inline)) static volatile FTFL_Type &flashController() { return ftfl(); }

   /**
    * Checks if the flexRAM has been configured.
    * Will wait for flash ready as necessary
    *
    * @return true if configured
    */
   static bool isFlexRamConfigured() {
#if 1
      return waitForFlashReady() && (flashController().FCNFG&FTFL_FCNFG_EEERDY_MASK);
#else
      console.write("flashController().FCNFG = ").writeln(flashController().FCNFG, Radix_16);
      console.write("flashController().FCNFG.FTFL_FCNFG_RAMRDY = ").writeln((bool)(flashController().FCNFG&FTFL_FCNFG_RAMRDY_MASK));
      console.write("flashController().FCNFG.FTFL_FCNFG_EEERDY = ").writeln((bool)(flashController().FCNFG&FTFL_FCNFG_EEERDY_MASK));

      uint8_t result[4];
      FlashDriverError_t rc = readFlashResource(0, DATA_ADDRESS_FLAG|0xFC, result);
      if (rc != 0) {
         console.write("IFR read failed, rc=").writeln(rc);
         return false;
      }
      uint8_t flexNvmPartitionSize = result[0];
      uint8_t eepromDatSetSize     = result[1];

      console.write("FlexNVM partition code = ").writeln(flexNvmPartitionSize, Radix_16);
      console.write("EEPROM data set size   = ").writeln(eepromDatSetSize, Radix_16);

      return (flashController().FCNFG&FTFL_FCNFG_EEERDY_MASK);
#endif
   }

   /**
    * Wait until flash is ready.
    * Any flash operations will have completed.
    *
    * @return true => OK, false => timeout
    */
   static bool waitForFlashReady() {
      for(int timeout=0; timeout<100000; timeout++) {
         if ((flashController().FSTAT&FTFL_FSTAT_CCIF_MASK) != 0) {
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
      return (Smc::getStatus() == SmcStatus_RUN);
   }

   /**
    * Waits until the current flash operation is complete with run mode check.
    * This is used to wait until a FlexRAM write has completed.
    *
    * @return true  => Operation complete and FlexRAM idle
    * @return false => timeout or flash not available
    */
   static bool waitUntilFlexIdle() {
      usbdm_assert(isFlashAvailable(), "Flash use in unsuitable run mode");
      return
            isFlashAvailable() &&
            waitForFlashReady();
   }

   /**
    * Enable interrupts in NVIC
    */
   static void enableNvicInterrupts() {
      NVIC_EnableIRQ(irqNums[0]);
   }

   /**
    * Enable and set priority of interrupts in NVIC
    * Any pending NVIC interrupts are first cleared.
    *
    * @param[in]  nvicPriority  Interrupt priority
    */
   static void enableNvicInterrupts(uint32_t nvicPriority) {
      enableNvicInterrupt(irqNums[0], nvicPriority);
   }

   /**
    * Disable interrupts in NVIC
    */
   static void disableNvicInterrupts() {
      NVIC_DisableIRQ(irqNums[0]);
   }

   /**
    * Enable interrupts in NVIC
    * Any pending NVIC interrupts are first cleared.
    */
   static void enableNvicCollisionInterrupts() {
      NVIC_EnableIRQ(irqNums[1]);
   }

   /**
    * Enable and set priority of interrupts in NVIC
    * Any pending NVIC interrupts are first cleared.
    *
    * @param[in]  nvicPriority  Interrupt priority
    */
   static void enableNvicCollisionInterrupts(uint32_t nvicPriority) {
      enableNvicInterrupt(irqNums[1], nvicPriority);
   }

   /**
    * Disable interrupts in NVIC
    */
   static void disableNvicCollisionInterrupts() {
      NVIC_DisableIRQ(irqNums[1]);
   }

private:
   /**
    * Program a phrase to Flash memory.
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
    * Program a range of bytes to Flash memory.
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
 * Class to wrap a scalar variable allocated within the FlexRam area.
 * Size is limited to 1, 2 or 4 bytes.
 *
 * Writing to the variable triggers an EEPROM update.\n
 * Ensures updates are completed before return.
 *
 * @tparam T Scalar type for variable
 *
 * @note Instances should be placed in FlexRAM segment e.g.\n
 * @code
 * __attribute__ ((section(".flexRAM")))
 * USBDM::Nonvolatile<char> a_nonvolatile_char;
 * @endcode
 */
template <typename T>
class Nonvolatile {

   static_assert((sizeof(T) == 1)||(sizeof(T) == 2)||(sizeof(T) == 4), "Size of non-volatile object must be 1, 2 or 4 bytes in size");

private:
   /**
    * Data value in FlexRAM.
    *
    * FlexRAM required data to be aligned according to its size.\n
    * Be careful how you order variables otherwise space will be wasted
    */
   __attribute__ ((aligned (sizeof(T))))
   T data;

public:
   /**
    * Assign to underlying type.
    * This adds a wait for the Flash to be updated
    *
    * @param[in]  data The data to assign
    */
   void operator=(const Nonvolatile<T> &data ) {
      this->data = (T)data;
      Flash::waitUntilFlexIdle();
   }
   /**
    * Assign to underlying type.
    * This adds a wait for the Flash to be updated
    *
    * @param[in]  data The data to assign
    */
   void operator=(const T &data ) {
      this->data = data;
      Flash::waitUntilFlexIdle();
   }
   /**
    * Increment underlying type.
    * This adds a wait for the Flash to be updated
    *
    * @param[in]  change The amount to increment
    */
   void operator+=(const Nonvolatile<T> &change ) {
      this->data += (T)change;
      Flash::waitUntilFlexIdle();
   }
   /**
    * Increment underlying type.
    * This adds a wait for the Flash to be updated
    *
    * @param[in]  change The amount to increment
    */
   void operator+=(const T &change ) {
      this->data += change;
      Flash::waitUntilFlexIdle();
   }
   /**
    * Decrement underlying type.
    * This adds a wait for the Flash to be updated
    *
    * @param[in]  change The amount to increment
    */
   void operator-=(const Nonvolatile<T> &change ) {
      this->data -= (T)change;
      Flash::waitUntilFlexIdle();
   }
   /**
    * Decrement underlying type.
    * This adds a wait for the Flash to be updated
    *
    * @param[in]  change The amount to increment
    */
   void operator-=(const T &change ) {
      this->data -= change;
      Flash::waitUntilFlexIdle();
   }
   /**
    * Return the underlying object - <b>read-only</b>.
    *
    * @return underlying object
    */
   operator T() const {
      Flash::waitUntilFlexIdle();
      return data;
   }
};

/**
 * Class to wrap an array of scalar variables allocated to the FlexRam area.
 *
 * Element size is limited to 1, 2 or 4 bytes.
 *
 * Writing to an element triggers an EEPROM update.\n
 * Ensures updates are completed before return.
 *
 * @tparam T         Scalar type for element
 * @tparam dimension Dimension of array
 *
 * @note Instances should be placed in FlexRAM segment e.g.\n
 * @code
 * __attribute__ ((section(".flexRAM")))
 * USBDM::NonvolatileArray<20, int> a_nonvolatile_array_of_ints;
 * @endcode
 */
template <typename T, int dimension>
class NonvolatileArray {

   static_assert((sizeof(T) == 1)||(sizeof(T) == 2)||(sizeof(T) == 4), "T must be 1, 2 or 4 bytes in size");

private:
   using TArray = T[dimension];
   using TPtr   = const T(*);

   /** Array of elements in FlexRAM.
    *
    *  FlexRAM required data to be aligned according to its size.\n
    *  Be careful how you order variables otherwise space will be wasted
    */
   __attribute__ ((aligned (sizeof(T))))
   T data[dimension];

public:
   /**
    * Assign to underlying array.
    *
    * @param[in]  other TArray to assign from
    *
    * This adds a wait for the Flash to be updated after each element is assigned
    */
   void operator=(const TArray &other ) {
      for (int index=0; index<dimension; index++) {
         data[index] = other[index];
         Flash::waitUntilFlexIdle();
      }
   }

   /**
    * Assign to underlying array.
    *
    * @param[in]  other NonvolatileArray to assign from
    *
    * This adds a wait for the Flash to be updated after each element is assigned
    */
   void operator=(const NonvolatileArray &other ) {
      if (this == &other) {
         // Identity check
         return;
      }
      for (int index=0; index<dimension; index++) {
         data[index] = other[index];
         Flash::waitUntilFlexIdle();
      }
   }

   /**
    * Assign to underlying array.
    *
    * @param[in]  other NonvolatileArray to assign to
    *
    * This adds a wait for the Flash to be updated after each element is assigned
    */
   void copyTo(T *other) const {
      for (int index=0; index<dimension; index++) {
         other[index] = data[index];
      }
   }

   /**
    * Return a reference to the underlying array element - read-only.
    *
    * @param[in]  index Index of element to return
    *
    * @return Reference to underlying array
    */
   const T operator [](int index) {
      return data[index];
   }

   /**
    * Return a pointer to the underlying array - read-only.
    */
   operator TPtr() const {
      return data;
   }

   /**
    * Set an element of the array to the value provided.
    *
    * @param[in]  index Array index of element to change
    * @param[in]  value Value to initialise array elements to
    */
   void set(int index, T value) {
      data[index] = value;
      Flash::waitUntilFlexIdle();
   }
   /**
    * Set all elements of the array to the value provided.
    *
    * @param[in]  value Value to initialise array elements to
    */
   void set(T value) {
      for (int index=0; index<dimension; index++) {
         data[index] = value;
         Flash::waitUntilFlexIdle();
      }
   }
};
/**
 * @}
 */

} // namespace USBDM

#endif /* SOURCES_FLASH_H_ */
