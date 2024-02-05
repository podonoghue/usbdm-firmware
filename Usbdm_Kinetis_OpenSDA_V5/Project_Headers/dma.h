/**
 * @file    dma.h  (180.ARM_Peripherals/Project_Headers/dma-MK.h)
 * @brief   Direct Memory Controller
 *
 * @version  V4.12.1.210
 * @date     30 September 2017
 */

#ifndef INCLUDE_USBDM_DMA_H_
#define INCLUDE_USBDM_DMA_H_

#include "pin_mapping.h"
#include "dmamux.h"
#include "cstring"

/*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
namespace USBDM {

#if false // /DMA/enablePeripheralSupport

typedef DmaBasicInfo::DmaTcdCsr DmaTcdCsr;
typedef DmaBasicInfo::DmaConfig DmaConfig;

/**
 * @addtogroup DMA_Group DMA, Direct Memory Access (DMA)
 * @brief Support for DMA operations
 * @{
 */

/**
 * DMA transfer sizes.
 */
enum DmaSize : uint8_t {
   DmaSize_8bit    = 0b000,  //!< 8-bit transfer
   DmaSize_16bit   = 0b001,  //!< 16-bit transfer
   DmaSize_32bit   = 0b010,  //!< 32-bit transfer
   DmaSize_16byte  = 0b100,  //!< 16-byte transfer
   DmaSize_32byte  = 0b101,  //!< 32-byte transfer
};

/**
 * DMA modulo size
 */
enum DmaModulo : uint8_t {
   DmaModulo_Disabled   = 0b00000, //!< Modulo function disabled
   DmaModulo_2byte      = 0b00001, //!< 2-byte modulo
   DmaModulo_4byte      = 0b00010, //!< 4-byte modulo
   DmaModulo_8byte      = 0b00011, //!< 8-byte modulo
   DmaModulo_16byte     = 0b00100, //!< 16-byte modulo
   DmaModulo_32byte     = 0b00101, //!< 32-byte modulo
   DmaModulo_64byte     = 0b00110, //!< 64-byte modulo
   DmaModulo_128byte    = 0b00111, //!< 128-byte modulo
   DmaModulo_256byte    = 0b01000, //!< 256-byte modulo
   DmaModulo_512byte    = 0b01001, //!< 512-byte modulo
   DmaModulo_1KiByte    = 0b01010, //!< 1-Kibibyte modulo
   DmaModulo_2KiByte    = 0b01011, //!< 2-Kibibyte modulo
   DmaModulo_4KiByte    = 0b01100, //!< 4-Kibibyte modulo
   DmaModulo_8KiByte    = 0b01101, //!< 5-Kibibyte modulo
   DmaModulo_16KiByte   = 0b01110, //!< 16-Kibibyte modulo
   DmaModulo_32KiByte   = 0b01111, //!< 32-Kibibyte modulo
   DmaModulo_64KiByte   = 0b10000, //!< 64-Kibibyte modulo
   DmaModulo_128KiByte  = 0b10001, //!< 128-Kibibyte modulo
   DmaModulo_256KiByte  = 0b10010, //!< 256-Kibibyte modulo
   DmaModulo_512KiByte  = 0b10011, //!< 512-Kibibyte modulo
   DmaModulo_1MiByte    = 0b10100, //!< 1-Mebibyte  modulo
   DmaModulo_2MiByte    = 0b10101, //!< 2-Mebibyte modulo
   DmaModulo_4MiByte    = 0b10110, //!< 4-Mebibyte modulo
   DmaModulo_8MiByte    = 0b10111, //!< 8-Mebibyte modulo
   DmaModulo_16MiByte   = 0b11000, //!< 16-Mebibyte modulo
   DmaModulo_32MiByte   = 0b11001, //!< 32-Mebibyte modulo
   DmaModulo_64MiByte   = 0b11010, //!< 64-Mebibyte modulo
   DmaModulo_128MiByte  = 0b11011, //!< 128-Mebibyte modulo
   DmaModulo_256MiByte  = 0b11100, //!< 256-Mebibyte modulo
   DmaModulo_512MiByte  = 0b11101, //!< 512-Mebibyte modulo
   DmaModulo_1GiByte    = 0b11110, //!< 1-Gibibyte modulo
   DmaModulo_2GiByte    = 0b11111, //!< 2-Gibibyte modulo
};

/**
 * Get DMA size of object.
 * For use in TCD.SMOD, TCD.DMOD value
 *
 * @param[in] obj Object to obtain DMA size value for
 *
 * @return one of the DmaSize_xxxx values
 */
template <class T>
static constexpr DmaSize dmaSize(const T &obj) {
   static_assert(((sizeof(obj)==1)||(sizeof(obj)==2)||(sizeof(obj)==4)||(sizeof(obj)==16)||(sizeof(obj)==32)), "Illegal DMA transfer size");
   return
      (sizeof(obj)==1) ?DmaSize_8bit:
      (sizeof(obj)==2) ?DmaSize_16bit:
      (sizeof(obj)==4) ?DmaSize_32bit:
      (sizeof(obj)==16)?DmaSize_16byte:
      /*          ==32*/DmaSize_32byte;
}

/**
 * Get DMA size of object.
 * For use in TCD.SMOD, TCD.DMOD value
 *
 * @param[in] obj Object to obtain DMA size value for
 *
 * @return one of the DmaSize_xxxx values
 */
template <class T>
static constexpr DmaSize dmaSize(const T *obj) {
   static_assert(((sizeof(*obj)==1)||(sizeof(*obj)==2)||(sizeof(*obj)==4)||(sizeof(*obj)==16)||(sizeof(*obj)==32)), "Illegal DMA transfer size");
   return
      (sizeof(*obj)==1) ?DmaSize_8bit:
      (sizeof(*obj)==2) ?DmaSize_16bit:
      (sizeof(*obj)==4) ?DmaSize_32bit:
      (sizeof(*obj)==16)?DmaSize_16byte:
      /*          ==32*/DmaSize_32byte;
}

/**
 * Get DMA size of object.
 * For use in TCD.SMOD, TCD.DMOD value
 *
 * @tparam T Type to get DMA size of
 *
 * @return one of the DmaSize_xxxx values
 */
template <class T>
static constexpr DmaSize dmaSize() {
   static_assert(((sizeof(T)==1)||(sizeof(T)==2)||(sizeof(T)==4)||(sizeof(T)==16)||(sizeof(T)==32)), "Illegal DMA transfer size");
   return
      (sizeof(T)==1) ?DmaSize_8bit:
      (sizeof(T)==2) ?DmaSize_16bit:
      (sizeof(T)==4) ?DmaSize_32bit:
      (sizeof(T)==16)?DmaSize_16byte:
      /*          ==32*/DmaSize_32byte;
}

/**
 * Get DMA source size of object.
 * For use in TCD ATTR value
 *
 * @param[in] obj Object to get size of
 *
 * @return mask suitable for use as part of TCD.ATTR value
 */
template <class T>
static constexpr uint32_t dmaSSize(const T &obj) {
   return DMA_ATTR_SSIZE(dmaSize(obj));
}

/**
 * Get DMA destination size of object.
 * For use in TCD ATTR value
 *
 * @param[in] obj Object to get size of
 *
 * @return mask suitable for use as part of TCD.ATTR value
 */
template <class T>
static constexpr uint32_t dmaDSize(const T &obj) {
   return DMA_ATTR_DSIZE(dmaSize(obj));
}

/**
 * Get DMA source and destination size of objects.
 * For use in TCD ATTR value
 *
 * @param[in] sobj DMA source object to get size of
 * @param[in] dobj DMA destination object to get size of
 *
 * @return mask suitable for use as part of TCD.ATTR value
 */
template <class Ts, class Td>
static constexpr uint16_t dmaSize(const Ts &sobj, const Td &dobj) {
   return DMA_ATTR_SSIZE(dmaSize(sobj))|DMA_ATTR_DSIZE(dmaSize(dobj));
}

/**
 * Creates DMA NBYTES entry for:
 * - Minor loop enabled CR[EMLN] = 1 (DmaMinorLoopMapping_Enabled)
 * - Source/Destination offset enabled
 *
 * In this mode address adjustments are made at the end of a minor loop
 *
 * @param offset                    20-bit signed value for offset to apply after minor loop completion
 * @param dmaMinorLoopOffsetSelect  Determines if the offset is applied to source and/or destination address
 * @param nBytes                    10-bit value for byte count
 *
 * @return TCD.NBYTES entry
 */
constexpr uint32_t dmaNBytes(uint16_t nBytes, DmaMinorLoopOffsetSelect dmaMinorLoopOffsetSelect,  int32_t offset) {
   return
#ifdef DEBUG_BUILD
   usbdm_assert((nBytes<(1<<10)), "nBytes is too large"),
   usbdm_assert(((offset>=-(1<<19))&&(offset<(1<<19))),"Offset is too large"),
#endif
   dmaMinorLoopOffsetSelect|DMA_NBYTES_MLOFFYES_MLOFF(offset)|DMA_NBYTES_MLOFFYES_NBYTES(nBytes);
}

struct __attribute__((__packed__)) DmaTcdAttr {
   union {
      struct {
         DmaSize       DSIZE:3;       //!< Destination size
         DmaModulo     DMOD:5;        //!< Destination modulo
         DmaSize       SSIZE:3;       //!< Source size
         DmaModulo     SMOD:5;        //!< Source modulo
      };
      uint16_t data;
   };
   /**
    * Default constructor
    */
   constexpr DmaTcdAttr() :
      DSIZE(DmaSize_8bit),
      DMOD(DmaModulo_Disabled),
      SSIZE(DmaSize_8bit),
      SMOD(DmaModulo_Disabled) {
   }
   /**
    * Constructor
	 *
    * @param dmaSizeSource          Size for source transfers
    * @param dmaModuloSource        Modulo for source transfers
    * @param dmaSizeDestination     Size for destination transfers
    * @param dmaModuloDestination   Modulo for destination transfers
    */
   constexpr DmaTcdAttr(
         DmaSize     dmaSizeSource,
         DmaModulo   dmaModuloSource,
         DmaSize     dmaSizeDestination,
         DmaModulo   dmaModuloDestination
   ) :
      DSIZE(dmaSizeDestination),
      DMOD(dmaModuloDestination),
      SSIZE(dmaSizeSource),
      SMOD(dmaModuloSource) {
   }
};

/**
 *  Used to create transfer information for source or destination
 */
class DmaInfo {

public:
   // Start address for transfer
   uint32_t    startAddress;

   // Signed adjustment of address after each transfer
   int32_t     offset;

   // Signed adjustment of address at the completion of the major iteration
   int32_t    lastAddressAdjustment;

   // Data transfer size
   DmaSize     dmaSize;

   // Modulo adjustment for address
   // This allows the address to be restricted to a binary range
   DmaModulo   dmaModulo;

   /**
    *
    * @param startAddress           Start address of transfer
    * @param offset                 Signed adjustment of address after each transfer
    * @param dmaSize                Data transfer size
    * @param dmaModulo              Modulo adjustment for address
    *                               This allows the address to be restricted to a binary range
    * @param lastAddressAdjustment  Signed adjustment of address at the completion of the major iteration
    */
   constexpr DmaInfo(
         uint32_t  startAddress,
         int32_t   offset,
         DmaSize   dmaSize,
         int32_t   lastAddressAdjustment  = 0,
         DmaModulo dmaModulo              = DmaModulo_Disabled) :

            startAddress(startAddress),
            offset(offset),
            lastAddressAdjustment(lastAddressAdjustment),
            dmaSize(dmaSize),
            dmaModulo(dmaModulo) {
   }
};

/**
 * Transfer Control Descriptor
 * @verbatim
 * +------------------------------+            Simple DMA mode (MLNO = Minor Loop Mapping Disabled)
 * | Major Loop =                 |            ==================================================
 * |    CITER x Minor Loop        |
 * |                              |            Each DMA request triggers a minor-loop transfer sequence.
 * | +--------------------------+ |<-DMA Req.  The minor loops are counted in the major-loop.
 * | | Minor Loop               | |
 * | | Each transfer            | |            The following are used during a minor loop:
 * | |   SADDR->DADDR           | |             - SADDR Source address
 * | |   SADDR += SOFF          | |             - SOFF  Adjustment applied to SADDR after each transfer
 * | |   DADDR += DOFF          | |             - DADDR Destination address
 * | | Total transfer is NBYTES | |             - DOFF  Adjustment applied to DADDR after each transfer
 * | +--------------------------+ |             - NBYTES Number of bytes to transfer
 * | +--------------------------+ |<-DMA Req.   - Attributes
 * | | Minor Loop               | |               - ATTR_SSIZE, ATTR_DSIZE Source and destination transfer sizes
 * |..............................|               - ATTR_SMOD, ATTR_DMOD Modulo
 * | |                          | |
 * | +--------------------------+ |             The number of reads and writes done will depend on NBYTES, SSIZE and DSIZE
 * | +--------------------------+ |<-DMA Req.   For example: NBYTES=12, SSIZE=16-bits, DSIZE=32-bits => 6 reads, 3 writes
 * | | Minor Loop               | |             NBYTES must be an even multiple of SSIZE and DSIZE in bytes.
 * | | Each transfer            | |
 * | |   SADDR->DADDR           | |            The following are used by the major loop
 * | |   SADDR += SOFF          | |             - SLAST Adjustment applied to SADDR at the end of each major loop
 * | |   DADDR += DOFF          | |             - DLAST Adjustment applied to DADDR at the end of each major loop
 * | | Total transfer is NBYTES | |             - CITER Major loop counter - counts how many completed major loops
 * | +--------------------------+ |
 * |                              |            SLAST and DLAST may be used to reset the addresses to the initial value or
 * | At end of Major Loop         |            link to the next transfer.
 * |    SADDR += SLAST            |            The total transferred for the entire sequence is CITER x NBYTES.
 * |    DADDR += DLAST            |
 * |                              |            Important options in the CSR:
 * | Total transfer =             |              - DMA_CSR_INTMAJOR = Generate interrupt at end of Major-loop
 * |    CITER*NBYTES              |              - DMA_CSR_DREQ     = Clear hardware request at end of Major-loop
 * +------------------------------+              - DMA_CSR_START    = Start transfer. Used for software transfers. Automatically cleared.
 * @endverbatim
 *
 * Structure to define a DMA transfer
 */
struct __attribute__((__packed__)) DmaTcd {

   uint32_t      SADDR = 0;  //!< :00 Source address
   int16_t       SOFF  = 0;  //!< :04 Source offset
   DmaTcdAttr    ATTR;       //!< :06 Transfer attributes
   uint32_t      NBYTES= 0;  //!< :08 Minor loop byte count
   int32_t       SLAST = 0;  //!< :0C Last source adjustment
   uint32_t      DADDR = 0;  //!< :10 Destination address
   int16_t       DOFF  = 0;  //!< :14 Destination offset
   uint16_t      CITER = 0;  //!< :16 Major loop count
   int32_t       DLAST = 0;  //!< :18 Last destination adjustment
   DmaTcdCsr     CSR;        //!< :1C Control and Status

   /**
    * Default constructor
    */
   constexpr DmaTcd() = default;

   /**
    * Default copy constructor
    */
   constexpr DmaTcd(const DmaTcd &other) = default;

   DmaTcd &operator=(const DmaTcd &other) = default;

   void operator=(const DmaTcd &other) volatile {
         *(DmaTcd *)this = other;
   };

   /**
    * Constructor.
    * This constructor should be used if minor loops are disabled (DmaMinorLoopMapping_Disabled).
    *
    * @param sourceInformation          The following information in this order:
    *        sourceAddress              Starting source address
    *        sourceOffset               Offset to apply to source address after each read
    *        sourceSize                 Source size
    *        lastSourceAdjustment       Source adjustment to apply after completion of the major iteration count
    *        sourceModulo               Source modulo
    *
    * @param destinationInfo            The following information in this order:
    *        destinationAddress         Starting destination address
    *        destinationOffset          Offset to apply to destination address after each read
    *        destinationSize            Destination size
    *        lastDestinationAdjustment  Destination adjustment to apply after completion of the major iteration count
    *        destinationModulo          Destination modulo
    *
    * @param minorLoopByteCount         Number of bytes to be transferred in each minor loop (per service request)
    * @param majorLoopCount             Number of major iterations
    *
    * @param dmaTcdCsr                  CSR option list (in any order, inactive options may be omitted)
    *        dmaStart                   Software start of channel
    *        dmaStopOnComplete          Disable hardware request on major loop completion
    *        dmaIntMajor                Interrupt request on on major loop completion
    *        dmaIntHalf                 Interrupt request on on major loop half completion
    *        dmaSpeed                   Control how channel monopolises the bus
    *        dmaScatterGather           Enable Scatter/Gather operations
    *        dmaMajorLink               Enable linking to another channel
    */
   constexpr DmaTcd(
         DmaInfo   sourceInformation,
         DmaInfo   destinationInfo,

         uint32_t  minorLoopByteCount,
         uint16_t  majorLoopCount,

         DmaTcdCsr dmaTcdCsr
   ) :
      SADDR(sourceInformation.startAddress),
      SOFF(sourceInformation.offset),
      ATTR(sourceInformation.dmaSize, sourceInformation.dmaModulo, destinationInfo.dmaSize, destinationInfo.dmaModulo),
      NBYTES(minorLoopByteCount),
      SLAST(sourceInformation.lastAddressAdjustment),
      DADDR(destinationInfo.startAddress),
      DOFF(destinationInfo.offset),
      CITER(majorLoopCount),
      DLAST(destinationInfo.lastAddressAdjustment),
      CSR(dmaTcdCsr) {
   }

   /**
    * Constructor.
    * This constructor should be used if minor loops are enabled (DmaMinorLoopMapping_Enabled).
    *
    * @param sourceInformation          The following information in this order:
    *        sourceAddress              Starting source address
    *        sourceOffset               Offset to apply to source address after each read
    *        sourceSize                 Source size
    *        lastSourceAdjustment       Source adjustment to apply after completion of the major iteration count
    *        sourceModulo               Source modulo
    *
    * @param destinationInfo            The following information in this order:
    *        destinationAddress         Starting destination address
    *        destinationOffset          Offset to apply to destination address after each read
    *        destinationSize            Destination size
    *        lastDestinationAdjustment  Destination adjustment to apply after completion of the major iteration count
    *        destinationModulo          Destination modulo
    *
    * @param dmaMinorLoopOffsetSelect   Selects if the minor loop offset is applied to source and/or destination
    * @param minorLoopOffset            Offset applied to source and/or destination after minor loop
    * @param minorLoopByteCount         Number of bytes to be transferred in each minor loop (per service request)
    *
    * @param majorLoopCount             Number of major iterations (may include link channel)
    *
    * @param dmaTcdCsr                  CSR option list (in any order, inactive options may be omitted)
    *        dmaStart                   Software start of channel
    *        dmaStopOnComplete          Disable hardware request on major loop completion
    *        dmaIntMajor                Interrupt request on on major loop completion
    *        dmaIntHalf                 Interrupt request on on major loop half completion
    *        dmaSpeed                   Control how channel monopolises the bus
    *        dmaScatterGather           Enable Scatter/Gather operations
    *        dmaMajorLink               Enable linking to another channel
    */
   constexpr DmaTcd(
         DmaInfo   sourceInformation,
         DmaInfo   destinationInfo,

         uint16_t                  minorLoopByteCount,
         DmaMinorLoopOffsetSelect  dmaMinorLoopOffsetSelect,
         int32_t                   minorLoopOffset,

         uint16_t                  majorLoopCount,

         DmaTcdCsr dmaTcdCsr
   ) :
      SADDR(sourceInformation.startAddress),
      SOFF(sourceInformation.offset),
      ATTR(sourceInformation.dmaSize, sourceInformation.dmaModulo, destinationInfo.dmaSize, destinationInfo.dmaModulo),
      NBYTES(dmaNBytes(minorLoopByteCount, dmaMinorLoopOffsetSelect, minorLoopOffset)),
      SLAST(sourceInformation.lastAddressAdjustment),
      DADDR(destinationInfo.startAddress),
      DOFF(destinationInfo.offset),
      CITER(majorLoopCount),
      DLAST(destinationInfo.lastAddressAdjustment),
      CSR(dmaTcdCsr) {
   }
};

/**
 * Creates DMA NBYTES entry for:
 * - Minor loop enabled CR[EMLN] = 1 (DmaMinorLoopMapping_Enabled)
 * - Source/Destination offset disabled TCD.SMLOE=TCD.DMLOE=0
 *
 * In this mode no address adjustments are made at the end of a minor loop
 *
 * @param nBytes 30-bit value for byte count
 *
 * @return TCD.NBYTES entry
 */
constexpr uint32_t dmaNBytes(uint32_t nBytes) {
   return
         usbdm_assert((nBytes<(1<<30)), "nBytes is too large"),
         DMA_NBYTES_MLOFFNO_NBYTES(nBytes);
}

/**
 * Creates DMA CITER entry for:
 * - Channel-to-channel linking disabled TCD.ELINK=0
 *
 * @param citer 15-bit value for major iteration count
 *
 * @return TCD.CITER entry
 */
constexpr uint16_t dmaCiter(uint16_t citer) {
   return
         usbdm_assert((citer<(1<<15)), "CITER is too large"),
         DMA_CITER_ELINKNO_CITER(citer);
}

/**
 * Creates DMA CITER entry for:
 * - Channel-to-channel linking enabled TCD.ELINK=1
 *
 * @param dmaChannelNum Channel to link to
 * @param citer         9-bit value for major iteration count
 *
 * @return TCD.CITER entry
 */
constexpr uint16_t dmaCiter(DmaChannelNum dmaChannelNum, uint16_t citer) {
   return
         usbdm_assert((citer<(1<<9)), "citer is too large"),
         DMA_CITER_ELINKNO_ELINK(1)|DMA_CITER_ELINKYES_LINKCH(dmaChannelNum)|DMA_CITER_ELINKYES_CITER(citer);
}

/**
 * Class representing a DMA controller.
 *
 * @tparam Info Information describing DMA controller
 */
template<class Info>
class DmaBase_T : public Info {

   using MuxInfo = Dmamux0Info;

protected:
   /** Hardware instance pointer */
   static constexpr HardwarePtr<DMA_Type> dma = Info::baseAddress;

   /** Bit-mask of allocated channels */
   static uint32_t allocatedChannels;

   /** Callback to catch unhandled interrupt */
   static void noHandlerCallback(DmaChannelNum, uint32_t) {
      setAndCheckErrorCode(E_NO_HANDLER);
   }

public:
   /**
    * Enable and configure shared DMA settings.
    * This also clears all DMA channels.
    *
    * @param[in] dmaConfig           Main configuration values
    */
   static void configure(const DmaConfig &dmaConfig) {

      // Enable clock to DMAC
      Info::enableClock();

      // Set shared control options
      dma->CR = dmaConfig.cr;

      dma->ERQ  = 0;
      dma->EEI  = 0;
#ifdef DMA_EARS_EDREQ_0_MASK
      dma->EARS = 0;
#endif
      dma->INT  = (uint32_t)-1;
      dma->ERR  = (uint32_t)-1;

      // Clear call-backs
      for (unsigned channel=0; channel<Info::NumVectors; channel++) {
         Info::sCallbacks[channel] = noHandlerCallback;
      }
#ifndef NDEBUG
      // Clear the TCDs
      for (unsigned index=0; index<sizeof(dma->TCD_RAW);index++) {
         dma->TCD_RAW[index] = 0;
      }
#endif
      // Reset record of allocated channels
      allocatedChannels = (1<<Info::NumChannels)-1;
   }

   /**
    * Enable and configure shared DMA settings from settings in Configure.usbdmProject
    * This also clears all DMA channels.
    */
   static void defaultConfigure() {
      configure(Dma0Info::DefaultDmaConfigValue);
   }

   /**
    * Set DMA halt on error
    *
    * @param dmaActionOnError Whether to halt transfer when a DMA error occurs
    */
   void SetActionOnError(DmaActionOnError dmaActionOnError) {
   
      dma->CR = (dma->CR & ~DMA_CR_HOE_MASK)|dmaActionOnError;
   }
   
   /**
    * Set Continuous Link mode
    *
    * @param dmaContinuousLink Whether to enable continuous link mode
    *        If enabled, on minor loop completion, the channel activates again if that
    *        channel has a minor loop channel link enabled and the link channel is itself.
    *        This effectively applies the minor loop offsets and restarts the next minor loop
    */
   void SetLinkMode(DmaContinuousLink dmaContinuousLink) {
   
      dma->CR = (dma->CR & ~DMA_CR_CLM_MASK)|dmaContinuousLink;
   }
   
   /**
    * Set Minor loop mapping
    *
    * @param dmaMinorLoopMapping Whether to enable minor loop mapping
    *        When enabled, TCDn.word2 is redefined to include individual enable fields, an offset field
    *        and the NBYTES field. The individual enable fields allow the minor loop offset to be
    *        applied to the source address, the destination address, or both.
    *        The NBYTES field is reduced when either offset is enabled.
    */
   void SetMinorLoopMapping(DmaMinorLoopMapping dmaMinorLoopMapping) {
   
      dma->CR = (dma->CR & ~DMA_CR_EMLM_MASK)|dmaMinorLoopMapping;
   }
   
   /**
    * Set Channel Arbitration
    *
    * @param dmaArbitration How to arbitrate between requests from different channels
    */
   void SetArbitration(DmaArbitration dmaArbitration) {
   
      dma->CR = (dma->CR & ~DMA_CR_ERCA_MASK)|dmaArbitration;
   }
   
   /**
    * Set Operation in Debug mode
    *
    * @param dmaInDebug Control DMA operation in debug mode
    */
   void SetActionInDebug(DmaInDebug dmaInDebug) {
   
      dma->CR = (dma->CR & ~DMA_CR_EDBG_MASK)|dmaInDebug;
   }
   

   /**
    * Set minor loop mapping modes.
    *
    * @param[in] dmaMinorLoopMapping Whether to enable minor loop mapping
    */
   void setMinorLoopMapping(DmaMinorLoopMapping  dmaMinorLoopMapping) {
      dma->CR = (dma->CR&~(DMA_CR_EMLM(1)))|dmaMinorLoopMapping;
   }

   /**
    * Set error handling
    *
    * @param[in] dmaActionOnError Whether to halt when a DMA error occurs
    */
   void setErrorHandling(DmaActionOnError dmaActionOnError) {
      dma->CR = (dma->CR&~(DMA_CR_HOE(1)))|dmaActionOnError;
   }

   /**
    * Enable and configure shared DMA settings.
    * This also clears all DMA channels.
    *
    * @param[in] dmaContinuousLink       Whether to enable continuous link mode
    */
   void setLinking(DmaContinuousLink dmaContinuousLink) {
      dma->CR = (dma->CR&~(DMA_CR_CLM(1)))|dmaContinuousLink;
   }

   /**
    * Allocate DMA channel.
    *
    * @return DmaChannelNum_None - No suitable channel available.  Error code set.
    * @return Channel number     - Number of allocated channel
    */
   static DmaChannelNum allocateChannel() {
      CriticalSection cs;

      unsigned channelNum;

#if true
      // Try non-PIT channel first
      channelNum = __builtin_ffs(allocatedChannels&~0xF);
      if (channelNum == 0) {
         channelNum = __builtin_ffs(allocatedChannels);
      }
#endif
      if (channelNum == 0) {
         setErrorCode(E_NO_RESOURCE);
         return DmaChannelNum_None;
      }
      channelNum--;
      allocatedChannels &= ~(1<<channelNum);
      return (DmaChannelNum) channelNum;
   }

   /**
    * Allocate Periodic DMA channel associated with given PIT channel.
    * This is a channel that may be throttled by the associated PIT channel.
    *
    * @param pitChannelNum PIT channel being used.
    * @return DmaChannelNum_None - No suitable channel available.  Error code set.
    * @return Channel number     - Number of allocated channel
    */
   static DmaChannelNum allocatePitAssociatedChannel(PitChannelNum pitChannelNum) {
      const uint32_t channelMask = (1<<pitChannelNum);
      usbdm_assert(pitChannelNum<Info::NumChannels,        "No DMA channel associated with PIT channel");
      usbdm_assert((allocatedChannels & channelMask) != 0, "DMA channel already allocated");

      CriticalSection cs;
      if ((allocatedChannels & channelMask) == 0) {
         setErrorCode(E_NO_RESOURCE);
         return DmaChannelNum_None;
      }
      allocatedChannels &= ~channelMask;
      return (DmaChannelNum) pitChannelNum;
   }

#if false
   /**
    * Allocate Periodic DMA channel.
    * This is a channel that may be throttled by an associated LPIT channel.
    *
    * @return Error DmaChannelNum_None - No suitable channel available.  Error code set.
    * @return Channel number           - Number of allocated channel
    */
   static DmaChannelNum allocatePeriodicChannel() {
      CriticalSection cs;
      unsigned channelNum = __builtin_ffs(allocatedChannels);
      if ((channelNum == 0)||(--channelNum>=Info::NumChannels)||(channelNum>=USBDM::Lpit0Info::NumChannels)) {
         setErrorCode(E_NO_RESOURCE);
         return DmaChannelNum_None;
      }
      allocatedChannels &= ~(1<<channelNum);
      return (DmaChannelNum) channelNum;
   }
#endif

#if true
   /**
    * Allocate Periodic DMA channel.
    * This is a channel that may be throttled by an associated PIT channel.
    *
    * @return Error DmaChannelNum_None - No suitable channel available.  Error code set.
    * @return Channel number           - Number of allocated channel
    */
   static DmaChannelNum allocatePeriodicChannel() {
      CriticalSection cs;
      unsigned channelNum = __builtin_ffs(allocatedChannels);
      if ((channelNum == 0)||(--channelNum>=Info::NumChannels)||(channelNum>=USBDM::PitInfo::NumChannels)) {
         setErrorCode(E_NO_RESOURCE);
         return DmaChannelNum_None;
      }
      allocatedChannels &= ~(1<<channelNum);
      return (DmaChannelNum) channelNum;
   }
#endif

   /**
    * Free DMA channel.
    *
    * @param dmaChannelNum Channel to release
    */
   static void freeChannel(DmaChannelNum dmaChannelNum) {
      const uint32_t channelMask = (1<<dmaChannelNum);
      usbdm_assert((allocatedChannels & channelMask) == 0, "Freeing unallocated DMA channel");

      CriticalSection cs;
      allocatedChannels |= channelMask;
   }

   /**
    * Set priority for a DMA channel.
    * This is only used if DmaArbitration_Fixed is used.
    *
    * @param[in] dmaChannelNum      Channel to modify
    * @param[in] priority           Priority for the channel
    * @param[in] dmaCanBePreempted  Controls whether the channel can be suspended by a higher priority channel
    * @param[in] dmaCanPreemptLower Controls whether the channel can suspend a lower priority channel
    *
    * @note The priority of each channel must be a unique number from [0..NumChannel-1]
    */
   static void setChannelPriority(
         DmaChannelNum      dmaChannelNum,
         DmaPriority        dmaPriority,
         DmaCanBePreempted  dmaCanBePreempted=DmaCanBePreempted_Enabled,
         DmaCanPreemptLower dmaCanPreemptLower=DmaCanPreemptLower_Enabled) {

      int index = (dmaChannelNum&0xFC)|(3-(dmaChannelNum&0x03));
      constexpr volatile uint8_t *priorities = &dma->DCHPRI3;
      priorities[index] = dmaCanBePreempted|dmaCanPreemptLower|dmaPriority;
   }

   /**
    * Configure channel for arbitrary transfer defined by DmaTcd.
    *
    * @param[in] dmaChannelNum DMA channel number
    * @param[in] tcd           DMA TCD block describing the transfer
    */
   static void configureTransfer(DmaChannelNum dmaChannelNum, const DmaTcd &tcd) {

      dma->TCD[dmaChannelNum].SADDR           = tcd.SADDR;
      dma->TCD[dmaChannelNum].SOFF            = tcd.SOFF;
      dma->TCD[dmaChannelNum].ATTR            = tcd.ATTR.data;
      dma->TCD[dmaChannelNum].NBYTES_MLNO     = tcd.NBYTES;
      dma->TCD[dmaChannelNum].SLAST           = tcd.SLAST;
      dma->TCD[dmaChannelNum].DADDR           = tcd.DADDR;
      dma->TCD[dmaChannelNum].DOFF            = tcd.DOFF;
      dma->TCD[dmaChannelNum].CITER_ELINKNO   = tcd.CITER;
      dma->TCD[dmaChannelNum].DLASTSGA        = tcd.DLAST;
      dma->TCD[dmaChannelNum].CSR             = 0;
      dma->TCD[dmaChannelNum].CSR             = tcd.CSR;
      dma->TCD[dmaChannelNum].BITER_ELINKNO   = tcd.CITER;
   }

   /**
    * Get status of last transfer error.
    *
    * @return 32-bit flag register see DMA_ES definitions
    */
   static uint32_t __attribute__((always_inline)) getLastError() {
      return dma->ES;
   }

   /**
    * Waits until the channel indicates the transaction has completed.
    *
    * @param[in] dmaChannelNum DMA channel number
    */
   static void waitUntilComplete(DmaChannelNum dmaChannelNum) {

      int lastCiter = dma->TCD[dmaChannelNum].CITER_ELINKNO;
      while ((dma->TCD[dmaChannelNum].CSR & DMA_CSR_DONE_MASK) == 0) {
         int currentCiter = dma->TCD[dmaChannelNum].CITER_ELINKNO;
         if (lastCiter != currentCiter) {
            lastCiter = currentCiter;
            __asm__ volatile("nop");
         }
      }
   }

   /**
    * Initiate a DMA software request on a channel.
    *
    * The channel should be configured beforehand using configureTransfer().\n
    * This is a convenience function that allows re-use of already configured channel.\n
    * All it does is set the START bit in the existing TCD.\n
    * There is no need to use this function for a single request as the START bit may be set\n
    * in the original TCD used with configureTransfer().
    *
    * @param[in] dmaChannelNum Channel being modified
    *
    * @note There is no clear option as the flag is automatically cleared by the DMA controller when
    *        the transfer starts.
    */
   static void __attribute__((always_inline)) startSoftwareRequest(DmaChannelNum dmaChannelNum) {

      dma->SSRT = dmaChannelNum;
   }

   /**
    * Enable/disable DMA hardware requests on multiple channels.
    * The channel should be configured beforehand using configureTransfer().
    *
    * @param[in]  dmaChannelMask Mask for channels being modified
    * @param[in]  enable         True => enable, False => disable
    */
   static void __attribute__((always_inline)) enableMultipleRequests(uint32_t dmaChannelMask, bool enable=true) {

      usbdm_assert((dmaChannelMask&~((1<<Info::NumChannels)-1)) != 0, "Illegal DMA channel");

      if (enable) {
         dma->ERQ = dma->ERQ | dmaChannelMask;
      }
      else {
         dma->ERQ = dma->ERQ & ~dmaChannelMask;
      }
   }

   /**
    * Enable/disable DMA hardware requests on a channel.
    * The channel should be configured beforehand using configureTransfer().
    *
    * @param[in]  dmaChannelNum  Channel being modified
    * @param[in]  enable         True => enable, False => disable
    *
    * @note May use DmaChannelNum_All to apply to all channels
    */
   static void __attribute__((always_inline)) enableRequests(DmaChannelNum dmaChannelNum, bool enable=true) {

      if (enable) {
         dma->SERQ = dmaChannelNum;
      }
      else {
         dma->CERQ = dmaChannelNum;
      }
   }

#ifdef DMA_EARS_EDREQ_0_MASK
   /**
    * Enable/disable DMA asynchronous requests on a channel\n
    * The channel should be configured beforehand using configureTransfer()
    *
    * @param[in]  dmaChannelNum Channel being modified
    * @param[in]  enable        True => enable, False => disable
    */
   static void __attribute__((always_inline)) enableAsynchronousRequests(DmaChannelNum dmaChannelNum, bool enable=true) {

      if (enable) {
         dma->EARS = dma->EARS | (1<<dmaChannelNum);
      }
      else {
         dma->EARS = dma->EARS & ~(1<<dmaChannelNum);
      }
   }
#endif

   /**
    * Enable/disable error interrupts on multiple channels.
    *
    * @param[in]  dmaChannelMask Mask for channels being modified
    * @param[in]  enable         True => enable, False => disable
    */
   static void __attribute__((always_inline)) enableMultipleErrorInterrupts(uint32_t dmaChannelMask, bool enable=true) {

      usbdm_assert((dmaChannelMask&~((1<<Info::NumChannels)-1)) == 0, "Illegal DMA channel");

      if (enable) {
         dma->EEI = dma->EEI | dmaChannelMask;
      }
      else {
         dma->EEI = dma->EEI & ~dmaChannelMask;
      }
   }

   /**
    * Enable/disable error interrupts for a channel.
    *
    * @param[in]  dmaChannelNum Channel being modified
    * @param[in]  enable        True => enable, False => disable
    *
    * @note May use DmaChannelNum_All to apply to all channels
    */
   static void __attribute__((always_inline)) enableErrorInterrupts(DmaChannelNum dmaChannelNum, bool enable=true) {

      if (enable) {
         dma->SEEI = dmaChannelNum;
      }
      else {
         dma->CEEI = dmaChannelNum;
      }
   }

   /**
    * Clear done flag for a channel.
    *
    * @param[in]  dmaChannelNum  Channel being modified
    *
    * @note May use DmaChannelNum_All to apply to all channels
    */
   static void __attribute__((always_inline)) clearDoneFlag(DmaChannelNum dmaChannelNum) {

      dma->CDNE = dmaChannelNum;
   }

   /**
    * Clear interrupt request flags on multiple channels.
    *
    * @param[in]  dmaChannelMask Mask for channels being modified
    * @param[in]  enable         True => enable, False => disable
    */
   static void __attribute__((always_inline)) clearMultipleInterruptRequests(uint32_t dmaChannelMask, bool enable=true) {

      usbdm_assert((dmaChannelMask&~((1<<Info::NumChannels)-1)) != 0, "Illegal DMA channel");

      if (enable) {
         dma->INT = dma->INT | dmaChannelMask;
      }
      else {
         dma->INT = dma->INT & ~dmaChannelMask;
      }
   }

   /**
    * Clear interrupt request flag for a channel.
    *
    * @param[in]  dmaChannelNum  Channel being modified
    *
    * @note May use DmaChannelNum_All to apply to all channels
    */
   static void __attribute__((always_inline)) clearInterruptRequest(DmaChannelNum dmaChannelNum) {

      dma->CINT = dmaChannelNum;
   }

};

/** Bit-mask of allocated channels */
template<class Info> uint32_t DmaBase_T<Info>::allocatedChannels = (1<<Info::NumChannels)-1;



/**
 * End DMA_Group
 * @}
 */
#endif
} // End namespace USBDM

#endif /* INCLUDE_USBDM_DMA_H_ */
