/**
 * @file     dac.h (180.ARM_Peripherals/Project_Headers/dac.h)
 * @brief    Abstraction layer for DAC interface
 *
 * @version  V4.12.1.240
 * @date     28/10/2018
 */

#ifndef HEADERS_DAC_H_
#define HEADERS_DAC_H_
 /*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include "derivative.h"
#include "hardware.h"

namespace USBDM {

/**
 * DAC Reference voltage Select
 */
enum DacReferenceSelect {
   DacReferenceSelect_Vrefh   = DAC_C0_DACRFS(0), //!< DACREF_0 Usually Vrefh or VrefOut if present.
   DacReferenceSelect_VrefOut = DAC_C0_DACRFS(0), //!< DACREF_0 Usually Vrefh or VrefOut if present.
   DacReferenceSelect_Vdda    = DAC_C0_DACRFS(1), //!< DACREF_1 Usually Vdda as the reference voltage.
   DacReferenceSelect_0       = DAC_C0_DACRFS(0), //!< Selects DACREF_0 as the reference voltage.
   DacReferenceSelect_1       = DAC_C0_DACRFS(1), //!< Selects DACREF_1 as the reference voltage.
};

/**
 * DAC Trigger Select
 */
enum DacTriggerSelect {
   DacTriggerSelect_Hardware = DAC_C0_DACTRGSEL(0), //!< The DAC hardware trigger is selected.
   DacTriggerSelect_Software = DAC_C0_DACTRGSEL(1), //!< The DAC software trigger is selected.
};

/**
 * DAC Power control
 */
enum DacPower {
   DacPower_High = DAC_C0_LPEN(0), //!< Low-Power mode
   DacPower_Low  = DAC_C0_LPEN(1), //!< High-Power mode
};

#ifdef DAC_C0_DACBWIEN
/**
 * DAC Buffer Watermark Interrupt Enable.
 * Control whether an interrupt is generated when SR.DACBFWMF is set i.e.
 * when the DAC buffer read pointer has reached the watermark level.
 */
enum DacWatermarkIrq {
   DacWatermarkIrq_Disabled = DAC_C0_DACBWIEN(0), //!< The DAC buffer watermark interrupt is disabled.
   DacWatermarkIrq_Enabled  = DAC_C0_DACBWIEN(1), //!< The DAC buffer watermark interrupt is enabled.
};
#endif

/**
 * DAC Buffer Read Pointer Top Flag Interrupt Enable.
 * Control whether an interrupt is generated when SR.DACBFRPTF is set i.e.
 * when the DAC buffer read pointer is zero.
 */
enum DacTopFlagIrq {
   DacTopFlagIrq_Disabled = DAC_C0_DACBTIEN(0), //!< The DAC buffer read pointer top flag interrupt is disabled.
   DacTopFlagIrq_Enabled  = DAC_C0_DACBTIEN(1), //!< The DAC buffer read pointer top flag interrupt is enabled.
};

/**
 * DAC Buffer Read Pointer Bottom Flag Interrupt Enable.
 * Control whether an interrupt is generated when SR.DACBFRPBF is set.
 * when the DAC buffer read pointer is equal to buffer upper limit (C2.DACBFUP).
 */
enum DacBottomFlagIrq {
   DacBottomFlagIrq_Disabled = DAC_C0_DACBBIEN(0), //!< The DAC buffer read pointer bottom flag interrupt is disabled.
   DacBottomFlagIrq_Enabled  = DAC_C0_DACBBIEN(1), //!< The DAC buffer read pointer bottom flag interrupt is enabled.
};

/**
 * DAC DMA control
 */
enum DacDma {
   DacDma_Disabled = DAC_C1_DMAEN(0),  //!< Disable DMA
   DacDma_Enabled  = DAC_C1_DMAEN(1),  //!< Enable DMA
};

#if DAC_C1_DACBFMD_MASK == 0x6
/**
 * If disabled then the first word of the buffer is used to control the DAC output level.\n
 * If enabled then the word that the read pointer points to is used.\n
 * The read pointer may be changed by hardware (Trigger = PIT0 usually) or software and advances
 * in normal or scan modes.
 * In FIFO mode the buffer as a FIFO with write pointer.
 */
enum DacBufferMode {
   DacBufferMode_Disabled  = DAC_C1_DACBFEN(0),                   //!< Read pointer disabled. The first word of the buffer is used.
   DacBufferMode_Normal    = DAC_C1_DACBFEN(1)|DAC_C1_DACBFMD(0), //!< Read pointer enabled. Read pointer points advances as circular buffer.
   DacBufferMode_Swing     = DAC_C1_DACBFEN(1)|DAC_C1_DACBFMD(1), //!< Read pointer enabled. Read pointer points advances and retreats.
   DacBufferMode_Scan      = DAC_C1_DACBFEN(1)|DAC_C1_DACBFMD(2), //!< Read pointer enabled. Read pointer points advances once only.
   DacBufferMode_Fifo      = DAC_C1_DACBFEN(1)|DAC_C1_DACBFMD(3), //!< Buffer acts as a FIFO. (If supported)
};
#else
/**
 * If disabled then the first word of the buffer is used to control the DAC output level.\n
 * If enabled then the word that the read pointer points to is used.\n
 * The read pointer may be changed by hardware (Trigger = PIT0 usually) or software and advances
 * in normal or scan modes.
 */
enum DacBufferMode {
   DacBufferMode_Disabled  = DAC_C1_DACBFEN(0),                   //!< Read pointer disabled. The first word of the buffer is used.
   DacBufferMode_Normal    = DAC_C1_DACBFEN(1)|DAC_C1_DACBFMD(0), //!< Read pointer enabled. Read pointer points advances as circular buffer.
   DacBufferMode_Scan      = DAC_C1_DACBFEN(1)|DAC_C1_DACBFMD(1), //!< Read pointer enabled. Read pointer points advances once only.
};
#endif

#ifdef DAC_C1_DACBFWM
/**
 * DAC Buffer Watermark Select.
 * Controls when SR[DACBFWMF] is set.
 *
 * Normal Mode:
 *   SR[DACBFWMF] will be set when the DAC buffer read pointer reaches this many words away
 *   from the upper limit (DACBUP). This allows user configuration of the watermark interrupt.
 *
 * FIFO mode:
 *   SR[DACBFWMF] will be set when there is a threshold number of entries left in the FIFO.
 */
enum DacWaterMark {
   DacWaterMark_Normal1       = DAC_C1_DACBFWM(0),  //!< Normal mode: Read pointer 1 entry away from upper limit
   DacWaterMark_Normal2       = DAC_C1_DACBFWM(0),  //!< Normal mode: Read pointer 2 entries away from upper limit
   DacWaterMark_Normal3       = DAC_C1_DACBFWM(0),  //!< Normal mode: Read pointer 3 entries away from upper limit
   DacWaterMark_Normal4       = DAC_C1_DACBFWM(0),  //!< Normal mode: Read pointer 4 entries away from upper limit
   DacWaterMark_Fifo2         = DAC_C1_DACBFWM(0),  //!< FIFO mode: Threshold <= 2 remaining FIFO entries
   DacWaterMark_FifoMaxDiv4   = DAC_C1_DACBFWM(0),  //!< FIFO mode: Threshold <= Max/4 remaining FIFO entries
   DacWaterMark_FifoMaxDiv2   = DAC_C1_DACBFWM(0),  //!< FIFO mode: Threshold <= Max/2 remaining FIFO entries
   DacWaterMark_FifoMaxMinus2 = DAC_C1_DACBFWM(0),  //!< FIFO mode: Threshold <= Max-2 remaining FIFO entries
};
#endif // DAC_C1_DACBFWM

/**
 * DAC status value as individual flags
 */
union DacStatusValue {
   uint8_t raw;
   struct {
      bool     readPointerBottomFlag:1;
      bool     readPointerTopFlag:1;
#ifdef DAC_C0_DACBWIEN
      bool     watermarkFlag:1;
#endif
   };
   constexpr DacStatusValue(uint8_t value) : raw(value) {
   }
};

/**
 * @addtogroup DAC_Group DAC, Digital-to-Analogue Converter
 * @brief Pins used for Digital-to-Analogue Converter
 * @{
 */
/**
 * Type definition for DAC interrupt call back
 */
typedef void (*DACCallbackFunction)(DacStatusValue);

/**
 * Template class representing a Digital to Analogue Converter
 *
 * @tparam info      Information class for DAC
 *
 * @code
 * using dac = Dac_T<Dac0Info>;
 *
 *  dac::configure();
 *
 * @endcode
 */
template<class Info>
class Dac_T {

protected:
   /** Class to static check outputNum is mapped to a pin */
   template<int outputNum> class CheckPinMapping {
      static_assert((outputNum>=Info::numSignals)||(Info::info[outputNum].gpioBit != UNMAPPED_PCR),
            "DAC output is not mapped to a pin - Modify Configure.usbdm");
   public:
      /** Dummy function to allow convenient in-line checking */
      static constexpr void check() {}
   };

   /** Class to static check valid outputNum - it does not check that it is mapped to a pin */
   template<int outputNum> class CheckOutput {
      static_assert((outputNum<Info::numSignals),
            "Non-existent DAC output  - Check Configure.usbdm for available outputs");
      static_assert((outputNum>=Info::numSignals)||(Info::info[outputNum].gpioBit != INVALID_PCR),
            "DAC output  doesn't exist in this device/package - Check Configure.usbdm for available outputs");
      static_assert((outputNum>=Info::numSignals)||((Info::info[outputNum].gpioBit == UNMAPPED_PCR)||(Info::info[outputNum].gpioBit == INVALID_PCR)||(Info::info[outputNum].gpioBit >= 0)),
            "Illegal DAC output - Check Configure.usbdm for available outputs");
   public:
      /** Dummy function to allow convenient in-line checking */
      static constexpr void check() {}
   };

//   CheckPinMapping<0> check;

   /**
    * Callback to catch unhandled interrupt
    */
   static void unhandledCallback(DacStatusValue) {
      setAndCheckErrorCode(E_NO_HANDLER);
   }

   /** Callback function for ISR */
   static DACCallbackFunction sCallback;

public:
   /**
    * Hardware instance pointer
    *
    * @return Reference to CMT hardware
    */
   static __attribute__((always_inline)) volatile DAC_Type &dac() { return Info::dac(); }

   /** Get base address of DAC hardware as uint32_t */
   static constexpr uint32_t dacBase() { return Info::baseAddress; }

   /** Get base address of DAC.DATA register as uint32_t */
   static constexpr uint32_t dacData() { return dacBase() + offsetof(DAC_Type, DATA[0]); }

   /** Get base address of DAC.DATA[index] register as uint32_t */
   static constexpr uint32_t dacData(unsigned index) { return dacBase() + offsetof(DAC_Type, DATA[index]); }

   /**
    * IRQ handler
    */
   static void irqHandler() {
      // Call handler
      sCallback(getAndClearStatus());
   }

   /**
    * Set callback function
    *
    * @param[in] callback Callback function to execute on interrupt.\n
    *                     Use nullptr to remove callback.
    */
   static void setCallback(DACCallbackFunction callback) {
      static_assert(Info::irqHandlerInstalled, "DAC not configured for interrupts");
      if (callback == nullptr) {
         callback = unhandledCallback;
      }
      sCallback = callback;
   }

   /**
    * Configures all mapped pins associated with this peripheral
    */
   static void __attribute__((always_inline)) configureAllPins() {
      // Configure pins
      Info::initPCRs();
   }

   /**
    * Basic enable DAC.
    * Includes enabling clock and configuring all pins of mapPinsOnEnable is selected on configuration
    */
   static void enable() {
      if (Info::mapPinsOnEnable) {
         configureAllPins();
      }

      // Enable clock to DAC interface
      Info::enableClock();
   }

   /**
    * Enable with default settings\n
    * Includes configuring all pins
    */
   static void defaultConfigure() {
      enable();

      // Initialise hardware
      dac().C0 = Info::c0|DAC_C0_DACEN_MASK;
      dac().C1 = Info::c1;
      dac().C2 = Info::c2;
   }

   /**
    * Configure DAC.
    * Interrupts are initially disabled.
    *
    * @param dacReferenceSelect     Reference Select
    * @param dacPower               Power control
    * @param dacTriggerSelect       Trigger Select
    */
   static void configure(
         DacReferenceSelect dacReferenceSelect = DacReferenceSelect_Vdda,
         DacPower           dacPower           = DacPower_Low,
         DacTriggerSelect   dacTriggerSelect   = DacTriggerSelect_Software) {

      enable();
      dac().C0 = DAC_C0_DACEN_MASK|dacReferenceSelect|dacTriggerSelect|dacPower;
   }

#ifdef DAC_C1_DACBFWM
   /**
    *  Configure DAC buffer operation
    *
    * @param dacBufferMode  Select if buffer is used and how the buffer pointer changes.
    * @param dacWaterMark   Selects water mark level for buffer.
    */
   static void configureBuffer(
         DacBufferMode dacBufferMode  = DacBufferMode_Disabled,
         DacWaterMark  dacWaterMark   = DacWaterMark_Normal1
          ) {
      dac().C1 =
            (dac().C1&~(DAC_C1_DACBFEN_MASK|DAC_C1_DACBFMD_MASK|DAC_C1_DACBFWM_MASK))|
            dacBufferMode|dacWaterMark;
   }
#else
   /**
    *  Configure DAC buffer operation
    *
    * @param dacBufferMode  Select if buffer is used and how the buffer pointer changes.
    */
   static void configureBuffer(
         DacBufferMode dacBufferMode  = DacBufferMode_Disabled
          ) {
      dac().C1 =
            (dac().C1&~(DAC_C1_DACBFEN_MASK|DAC_C1_DACBFMD_MASK))|
            dacBufferMode;
   }
#endif

   /**
    * Get output range of DAC
    *
    * @return Range of DAC e.g. 2^12-1
    */
   static constexpr unsigned getRange() {
      return DAC_DATA_DATA_MASK;
   }

   /**
    * Get size of ADC buffer
    *
    * @return size in entries
    */
   static constexpr unsigned getBufferSize() {
      return sizeof(dac().DATA)/sizeof(dac().DATA[0]);
   }

   /**
    * Used to modify the FIFO read and write pointers in FIFO mode.
    *
    * @param writePtr  Write pointer
    * @param readPtr   Read pointer
    */
   static void setFifoPointers(uint8_t writePtr, uint8_t readPtr) {
      usbdm_assert(readPtr<getBufferSize(), "Illegal read pointer");
      usbdm_assert(writePtr<getBufferSize(),"Illegal write pointer");
      dac().C2 = DAC_C2_DACBFRP(readPtr)|DAC_C2_DACBFUP(writePtr);
   }

   /**
    * Clear (empty) FIFO.
    */
   static void clearFifo() {
      dac().C2 = DAC_C2_DACBFRP(0)|DAC_C2_DACBFUP(0);
   }

   /**
    * Set buffer upper limit in buffered modes.
    *
    * @param limit Upper limit for buffer index (inclusive)
    */
   static void setBufferLimit(uint8_t limit) {
      usbdm_assert(limit<getBufferSize(),"Illegal limit value");
      dac().C2 = (dac().C2&~DAC_C2_DACBFUP_MASK)|DAC_C2_DACBFUP(limit);
   }

   /**
    * Set buffer write index in buffered modes.
    *
    * @param index Write index (0..N)
    */
   static void setBufferWritePointer(uint8_t index) {
      usbdm_assert(index<getBufferSize(),"Illegal write index");
      dac().C2 = (dac().C2&~DAC_C2_DACBFRP_MASK)|DAC_C2_DACBFRP(index);
   }

   /**
    * Enable DMA mode
    */
   static void enableDma() {
      dac().C1 |= DAC_C1_DMAEN_MASK;
   }

   /**
    * Disable DMA mode
    */
   static void disableDma() {
      dac().C1 &= ~DAC_C1_DMAEN_MASK;
   }

   /**
    * Do a software trigger on the DAC.
    * If DAC software trigger is selected and buffer is enabled then
    * the buffer read pointer will be advanced once.
    */
   static void softwareTrigger() {
      dac().C0 |= DAC_C0_DACSWTRG_MASK;
   }

#ifdef DAC_C0_DACBWIEN_MASK
   /**
    * Enable or disable interrupts.
    * The flags are cleared first.
    *
    * @param dacTopFlagIrq       Interrupt Enable for buffer read pointer is zero.
    * @param dacBottomFlagIrq    Interrupt Enable for buffer read pointer is equal to buffer limit C2.DACBFUP.
    * @param dacWatermarkIrq     Interrupt Enable for buffer read pointer has reached the watermark level.
    */
   static void enableInterrupts(
         DacTopFlagIrq      dacTopFlagIrq,
         DacBottomFlagIrq   dacBottomFlagIrq,
         DacWatermarkIrq    dacWatermarkIrq   = DacWatermarkIrq_Disabled) {

      // Clear flags
      dac().SR = DAC_SR_DACBFRPBF_MASK|DAC_SR_DACBFRPTF_MASK|DAC_SR_DACBFWMF_MASK;

      // Enable/disable flags
      dac().C0 = (dac().C0&~(DAC_C0_DACBWIEN_MASK|DAC_C0_DACBTIEN_MASK|DAC_C0_DACBBIEN_MASK)) |
            dacTopFlagIrq|dacBottomFlagIrq|dacWatermarkIrq;
   }
#else
   /**
    * Enable or disable interrupts.
    * The flags are cleared first.
    *
    * @param dacTopFlagIrq       Buffer Read Pointer Top Flag Interrupt Enable
    * @param dacBottomFlagIrq    Buffer Read Pointer Bottom Flag Interrupt Enable
    */
   static void enableInterrupts(
         DacTopFlagIrq      dacTopFlagIrq,
         DacBottomFlagIrq   dacBottomFlagIrq) {

      // Clear flags
      dac().SR = 0;

      // Enable/disable flags
      dac().C0 = (dac().C0&~(DAC_C0_DACBTIEN_MASK|DAC_C0_DACBBIEN_MASK)) |
            dacTopFlagIrq|dacBottomFlagIrq;
   }
#endif

   /**
    * Enable interrupts in NVIC
    */
   static void enableNvicInterrupts() {
      NVIC_EnableIRQ(Info::irqNums[0]);
   }

   /**
    * Enable and set priority of interrupts in NVIC
    * Any pending NVIC interrupts are first cleared.
    *
    * @param[in]  nvicPriority  Interrupt priority
    */
   static void enableNvicInterrupts(uint32_t nvicPriority) {
      enableNvicInterrupt(Info::irqNums[0], nvicPriority);
   }

   /**
    * Disable interrupts in NVIC
    */
   static void disableNvicInterrupts() {
      NVIC_DisableIRQ(Info::irqNums[0]);
   }

   /**
    * Enable DAC output pin as output.
    * Configures all Pin Control Register (PCR) values
    */
   static void setOutput() {
      CheckOutput<0>::check();
      using Pcr = PcrTable_T<Info, 0>;

      // Enable and map pin to CMP_OUT
      Pcr::setPCR(Info::info[0].pcrValue&PORT_PCR_MUX_MASK);
   }

   /**
    * Get DAC status
    *
    * @return DAC status value see DacStatusValue
    */
   static DacStatusValue getStatus() {
      return (DacStatusValue)dac().SR;
   }

   /**
    * Get and clear DAC status
    *
    * @return DAC status value see DacStatusValue
    */
   static DacStatusValue getAndClearStatus() {
      // Get status
      uint8_t status = dac().SR;
      // Clear set flags
      dac().SR = ~status;
      // return original status
      return (DacStatusValue)status;
   }
   /**
    *   Disable the DAC
    */
   static void finalise() {
      // Enable timer
      dac().C0 = 0;
      dac().C1 = 0;
      Info::disableClock();
   }
   /**
    * Write output value for non-buffered mode or write value to FIFO for FIFO mode.
    *
    * @param value 12-bit value to write to DAC or FIFO
    */
   static void writeValue(uint16_t value) {
      dac().DATA[0] = DAC_DATA_DATA(value);
   }

   /**
    * Write DAC value to buffer (for buffered non-FIFO modes).
    *
    * @param index Index value for output buffer
    * @param value 12-bit value to write to DAC output buffer
    */
   static void writeValue(unsigned index, uint16_t value) {
      usbdm_assert(index<getBufferSize(), "Buffer index out of range");
      dac().DATA[index] = DAC_DATA_DATA(value);
   }

};

/**
 * Callback table for programmatically set handlers
 */
template<class Info> DACCallbackFunction Dac_T<Info>::sCallback =  Dac_T<Info>::unhandledCallback;

#if defined(USBDM_DAC0_IS_DEFINED)
using Dac0 = Dac_T<Dac0Info>;
#endif

#if defined(USBDM_DAC1_IS_DEFINED)
using Dac1 = Dac_T<Dac1Info>;
#endif
/**
 * @}
 */
} // End namespace USBDM

#endif /* HEADERS_DAC_H_ */
