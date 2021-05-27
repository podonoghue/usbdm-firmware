/**
 * @file     llwu.h (180.ARM_Peripherals/Project_Headers/llwu.h)
 * @brief    Low leakage wake-up unit
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */

#ifndef HEADER_LLWU_H
#define HEADER_LLWU_H
 /*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include "hardware.h"

namespace USBDM {

/**
 * @addtogroup LLWU_Group LLWU, Low-leakage Wake-up Unit
 * @brief Abstraction for Low-leakage Wake-up Unit
 * @{
 */

/**
 * Pin filter numbers
 *
 * @note These are used as an index into the FILT table so numbers do NOT correspond to filter names FILT[0] <=> FILT1 etc
 */
enum LlwuFilterNum {
   LlwuFilterNum_0, //!< FILT1
#ifdef LLWU_FILT2_FILTE_MASK
   LlwuFilterNum_1, //!< FILT2
#endif
#ifdef LLWU_FILT3_FILTE_MASK
   LlwuFilterNum_2, //!< FILT3
#endif
#ifdef LLWU_FILT4_FILTE_MASK
   LlwuFilterNum_3, //!< FILT4
#endif
};

/**
 * LLWU pin wake-up mode
 */
enum LlwuPinMode {
   LlwuPinMode_Disabled    = LLWU_PE1_WUPE0(0)|LLWU_PE1_WUPE1(0)|LLWU_PE1_WUPE2(0)|LLWU_PE1_WUPE3(0), //!< Wake-up by pin change disabled
   LlwuPinMode_RisingEdge  = LLWU_PE1_WUPE0(1)|LLWU_PE1_WUPE1(1)|LLWU_PE1_WUPE2(1)|LLWU_PE1_WUPE3(1), //!< Wake-up on pin rising edge
   LlwuPinMode_FallingEdge = LLWU_PE1_WUPE0(2)|LLWU_PE1_WUPE1(2)|LLWU_PE1_WUPE2(2)|LLWU_PE1_WUPE3(2), //!< Wake-up on pin falling edge
   LlwuPinMode_EitherEdge  = LLWU_PE1_WUPE0(3)|LLWU_PE1_WUPE1(3)|LLWU_PE1_WUPE2(3)|LLWU_PE1_WUPE3(3), //!< Wake-up on pin rising or falling edge
};

/**
 * LLWU peripheral wake-up mode
 */
enum LlwuPeripheralMode {
   LlwuPeripheralMode_Disabled = false, //!< Wake-up by peripheral disabled
   LlwuPeripheralMode_Enabled  = true,  //!< Wake-up by peripheral enabled
};

/**
 * LLWU pin sources
 */
enum LlwuPin : uint32_t {
   LlwuPin_0            =   0,  //!< Wake-up pin LLWU_P0 
   LlwuPin_1            =   1,  //!< Wake-up pin LLWU_P1 
   LlwuPin_2            =   2,  //!< Wake-up pin LLWU_P2 
   LlwuPin_3            =   3,  //!< Wake-up pin LLWU_P3 
#ifdef LLWU_PE2_WUPE4_MASK
   LlwuPin_4            =   4,  //!< Wake-up pin LLWU_P4 
   LlwuPin_5            =   5,  //!< Wake-up pin LLWU_P5 
   LlwuPin_6            =   6,  //!< Wake-up pin LLWU_P6 
   LlwuPin_7            =   7,  //!< Wake-up pin LLWU_P7 
#endif
#ifdef LLWU_PE3_WUPE8_MASK
   LlwuPin_8            =   8,  //!< Wake-up pin LLWU_P8 
   LlwuPin_9            =   9,  //!< Wake-up pin LLWU_P9 
   LlwuPin_10           =  10,  //!< Wake-up pin LLWU_P10
   LlwuPin_11           =  11,  //!< Wake-up pin LLWU_P11
#endif
#ifdef LLWU_PE4_WUPE12_MASK
   LlwuPin_12           =  12,  //!< Wake-up pin LLWU_P12
   LlwuPin_13           =  13,  //!< Wake-up pin LLWU_P13
   LlwuPin_14           =  14,  //!< Wake-up pin LLWU_P14
   LlwuPin_15           =  15,  //!< Wake-up pin LLWU_P15
#endif
#ifdef LLWU_PE5_WUPE16_MASK
   LlwuPin_16           =  16,  //!< Wake-up pin LLWU_P16
   LlwuPin_17           =  17,  //!< Wake-up pin LLWU_P17
   LlwuPin_18           =  18,  //!< Wake-up pin LLWU_P18
   LlwuPin_19           =  19,  //!< Wake-up pin LLWU_P19
#endif
#ifdef LLWU_PE6_WUPE20_MASK
   LlwuPin_20           =  20,  //!< Wake-up pin LLWU_P20
   LlwuPin_21           =  21,  //!< Wake-up pin LLWU_P21
   LlwuPin_22           =  22,  //!< Wake-up pin LLWU_P22
   LlwuPin_23           =  23,  //!< Wake-up pin LLWU_P23
#endif
#ifdef LLWU_PE7_WUPE24_MASK
   LlwuPin_24           =  24,  //!< Wake-up pin LLWU_P24
   LlwuPin_25           =  25,  //!< Wake-up pin LLWU_P25
   LlwuPin_26           =  26,  //!< Wake-up pin LLWU_P26
   LlwuPin_27           =  27,  //!< Wake-up pin LLWU_P27
#endif
#ifdef LLWU_PE8_WUPE28_MASK
   LlwuPin_28           =  28,  //!< Wake-up pin LLWU_P28
   LlwuPin_29           =  29,  //!< Wake-up pin LLWU_P29
   LlwuPin_30           =  30,  //!< Wake-up pin LLWU_P30
   LlwuPin_31           =  31,  //!< Wake-up pin LLWU_P31
#endif

   // Mapped pins

};

/**
 * LLWU peripheral sources
 */
enum LlwuPeripheral : uint32_t {
   LlwuPeripheral_0               = (1<<0), //!< Wake-up peripheral LLWU_M0IF
   LlwuPeripheral_1               = (1<<1), //!< Wake-up peripheral LLWU_M1IF
   LlwuPeripheral_2               = (1<<2), //!< Wake-up peripheral LLWU_M2IF
   LlwuPeripheral_3               = (1<<3), //!< Wake-up peripheral LLWU_M3IF
   LlwuPeripheral_4               = (1<<4), //!< Wake-up peripheral LLWU_M4IF
   LlwuPeripheral_5               = (1<<5), //!< Wake-up peripheral LLWU_M5IF
   LlwuPeripheral_6               = (1<<6), //!< Wake-up peripheral LLWU_M6IF
   LlwuPeripheral_7               = (1<<7), //!< Wake-up peripheral LLWU_M7IF

   // Connected peripherals
   LlwuPeripheral_Lptmr           = LlwuPeripheral_0, //!< LPTMR wake-up
   LlwuPeripheral_Cmp0            = LlwuPeripheral_1, //!< CMP0 wake-up
   LlwuPeripheral_Tsi0            = LlwuPeripheral_4, //!< TSI0 wake-up
   LlwuPeripheral_RtcAlarm        = LlwuPeripheral_5, //!< RTC Alarm wake-up
   LlwuPeripheral_RtcSeconds      = LlwuPeripheral_7, //!< RTC Seconds wake-up

};

/**
 * LLWU pin wake-up mode
 */
enum LlwuFilterPinMode {
   LlwuFilterPinMode_Disabled    = LLWU_FILT_FILTE(0), //!< Wake-up by pin change disabled
   LlwuFilterPinMode_RisingEdge  = LLWU_FILT_FILTE(1), //!< Wake-up on pin rising edge
   LlwuFilterPinMode_FallingEdge = LLWU_FILT_FILTE(2), //!< Wake-up on pin falling edge
   LlwuFilterPinMode_EitherEdge  = LLWU_FILT_FILTE(3), //!< Wake-up on pin rising or falling edge
};

#ifdef LLWU_RST_LLRSTE
/**
 * Controls Low-Leakage Mode RESET Enable
 */
enum LlwuResetWakeup {
   LlwuResetWakeup_Disabled = LLWU_RST_LLRSTE(0), //!< Wake-up by Reset disabled
   LlwuResetWakeup_Enabled  = LLWU_RST_LLRSTE(1), //!< Wake-up by Reset enabled
};

/**
 * Controls Digital Filter On RESET Pin
 */
enum LlwuResetFilter {
   LlwuResetFilter_Disabled = LLWU_RST_RSTFILT(0), //!< Reset filter disabled
   LlwuResetFilter_Enabled  = LLWU_RST_RSTFILT(1), //!< Reset filter enabled
};
#endif

/**
 * Type definition for LLWU interrupt call back
 */
typedef void (*LLWUCallbackFunction)();

/**
 * Template class providing interface to Low Leakage Wake-up Unit
 *
 * @tparam info      Information class for LLWU
 *
 * @code
 * using llwu = LlwuBase_T<LlwuInfo>;
 *
 *  llwu::configure();
 *
 * @endcode
 */
template <class Info>
class LlwuBase_T {

protected:
   /** Class to static check channel pin mapping is valid */
   template<LlwuPin llwuPin> class CheckSignal {
      static_assert((llwuPin<Info::numSignals), "Non-existent LLWU Input - Modify Configure.usbdm");
      static_assert((llwuPin>=Info::numSignals)||(Info::info[llwuPin].gpioBit != UNMAPPED_PCR), "LLWU Input is not mapped to a pin - Modify Configure.usbdm");
      static_assert((llwuPin>=Info::numSignals)||(Info::info[llwuPin].gpioBit != INVALID_PCR),  "LLWU Input doesn't exist in this device/package - Modify Configure.usbdm");
      static_assert((llwuPin>=Info::numSignals)||((Info::info[llwuPin].gpioBit == UNMAPPED_PCR)||(Info::info[llwuPin].gpioBit == INVALID_PCR)||(Info::info[llwuPin].gpioBit >= 0)), "Illegal LLWU Input - Modify Configure.usbdm");
   public:
      /** Dummy function to allow convenient in-line checking */
      static constexpr void check() {}
   };

   /** Callback function for ISR */
   static LLWUCallbackFunction sCallback;

   /** Callback to catch unhandled interrupt */
   static void unhandledCallback() {
      setAndCheckErrorCode(E_NO_HANDLER);
   }

public:
   /**
    * IRQ handler
    */
   static void irqHandler(void) {
      sCallback();
   }

   /**
    * Set Callback function
    *
    *   @param[in]  callback Callback function to be executed on interrupt\n
    *                        Use nullptr to remove callback.
    */
   static void setCallback(LLWUCallbackFunction callback) {
      static_assert(Info::irqHandlerInstalled, "LLWU not configured for interrupts");
      if (callback == nullptr) {
         callback = unhandledCallback;
      }
      sCallback = callback;
   }

protected:
   /** Pointer to hardware */
   static __attribute__((always_inline)) volatile LLWU_Type &llwu() { return Info::llwu(); }

public:

   /**
    * Configures all mapped pins associated with this peripheral
    */
   static void configureAllPins() {
      // Configure pins
      Info::initPCRs();
   }

   /**
    * Basic enable of LLWU\n
    * Includes configuring all pins
    */
   static void enable() {
      configureAllPins();
   }

   /**
    * Configure with settings from Configure.usbdmProject.
    */
   static void defaultConfigure() {

      // Configure pins
      Info::initPCRs();

      llwu().PE1   = Info::pe1;
#ifdef LLWU_PE2_WUPE4_MASK
      llwu().PE2   = Info::pe2;
#endif
#ifdef LLWU_PE3_WUPE8_MASK
      llwu().PE3   = Info::pe3;
#endif
#ifdef LLWU_PE4_WUPE12_MASK
      llwu().PE4   = Info::pe4;
#endif
#ifdef LLWU_PE5_WUPE16_MASK
      llwu().PE5   = Info::pe5;
#endif
#ifdef LLWU_PE6_WUPE20_MASK
      llwu().PE6   = Info::pe6;
#endif
#ifdef LLWU_PE7_WUPE24_MASK
      llwu().PE7   = Info::pe7;
#endif
#ifdef LLWU_PE8_WUPE28_MASK
      llwu().PE8   = Info::pe8;
#endif

      llwu().ME    = Info::me;

      llwu().FILT1 = Info::filt1|LLWU_FILT_FILTF_MASK;
#ifdef LLWU_FILT2_FILTE_MASK
      llwu().FILT2 = Info::filt2|LLWU_FILT_FILTF_MASK;
#endif
#ifdef LLWU_FILT3_FILTE_MASK
      llwu().FILT3 = Info::filt3|LLWU_FILT_FILTF_MASK;
#endif
#ifdef LLWU_FILT4_FILTE_MASK
      llwu().FILT4 = Info::filt4|LLWU_FILT_FILTF_MASK;
#endif

#ifdef LLWU_RST_LLRSTE
      llwu().RST   = Info::rst;
#endif

      enableNvicInterrupts(Info::irqLevel);
   }

   /*
    * ***************************************************
    * Wake-up pins
    * ***************************************************
    */
   /**
    * Configure pin as wake-up source
    *
    * @param[in] llwuPin       Pin to configure
    * @param[in] llwuPinMode   Mode for pin as wake-up input
    */
   static void configurePinSource(
         LlwuPin     llwuPin,
         LlwuPinMode llwuPinMode) {

      static const uint8_t masks[] = {(0x3<<0),(0x3<<2),(0x3<<4),(0x3<<6)};
      volatile uint8_t &llwuPe = llwu().PE[llwuPin>>2];
      uint8_t mask = masks[llwuPin&3];
      llwuPe = (llwuPe&~mask) | (llwuPinMode&mask);
   }

   /**
    * Get bit mask indicating wake-up pin sources\n
    * The pin masks correspond to Pin sources.
    *
    * @code
    *    if ((pinWakeupSource&(1<<LlwuPin_ptc1)) != 0) {
    *       // Wakeup from LPTMR
    *    }
    * @endcode
    *
    * @return Bit mask
    */
   static uint32_t getPinWakeupSources() {
      constexpr unsigned PF_SIZE = sizeof(llwu().PF)/sizeof(llwu().PF[0]);
      if constexpr(PF_SIZE==4) {
         return (llwu().PF[1]<<24)|(llwu().PF[1]<<16)|(llwu().PF[1]<<8)|llwu().PF[0];
      }
      else if constexpr(PF_SIZE==3) {
         return (llwu().PF[1]<<16)|(llwu().PF[1]<<8)|llwu().PF[0];
      }
      else if constexpr(PF_SIZE==2) {
         return (llwu().PF[1]<<8)|llwu().PF[0];
      }
      else {
         return llwu().PF[0];
      }
   }

   /**
    * Check if pin is source of wake-up
    *
    * @param[in] llwuPin  Pin to check
    *
    * @return false Given pin is not source of wake-up.
    * @return true  Given pin is source of wake-up.
    */
   static bool isPinWakeupSource(LlwuPin llwuPin) {
      return getPinWakeupSources() & (1<<llwuPin);
   }

   /**
    * Clear wake-up pin flag
    *
    *  @param[in] llwuPin Pin indicating which flag to clear
    */
   static void clearPinWakeupFlag(LlwuPin llwuPin) {
      llwu().PF[llwuPin>>3] = (1<<(llwuPin&0x7));
   }

   /**
    * Clear all wake-up flags\n
    * Peripherals sources are not cleared since they are controlled by the peripheral
    */
   static void clearAllFlags() {
      clearPinWakeupFlags();
      clearFilteredPinWakeupFlags();
   }

   /**
    * Clear all wake-up pin flags
    */
   static void clearPinWakeupFlags() {
      for(unsigned index=0; index<(sizeof(llwu().PF)/sizeof(llwu().PF[0])); index++) {
         llwu().PF[index] = 0xFF;
      }
   }

   /*
    * ***************************************************
    * Wake-up filtered pins
    * ***************************************************
    */
   /**
    * Configure one of the input pins as a filtered wake-up source
    *
    * @param[in] filterNum          Filter to configure - number available depends on device
    * @param[in] llwuPin            Pin to assign to filter
    * @param[in] llwuFilterPinMode  Mode for pin as wake-up input
    *
    * @note Filtering is bypassed in VLLS0
    */
   static ErrorCode configureFilteredPinSource(
         LlwuFilterNum     filterNum,
         LlwuPin           llwuPin,
         LlwuFilterPinMode llwuFilterPinMode) {

      llwu().FILT[filterNum] = llwuPin|llwuFilterPinMode;
      return E_NO_ERROR;
   }

   /**
    * Check if filtered wake-up pin is source of wake-up
    *
    * @param[in] filterNum Pin Filter to check
    *
    * @return false Given filtered pin is not source of wake-up.
    * @return true  Given filtered pin is source of wake-up.
    */
   static bool isFilteredPinWakeupSource(unsigned filterNum) {
      return (llwu().FILT[filterNum] & LLWU_FILT_FILTF_MASK);
   }

   /**
    * Clear filtered wake-up pin flag
    *
    * @param[in] filterNum Pin Filter to clear flag
    */
   static void clearFilteredPinWakeupFlag(LlwuFilterNum filterNum) {
      llwu().FILT[filterNum] |= LLWU_FILT_FILTF_MASK;
   }

   /**
    * Clear all filtered wake-up pin flags
    */
   static void clearFilteredPinWakeupFlags() {
      for (unsigned index=0; index<(sizeof(llwu().FILT)/sizeof(llwu().FILT[0])); index++) {
         llwu().FILT[index] |= LLWU_FILT_FILTF_MASK;
      }
   }

#ifdef LLWU_RST_LLRSTE
   /**
    * Controls Reset wake-up control
    *
    * @param llwuResetFilter  Whether filtering is applied to reset pin
    * @param llwuResetWakeup  Whether reset is enabled as a wake-up source
    */
   static void configureResetFilter(LlwuResetFilter llwuResetFilter, LlwuResetWakeup llwuResetWakeup=LlwuResetWakeup_Enabled) {
      llwu().RST = llwuResetFilter|llwuResetWakeup;
   }
#endif

   /*
    * ***************************************************
    * Wake-up peripherals
    * ***************************************************
    */
   /**
    * Configure peripheral as wake-up source
    *
    * @param[in] llwuPeripheral     Peripheral to configure
    * @param[in] llwuPeripheralMode Whether to enable peripheral as wake-up source
    */
   static void configurePeripheralSource(
         LlwuPeripheral       llwuPeripheral,
         LlwuPeripheralMode   llwuPeripheralMode=LlwuPeripheralMode_Enabled) {

      if (llwuPeripheralMode) {
         llwu().ME |= llwuPeripheral;
      }
      else {
         llwu().ME &= (uint8_t)~llwuPeripheral;
      }
   }

   /**
    * Disable all wake-up sources (pins and peripherals)
    */
   static void disableAllSources() {
      for (unsigned index=0; index<(sizeof(llwu().PE)/(sizeof(llwu().PE[0]))); index++) {
         llwu().PE[index] = 0;
      }
      llwu().ME  = 0;
   }

   /**
    * Get flag bit mask indicating wake-up peripheral sources\n
    * The mask returned correspond to (multiple) peripheral sources.\n
    * These flags are cleared through the originating peripheral.
    *
    *
    * Example checking source
    * @code
    *    if ((peripheralWakeupSource&LlwuPeripheral_Lptmr) != 0) {
    *       // Wakeup from LPTMR
    *    }
    * @endcode
    *
    * @return Bit mask
    */
   static uint32_t getPeripheralWakeupSources() {
      return llwu().MF;
   }

   /**
    *  Check if peripheral is source of wake-up\n
    *  These flags are cleared through the originating peripheral.
    *
    * @param[in] llwuPeripheral  Peripheral to check
    *
    * @return false Given peripheral is not source of wake-up.
    * @return true  Given peripheral is source of wake-up.
    */
   static bool isPeripheralWakeupSource(LlwuPeripheral llwuPeripheral) {
      return llwu().MF & llwuPeripheral;
   }

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

   template<LlwuPin llwuPin>
   class Pin {

   private:
      // Checks pin mapping is valid
      LlwuBase_T::CheckSignal<llwuPin> check;

      using Pcr = PcrTable_T<Info, llwuPin>;

   public:
      static constexpr LlwuPin  pin = llwuPin;

      static constexpr int      PORT_BITNUM  = Pcr::BITNUM;    ///< Bit number for bit within associated port
      static constexpr uint32_t PORT_BITMASK = Pcr::BITMASK;   ///< Bit mask for bit within associated port

      /**
       * Set Pin Control Register (PCR) value \n
       * This will map the pin to the LLWU function (mux value) \n
       * The clock to the port will be enabled before changing the PCR
       *
       * @tparam llwuPin LLWU pin to configure e.g. LlwuPin_Pte1
       *
       * @param[in]  pinPull          One of PinPull_None, PinPull_Up, PinPull_Down
       * @param[in]  pinAction        One of PinAction_None, etc (defaults to PinAction_None)
       * @param[in]  pinFilter        One of PinFilter_None, PinFilter_Passive (defaults to PinFilter_None)
       */
      static void setInput(
            PinPull           pinPull           = PinPull_None,
            PinAction         pinAction         = PinAction_None,
            PinFilter         pinFilter         = PinFilter_None
            ) {
         Pcr::setPCR(pinPull|pinAction|pinFilter|(Info::info[llwuPin].pcrValue&PORT_PCR_MUX_MASK));
      }

      /**
       * Clear pin interrupt flag.
       * Assumes clock to the port has already been enabled.
       */
      static void clearInterruptFlag() {
         Pcr::clearInterruptFlag();
      }

      /**
       * Set callback for Pin interrupts
       *
       * @param[in] pinCallback The function to call on Pin interrupt. \n
       *                        nullptr to indicate none
       *
       * @return E_NO_ERROR            No error
       * @return E_HANDLER_ALREADY_SET Handler already set
       *
       * @note There is a single callback function for all pins on the related port.
       *       It is necessary to identify the originating pin in the callback
       */
      static void setCallback(PinCallbackFunction pinCallback) {
         Pcr::setCallback(pinCallback);
      }

      /**
       * Enable Pin interrupts in NVIC
       */
      static void enablePinNvicInterrupts() {
         Pcr::enableNvicInterrupts();
      }

      /**
       * Enable and set priority of Pin interrupts in NVIC
       * Any pending NVIC interrupts are first cleared.
       *
       * @param[in]  nvicPriority  Interrupt priority
       */
      static void enablePinNvicInterrupts(uint32_t nvicPriority) {
         Pcr::enableNvicInterrupts(nvicPriority);
      }

      /**
       * Disable Pin interrupts in NVIC
       */
      static void disablePinNvicInterrupts() {
         Pcr::disableNvicInterrupts();
      }
   };
};

template<class Info> LLWUCallbackFunction LlwuBase_T<Info>::sCallback = LlwuBase_T<Info>::unhandledCallback;

#ifdef USBDM_LLWU_IS_DEFINED
/**
 * Class representing LLWU
 */
class Llwu : public LlwuBase_T<LlwuInfo> {};

#endif

/**
 * End LLWU_Group
 * @}
 */

} // End namespace USBDM

#endif /* HEADER_LLWU_H */
