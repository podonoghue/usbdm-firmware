/**
 * @file    tsi.h (180.ARM_Peripherals/Project_Headers/tsi-MK.h)
 * @brief   Touch Sense Interface
 *
 * @date     13 April 2016
 *      Author: podonoghue
 */

#ifndef TSI_MK_H_
#define TSI_MK_H_
/*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include <math.h>
#include "pin_mapping.h"

namespace USBDM {

/**
 * @addtogroup TSI_Group TSI, Touch Sense Interface
 * @brief C++ Class allowing access to Touch Sense Interface
 * @{
 */



/**
 * Type definition for TSI interrupt call back
 *
 *  @param[in]  status - Interrupt flags e.g. TSI_GENCS_EOSF_MASK, TSI_GENCS_OVRF_MASK, TSI_GENCS_EXTERF_MASK
 */
typedef void (*TSICallbackFunction)(uint32_t status);

/**
 * Interface for Touch Sense Module (TSI)
 *
 * Notes:
 *  The TSI operates in active mode for MCU in Run, Wait, Stop, VLPRun, VLPWait and VLPStop.
 *  The TSI operates on low-power mode for MCU in low-leakage modes LLS, VLLSx.
 *  Only a single electrode is scanned (PEN.LPSP) for low-power mode.
 */
template <class Info>
class TsiBase_T : public Info {

protected:
   /**
    * Used to suppress error messages that are already checked
    * by static assertions in a more meaningful manner.
    *
    * @return Unchanged validate channel number or 0
    */
   static constexpr int limitElectrode(int electrode) {
      return (electrode>=Info::numSignals)?0:electrode;
   }

   /**
    * Class to static check electrodeNum exists and is mapped to a pin
    *
    * @tparam electrodeNum Electrode number to check
    */
   template<int electrodeNum> class CheckPinExistsAndIsMapped {
      // Tests are chained so only a single assertion can fail so as to reduce noise

      // Out of bounds value for function index
      static constexpr bool Test1 = (electrodeNum>=0) && (electrodeNum<(Info::numSignals));
      // Function is not currently mapped to a pin
      static constexpr bool Test2 = !Test1 || (Info::info[electrodeNum].pinIndex != PinIndex::UNMAPPED_PCR);
      // Non-existent function and catch-all. (should be INVALID_PCR)
      static constexpr bool Test3 = !Test1 || !Test2 || (Info::info[electrodeNum].pinIndex >= PinIndex::MIN_PIN_INDEX);

      static_assert(Test1, "Illegal TSI Input - Check Configure.usbdm for available inputs");
      static_assert(Test2, "TSI input is not mapped to a pin - Modify Configure.usbdm");
      static_assert(Test3, "TSI input doesn't exist in this device/package - Check Configure.usbdm for available input pins");

   public:
      /** Dummy function to allow convenient in-line checking */
      static constexpr void check() {}
   };

   /** Callback function for ISR */
   static TSICallbackFunction sCallback;

public:
   /**
    * Set Callback function
    *
    *  @param[in]  callback  Callback function to be executed on interrupt.\n
    *                        Use nullptr to remove callback.
    */
   static void setCallback(TSICallbackFunction callback) {
      static_assert(Info::irqHandlerInstalled, "TSI not configure for interrupts");
      if (callback == nullptr) {
         callback = TsiBase_T<Info>::unhandledCallback;
      }
      sCallback = callback;
   }

public:
   /** Pointer to hardware */
   static constexpr HardwarePtr<TSI_Type> tsi = Info::baseAddress;

   /**
    * Handler for unexpected interrupts i.e. handler not installed.
    */
   static void unhandledCallback(uint32_t) {
      setAndCheckErrorCode(E_NO_HANDLER);
   }

   // /TSI/classInfo not found
   // /TSI/InitMethod not found
   /**
    * Set the electrode scan configuration for active mode.\n
    * This controls the count interval of the internal oscillator when sampling a pin:\n
    *   T = (Prescaler*consecutiveScans)/Felectrode\n
    * The reference oscillator is counted for the above interval to get the electrode measurement.
    *
    * tsiElectrodePrescaler Electrode oscillator prescaler.
    * consecutiveScans      The number of consecutive scans of an electrode
    */
   static void setScans(TsiElectrodePrescaler tsiElectrodePrescaler, int consecutiveScans) {

      tsi->GENCS = (tsi->GENCS&~(TSI_GENCS_PS_MASK|TSI_GENCS_NSCN_MASK))|tsiElectrodePrescaler|TSI_GENCS_NSCN(consecutiveScans-1);
   }
   /**
    * Set clock source, clock divider and modulus for active mode
    *
    * @param[in] tsiClockSource  Clock source
    * @param[in] tsiClockDivider Clock divider
    * @param[in] scanModulus     Scan modulus (0=continuous, otherwise scan period)
    */
   static void setClock(
         TsiClockSource  tsiClockSource   = TsiClockSource_LpoClk,
         TsiClockDivider tsiClockDivider  = TsiClockDivider_DivBy128,
         uint16_t        scanModulus      = 0 ) {

      tsi->SCANC = (tsi->SCANC&~(TSI_SCANC_AMCLKS_MASK|TSI_SCANC_AMPSC_MASK|TSI_SCANC_SMOD_MASK))|
            TSI_SCANC_SMOD(scanModulus)|tsiClockSource|tsiClockDivider;
   }
   /**
    * Sets how often the inputs are scanned.
    *
    * @param[in] period The period of scanning.
    *
    * @note This period must be larger than the total measurement time as determined by other parameters e.g.
    *  - Electrode oscillation period (electrode/finger capacitance, electrode current)
    *  - Electrode oscillator divider and consecutive scans settings
    *  - Number of electrodes scanned.
    */
   static ErrorCode setSamplePeriod(float period) {
      float inputClock = Info::getInputClockFrequency();
      int prescaleFactor=1;
      int prescalerValue=0;

      // Maximum period value in ticks
      uint32_t maxPeriodInTicks = 255;

      while (prescalerValue<=7) {
         float    clock = inputClock/prescaleFactor;
         uint32_t periodInTicks = round(period*clock);
         if (periodInTicks <= maxPeriodInTicks) {
            tsi->SCANC = (tsi->SCANC&~(TSI_SCANC_AMPSC_MASK|TSI_SCANC_SMOD_MASK))|
                  TSI_SCANC_AMPSC(prescalerValue)|TSI_SCANC_SMOD(periodInTicks);
            return E_NO_ERROR;
         }
         prescalerValue++;
         prescaleFactor <<= 1;
      }
      // Too long a period
      return setErrorCode(E_TOO_LARGE);
   }
   /**
    * Set mode, clock source and scan interval for low power mode
    *
    * @param[in] tsiStopMode             Selects if the module operates in low power modes
    * @param[in] tsiLowPowerScanInterval Scan interval in low power modes
    * @param[in] tsiLowPowerClockSource  Clock source in low power modes
    */
   static void setLowPowerClock(
         TsiStopMode             tsiStopMode,
         TsiLowPowerScanInterval tsiLowPowerScanInterval = TsiLowPowerScanInterval_500ms,
         TsiLowPowerClockSource  tsiLowPowerClockSource  = TsiLowPowerClockSource_LpoClk) {

      tsi->GENCS = (tsi->GENCS&~(TSI_GENCS_STPE_MASK|TSI_GENCS_LPCLKS_MASK|TSI_GENCS_LPSCNITV_MASK))|
            tsiStopMode|tsiLowPowerScanInterval|tsiLowPowerClockSource;
   }
   /**
    * Set reference and external charge currents
    *
    * @param[in] referenceCharge Reference charge current (uA in 2uA steps)
    * @param[in] externalCharge  External oscillator charge current (uA in 2uA steps)
    */
   static void setCurrents(uint16_t referenceCharge=16, uint16_t externalCharge=16) {

      tsi->SCANC = (tsi->SCANC&~(TSI_SCANC_REFCHRG_MASK|TSI_SCANC_EXTCHRG_MASK))|
            TSI_SCANC_REFCHRG((referenceCharge+1)/2)|TSI_SCANC_EXTCHRG((externalCharge+1)/2);
   }
   /**
    * Select inputs to be scanned
    *
    * @param[in] activeChannels  Bit-mask for channels in use in active mode
    * @param[in] lowpowerChannel Channel used in low-power mode
    */
   static void selectInputs(uint16_t activeChannels, int lowpowerChannel) {

      TSI0->PEN = TSI_PEN_PEN(activeChannels)|TSI_PEN_LPSP(lowpowerChannel);
   }
   /**
    * Set low-power channel thresholds
    *
    * @param[in] low  Low threshold
    * @param[in] high High threshold
    */
   static void setLowPowerThresholds(uint16_t low, int high) {

      TSI0->THRESHOLD = TSI_THRESHOLD_HTHH(high)|TSI_THRESHOLD_LTHH(low);
   }
   /**
    * Configure touch sensing interrupts
    *
    * @param[in] tsiEventSource     Selects End-of-Scan or Out-of-Range interrupt behaviour
    * @param[in] tsiErrorInterrupt  Selects error interrupt behaviour
    */
   static void enableTsiInterrupts(
         TsiEventSource    tsiEventSource,
         TsiErrorAction    tsiErrorAction = TsiErrorAction_None) {
      tsi->GENCS = (tsi->GENCS&~(TSI_GENCS_TSIIE_MASK|TSI_GENCS_ERIE_MASK))|tsiEventSource|tsiErrorAction;
   }

   /**
    * Get channel count value
    *
    * @param[in]  channel Channel number
    *
    * @return 16-bit count value
    */
   static uint16_t getCount(int channel) {
      return Info::tsi->CNTR[channel];
   }

   /**
    * Enable TSI and start scan
    *
    * @param tsiScanMode Controls if a single software scan is initiated or periodic scans
    */
   static void startScan(TsiScanMode tsiScanMode) {
      // Disable module so changes have effect
      // This also helps with errata e4181
      tsi->GENCS = tsi->GENCS & ~(TSI_GENCS_TSIEN_MASK|TSI_GENCS_SWTS_MASK);

      // Select Hardware/Software mode
      tsi->GENCS = tsi->GENCS | (tsiScanMode&TSI_GENCS_STM_MASK);

      // Enable
      tsi->GENCS = tsi->GENCS | TSI_GENCS_TSIEN_MASK;

      // Clear flags and start scan
      tsi->GENCS = tsi->GENCS |
            tsiScanMode|            // Software/Hardware mode + optional software trigger
            TSI_GENCS_EOSF_MASK|    // Clear flags
            TSI_GENCS_OUTRGF_MASK|
            TSI_GENCS_EXTERF_MASK|
            TSI_GENCS_OVRF_MASK;
   }

   /**
    * Start software scan and wait for completion\n
    *
    * @note This routine will hang if interrupts are used as the flags will be automatically
    *       cleared by the stub ISR
    */
   static ErrorCode startScanAndWait() {
      // Clear flags and start scan
      startScan(TsiScanMode_Triggered);

      // Wait for complete flag or error
      while ((tsi->GENCS&(TSI_GENCS_EOSF_MASK|TSI_GENCS_OUTRGF_MASK|TSI_GENCS_EXTERF_MASK|TSI_GENCS_OVRF_MASK)) == 0) {
      }

      return (tsi->GENCS&(TSI_GENCS_OUTRGF_MASK|TSI_GENCS_EXTERF_MASK|TSI_GENCS_OVRF_MASK))?E_ERROR:E_NO_ERROR;
   }

   /**
    * Class representing a TSI input pin
    *
    * @tparam tsiChannel Number of TSI electrode (input) to configure
    */
   template<TsiInput tsiChannel>
   class Pin {

   private:
      // Check if electrode mapped to pin
      static CheckPinExistsAndIsMapped<tsiChannel> check;

      // PCR for pin associated with electrode
      using Pcr = PcrTable_T<Info, limitElectrode(tsiChannel)>;

   public:
      /// TSI input number
      static constexpr TsiInput         TSI_INPUT          = tsiChannel;

      /// TSI input as low-power input number
      static constexpr TsiLowPowerInput TSI_LOWPOWER_INPUT = TsiLowPowerInput(TSI_PEN_LPSP(tsiChannel));

      /**
       * Configure the pin associated with a TSI input electrode.
       */
      static void setInput() {
         // Configure associated pin as analogue input
         Pcr::setPCR();
      }

      /**
       * Get channel count value
       *
       * @return 16-bit count value
       */
      static uint16_t getCount() {
         return Info::tsi->CNTR[tsiChannel];
      }
   };

};

template<class Info> TSICallbackFunction TsiBase_T<Info>::sCallback = TsiBase_T<Info>::unhandledCallback;


/**
 * End TSI_Group
 * @}
 */

} // End namespace USBDM

#endif /* TSI_MK_H_ */
