/*
 *  clock-MKL0x.c
 *
 *  Used for MKL02, MKL04, MKL05
 *
 *  Created on: 04/03/2012
 *      Author: podonoghue
 */
#include "string.h"
#include "derivative.h" /* include peripheral declarations */
#include "system.h"
#include "clock_configure.h"
#include "utilities.h"
#include "stdbool.h"

// Some MCUs call OSC0 just OSC
#ifndef OSC0
#define OSC0 OSC
#endif

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK;   // Hz
uint32_t SystemBusClock  = SYSTEM_BUS_CLOCK; // Hz

/*! @brief Sets up the clock out of RESET
 *
 */
void clock_initialise(void) {

#if (CLOCK_MODE == CLOCK_MODE_NONE)
   // No clock setup
#else
   // XTAL/EXTAL Pins
   SIM->SCGC5  |= SIM_SCGC5_PORTA_MASK;
   PORTA->PCR[3]  = PORT_PCR_MUX(0);
   PORTA->PCR[4]  = PORT_PCR_MUX(0);

   // Configure the Crystal Oscillator
   OSC0->CR = OSC_CR_ERCLKEN_M|OSC_CR_EREFSTEN_M|OSC_CR_SCP_M;

   // Fast Internal Clock divider
   MCG->SC = MCG_SC_FCRDIV_M;

   // Out of reset MCG is in FEI mode
   // =============================================================

   SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV2(7) | SIM_CLKDIV1_OUTDIV3(3) | SIM_CLKDIV1_OUTDIV4(7);

   // Switch from FEI -> FEI/FBI/FEE/FBE
   // =============================================================

   // Set up crystal or external clock source
   MCG->C2 =
            MCG_C2_LOCRE0_M     | // LOCRE0 = 0,1   -> Loss of clock reset enable
            MCG_C2_RANGE0_M     | // RANGE0 = 0,1,2 -> Oscillator low/high/very high clock range
            MCG_C2_HGO0_M       | // HGO0   = 0,1   -> Oscillator low power/high gain
            MCG_C2_EREFS0_M     | // EREFS0 = 0,1   -> Select external clock/crystal oscillator
            MCG_C2_IRCS_M;        // IRCS   = 0,1   -> Select slow/fast internal clock for internal reference

#if ((CLOCK_MODE == CLOCK_MODE_FEI) || (CLOCK_MODE == CLOCK_MODE_FBI) || (CLOCK_MODE == CLOCK_MODE_BLPI) )
   // Transition via FBI
   //=====================================
#define BYPASS (1) // CLKS value used while FLL locks
   MCG->C1 =  MCG_C1_CLKS(BYPASS)     | // CLKS     = X     -> External reference source while PLL locks
              MCG_C1_FRDIV_M          | // FRDIV    = N     -> XTAL/2^n ~ 31.25 kHz
              MCG_C1_IREFS_M          | // IREFS    = 0,1   -> External/Slow IRC for FLL source
              MCG_C1_IRCLKEN_M        | // IRCLKEN  = 0,1   -> IRCLK disable/enable
              MCG_C1_IREFSTEN_M;        // IREFSTEN = 0,1   -> Internal reference enabled in STOP mode

   // Wait for S_IREFST to indicate FLL Reference has switched
   do {
      __asm__("nop");
   } while ((MCG->S & MCG_S_IREFST_MASK) != (MCG_C1_IREFS_V<<MCG_S_IREFST_SHIFT));

   // Wait for S_CLKST to indicating that OUTCLK has switched to bypass PLL/FLL
   do {
      __asm__("nop");
   } while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(BYPASS));

   // Set FLL Parameters
   MCG->C4 = (MCG->C4&~(MCG_C4_DMX32_MASK|MCG_C4_DRST_DRS_MASK))|MCG_C4_DMX32_M|MCG_C4_DRST_DRS_M;
#endif

#if ((CLOCK_MODE == CLOCK_MODE_FBE) || (CLOCK_MODE == CLOCK_MODE_FEE) || (CLOCK_MODE == CLOCK_MODE_PLBE) || (CLOCK_MODE == CLOCK_MODE_PBE) || (CLOCK_MODE == CLOCK_MODE_PEE))

   // Transition via FBE
   //=====================================
#define BYPASS (2) // CLKS value used while PLL locks
   MCG->C1 =  MCG_C1_CLKS(BYPASS)     | // CLKS     = 2     -> External reference source while PLL locks
              MCG_C1_FRDIV_M          | // FRDIV    = N     -> XTAL/2^n ~ 31.25 kHz
              MCG_C1_IREFS_M          | // IREFS    = 0,1   -> External/Slow IRC for FLL source
              MCG_C1_IRCLKEN_M        | // IRCLKEN  = 0,1   -> IRCLK disable/enable
              MCG_C1_IREFSTEN_M;        // IREFSTEN = 0,1   -> Internal reference enabled in STOP mode

#if (MCG_C2_EREFS_V != 0)
   // Wait for oscillator stable (if used)
   do {
      __asm__("nop");
   } while ((MCG->S & MCG_S_OSCINIT0_MASK) == 0);
#endif

   // Wait for S_IREFST to indicate FLL Reference has switched
   do {
      __asm__("nop");
   } while ((MCG->S & MCG_S_IREFST_MASK) != (MCG_C1_IREFS_V<<MCG_S_IREFST_SHIFT));

   // Wait for S_CLKST to indicating that OUTCLK has switched to bypass PLL/FLL
   do {
      __asm__("nop");
   } while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(BYPASS));

   // Set FLL Parameters
   MCG->C4 = (MCG->C4&~(MCG_C4_DMX32_MASK|MCG_C4_DRST_DRS_MASK))|MCG_C4_DMX32_M|MCG_C4_DRST_DRS_M;
#endif

   // Select FEI/FBI/FEE/FBE clock mode
   MCG->C1 =  MCG_C1_CLKS_M       | // CLKS     = 0,1,2 -> Select FLL/IRCSCLK/ERCLK
              MCG_C1_FRDIV_M      | // FRDIV    = N     -> XTAL/2^n ~ 31.25 kHz
              MCG_C1_IREFS_M      | // IREFS    = 0,1   -> External/Slow IRC for FLL source
              MCG_C1_IRCLKEN_M    | // IRCLKEN  = 0,1   -> IRCLK disable/enable
              MCG_C1_IREFSTEN_M;    // IREFSTEN = 0,1   -> Internal reference enabled in STOP mode

   // Wait for mode change
   do {
      __asm__("nop");
   } while ((MCG->S & MCG_S_IREFST_MASK) != (MCG_C1_IREFS_V<<MCG_S_IREFST_SHIFT));

#if defined (MCG_C6_PLLS_V) && (MCG_C1_CLKS_V == 0) // FLL or PLL
#define MCG_S_CLKST_M MCG_S_CLKST(MCG_C6_PLLS_V?3:0)
#else
   #define MCG_S_CLKST_M MCG_S_CLKST(MCG_C1_CLKS_V)
#endif

   // Wait for S_CLKST to indicating that OUTCLK has switched
   do {
      __asm__("nop");
   } while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST_M);

   // Set the SIM _CLKDIV dividers
   SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1_M | SIM_CLKDIV1_OUTDIV2_M | SIM_CLKDIV1_OUTDIV3_M | SIM_CLKDIV1_OUTDIV4_M;

#if (CLOCK_MODE == CLOCK_MODE_BLPE) || (CLOCK_MODE == CLOCK_MODE_BLPI)
   // Select BLPE/BLPI clock mode
   MCG->C2 =
            MCG_C2_LOCRE0_M      | // LOCRE0 = 0,1   -> Loss of clock reset
            MCG_C2_RANGE0_M      | // RANGE0 = 0,1,2 -> Oscillator low/high/very high clock range
            MCG_C2_HGO0_M        | // HGO0   = 0,1   -> Oscillator low power/high gain
            MCG_C2_EREFS0_M      | // EREFS0 = 0,1   -> Select external clock/crystal oscillator
            MCG_C2_LP_M          | // LP     = 0,1   -> Select FLL enabled/disabled in bypass mode
            MCG_C2_IRCS_M;         // IRCS   = 0,1   -> Select slow/fast internal clock for internal reference

#endif // (CLOCK_MODE == CLOCK_MODE_BLPE) || (CLOCK_MODE == CLOCK_MODE_BLPI)
#endif // (CLOCK_MODE == CLOCK_MODE_NONE)


   /*!
    * SOPT1 Clock multiplexing
    */
#if defined(SIM_SOPT1_OSC32KSEL_MASK) && defined(SIM_SOPT1_OSC32KSEL_M) // ERCLK32K source
   SIM->SOPT1 = (SIM->SOPT1&~SIM_SOPT1_OSC32KSEL_MASK)|SIM_SOPT1_OSC32KSEL_M;
#endif

   /*!
    * SOPT2 Clock multiplexing
    */
#if defined(SIM_SOPT2_SDHCSRC_MASK) && defined(SIM_SOPT2_SDHCSRC_M) // SDHC clock
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_SDHCSRC_MASK)|SIM_SOPT2_SDHCSRC_M;
#endif

#if defined(SIM_SOPT2_TIMESRC_MASK) && defined(SIM_SOPT2_TIMESRC_M) // Ethernet time-stamp clock
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_TIMESRC_MASK)|SIM_SOPT2_TIMESRC_M;
#endif

#if defined(SIM_SOPT2_RMIISRC_MASK) && defined(SIM_SOPT2_RMIISRC_M) // RMII clock
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_RMIISRC_MASK)|SIM_SOPT2_RMIISRC_M;
#endif

#ifdef SIM_SCGC4_USBOTG_MASK
   // !! WARNING !! The USB interface must be disabled for clock changes to have effect !! WARNING !!
   SIM->SCGC4 &= ~SIM_SCGC4_USBOTG_MASK;
#endif

#if defined(SIM_SOPT2_USBSRC_MASK) && defined(SIM_SOPT2_USBSRC_M) // USB clock (48MHz req.)
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_USBSRC_MASK)|SIM_SOPT2_USBSRC_M;
#endif

#if defined(SIM_SOPT2_USBFSRC_MASK) && defined(SIM_SOPT2_USBFSRC_M) // USB clock (48MHz req.)
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_USBFSRC_MASK)|SIM_SOPT2_USBFSRC_M;
#endif

#if defined(SIM_SOPT2_PLLFLLSEL_MASK) && defined(SIM_SOPT2_PLLFLLSEL_M) // Peripheral clock
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_PLLFLLSEL_MASK)|SIM_SOPT2_PLLFLLSEL_M;
#endif

#if defined(SIM_SOPT2_UART0SRC_MASK) && defined(SIM_SOPT2_UART0SRC_M) // UART0 clock
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_UART0SRC_MASK)|SIM_SOPT2_UART0SRC_M;
#endif

#if defined(SIM_SOPT2_TPMSRC_MASK) && defined(SIM_SOPT2_TPMSRC_M) // TPM clock
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_TPMSRC_MASK)|SIM_SOPT2_TPMSRC_M;
#endif

#if defined(SIM_SOPT2_CLKOUTSEL_MASK) && defined(SIM_SOPT2_CLKOUTSEL_M)
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_CLKOUTSEL_MASK)|SIM_SOPT2_CLKOUTSEL_M;
#endif

#if defined(SIM_SOPT2_RTCCLKOUTSEL_MASK) && defined(SIM_SOPT2_RTCCLKOUTSEL_M)
   SIM->SOPT2 = (SIM->SOPT2&~SIM_SOPT2_RTCCLKOUTSEL_MASK)|SIM_SOPT2_RTCCLKOUTSEL_M;
#endif

#if defined(SIM_CLKDIV2_USBDIV_MASK) && defined(SIM_CLKDIV2_USBFRAC_MASK) && defined(SIM_CLKDIV2_USB_M)
   SIM->CLKDIV2 = (SIM->CLKDIV2&~(SIM_CLKDIV2_USBDIV_MASK|SIM_CLKDIV2_USBFRAC_MASK)) | SIM_CLKDIV2_USB_M;
#endif

   SystemCoreClockUpdate();
}

/*!
 * @brief Update SystemCoreClock variable
 *
 * Updates the SystemCoreClock variable with current core Clock retrieved from CPU registers.
 */
void SystemCoreClockUpdate(void) {
   uint32_t oscerclk = OSCCLK_CLOCK;
   switch (MCG->S&MCG_S_CLKST_MASK) {
      case MCG_S_CLKST(0) : // FLL
         if ((MCG->C1&MCG_C1_IREFS_MASK) == 0) {
            SystemCoreClock = oscerclk/(1<<((MCG->C1&MCG_C1_FRDIV_MASK)>>MCG_C1_FRDIV_SHIFT));
            if ((MCG->C2&MCG_C2_RANGE0_MASK) != 0) {
               if ((MCG->C1&MCG_C1_FRDIV_MASK) == MCG_C1_FRDIV(6)) {
                  SystemCoreClock /= 20;
               }
               else if ((MCG->C1&MCG_C1_FRDIV_MASK) == MCG_C1_FRDIV(7)) {
                  SystemCoreClock /= 12;
               }
               else {
                  SystemCoreClock /= 32;
               }
            }
         }
         else {
            SystemCoreClock = SYSTEM_SLOW_IRC_CLOCK;
         }
         SystemCoreClock *= (MCG->C4&MCG_C4_DMX32_MASK)?732:640;
         SystemCoreClock *= ((MCG->C4&MCG_C4_DRST_DRS_MASK)>>MCG_C4_DRST_DRS_SHIFT)+1;
      break;
      case MCG_S_CLKST(1) : // Internal Reference Clock
         if ((MCG->C2&MCG_C2_IRCS_MASK) != 0) {
            SystemCoreClock = SYSTEM_FAST_IRC_CLOCK/(1<<((MCG->SC&MCG_SC_FCRDIV_MASK)>>MCG_SC_FCRDIV_SHIFT));
         }
         else {
            SystemCoreClock = SYSTEM_SLOW_IRC_CLOCK;
         }
         break;
      case MCG_S_CLKST(2) : // External Reference Clock
         SystemCoreClock = oscerclk;
         break;
   }
   SystemCoreClock   = SystemCoreClock/(((SIM->CLKDIV1&SIM_CLKDIV1_OUTDIV1_MASK)>>SIM_CLKDIV1_OUTDIV1_SHIFT)+1);
   SystemBusClock    = SystemCoreClock/(((SIM->CLKDIV1&SIM_CLKDIV1_OUTDIV4_MASK)>>SIM_CLKDIV1_OUTDIV4_SHIFT)+1);
}

