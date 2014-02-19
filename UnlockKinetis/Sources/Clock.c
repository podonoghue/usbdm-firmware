/*
 * clock.c
 *
 *  Created on: 04/03/2012
 *      Author: podonoghue
 */
#include "derivative.h" /* include peripheral declarations */
#include "clock.h"

const uint32_t SystemCoreClock = 48000000; // Hz
const uint32_t PeripheralClock = 48000000; // Hz

/*! Sets up the clock for USB operation (out of RESET)
 *!
 *! MCGOUTCLK = 48MHz
 *! core/platform/system clock = PLL (48MHz), 
 *! bus clock = PLL (48MHz),
 *! flash clock = PLL/2 (24MHz)
 *!
 *! Assumes 8 MHz external crystal
 *!
 *! Modes: FEI [FLL engaged internal] -> 
 *!        FBE [FLL bypassed external] ->
 *!        PBE [PLL bypassed external] ->
 *!        PEE [PLL engaged external]
 *!
 *! Refer 24.5.3.1 of K20 Family reference
 */
void initClock(void) {
   MCG_C7 = 0; // OSCSEL = 0 => Crystal Osc
   
   // Out of reset MCG is in FEI mode
   // Configure the Crystal Oscillator
   OSC0_CR = OSC_CR_SC16P_MASK; // OSC_CR_SC16P_MASK; 
   
   // 1. Switch from FEI (FLL engaged internal) to FBE (FLL bypassed external)
   
   // 1 a) Set up crystal 
   MCG_C2 =                         // oscillator in low power mode (w/o Rf)
            MCG_C2_EREFS0_MASK |    // because crystal is being used
            MCG_C2_RANGE0(1)   |    // 4 or 8 MHz is in high freq range
            MCG_C2_IRCS_MASK;
 
   // 1 b) Select clock mode
   MCG_C1 =  MCG_C1_CLKS(2)       |   // CLKS = 10   -> External reference clock
             MCG_C1_FRDIV(3)      |   // FRDIV = 3   -> 8MHz/256 = 31.25 kHz
             MCG_C1_IRCLKEN_MASK;     // IRCLKEN = 1 -> MCGIRCLK active
   
   // Make sure - reset default
   MCG_C4 &= ~(MCG_C4_DMX32_MASK|MCG_C4_DRST_DRS_MASK);
   
   // 1 c) Wait for crystal to start up        
   while ((MCG_S & MCG_S_OSCINIT0_MASK) == 0) {
   }
   // 1 d) Wait for mode change
   while ((MCG_S & MCG_S_IREFST_MASK) != 0) {
   }
   // 1 e) Wait for MCGOUT indicating that the external reference to be fed to MCGOUT
   while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)) {
   }
   
   // 2. Configure PLL Ref. Freq.
   
   // 2 a) Set PRDIV for correct range
   MCG_C5 =  MCG_C5_PLLCLKEN0_MASK |
             MCG_C5_PRDIV0(3);    // PRDIV=3, PLL Ref Freq. = 8MHz/4 => 2 MHz

   MCG_C6 = 0;
   
   // 3. FBE => BLPE => PBE
   // 3 a) BLPE
   MCG_C2 =                         // oscillator in low power mode (w/o external Rf)
            MCG_C2_EREFS0_MASK |    // because crystal is being used
            MCG_C2_RANGE0(1)   |    // 4 or 8 MHz is in high freq range
            MCG_C2_IRCS_MASK ;
   // 3 b) BLPE/PBE
   MCG_C6 = MCG_C6_PLLS_MASK|MCG_C6_VDIV0(0); // 2MHz x 24 = 96MHz
   // 3 c) BPE
   MCG_C2 =                         // oscillator in low power mode (w/o Rf)
            MCG_C2_EREFS0_MASK |    // because crystal is being used
            MCG_C2_RANGE0(1)   |    // 4 or 8 MHz is in high freq range
            MCG_C2_IRCS_MASK;
   // 3 d) Wait until PLLS clock source changes to the PLL
   while((MCG_S & MCG_S_PLLST_MASK) == 0) {
   }
   // 3 e)  Wait for PLL to acquired lock
   while((MCG_S & MCG_S_LOCK0_MASK) == 0) {
   }
   // Set up the SIM clock dividers BEFORE switching to the PLL to ensure the
   // system clock speeds are in spec.
#if 1 || (CPU == MK20D5)
   {
   // Disable Flash pre-fetch while changing flash divider
   uint32_t temp = FMC_PFAPR;
   FMC_PFAPR |= FMC_PFAPR_M3AP_MASK|FMC_PFAPR_M2AP_MASK|FMC_PFAPR_M1AP_MASK|FMC_PFAPR_M0AP_MASK;
   // core/platform/system clock = PLL (48MHz), bus clock = PLL (48MHz), flash clock = PLL/2 (24MHz)
   SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV4(1);
   FMC_PFAPR = temp;
   }
#elif CPU == MKL25Z4
   {
   // Disable Flash pre-fetch while changing flash divider
   uint32_t temp = FMC_PFAPR;
   FMC_PFAPR |= FMC_PFAPR_M3AP_MASK|FMC_PFAPR_M2AP_MASK|FMC_PFAPR_M1AP_MASK|FMC_PFAPR_M0AP_MASK;
   // core/platform/system clock = PLL/2 (48MHz), bus clock/flash clock = PLL/2/2 (24MHz)
   SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(1);
   FMC_PFAPR = temp;
   }
#endif
   
   // 4. PBE mode transitions into PEE mode:
   // 4 a) Select clock mode
   MCG_C1 = MCG_C1_CLKS(0)  |    // CLKS  = 0 -> FLL or PLL is selected
            MCG_C1_FRDIV(3) |    // FRDIV = 3 -> 8MHz/256 = 31.25 kHz
            MCG_C1_IRCLKEN_MASK;
   
   // 4 b)  Wait for clock stable
   while ((MCG_S & MCG_S_CLKST_MASK) != (3<<MCG_S_CLKST_SHIFT)) {
   }
   // Now MCGOUTCLK=MCGPLLCLK=[(2 MHz / 1) * 24] = 48 MHz
   
   // Basic clock selection
#if 1 || (CPU == MK20D5)
   // Peripheral clock choice (incl. USB), USBCLK = MCGCLK
   SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK | // PLL rather than FLL for peripheral clock
                SIM_SOPT2_USBSRC_MASK;     // MCGPLLCLK Source as USB clock (48MHz req.)
   
   // USB Clock = Divider input clock x [(USBFRAC+1)/(USBDIV+1)] 
   // USBDIV 0-7, USBFRAC 0-1
//   SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
   SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(0);

#elif CPU == MKL25Z4 
   SIM_SOPT2 = SIM_SOPT2_UART0SRC(1)     | // MCGPLLCLK/2 as UART0 clock
		         SIM_SOPT2_TPMSRC(1)       | // MCGPLLCLK/2 as TPM clock
			      SIM_SOPT2_PLLFLLSEL_MASK  | // PLL rather than FLL for peripheral clock 
               SIM_SOPT2_USBSRC_MASK;      // MCGPLLCLK/2 Source as USB clock (48MHz req.)
#else
   #error "CPU not set"
#endif
}
