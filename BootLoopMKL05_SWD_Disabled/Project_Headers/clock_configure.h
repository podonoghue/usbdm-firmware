/*
 * clock_configure-MCG-MKL0x.h
 *
 *  Used for MK02, MKL04, MKL05
 *
 *  Created on: Nov 6, 2012
 *      Author: podonoghue
 */

#ifndef CLOCK_PRIVATE_H_
#define CLOCK_PRIVATE_H_

#include <stdint.h>
#include "derivative.h"
#include "system.h"

#ifdef __cplusplus
extern "C" {
#endif

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------

//===================================
// Validators
// Common clock settings                                                          Core      Bus
// <validate=net.sourceforge.usbdm.annotationEditor.validators.ClockValidate_KLxx(48000000, 24000000)>
// FLL clock
// <validate=net.sourceforge.usbdm.annotationEditor.validators.FllClockValidate>

// Convention
// name_V = field value
// name_M = field mask i.e. value in correct position for register

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------

// OSCCLK_CLOCK ==============================
//
//   <o> Frequency of Main External Clock or Crystal (Hz)  <name=oscclk_clock> <0-48000000>
//   <i> Frequency of external crystal or clock on XTAL/EXTAL
//   <i> See MCG_C2_EREFS0 for XTAL/Clock selection
#define OSCCLK_CLOCK (32768UL)

// OSC32KCLK_CLOCK
// Assumed to be only available when main oscilator operating with 32kHz crystal on XTAL/EXTAL
#if (OSCCLK_CLOCK < 31000) || (OSCCLK_CLOCK> 33000)
#define OSC32KCLK_CLOCK (0)
#else
#define OSC32KCLK_CLOCK OSCCLK_CLOCK
#endif

// RTC_CLKIN_CLOCK ==============================
//
//   <o> Frequency of external RTC clock (Hz)
//   <i> Frequency of external clock provided to RTC clock input pin
//   <i> Not available on KL02 devices
#define RTC_CLKIN_CLOCK (0UL)

// SYSTEM_LOW_POWER_CLOCK ==============================
//
//   <o> Frequency of Internal Low Power Oscillator Clock (Hz) <constant> <name=system_low_power_clock>
//   <i> Dependent on device. [Typically ~1kHz]
#define SYSTEM_LOW_POWER_CLOCK (1000UL)

// SYSTEM_ERC_CLOCK ==============================
//
//   <o> External Reference Clock (Hz) <constant> <name=system_erc_clock>
//   <i> Derived from the external crystal or clock source on XTAL/EXTAL
#define SYSTEM_ERC_CLOCK (32768UL)

// SYSTEM_SLOW_IRC_CLOCK ==============================
//
//   <o> Frequency of Slow Internal Reference Clock (Hz) <constant> <name=system_slow_irc_clock>
//   <i> Dependent on device and clock Trim. [Typically ~32kHz]
#define SYSTEM_SLOW_IRC_CLOCK (32768UL)

// SYSTEM_FAST_IRC_CLOCK ==============================
//
//   <o> Frequency of Fast Internal Reference Clock (Hz) <constant> <name=system_fast_irc_clock>
//   <i> Dependent on device and clock Trim. [Typically ~4MHz]
#define SYSTEM_FAST_IRC_CLOCK (4000000UL)

//========================================================================================
//========================================================================================
// CLOCK_MODE ===============================
//
//   <o> Clock Mode <name=clock_mode>
//   <i> Basic choice for final clock mode
//<i> FLL Engaged Internal(FEI)
//<i> In FEI mode, MCGOUT is derived from the FLL clock (DCOCLK) that is controlled by the 32 kHz Internal Reference Clock (IRC).
//<i> The FLL loop will lock the DCO frequency to the FLL factor, as selected by the C4[DRST_DRS] and C4[DMX32] bits, times the
//<i> internal reference frequency.
//<i>
//<i> FLL Engaged External(FEE)
//<i> In FEE mode, MCGOUT is derived from the FLL clock (DCOCLK) that is controlled by the external reference clock. The FLL loop
//<i> will lock the DCO frequency to the FLL factor, as selected by C4[DRST_DRS] and C4[DMX32] bits, times the external reference
//<i> frequency, as specified by the C1[FRDIV] and C2[RANGE].
//<i>
//<i> FLL Bypassed Internal(FBI)
//<i> In FBI mode, the MCGOUT clock is derived either from the slow (32 kHz IRC) or fast (2 MHz IRC) internal reference clock,
//<i> as selected by the C2[IRCS] bit. The FLL is operational but its output is not used. This mode is useful to allow the FLL
//<i> to acquire its target frequency while the MCGOUT clock is driven from the C2[IRCS] selected internal reference clock. The
//<i> FLL clock (DCOCLK) is controlled by the slow internal reference clock, and the DCO clock frequency locks to a multiplication
//<i> factor, as selected by the C4[DRST_DRS] and C4[DMX32] bits, times the internal reference frequency.
//<i>
//<i> FLL Bypassed External(FBE)
//<i> In FBE mode, the MCGOUT clock is derived from the external reference clock. The FLL is operational but its output is not
//<i> used. This mode is useful to allow the FLL to acquire its target frequency while the MCGOUT clock is driven from the
//<i> external reference clock. The FLL clock (DCOCLK) is controlled by the external reference clock, and the DCO clock frequency
//<i> locks to a multiplication factor, as selected by the C4[DRST_DRS] and C4[DMX32] bits, times the divided external reference
//<i> frequency.
//<i>
//<i> PLL Engaged External(PEE)
//<i> In PEE mode, the MCGOUT is derived from the PLL clock, which is controlled by the external reference clock. The PLL clock
//<i> frequency locks to a multiplication factor, as specified by C6[VDIV], times the external reference frequency, as specified
//<i> by C5[PRDIV].
//<i>
//<i> PLL Bypassed External(PBE)
//<i> In PBE mode, MCGOUT is derived from the OSCSEL external reference clock; the PLL is operational, but its output clock is
//<i> not used. This mode is useful to allow the PLL to acquire its target frequency while MCGOUT is driven from the external
//<i> reference clock. The PLL clock frequency locks to a multiplication factor, as specified by its [VDIV], times the external
//<i> reference frequency, as specified by its [PRDIV].
//<i>
//<i> Bypassed Low Power Internal (BLPI/FBILP)
//<i> In BLPI mode, MCGOUT is derived from the internal reference clock. The FLL is disabled and PLL is disabled even if the
//<i> C5[PLLCLKEN] is set to 1.
//<i>
//<i> Bypassed Low Power External (BLPE/FBELP)
//<i> In BLPE mode, MCGOUT is derived from the external reference clock. The FLL is disabled and PLL is disabled even if the
//<i> C5[PLLCLKEN] is set to 1.
//     <0=> No setup (Reset default)
//     <1=> FLL Engaged Internal (FEI)
//     <2=> FLL Engaged External (FEE)
//     <3=> FLL bypassed internal (FBI)
//     <4=> FLL bypassed internal low power (BLPI/FBILP)
//     <5=> FLL bypassed external (FBE)
//     <6=> FLL bypassed external low power (BLPE/FBELP)
#define CLOCK_MODE 1

// Clock modes
#define CLOCK_MODE_NONE     0
#define CLOCK_MODE_FEI      1
#define CLOCK_MODE_FEE      2
#define CLOCK_MODE_FBI      3
#define CLOCK_MODE_BLPI     4
#define CLOCK_MODE_FBE      5
#define CLOCK_MODE_BLPE     6

// FLL_TARGET_CLOCK =======================================
//
//  <o> FLL Output clock frequency (Hz) <name=fllTargetFrequency>
//  <i> Used for MCGFLLCLK system clock
//  <i> Used for main MCGOUTCLK system clock if FEI or FEE mode is selected.
#define FLL_TARGET_CLOCK 83886080UL

// SYSTEM_MCGOUT_CLOCK =======================================
//
//  <o> System MCGOUT Clock (Hz) <name=system_mcgout_clock> <constant>
//  <i> MCG Main clock output
//  <i> Derived from slow IRC, fast IRC, ERC, FLL or PLL
#define SYSTEM_MCGOUT_CLOCK 83886080UL

// SYSTEM_MCGIR_CLOCK =======================================
//
//  <o> System MCGIR Clock (Hz) <name=system_mcgir_clock> <constant>
//  <i> MCG Internal Reference Clock
//  <i> Derived from slow IRC or fast IRC divided by SC.FCRDIV
#define SYSTEM_MCGIR_CLOCK 4000000UL

// SYSTEM_CORE_CLOCK =======================================
//
//  <o> System Core Clock (Hz) <name=system_core_clock> <constant>
//  <i> Clocks the ARM Cortex-M4 core
//  <i> Derived from MCGOUT Clock after division by OUTDIV1
#define SYSTEM_CORE_CLOCK 41943040UL

// SYSTEM_BUS_CLOCK =======================================
//
//  <o> System Bus and Flash Clock (Hz) <name=system_bus_clock> <constant>
//  <i> Clocks the bus slaves & peripheral and flash
//  <i> Derived from Core Clock after division by OUTDIV4
//  <i> Must be &lt;= Core Clock frequency.
#define SYSTEM_BUS_CLOCK 20971520UL

// SYSTEM_FLASH_CLOCK =======================================
//
// Not used
#define SYSTEM_FLASH_CLOCK SYSTEM_BUS_CLOCK

// <h> System Clock dividers
// SIM_CLKDIV1_OUTDIV1 ================================
//
//   <o> Core & System Clock Divider (OUTDIV1) - Divide by <1-16> <#-1> <name=sim_clkdiv1_outdiv1>
//   <i> Clocks the ARM Cortex-M4 core and bus masters [SIM_CLKDIV1_OUTDIV1]
//   <i> MCGOUTCLK clock is source. Default /2
//   <i> Must be less than or equal to 48 MHz.
#define SIM_CLKDIV1_OUTDIV1_V (1)
#define SIM_CLKDIV1_OUTDIV1_M (SIM_CLKDIV1_OUTDIV1_V<<SIM_CLKDIV1_OUTDIV1_SHIFT)

// SIM_CLKDIV1_OUTDIV2 ================================
// Not used
#define SIM_CLKDIV1_OUTDIV2(x) (0)
#define SIM_CLKDIV1_OUTDIV2_M  (0)

// SIM_CLKDIV1_OUTDIV3 ================================
// Not used
#define SIM_CLKDIV1_OUTDIV3(x)  (0)
#define SIM_CLKDIV1_OUTDIV3_M   (0)

// SIM_CLKDIV1_OUTDIV4 ================================
//
//   <o> Bus & Flash Clock Divider (OUTDIV4) - Divide by <1-16> <#-1> <name=sim_clkdiv1_outdiv4>
//   <i> Clocks the bus slaves, peripherals and flash memory [SIM_CLKDIV1_OUTDIV4]
//   <i> MCGOUTCLK clock divided by OUTDIV1 is source. Default /2
//   <i> Must be less than or equal to 24 MHz and less than or equal to the Bus Clock frequency.
#define SIM_CLKDIV1_OUTDIV4_V (1)
#define SIM_CLKDIV1_OUTDIV4_M (SIM_CLKDIV1_OUTDIV4_V<<SIM_CLKDIV1_OUTDIV4_SHIFT)

// </h>

//========================================================================================
//========================================================================================
// <h> OSC Control Register (OSC_CR)

// OSC_CR_ERCLKEN ===============================
//
//   <q> External Reference Enable (ERCLKEN)
//   <i> Enables external reference clock [OSC_CR_ERCLKEN]
//     <0=> Disabled
//     <1=> Enabled
#define OSC_CR_ERCLKEN_V 1
#define OSC_CR_ERCLKEN_M (OSC_CR_ERCLKEN_V<<OSC_CR_ERCLKEN_SHIFT)

// OSC_CR_EREFSTEN ===============================
//
//   <q> External Reference Stop Enable (EREFSTEN)
//   <i> Determines if external reference clock is enabled in Stop mode [OSC_CR_EREFSTEN]
//     <0=> Disabled in Stop mode
//     <1=> Enabled in Stop mode
#define OSC_CR_EREFSTEN_V 1
#define OSC_CR_EREFSTEN_M (OSC_CR_EREFSTEN_V<<OSC_CR_EREFSTEN_SHIFT)

// OSC_CR_SC2P ===============================
//
//   <q0> Oscillator Capacitor Load Configure
//   <i> Configures the oscillator load capacitance [OSC_CR_SC2P]
//     <0=>
//     <1=> +2pF

// OSC_CR_SC4P ===============================
//
//   <q1> Oscillator Capacitor Load Configure
//   <i> Configures the oscillator load capacitance [OSC_CR_SC4P]
//     <0=>
//     <1=> +4pF

// OSC_CR_SC8P ===============================
//
//   <q2> Oscillator Capacitor Load Configure
//   <i> Configures the oscillator load capacitance [OSC_CR_SC8P]
//     <0=>
//     <1=> +8pF

// OSC_CR_SC16P ===============================
//
//   <q3> Oscillator Capacitor Load Configure
//   <i> Configures the oscillator load capacitance [OSC_CR_SC16P]
//     <0=>
//     <1=> +16pF

#define OSC_CR_SCP_M ((0<<OSC_CR_SC2P_SHIFT)|(0<<OSC_CR_SC4P_SHIFT)|(1<<OSC_CR_SC8P_SHIFT)|(0<<OSC_CR_SC16P_SHIFT))
// </h>

//========================================================================================
//========================================================================================
// <h> MCG Control Register 1 (MCG_C1)

// MCG_C1_CLKS =======================================
//
//  <o> MCGOUTCLK Clock Source Select (CLKS) [0-2] <constant> <name=mcg_c1_clks>
//  <i> Selects the clock source for MCGOUTCLK [MCG_C1_CLKS]
//  <i> This option is determined by the Clock Mode selection
//      <0=> Output of FLL is selected
//      <1=> Internal reference clock is selected
//      <2=> External reference clock is selected
#define MCG_C1_CLKS_V 0
#define MCG_C1_CLKS_M (MCG_C1_CLKS_V<<MCG_C1_CLKS_SHIFT)

// MCG_C1_FRDIV ================================
//
//   <o> FLL External Reference Divider (FRDIV) <0-7> <constant> <name=mcg_c1_frdiv>
//   <i> Selects the amount to divide down the external reference clock for the FLL. [MCG_C1_FRDIV]
//   <i> The resulting frequency must be in the range 31.25 kHz to 39.0625 kHz
//   <i> Division factor depends on Clock Range [MGC_C2_RANGE0]
//   <i> This option is determined by the Clock Mode and FLL input clock
//      <0=> Divide by 1 or 32
//      <1=> Divide by 2 or 64
//      <2=> Divide by 4 or 128
//      <3=> Divide by 8 or 256
//      <4=> Divide by 16 or 512
//      <5=> Divide by 32 or 1024
//      <6=> Divide by 64 or 1280
//      <7=> Divide by 128 or 1536
#define MCG_C1_FRDIV_V 0
#define MCG_C1_FRDIV_M (MCG_C1_FRDIV_V<<MCG_C1_FRDIV_SHIFT)

// MCG_C1_IREFS ================================
//
//   <o> Internal Reference Select (IREFS) <constant> <name=mcg_c1_irefs>
//   <i> Selects the reference clock source for the FLL [MCG_C1_IREFS]
//   <i> This option is determined by the Clock Mode selection
//      <0=> External Reference Clock
//      <1=> Slow Internal Clock
#define MCG_C1_IREFS_V 1
#define MCG_C1_IREFS_M (MCG_C1_IREFS_V<<MCG_C1_IREFS_SHIFT)

// MCG_C1_IRCLKEN ==============================
//
//   <q> Internal Reference Clock Enable (IRCLKEN)
//   <i> Enables the internal reference clock for use as MCGIRCLK [MCG_C1_IRCLKEN]
//      <0=> Inactive
//      <1=> Active
#define MCG_C1_IRCLKEN_V   0
#define MCG_C1_IRCLKEN_M   (1<<MCG_C1_IRCLKEN_SHIFT)

// MCG_C1_IREFSTEN =============================
//
//   <q> Internal Reference Stop Enable (IREFSTEN)
//   <i> Determines if IRS is enabled in Stop mode [MCG_C1_IREFSTEN]
//      <0=> IR disabled in STOP
//      <1=> IR enabled in STOP
#define MCG_C1_IREFSTEN_V   0
#define MCG_C1_IREFSTEN_M   (MCG_C1_IREFSTEN_V<<MCG_C1_IREFSTEN_SHIFT)

// </h>

//========================================================================================
//========================================================================================
// <h> MCG Control Register 2 (MCG_C2)

// MCG_C2_LOCRE0 =============================
//
//   <q> Action on Loss of Clock (LOCRE0)
//   <i> Determines if an Interrupt or Reset occurs on loss of OSC0 external reference [MCG_C2_LOCRE0]
//   <i> This option only has effect if the clock monitor is first enabled CME0 = 1
//      <0=> Interrupt
//      <1=> Reset
#define MCG_C2_LOCRE0_V (0)
#define MCG_C2_LOCRE0_M (MCG_C2_LOCRE0_V<<MCG_C2_LOCRE0_SHIFT)

// MCG_C2_RANGE0 =============================
//
//   <o> Frequency Range Select (RANGE0) <constant> <name=mcg_c2_range0>
//   <i> Selects the frequency range for the crystal oscillator or external clock source [MCG_C2_RANGE0]
//   <i> This value is calculated from the FLL input clock frequency
//      <0=> Low range
//      <1=> High range
//      <2=> Very High range
#define MCG_C2_RANGE0_V   0
#define MCG_C2_RANGE0_M   (MCG_C2_RANGE0_V<<MCG_C2_RANGE0_SHIFT)

// MCG_C2_HGO0 =============================
//
//   <q> Oscillator Gain (HGO0)
//   <i> Controls the crystal oscillator mode of operation [MCG_C2_HGO0]
//      <0=> Low power
//      <1=> High gain
#define MCG_C2_HGO0_V   0
#define MCG_C2_HGO0_M   (MCG_C2_HGO0_V<<MCG_C2_HGO0_SHIFT)

// MCG_C2_EREFS0 =============================
//
//   <q> External Reference Select (EREFS0) <name=mcg_c2_erefs0>
//   <i> Determines whether a clock or crystal is used for the external reference clock [C2_EREFS0]
//      <0=> External clock
//      <1=> Oscillator
#define MCG_C2_EREFS0_V  1
#define MCG_C2_EREFS0_M (MCG_C2_EREFS0_V<<MCG_C2_EREFS0_SHIFT)

// MCG_C2_LP =============================
//
//   <o> Low Power Select (LP) <constant> <name=mcg_c2_lp>
//   <i> Whether FLL continues operation when bypassed [MCG_C2_LP]
//   <i> This option is determined by the Clock Mode selection
//      <0=> FLL is enabled in bypass modes
//      <1=> FLL is disabled in bypass modes
#define MCG_C2_LP_V  0
#define MCG_C2_LP_M (MCG_C2_LP_V<<MCG_C2_LP_SHIFT)

// MCG_C2_IRCS ==============================
//
//   <q> MCG IRC Clock Source (IRCS) <name=mcg_c2_ircs>
//   <i> MCG Internal Clock Source [MCG_C2_IRCS]
//     <0=> Slow internal reference clock
//     <1=> Fast internal reference clock
#define MCG_C2_IRCS_V 1
#define MCG_C2_IRCS_M (MCG_C2_IRCS_V<<MCG_C2_IRCS_SHIFT)

// </h>

//========================================================================================
//========================================================================================
// <h> MCG Control Register 4 (MCG_C4)

// MCG_C4_DMX32 ==============================
//
//   <q> DMX32 DCO lock range (DMX32) <name=mcg_c4_dmx32>
//   <i> Optimise for 32.768 kHz Reference [MCG_C4_DMX32]
//     <0=> Wide lock range 31.25-39.06 kHz
//     <1=> Optimised for 32.768 kHz reference
#define MCG_C4_DMX32_V  0
#define MCG_C4_DMX32_M (MCG_C4_DMX32_V<<MCG_C4_DMX32_SHIFT)

// MCG_C4_DRST_DRS =============================
//
//   <o> DCO Range Select (DRST_DRS) <constant> <0-3> <name=mcg_c4_drst_drs>
//   <i> Frequency range for the FLL output, DCOOUT [MCG_C4_DRST_DRS]
//      <0=> Low (x640/x732, 20-25/24 MHz)
//      <1=> Mid (x1280/x1464, 40-50/48 MHz)
//      <2=> Mid-high (x1920/x2197, 60-75/72 MHz)
//      <3=> High (x2560/x2929, 80-100/96 MHz)
#define MCG_C4_DRST_DRS_V  3
#define MCG_C4_DRST_DRS_M (MCG_C4_DRST_DRS_V<<MCG_C4_DRST_DRS_SHIFT)

// </h>

//========================================================================================
//========================================================================================
// <h> MCG Control Register 6 (MCG_C6)

// MCG_C6_CME ==============================
//
//   <q> Clock Monitor Enable (CME0)
//   <i> Determines if a reset request is made following a loss of external clock indication. [MCG_C6_CME0]
//   <i> This field must be set to a logic 1 only when the ICS is in an operational mode that uses the external clock (FEE, FBE, or FBELP).
//      <0=> Clock monitor is disabled.
//      <1=> Clock monitor is enabled.
#define MCG_C6_CME0_V (0)
#define MCG_C6_CME0_M (MCG_C6_CME0_V<<MCG_C6_CME0_SHIFT)

// </h>

//========================================================================================
//========================================================================================
// <h> MCG Status and Control Register (MCG_SC)

// MCG_SC_FCRDIV ==============================
//
//   <o> Fast Internal Clock Reference Divider (FCRDIV) <0-7> <name=mcg_sc_fcrdiv>
//   <i> Selects the amount to divide down the fast internal reference clock [MCG_SC_FCRDIV]
//   <i> The FIR clock is available for use as MCGIRCLK or MCGOUTCLK.
//   <0=> Divide by 1
//   <1=> Divide by 2
//   <2=> Divide by 4
//   <3=> Divide by 8
//   <4=> Divide by 16
//   <5=> Divide by 32
//   <6=> Divide by 64
//   <7=> Divide by 128
#define MCG_SC_FCRDIV_V  0
#define MCG_SC_FCRDIV_M (MCG_SC_FCRDIV_V<<MCG_SC_FCRDIV_SHIFT)

// </h>


// ERC_AFTER_FRDIV_CLOCK = External reference clock after dividers
#if (MCG_C2_RANGE0_V == 0)
#define ERC_AFTER_FRDIV_CLOCK (SYSTEM_OSCER_CLOCK/(1<<MCG_C1_FRDIV_V))
#elif (MCG_C1_FRDIV_V <= 5)
#define ERC_AFTER_FRDIV_CLOCK (SYSTEM_OSCER_CLOCK/(1<<(MCG_C1_FRDIV_V+5)))
#elif (MCG_C1_FRDIV_V == 6)
#define ERC_AFTER_FRDIV_CLOCK (SYSTEM_OSCER_CLOCK/(1<<5)/40)
#elif (MCG_C1_FRDIV_V == 7)
#define ERC_AFTER_FRDIV_CLOCK (SYSTEM_OSCER_CLOCK/(1<<5)/48)
#endif

// SYSTEM_MCGFF_CLOCK  ==============================
// Only available if less than 1/8 MCGOUT clock
#if (MCG_C1_IREFS_V == 0)
#define SYSTEM_MCGFF_CLOCK ERC_AFTER_FRDIV_CLOCK  // External Reference clock after dividers
#else
#define SYSTEM_MCGFF_CLOCK SYSTEM_SLOW_IRC_CLOCK  // Slow internal clock (nominally 32 kHz)
#endif

// SYSTEM_PERIPHERAL_CLOCK  =====================================
// <h> Peripheral Clock Source Selection (Not all may be present)

// SIM_SOPT1_OSC32KSEL ================================
//
//   <o> 32kHz Clock Source (ERCLK32)
//   <i> Source for nominal 32K clock for peripherals [SIM_SOPT1_OSC32KSEL]
//   <i> Not available on KL02 devices
//     <0=> System Oscillator (OSC32KCLK)
//     <2=> Real Time Clock CLKIN pin
//     <3=> Low power oscillator (LPO - 1kHz)
#ifdef SIM_SOPT1_OSC32KSEL
#define SIM_SOPT1_OSC32KSEL_V 3
#define SIM_SOPT1_OSC32KSEL_M (SIM_SOPT1_OSC32KSEL_V<<SIM_SOPT1_OSC32KSEL_SHIFT)

#if SIM_SOPT1_OSC32KSEL_V == 0
#define SYSTEM_ERCLK32_CLOCK OSC32KCLK_CLOCK          // Main Oscillator operating in 32kHz mode
#elif SIM_SOPT1_OSC32KSEL_V == 2
#define SYSTEM_ERCLK32_CLOCK RTC_CLKIN_CLOCK   // External RTC clock pin
#elif SIM_SOPT1_OSC32KSEL_V == 3
#define SYSTEM_ERCLK32_CLOCK SYSTEM_LOW_POWER_CLOCK   // LPO
#else
#error "Invalid ERCLK32 clock selected"
#endif
#else
#define SYSTEM_ERCLK32_CLOCK SYSTEM_OSC32KCLK_CLOCK   // Main/32kHz Oscillator (depends on chip)
#endif

// SIM_SOPT2_UART0SRC =================================
//
//   <o> UART0 Clock Source
//   <i> Universal Asynchronous Receiver/Transmitter clock source [SIM_SOPT2_UART0SRC]
//     <0=> Disabled
//     <1=> MCG FLL Clock (MCGFLLCLK)
//     <2=> Oscillator External Reference Clock (OSCERCLK)
//     <3=> MCG Internal Reference Clock (MCGIRCLK)
#define SIM_SOPT2_UART0SRC_V  (1)
#define SIM_SOPT2_UART0SRC_M  SIM_SOPT2_UART0SRC(SIM_SOPT2_UART0SRC_V)

#if (SIM_SOPT2_UART0SRC_V == (0))
#define SYSTEM_UART0_CLOCK (0)
#elif (SIM_SOPT2_UART0SRC_V == (1))
#define SYSTEM_UART0_CLOCK SYSTEM_MCGFLL_CLOCK
#elif (SIM_SOPT2_UART0SRC_V == (2))
#define SYSTEM_UART0_CLOCK SYSTEM_OSCER_CLOCK
#elif (SIM_SOPT2_UART0SRC_V == (3))
#define SYSTEM_UART0_CLOCK SYSTEM_MCGIR_CLOCK
#endif

#define SYSTEM_UART1_CLOCK SystemBusClock
#define SYSTEM_UART2_CLOCK SystemBusClock

// SIM_SOPT2_TPMSRC ===================================
//
//   <o> TPM Clock Source
//   <i> Timer/PWM Module clock source [SIM_SOPT2_TPMSRC]
//     <0=> Disabled
//     <1=> MGC FLL Clock (MCGFLLCLK)
//     <2=> Oscillator External Reference Clock (OSCERCLK)
//     <3=> MCG Internal Reference Clock (MCGIRCLK)
#define SIM_SOPT2_TPMSRC_V (1)
#define SIM_SOPT2_TPMSRC_M SIM_SOPT2_TPMSRC(SIM_SOPT2_TPMSRC_V)

#if (SIM_SOPT2_TPMSRC_V == (0))
#define SYSTEM_TPM_CLOCK (0)
#elif (SIM_SOPT2_TPMSRC_V == (1))
#define SYSTEM_TPM_CLOCK SYSTEM_MCGFLL_CLOCK
#elif (SIM_SOPT2_TPMSRC_V == (2))
#define SYSTEM_TPM_CLOCK SYSTEM_OSCER_CLOCK
#elif (SIM_SOPT2_TPMSRC_V == (3))
#define SYSTEM_TPM_CLOCK SYSTEM_MCGIRCLK_CLOCK
#endif

// SIM_SOPT2_CLKOUTSEL ================================
//
//   <o> CLKOUT select
//   <i> Selects the clock to output on the CLKOUT pin. [SIM_SOPT2_CLKOUTSEL]
//     <2=> Bus clock
//     <3=> LPO clock (1 kHz)
//     <4=> MCGIRCLK
//     <6=> OSCERCLK
#define SIM_SOPT2_CLKOUTSEL_V 0
#define SIM_SOPT2_CLKOUTSEL_M (SIM_SOPT2_CLKOUTSEL_V<<SIM_SOPT2_CLKOUTSEL_SHIFT)

// SIM_SOPT2_RTCCLKOUTSEL ================================
//
//   <o> RTC clock out select
//   <i> Selects the clock to be output on the RTC_CLKOUT pin [SIM_SOPT2_RTCCLKOUTSEL]
//     <0=> RTC 1 Hz clock
//     <1=> RTC 32.768kHz clock
#define SIM_SOPT2_RTCCLKOUTSEL_V 0
#define SIM_SOPT2_RTCCLKOUTSEL_M (SIM_SOPT2_RTCCLKOUTSEL_V<<SIM_SOPT2_RTCCLKOUTSEL_SHIFT)

// </h>

// SYSTEM_OSCER_CLOCK ================================
// Always connected to main oscillator
#define SYSTEM_OSCER_CLOCK OSCCLK_CLOCK

// MCGFLLCLK_CLOCK  ==============================
#define SYSTEM_MCGFLL_CLOCK FLL_TARGET_CLOCK

/*
 * The following are 'public' definitions
 *
 * SYSTEM_MCGIR_CLOCK       MCG Internal Reference clock
 * SYSTEM_MCGFF_CLOCK       MCG Fixed Frequency ! (from SlowIRC/ERC_DIV/Peripheral bus clock)
 * SYSTEM_MCGFLL_CLOCK      MCG FLL clock (from FLL)
 * SYSTEM_MCGOUT_CLOCK      MCG OUT clock (from SlowIRC/FastIRC/ExternalRC/PLL/FLL)
 * SYSTEM_OSCER_CLOCK       Main oscillator/clock
 * SYSTEM_BUS_CLOCK
 * SYSTEM_CORE_CLOCK
 */

void clock_initialise(void);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_PRIVATE_H_ */
