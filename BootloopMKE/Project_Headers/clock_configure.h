/*
 * clock_private-MKE02M4.h
 *
 *  Used for MKE02M4
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
// Common clock settings                                                           Core       Bus
// <validate=net.sourceforge.usbdm.annotationEditor.validators.ClockValidate_MKE02(40000000, 20000000)>
//
// FLL clock                                                                    fllFactor
// <validate=net.sourceforge.usbdm.annotationEditor.validators.ICSClockValidate(1024     )>

// Convention
// name_V = field value
// name_M = field mask i.e. value in correct position for register

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------

// OSCCLK_CLOCK ==============================
//
//   <o> Frequency of Main External Clock or Crystal (Hz)  <name=oscclk_clock> <0-48000000>
//   <i> Frequency of external crystal or clock on XTAL/EXTAL
#define OSCCLK_CLOCK (10000000UL)

// SYSTEM_LOW_POWER_CLOCK ==============================
//
//   <o> Frequency of Internal Low Power Oscillator Clock (Hz) <constant> <name=system_low_power_clock>
//   <i> Dependent on device. [Typically ~1kHz]
#define SYSTEM_LOW_POWER_CLOCK (1000UL)

// SYSTEM_SLOW_IRC_CLOCK ==============================
//
//   <o> Frequency of Slow Internal Reference Clock (Hz) <constant> <name=system_slow_irc_clock>
//   <i> Dependent on device and clock Trim. [Typically ~32kHz]
#define SYSTEM_SLOW_IRC_CLOCK (31250UL)

//========================================================================================
//========================================================================================
// CLOCK_MODE ===============================
//
//   <o> Clock Mode <name=clock_mode>
//   <i> Basic choice for final clock mode
//<i> FLL Engaged Internal(FEI)
//<i> In FEI mode, ICSOUT is derived from the FLL clock (DCOCLK) that is controlled by the 32 kHz Internal Reference Clock (IRC).
//<i> The FLL loop will lock the DCO frequency to the FLL factor, as selected by the C4[DRST_DRS] and C4[DMX32] bits, times the
//<i> internal reference frequency.
//<i>
//<i> FLL Engaged External(FEE)
//<i> In FEE mode, ICSOUT is derived from the FLL clock (DCOCLK) that is controlled by the external reference clock. The FLL loop
//<i> will lock the DCO frequency to the FLL factor, as selected by C4[DRST_DRS] and C4[DMX32] bits, times the external reference
//<i> frequency, as specified by the C1[FRDIV] and C2[RANGE].
//<i>
//<i> FLL Bypassed Internal(FBI)
//<i> In FBI mode, the ICSOUT clock is derived either from the slow (32 kHz IRC) or fast (2 MHz IRC) internal reference clock,
//<i> as selected by the C2[IRCS] bit. The FLL is operational but its output is not used. This mode is useful to allow the FLL
//<i> to acquire its target frequency while the ICSOUT clock is driven from the C2[IRCS] selected internal reference clock. The
//<i> FLL clock (DCOCLK) is controlled by the slow internal reference clock, and the DCO clock frequency locks to a multiplication
//<i> factor, as selected by the C4[DRST_DRS] and C4[DMX32] bits, times the internal reference frequency.
//<i>
//<i> FLL Bypassed External(FBE)
//<i> In FBE mode, the ICSOUT clock is derived from the external reference clock. The FLL is operational but its output is not
//<i> used. This mode is useful to allow the FLL to acquire its target frequency while the ICSOUT clock is driven from the
//<i> external reference clock. The FLL clock (DCOCLK) is controlled by the external reference clock, and the DCO clock frequency
//<i> locks to a multiplication factor, as selected by the C4[DRST_DRS] and C4[DMX32] bits, times the divided external reference
//<i> frequency.
//<i>
//<i> FLL Bypassed Low Power Internal (BLPI/FBILP)
//<i> In BLPI mode, ICSOUT is derived from the internal reference clock. The FLL is disabled and PLL is disabled even if the
//<i> C5[PLLCLKEN] is set to 1.
//<i>
//<i> FLL Bypassed Low Power External (BLPE/FBELP)
//<i> In BLPE mode, ICSOUT is derived from the external reference clock. The FLL is disabled and PLL is disabled even if the
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
#define CLOCK_MODE_FBILP    4
#define CLOCK_MODE_FBE      5
#define CLOCK_MODE_FBELP    6

// FLL_TARGET_CLOCK =======================================
//
//  <o> FLL Output clock frequency (Hz) <name=fllTargetFrequency>
//  <i> Used for ICSFLLCLK system clock
//  <i> Used for main ICSOUTCLK system clock if FEI or FEE mode is selected.
#define FLL_TARGET_CLOCK 32000000UL

// SYSTEM_ICSOUT_CLOCK =======================================
//
//  <o> System ICSOUT Clock (Hz) <name=system_icsout_clock> <constant>
//  <i> ICS Main clock output
//  <i> Derived from slow IRC, fast IRC, ERC, FLL or PLL
#define SYSTEM_ICSOUT_CLOCK 32000000UL

// SYSTEM_ICSIR_CLOCK =======================================
//
//  <o> System ICSIR Clock (Hz) <name=system_icsir_clock> <constant>
//  <i> ICS Internal Reference Clock
//  <i> Derived from slow IRC or fast IRC
#define SYSTEM_ICSIR_CLOCK 31250UL

// SYSTEM_CORE_CLOCK =======================================
//
//  <o> System Core Clock (Hz) <name=system_core_clock> <constant>
//  <i> Clocks the ARM Cortex-M4 core
//  <i> From ICSOUT Clock
//  <i> Must be less than or equal to 40 MHz.
#define SYSTEM_CORE_CLOCK 32000000UL

// SYSTEM_BUS_CLOCK =======================================
//
//  <o> System Bus and Flash Clock (Hz) <name=system_bus_clock> <constant>
//  <i> Clocks the bus slaves & peripheral and flash
//  <i> Derived from Core Clock after division by BUSDIV
//  <i> Must be less than or equal to 20 MHz.
#define SYSTEM_BUS_CLOCK 16000000UL

// <h> System Clock dividers
// SIM_BUSDIV_BUSDIV ================================
//
//   <o> Core & System Clock Divider (BUSDIV) - Divide by <1-2> <#-1> <name=sim_busdiv_busdiv>
//   <i> Clocks the ARM Cortex-M4 core and bus masters [SIM_BUSDIV_BUSDIV]
//   <i> ICSOUTCLK clock is source. Default /2
#define SIM_BUSDIV_BUSDIV_V (1)
#define SIM_BUSDIV_BUSDIV_M (SIM_BUSDIV_BUSDIV_V<<SIM_BUSDIV_BUSDIV_SHIFT)

// </h>

//========================================================================================
//========================================================================================
// <h> OSC Control Register (OSC_CR)

// OSC_CR_OSCEN ===============================
//
//   <q> OSC Enable (OSCEN)
//   <i> Enables the OSC module. The OSC module can also be enabled by the ICS module. [OSC_CR_OSCEN]
//     <0=> Disabled
//     <1=> Enabled
#define OSC_CR_OSCEN_V 0
#define OSC_CR_OSCEN_M (OSC_CR_OSCEN_V<<OSC_CR_OSCEN_SHIFT)

// OSC_CR_OSCSTEN ===============================
//
//   <q> OSC Enable in Stop mode (OSCSTEN)
//   <i> Controls whether or not the OSC clock remains enabled when MCU enters Stop mode and OSCEN is set. [OSC_CR_OSCSTEN]
//     <0=> Disabled in Stop mode
//     <1=> Enabled in Stop mode
#define OSC_CR_OSCSTEN_V 0
#define OSC_CR_OSCSTEN_M (OSC_CR_OSCSTEN_V<<OSC_CR_OSCSTEN_SHIFT)

// OSC_CR_OSCOS ===============================
//
//   <q> OSC Output Select (OSCOS) <name=osc_cr_oscos>
//   <i> Selects the output clock of the OSC module. [OSC_CR_OSCOS]
//     <0=> External clock source from EXTAL pin is selected.
//     <1=> Oscillator clock source is selected
#define OSC_CR_OSCOS_V 1
#define OSC_CR_OSCOS_M (OSC_CR_OSCOS_V<<OSC_CR_OSCOS_SHIFT)

// OSC_CR_RANGE =============================
//
//   <o> Frequency Range Select (RANGE) <constant> <name=osc_cr_range>
//   <i> Selects the frequency range for the crystal oscillator or external clock source [OSC_CR_RANGE]
//   <i> This value is calculated from the ICS input clock frequency
//      <0=> Low range
//      <1=> High range
#define OSC_CR_RANGE_V   1
#define OSC_CR_RANGE_M   (OSC_CR_RANGE_V<<OSC_CR_RANGE_SHIFT)

// OSC_CR_HGO =============================
//
//   <q> High Gain Oscillator Select (HGO)
//   <i> Controls the crystal oscillator mode of operation [OSC_CR_HGO]
//      <0=> Low power
//      <1=> High gain
#define OSC_CR_HGO_V   0
#define OSC_CR_HGO_M   (OSC_CR_HGO_V<<OSC_CR_HGO_SHIFT)

// </h>

//========================================================================================
//========================================================================================
// <h> ICS Control Register 1 (ICS_C1)

// ICS_C1_CLKS =======================================
//
//  <o> ICSOUT Clock Source Select (CLKS) <0-2> <constant> <name=ics_c1_clks>
//  <i> Selects the clock source for ICSOUTCLK [ICS_C1_CLKS]
//  <i> This option is determined by the Clock Mode selection
//      <0=> Output of FLL is selected
//      <1=> Internal reference clock is selected
//      <2=> External reference clock is selected
#define ICS_C1_CLKS_V 0
#define ICS_C1_CLKS_M (ICS_C1_CLKS_V<<ICS_C1_CLKS_SHIFT)

// ICS_C1_RDIV =======================================
//
//   <o> FLL External Reference Divider (RDIV) <0-7> <constant> <name=ics_c1_rdiv>
//   <i> Selects the amount to divide down the external reference clock for the FLL. [ICS_C1_RDIV]
//   <i> The resulting frequency must be in the range 31.25 kHz to 39.0625 kHz
//   <i> Division factor depends on Clock Range [OSC_CR_RANGE0]
//   <i> This option is determined by the Clock Mode and FLL input clock
//      <0=> Divide by 1 or 32
//      <1=> Divide by 2 or 64
//      <2=> Divide by 4 or 128
//      <3=> Divide by 8 or 256
//      <4=> Divide by 16 or 512
//      <5=> Divide by 32 or 1024
//      <6=> Divide by 64
//      <7=> Divide by 128
#define ICS_C1_RDIV_V 3
#define ICS_C1_RDIV_M (ICS_C1_RDIV_V<<ICS_C1_RDIV_SHIFT)

// ICS_C1_IREFS ================================
//
//   <o> Internal Reference Select (IREFS) <constant> <name=ics_c1_irefs>
//   <i> Selects the reference clock source for the FLL. [ICS_C1_IREFS]
//   <i> This option is determined by the Clock Mode selection
//      <0=> External Reference Clock
//      <1=> Internal Reference Clock
#define ICS_C1_IREFS_V 1
#define ICS_C1_IREFS_M (ICS_C1_IREFS_V<<ICS_C1_IREFS_SHIFT)

// ICS_C1_IRCLKEN ==============================
//
//   <q> Internal Reference Clock Enable (IRCLKEN)
//   <i> Enables the internal reference clock for use as ICSIRCLK. [ICS_C1_IRCLKEN]
//      <0=> Inactive
//      <1=> Active
#define ICS_C1_IRCLKEN_V   0
#define ICS_C1_IRCLKEN_M   (1<<ICS_C1_IRCLKEN_SHIFT)

// ICS_C1_IREFSTEN =============================
//
//   <q> Internal Reference Stop Enable (IREFSTEN)
//   <i> Determines if IRS is enabled in Stop mode [ICS_C1_IREFSTEN]
//      <0=> IR disabled in STOP
//      <1=> IR enabled in STOP if IRCLKEN is set or in FEI, FBI, or FBILP mode
#define ICS_C1_IREFSTEN_V   0
#define ICS_C1_IREFSTEN_M   (ICS_C1_IREFSTEN_V<<ICS_C1_IREFSTEN_SHIFT)

// </h>

//========================================================================================
//========================================================================================
// <h> ICS Control Register 2 (ICS_C2)

// ICS_C2_BDIV =============================
//
//   <o> Bus Frequency Divider (BDIV) <0-7> <name=ics_c2_bdiv>
//   <i> Selects the amount to divide down the clock source selected by ICS_C1_CLKS [ICS_C2_BDIV]
//   <i> This controls the bus frequency.
//      <0=> Divide by 1
//      <1=> Divide by 2 (reset default)
//      <2=> Divide by 4
//      <3=> Divide by 8
//      <4=> Divide by 16
//      <5=> Divide by 32
//      <6=> Divide by 64
//      <7=> Divide by 128
#define ICS_C2_BDIV_V  0
#define ICS_C2_BDIV_M (ICS_C2_BDIV_V<<ICS_C2_BDIV_SHIFT)

// ICS_C2_LP =============================
//
//   <o> Low Power Select (LP) <constant> <name=ics_c2_lp>
//   <i> Whether FLL continues operation when bypassed [ICS_C2_LP]
//   <i> This option is determined by the Clock Mode selection
//      <0=> FLL is enabled in bypass modes
//      <1=> FLL is disabled in bypass modes
#define ICS_C2_LP_V  0
#define ICS_C2_LP_M (ICS_C2_LP_V<<ICS_C2_LP_SHIFT)

// </h>

//========================================================================================
//========================================================================================
// <h> ICS Control Register 4 (ICS_C4)

// ICS_C4_LOLIE =============================
//
//   <q> Loss of Lock Interrupt (LOLIE)
//   <i> Determines if an interrupt request is made following a loss of lock indication. [ICS_C4_LOLIE]
//      <0=> No request on loss of lock.
//      <1=> Request on loss of lock.
#define ICS_C4_LOLIE_V (0)
#define ICS_C4_LOLIE_M (ICS_C4_LOLIE_V<<ICS_C4_LOLIE0_SHIFT)

// ICS_C4_CME =============================
//
//   <q> Clock Monitor Enable (CME)
//   <i> Determines if a reset request is made following a loss of external clock indication. [ICS_C4_CME]
//   <i> This field must be set to a logic 1 only when the ICS is in an operational mode that uses the external clock (FEE, FBE, or FBELP).
//      <0=> Clock monitor is disabled.
//      <1=> Generates a reset request on loss of external clock.
#define ICS_C4_CME_V (0)
#define ICS_C4_CME_M (ICS_C4_CME_V<<ICS_C4_CME_SHIFT)

// </h>

#define SYSTEM_OSCER_CLOCK    OSCCLK_CLOCK         // External reference clock

// ERC_AFTER_RDIV_CLOCK = External reference clock after dividers
#if (OSC_CR_RANGE_V == 0)
#define ERC_AFTER_RDIV_CLOCK (SYSTEM_OSCER_CLOCK/(1<<ICS_C1_RDIV_V))
#else
#define ERC_AFTER_RDIV_CLOCK (SYSTEM_OSCER_CLOCK/(1<<(ICS_C1_RDIV_V+5)))
#endif

#if (ERC_AFTER_RDIV_CLOCK > 39062UL) || (ERC_AFTER_RDIV_CLOCK < 31250UL)
#error "External reference clock must be in range 31.25 kHz to 39.0625 kHz"
#endif

// SYSTEM_ICSFF_CLOCK  ==============================
#if (ICS_C1_IREFS_V == 0)
#define SYSTEM_ICSFF_CLOCK ERC_AFTER_RDIV_CLOCK  // External Reference clock after dividers
#else
#define SYSTEM_ICSFF_CLOCK SYSTEM_IRC_CLOCK  // Internal clock (nominally ~31.25 kHz)
#endif

// SYSTEM_PERIPHERAL_CLOCK  ==============================
// <h> Peripheral Clock Source Selection

#ifdef RTC_SC
// RTC_SC_RTCLKS ================================
//
//   <o> Real-Time Clock Source Select (RTCLKS)
//   <i> This field selects the clock source input to the RTC prescaler. [RTC_SC_RTCLKS]
//     <0=> External clock source
//     <1=> Low Power Oscillator ~1 kHz (LPOCLK)
//     <2=> Internal reference clock (ICSIRCLK)
//     <3=> Bus clock
#define RTC_SC_RTCLKS_V (0)
#define RTC_SC_RTCLKS_M (RTC_SC_RTCLKS_V<<RTC_SC_RTCLKS_SHIFT)

#if RTC_SC_RTCLKS_V == (0)
#define SYSTEM_RTC_CLOCK RTC_CLKIN_CLOCK
#elif RTC_SC_RTCLKS_V == (1)
#define SYSTEM_RTC_CLOCK SYSTEM_LOW_POWER_CLOCK
#elif RTC_SC_RTCLKS_V == (2)
#define SYSTEM_RTC_CLOCK SYSTEM_ICSIR_CLOCK
#elif RTC_SC_RTCLKS_V == (3)
#define SYSTEM_RTC_CLOCK SYSTEM_BUS_CLOCK
#else
#error "Invalid RTC_SC_RTCLKS clock selected"
#endif
#else
#define SYSTEM_ERCLK32_CLOCK SYSTEM_OSC32KCLK_CLOCK   // Main/32kHz Oscillator (depends on chip)
#endif

#define SYSTEM_UART0_CLOCK SystemBusClock
#define SYSTEM_UART1_CLOCK SystemBusClock
#define SYSTEM_UART2_CLOCK SystemBusClock

// TODO check other sources
#define SYSTEM_FTM0_CLOCK SystemBusClock
#define SYSTEM_FTM0_CLOCK SystemBusClock
#define SYSTEM_FTM0_CLOCK SystemBusClock
// </h>

// SYSTEM_OSCER_CLOCK ================================
// Always connected to main oscillator
#define SYSTEM_OSCER_CLOCK OSCCLK_CLOCK

// ICSFLLCLK_CLOCK  ==============================
#define SYSTEM_ICSFLL_CLOCK FLL_TARGET_CLOCK

/*
 * The following are 'public' definitions
 *
 * SYSTEM_ICSIR_CLOCK    ICS Internal Reference clock (gated by IRCLKEN)
 * SYSTEM_ICSFF_CLOCK    ICS Fixed Frequency ! (from IRC/ERC_AFTER_RDIV_CLOCK)
 * SYSTEM_ICSFLL_CLOCK   ICS FLL clock output
 * SYSTEM_ICSOUT_CLOCK   ICS OUT clock (from IRC/ExternalRC/FLL depending on CLKS)
 * SYSTEM_OSCER_CLOCK    Main oscillator/clock
 * SYSTEM_BUS_CLOCK
 * SYSTEM_CORE_CLOCK
 */

void clock_initialise(void);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_PRIVATE_H_ */
