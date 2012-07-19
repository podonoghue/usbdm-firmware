/*! \file
    \brief USBDM - Common BDM routines.

   \verbatim
   This software was modified from \e TBLCF software
   This software was modified from \e TBDML software

   USBDM
   Copyright (C) 2007  Peter O'Donoghue

   Turbo BDM Light
   Copyright (C) 2005  Daniel Malik

   Turbo BDM Light ColdFire
   Copyright (C) 2005  Daniel Malik

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   \endverbatim

   \verbatim
   Change History
   +================================================================================================
   | 22 Nov 2011 | More thoroughly disabled interfaces when off                       - pgo, ver 4.8 
   | 27 Oct 2011 | Modified timer code to avoid TSCR1 changes & TCNT resets           - pgo, ver 4.8 
   |  8 Aug 2010 | Re-factored interrupt handling                                     - pgo 
   | 10 Apr 2010 | Changed to accommodate changes to Vpp interface                    - pgo 
   |  5 Feb 2010 | bdm_cycleTargetVdd() now disables Vdd monitoring                   - pgo
   |  4 Feb 2010 | bdm_cycleTargetVdd() parametised for mode                          - pgo
   | 19 Oct 2009 | Modified Timer code - Folder together with JS16 code               - pgo
   | 20 Sep 2009 | Increased Reset wait                                               - pgo
   |    Sep 2009 | Major changes for V2                                               - pgo
   +================================================================================================
   \endverbatim
*/

#include <hidef.h> /* for EnableInterrupts macro */
#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "BDMCommon.h"
#include "BDM.h"
#include "BDM_CF.h"
#include "BDM_RS08.h"
#include "CmdProcessing.h"

//=============================================================================================================================================
#define SOFT_RESETus       1280U //!< us - longest time needed for soft reset of the BDM interface (512 BDM cycles @ 400kHz = 1280us)

//! Status of the BDM
//!
static const CableStatus_t cable_statusDefault =  {
   T_OFF,             // target_type
   WAIT,              // ackn
   NO_RESET_ACTIVITY, // reset
   SPEED_NO_INFO,     // speed
   0,                 // power
   0,                 // wait150_cnt
   0,                 // wait64_cnt
   0,                 // sync_length
   0,                 // bdmpprValue
};

//=========================================================================
// Timer routines
//
//=========================================================================

//! Wait for given time in timer ticks
//!
//!  @param delay Delay time in fast timer ticks
//!
//!  @note Limited to 2 ms
//!
void fastTimerWait(U16 delay) {
   TIMEOUT_TPMxCnVALUE   = TPMCNT+delay;           // Schedule event NOW+delay
   TIMEOUT_TPMxCnSC_CHF  = 0;                      // Clear timeout flag
   while (TIMEOUT_TPMxCnSC_CHF==0) {               // Wait for timeout
   }
}

//! Wait for given time in milliseconds
//!
//!  @param delay Delay time in milliseconds
//!
void millisecondTimerWait(U16 delay) {
   TIMEOUT_TPMxCnVALUE = TPMCNT+TIMER_MICROSECOND(1000); // Set initial timeout value
   while (delay-->0) {
      TIMEOUT_TPMxCnSC_CHF = 0;                    // Clear timeout flag
      while (TIMEOUT_TPMxCnSC_CHF==0) {            // Wait for timeout
      }
      TIMEOUT_TPMxCnVALUE   += TIMER_MICROSECOND(1000);  // Schedule next timeout value
   }
}

//! Initialises the timers, input captures and interrupts
//!
U8 initTimers(void) {

   //====================================================================
   // Set up timers
   TPMSC      = TIMER_TPMxSC_VALUE;     // Set timer tick rate

   // Set up Input capture & timeout timers
   TIMEOUT_TPMxCnSC    = TIMEOUT_TPMxCnSC_OC_MASK;         // TPMx.CHa : Output compare no pin
   
#if (HW_CAPABILITY&CAP_BDM)
   BKGD_TPMxCnSC       = BKGD_TPMxCnSC_FALLING_EDGE_MASK;  // TPMx.CHb : Input capture, falling edge on pin
#endif
	   
#if (HW_CAPABILITY&CAP_VDDSENSE)
   //====================================================================
   // Set up Vdd monitoring (ACMP interrupts or Input capture)
   // Clear existing flag, enable falling & rising transitions (Vdd rising & falling!), enable Bandgap (~1.2V)
   CONFIGURE_VDD_SENSE();         // Capture Vdd rising & falling edges
   CLEAR_VDD_SENSE_FLAG();        // Clear Vdd Change Event
   ENABLE_VDD_SENSE_INT();        // Enable Vdd IC interrupts
#endif

#if (HW_CAPABILITY&CAP_RST_IO)
   //===================================================================
   // Setup RESET detection (Input Capture or keyboard interrupt)
   if (bdm_option.useResetSignal) {
      CONFIGURE_RESET_SENSE();         // Capture RESET falling edges
      CLEAR_RESET_SENSE_FLAG();        // Clear RESET IC Event
      ENABLE_RESET_SENSE_INT();        // Enable RESET IC interrupts
      }
   else
      DISABLE_RESET_SENSE_INT();       // Disable RESET IC interrupts
#endif

   cable_status.reset = NO_RESET_ACTIVITY;  // Clear the reset detection flag

   return BDM_RC_OK;
}

//=========================================================================
// Target monitoring and status routines
//
//=========================================================================

//! Interrupt function servicing the IC interrupt from Vdd changes
//! This routine has several purposes:
//!  - Triggers POR into Debug mode on HCS08/RS08 targets\n
//!  - Turns off Target power on short circuits\n
//!  - Updates Target power status\n
//!
void bdm_targetVddSense(void) {

#if (HW_CAPABILITY&CAP_VDDSENSE)
   CLEAR_VDD_SENSE_FLAG(); // Clear Vdd Change Event

   if (VDD_SENSE) {  // Vdd rising
      // Needs to be done on non-interrupt thread?
      switch (cable_status.target_type) {
#if (HW_CAPABILITY&CAP_BDM)    	  
         case    T_HC12:
         case    T_HCS08:
         case    T_RS08:
         case    T_CFV1:
            (void)bdmHCS_powerOnReset();
            break;
#endif
#if (HW_CAPABILITY&CAP_CFVx_HW)
         case    T_CFVx:
            (void)bdmCF_powerOnReset();
            break;
#endif
         case    T_JTAG:
         case    T_EZFLASH:
         case    T_MC56F80xx:
         case    T_ARM_JTAG:
         case    T_OFF:
         default:
            break;
      }
      }
   else { // Vdd falling
      VDD_OFF();   // Turn off Vdd in case it's an overload
      }
   // Update power status
   (void)bdm_checkTargetVdd();
#endif // CAP_VDDSENSE
}

#if (HW_CAPABILITY&CAP_RST_IO)
//! Interrupt function servicing the IC interrupt from RESET_IN assertion
//!
void bdm_resetSense(void) {
   CLEAR_RESET_SENSE_FLAG();             // Acknowledge RESET IC Event
   if (RESET_IS_LOW)
      cable_status.reset = RESET_DETECTED;  // Record that reset was asserted
}
#endif

//! Keyboard interrupt (KBI) handler
//!
//! Reset sensing on JS16CWJ(KBIP3), 56F8006Demo(KBIP2)
//! Target Vdd sense on 56F8006Demo(KBIP5) - incomplete
#pragma TRAP_PROC
void kbiHandler(void) {
#if (HW_CAPABILITY&CAP_RST_IO)
	bdm_resetSense();
#endif	
#if TARGET_HARDWARE==H_USBDM_MC56F8006DEMO   
//	bdm_targetVddSense();
#endif   
}

//! Timer interrupt handling
//!
//! Reset sensing on CF_JMxxCLD, JMxxCLD, JMxxCLC
//!
#pragma TRAP_PROC
void timerHandler(void) {
#if (HW_CAPABILITY&CAP_RST_IO)
	bdm_resetSense();
#endif
}

//! Analogue comparator handling
//!
//! Target Vdd sensing on CF_JMxxCLD, JMxxCLD, JMxxCLC
//!
#pragma TRAP_PROC
void acmpHandler(void) {
	// bdm_targetVddSense() on CF_JMxxCLD, JMxxCLD, JMxxCLC
	bdm_targetVddSense();
}

//=========================================================================
// Target power control
//
//=========================================================================

#define VDD_2v  (((2*255)/5)*9/10)  // 10% allowance on 2V
#define VDD_3v3 (((3*255)/5)*9/10)  // 10% allowance on 3.3V
#define VDD_5v  (((5*255)/5)*9/10)  // 10% allowance on 5V

//!  Checks Target Vdd  - Updates Target Vdd LED & status
//!
//!  Updates \ref cable_status
//!
U8 bdm_checkTargetVdd(void) {
#if (HW_CAPABILITY&CAP_VDDSENSE)
   if (bdm_targetVddMeasure() > VDD_2v) {
      RED_LED_ON();
      if (bdm_option.targetVdd == BDM_TARGET_VDD_OFF)
         cable_status.power = BDM_TARGET_VDD_EXT;
      else
         cable_status.power = BDM_TARGET_VDD_INT;
   }
   else {
      RED_LED_OFF();
      if (bdm_option.targetVdd == BDM_TARGET_VDD_OFF)
         cable_status.power = BDM_TARGET_VDD_NONE;
      else {
    	 // Possible overload
         cable_status.power = BDM_TARGET_VDD_ERR;
         VDD_OFF();
      }
   }
#else
   // No target Vdd sensing - assume external Vdd is present
   cable_status.power = BDM_TARGET_VDD_EXT;
#endif // CAP_VDDSENSE
   if ((cable_status.power == BDM_TARGET_VDD_NONE) ||
       (cable_status.power == BDM_TARGET_VDD_ERR))
      return BDM_RC_VDD_NOT_PRESENT;
   return BDM_RC_OK;
}

#if (HW_CAPABILITY&CAP_VDDCONTROL)

//! Turns on Target Vdd if enabled.
//!
//!  @return
//!   \ref BDM_RC_OK                => Target Vdd confirmed on target \n
//!   \ref BDM_RC_VDD_NOT_PRESENT   => Target Vdd not present
//!
U8 bdm_setTargetVdd( void ) {
U8 rc = BDM_RC_OK;

#if (HW_CAPABILITY&CAP_VDDSENSE)
   DISABLE_VDD_SENSE_INT();
#endif
   
   switch (bdm_option.targetVdd) {
   case BDM_TARGET_VDD_OFF :
	   VDD_OFF();
	   // Check for externally supplied target Vdd (> 2 V)
	   WAIT_US(VDD_RISE_TIMEus); // Wait for Vdd to rise & stabilise
	   if (bdm_targetVddMeasure()<VDD_2v)
		   rc = BDM_RC_VDD_NOT_PRESENT;
	   break;
   case BDM_TARGET_VDD_3V3 :
	   VDD3_ON();
	   // Wait for Vdd to rise to 90% of 3V
	   WAIT_WITH_TIMEOUT_MS( 100 /* ms */, (bdm_targetVddMeasure()>VDD_3v3));
	   WAIT_US(VDD_RISE_TIMEus); // Wait for Vdd to rise & stabilise
	   if (bdm_targetVddMeasure()<VDD_3v3) {
		   VDD_OFF(); // In case of Vdd overload
		   rc = BDM_RC_VDD_NOT_PRESENT;
	   }
	   break;
   case BDM_TARGET_VDD_5V  :
	   VDD5_ON();
	   // Wait for Vdd to rise to 90% of 5V
	   WAIT_WITH_TIMEOUT_MS( 100 /* ms */, (bdm_targetVddMeasure()>VDD_5v));
	   WAIT_US(VDD_RISE_TIMEus); // Wait for Vdd to rise & stabilise
	   if (bdm_targetVddMeasure()<VDD_5v) {
		   VDD_OFF(); // In case of Vdd overload
		   rc = BDM_RC_VDD_NOT_PRESENT;
	   }
	   break;
   }
#if (HW_CAPABILITY&CAP_VDDSENSE)
   CLEAR_VDD_SENSE_FLAG(); // Clear Vdd Change Event
   ENABLE_VDD_SENSE_INT();
#endif

   (void)bdm_checkTargetVdd(); // Update Target Vdd LED & status
   return (rc);
}

#endif // CAP_VDDCONTROL

//!  Cycle power ON to target
//!
//! @param mode
//!    - \ref RESET_SPECIAL => Power on in special mode,
//!    - \ref RESET_NORMAL  => Power on in normal mode
//!
//!  BKGD/BKPT is held low when power is re-applied to start
//!  target with BDM active if RESET_SPECIAL
//!
//!  @return
//!   \ref BDM_RC_OK                	=> Target Vdd confirmed on target \n
//!   \ref BDM_RC_VDD_WRONG_MODE    	=> Target Vdd not controlled by BDM interface \n
//!   \ref BDM_RC_VDD_NOT_PRESENT   	=> Target Vdd failed to rise 		\n
//!   \ref BDM_RC_RESET_TIMEOUT_RISE    => RESET signal failed to rise 		\n
//!   \ref BDM_RC_BKGD_TIMEOUT      	=> BKGD signal failed to rise
//!
U8 bdm_cycleTargetVddOn(U8 mode) {
U8 rc = BDM_RC_OK;

   mode &= RESET_MODE_MASK;

#if (HW_CAPABILITY&CAP_VDDCONTROL)

   switch(cable_status.target_type) {
#if (HW_CAPABILITY&CAP_CFVx_HW)
   case T_CFVx:
      bdmcf_interfaceIdle();  // Make sure BDM interface is idle
      if (mode == RESET_SPECIAL)
         BKPT_LOW();
      break;
#endif
#if (HW_CAPABILITY&CAP_BDM)     
   case T_HC12:
   case T_HCS08:
   case T_RS08:
   case T_CFV1:
      bdmHCS_interfaceIdle();  // Make sure BDM interface is idle
      if (mode == RESET_SPECIAL) {
         BDM_LOW();  // BKGD pin=L
      }
      break;
#endif      
#if (HW_CAPABILITY&CAP_JTAG_HW)     
   case T_JTAG:
   case T_MC56F80xx:
   case T_ARM_JTAG:
      jtag_interfaceIdle();  // Make sure BDM interface is idle
#endif      
      break;
   }
#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
#endif //  (DEBUG&CYCLE_DEBUG)

   // Power on with TargetVdd monitoring off
   rc = bdm_setTargetVdd();
   if (rc != BDM_RC_OK) // No target Vdd
      goto cleanUp;

#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 1;
   DEBUG_PIN     = 0;
#endif //  (DEBUG&CYCLE_DEBUG)
#if (HW_CAPABILITY&CAP_RST_IO)
   // RESET rise may be delayed by target POR
   if (bdm_option.useResetSignal) {
      WAIT_WITH_TIMEOUT_S( 2 /* s */, (RESET_IN!=0) );
   }
#endif
#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN   = 0;
   DEBUG_PIN   = 1;
#endif // (DEBUG&CYCLE_DEBUG)

   // Let signals settle & CPU to finish reset (with BKGD held low)
   WAIT_US(BKGD_WAITus);

#if (HW_CAPABILITY&CAP_RST_IO)
   if (bdm_option.useResetSignal && (RESET_IN==0)) {
      // RESET didn't rise
      rc = BDM_RC_RESET_TIMEOUT_RISE;
      goto cleanUp;
      }
#endif //(HW_CAPABILITY&CAP_RST_IO)

#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 1;
   DEBUG_PIN     = 0;
#endif // (DEBUG&CYCLE_DEBUG)

#if (HW_CAPABILITY&CAP_CFVx_HW)
   if  (cable_status.target_type == T_CFVx)
      bdmcf_interfaceIdle();  // Release BKPT etc
   else
#endif
#if (HW_CAPABILITY&CAP_BDM)     
      bdmHCS_interfaceIdle();  // Release BKGD
#endif
   // Let processor start up
   WAIT_MS(RESET_RECOVERYms);

#if 0
// Removed - some targets may be holding BKGD low (e.g. used as port pin)
// This situation is handled elsewhere (requires power cycle)
   if (BDM_IN==0) { // BKGD didn't rise!
      rc = BDM_RC_BKGD_TIMEOUT;
      goto cleanUp;
      }
#endif // 0

   cable_status.reset  = RESET_DETECTED; // Cycling the power should have reset it!

cleanUp:
#if (HW_CAPABILITY&CAP_CFVx_HW)
   if  (cable_status.target_type == T_CFVx)
      bdmcf_interfaceIdle();  // Release BKPT etc
   else
#endif
#if (HW_CAPABILITY&CAP_BDM)     
      bdmHCS_interfaceIdle();  // Release BKGD
#endif
   
   WAIT_MS( 250 /* ms */);

//   EnableInterrupts;
#endif // CAP_VDDCONTROL

   (void)bdm_checkTargetVdd(); // Update Target Vdd LED & power status

   return(rc);
}

//!  Cycle power OFF to target
//!
//!  @return
//!   \ref BDM_RC_OK                => No error  \n
//!   \ref BDM_RC_VDD_WRONG_MODE    => Target Vdd not controlled by BDM interface \n
//!   \ref BDM_RC_VDD_NOT_REMOVED   => Target Vdd failed to fall \n
//!
U8 bdm_cycleTargetVddOff(void) {
U8 rc = BDM_RC_OK;

#if (HW_CAPABILITY&CAP_VDDCONTROL)

   (void)bdm_checkTargetVdd();

   if (bdm_option.targetVdd == BDM_TARGET_VDD_OFF)
      return BDM_RC_VDD_WRONG_MODE;

#if (HW_CAPABILITY&CAP_CFVx_HW)
   if  (cable_status.target_type == T_CFVx)
      bdmcf_interfaceIdle();  // Make sure BDM interface is idle
   else
#endif 
   {
#if (HW_CAPABILITY&CAP_BDM)    	  
	  bdmHCS_interfaceIdle();  // Make sure BDM interface is idle
#endif
   }
#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
#endif

   // Power off & wait for Vdd to fall to ~5%
   VDD_OFF();
   WAIT_WITH_TIMEOUT_S( 5 /* s */, (bdm_targetVddMeasure()<10) );

#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   if (bdm_targetVddMeasure()>=15) // Vdd didn't turn off!
      rc = BDM_RC_VDD_NOT_REMOVED;

#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
#endif

   (void)bdm_checkTargetVdd(); // Update Target Vdd LED

   // Wait a while with power off
   WAIT_US(RESET_SETTLEms);

   // Clear Vdd monitoring interrupt
#if (HW_CAPABILITY&CAP_VDDSENSE)
   CLEAR_VDD_SENSE_FLAG();  // Clear Vdd monitoring flag
#endif
   (void)bdm_checkTargetVdd();    // Update Target Vdd LED

#endif // CAP_VDDCONTROL

   return(rc);
}

//!  Cycle power to target
//!
//! @param mode
//!    - \ref RESET_SPECIAL => Power on in special mode,
//!    - \ref RESET_NORMAL  => Power on in normal mode
//!
//!  BKGD/BKPT is held low when power is re-applied to start
//!  target with BDM active if RESET_SPECIAL
//!
//!  @return
//!   \ref BDM_RC_OK                 => No error \n
//!   \ref BDM_RC_VDD_WRONG_MODE     => Target Vdd not controlled by BDM interface \n
//!   \ref BDM_RC_VDD_NOT_REMOVED    => Target Vdd failed to fall \n
//!   \ref BDM_RC_VDD_NOT_PRESENT    => Target Vdd failed to rise \n
//!   \ref BDM_RC_RESET_TIMEOUT_RISE => RESET signal failed to rise \n
//!
U8 bdm_cycleTargetVdd(U8 mode) {
U8 rc;

   // This may take a while
   setBDMBusy();

   rc = bdm_cycleTargetVddOff();
   if (rc != BDM_RC_OK)
      return rc;
   WAIT_MS(1000);
   rc = bdm_cycleTargetVddOn(mode);
   return rc;
}

//!  Measures Target Vdd
//!
//!  @return
//!  8-bit value representing the Target Vdd, N ~ (N/255) * 5V \n
//!  On JM60 hardware the internal ADC is used.
//!  JB16/UF32 doesn't have an ADC so an external comparator is used.  In this case this routine only
//!  returns an indication if Target Vdd is present [255 => Vdd present, 0=> Vdd not present].
//!
U16 bdm_targetVddMeasure(void) {

#if ((HW_CAPABILITY&CAP_VDDSENSE) == 0)
   // No Target Vdd measurement - Assume external Vdd supplied
   return 255;
#elif defined(VDD_MEASURE_CHANNEL)
	#if (VDD_MEASURE_CHANNEL==5)
	   int timeout = 1000;
	
	   ADCCFG = ADCCFG_ADLPC_MASK|ADCCFG_ADIV1_MASK|ADCCFG_ADIV0_MASK|
				ADCCFG_ADLSMP_MASK|ADCCFG_ADIV1_MASK|ADCCFG_ADIV0_MASK|ADCCFG_ADICLK0_MASK;
	   APCTL1_ADPC5 = 1;  // Using channel 5
	   ADCSC2 = 0;        // Single software triggered conversion
	   ADCSC1 = 5;        // Trigger single conversion on Channel
	   while ((timeout-->0) && (ADCSC1_COCO == 0)) {
	   }
	#elif (VDD_MEASURE_CHANNEL==9)
	   int timeout = 1000;
	
	   ADCCFG = ADCCFG_ADLPC_MASK|ADCCFG_ADIV1_MASK|ADCCFG_ADIV0_MASK|
				ADCCFG_ADLSMP_MASK|ADCCFG_ADIV1_MASK|ADCCFG_ADIV0_MASK|ADCCFG_ADICLK0_MASK;
	   APCTL2 = APCTL2_ADPC9_MASK; // Using channel 9
	   ADCSC2 = 0;                 // Single software triggered conversion
	   ADCSC1 = 9;                 // Trigger single conversion on Channel
	   while ((timeout-->0) && (ADCSC1_COCO == 0)) {
	   }
	#else
	#error "Invalid VDD_MEASURE_CHANNEL definition"
	#endif

#ifdef VDD_HAS_DIVIDER
	   return VDD_HAS_DIVIDER*ADCR;
#else
	   return ADCR;
#endif
#else
   // Simple yes/no code as JB16/JS16/JB8 doesn't have a ADC
   VDD_SENSE_DDR = 0;
   asm {
      nop
      nop
      nop
   }
   return (VDD_SENSE?255:0); 
#endif
}

//=========================================================================
// Common BDM routines
//
//=========================================================================

//! Once off initialisation
//!
void bdm_init(void) {

   // Turn off important things
#if (HW_CAPABILITY&CAP_FLASH)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif   
   VDD_OFF();
   cable_status.target_type = T_OFF;
   (void)initTimers(); // Initialise Timer system & input monitors
   // Update power status
   (void)bdm_checkTargetVdd();
}

//!  Sets the BDM interface to a suspended state
//!
//!  - All signals idle \n
//!  - All voltages off.
//!
void bdm_suspend(void){
#if (HW_CAPABILITY&CAP_FLASH)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif
#if (HW_CAPABILITY&CAP_CFVx_HW)
   bdmCF_suspend();
#endif
#if (HW_CAPABILITY&CAP_BDM)    	  
   bdmHCS_suspend();
#endif   
}

//!  Turns off the BDM interface
//!
void bdm_interfaceOff( void ) {
#if (HW_CAPABILITY&CAP_CFVx)
#ifndef SPI1C1
#define SPI1C1 SPIC1
#endif
   SPI1C1 = 0; // Disable SPI1
#endif
#if (HW_CAPABILITY&CAP_FLASH)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif
#ifdef TA_3STATE
   TA_3STATE();
#endif
#ifdef JTAG_DRV_DISABLE
   JTAG_DRV_DISABLE();
#endif
#ifdef BKPT_DISABLE
   BKPT_DISABLE();
#endif
#ifdef RESET_3STATE
   RESET_3STATE();
#endif
#ifdef CF_DRV_DISABLE
   CF_DRV_DISABLE();
#endif
#ifdef TCLK_DISABLE
   TCLK_DISABLE();
#endif   
#ifdef DSCLK_DRV_DISABLE
   DSCLK_DRV_DISABLE();
#endif
#ifdef TRST_3STATE
   TRST_3STATE();
#endif
#ifdef TDI_DISABLE
   TDI_DISABLE();
#endif
#ifdef BDM_3STATE
   BDM_3STATE();
#endif
#ifdef TCLK_CTL_DISABLE
   TCLK_CTL_DISABLE();
#endif
}

//!  Turns off the BDM interface
//!
//!  Depending upon settings, may leave target power on.
//!
void bdm_off( void ) {
   bdm_interfaceOff();
   if (!bdm_option.leaveTargetPowered)
      VDD_OFF();
}

//! Initialises BDM module for the given target type
//!
//!  @param target = Target processor (see \ref TargetType_t)
//!
U8 bdm_setTarget(U8 target) {
U8 rc = BDM_RC_OK;

#ifdef RESET_OUT_PER
   RESET_OUT_PER    = 1;    // Holds RESET_OUT inactive when unused
   RESET_IN_PER     = 1;    // Needed for input level translation to 5V
#endif

   if (target == T_OFF) {
	   bdm_off();             // Turn off the interface
   }   
   bdm_interfaceOff();

   cable_status             = cable_statusDefault; // Set default status/settings
   cable_status.target_type = target; // Assume mode is valid

#if (HW_CAPABILITY&CAP_FLASH)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif

   rc = initTimers();         // re-init timers in case settings changed
   if (rc != BDM_RC_OK)
      return rc;

   switch (target) {
#if (TARGET_CAPABILITY & CAP_HCS12)   
      case T_HC12:
         bdm_option.useResetSignal = 1; // Must use RESET signal on HC12
         bdmHCS_init();
         break;
#endif         
#if (TARGET_CAPABILITY & CAP_RS08)
      case T_RS08:
#endif
#if (TARGET_CAPABILITY & CAP_HCS08)
      case T_HCS08:
#endif    	  
#if (TARGET_CAPABILITY & CAP_CFV1)
      case T_CFV1:
#endif    	  
#if (TARGET_CAPABILITY & (CAP_RS08|CAP_HCS08|CAP_CFV1))
         bdmHCS_init();
         break;
#endif    	  
#if (TARGET_CAPABILITY&CAP_CFVx)
      case T_CFVx:
         bdm_option.useResetSignal = 1; // Must use RESET signal on CFVx
         bdmcf_init();                  // Initialise the BDM interface
//         (void)bdmcf_resync();          // Synchronise with the target (ignore error?)
         break;
#endif
#if (TARGET_CAPABILITY&CAP_JTAG)
      case T_JTAG:
#endif
#if (TARGET_CAPABILITY&CAP_DSC)
      case T_MC56F80xx:
#endif
#if (TARGET_CAPABILITY&(CAP_JTAG|CAP_DSC))
         bdm_option.useResetSignal = 1; // Must use RESET signal on JTAG etc
         jtag_init();                   // Initialise JTAG
         break;
#endif
#if (TARGET_CAPABILITY&CAP_ARM_JTAG)
      case T_ARM_JTAG:
          jtag_init();                   // Initialise JTAG
          break;
#endif
      case T_OFF:
    	  return BDM_RC_OK;
    	  
      default:
         bdm_off();                        // Turn off the interface
         cable_status.target_type = T_OFF; // Safe mode!
         return BDM_RC_UNKNOWN_TARGET;
   }
   return rc;
}
