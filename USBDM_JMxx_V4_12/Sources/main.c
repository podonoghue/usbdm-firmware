/*! \file
    \brief Contains startup code and main loop.
    
    This file contains the main entry point for the program.
    It initialises the system and then remains in the main idle loop.
   
   The basic program flow:
   \li   All BDM activity is driven by commands received from the USB
   \li   All BDM commands are executed from within the ISRs servicing the USB IRQ
              (i.e. with interrupts disabled)
   \li   The only asynchronous activity is:
        \n      * Detection of external resets on RESET_IN pin through KBD interrupts
        \n      * Detection of Target Vdd changes on ?? pin through Timer input capture events
           
    \verbatim
    Open Source BDM/Turbo BMD Light
    
    Original TBDML Copyright (C) 2005  Daniel Malik
     **  Prominent Notice-This software was modified from TBDML software - 12/05  **

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

   Change History

+===============================================================================
| 18 Oct 2011 | Minor changes for TWR version (initial power etc)  - pgo v4.8
|  8 Oct 2011 | Added wait for xtal stabilisation                  - pgo v4.7.1
|             | Changed 8MHz auto threshold for JMxx devices       - pgo v4.7.1
| 10 Aug 2011 | Added SCI DEBUG code                               - pgo v4.7.1
|  1 Jan 2009 | Ported from USBDM                                  - pgo
| 17 Apr 2009 | Removed hardware ICP from user code                - pgo
| 23 Jul 2008 | Added ICP code                                     - pgo
| 23 Jul 2008 | Added Vector re-location                           - pgo
|  3 Mar 2008 | Started changes - lots                             - pgo
+===============================================================================
 \endverbatim
*/

#include <hidef.h> /* for EnableInterrupts macro */
#include "Common.h"
#include "ICP.h"
#include "Configure.h"
#include "Commands.h"
#include "USB.h"
#include "BDM.h"
#include "main.h"
#include "BDMCommon.h"
#include "CmdProcessing.h"
#include "SCI_Debug.h"
#include "BDM_RS08.h"

/*! \brief Detect In-Circuit Programming (ICP) mode.
 *
 * This routine is called DIRECTLY from the ICP boot code - minimal C setup (stack only)!
 *
 * @return 0 => ICP not required, \n
 *         1 => ICP required
 */
uint8_t userDetectICP(void) {
   VDD_OFF(); // Turn off Vdd as early as possible
   return 0;  // ICP not required (rely on bootstrap in ICP code)
}

typedef struct {
   uint8_t mcgC1; 
   uint8_t mcgC3; 
} ClockFactors;

enum {clksel_4MHz, clksel_8MHz, clksel_12MHz};

static const ClockFactors clockFactors[] = {
      { // 4 MHz
        (1<<MCGC1_RDIV_BITNUM),                   // RDIV = 1 -> 4MHz/2=2 MHz
        MCGC3_PLLS_MASK | (6<<MCGC3_VDIV_BITNUM)  // VDIV = 6 -> multiply by 24 -> 2MHz * 24 = 48MHz
      },
      { // 8 MHz
        (2<<MCGC1_RDIV_BITNUM),                   // RDIV = 2 -> 8MHz/2=2 MHz
        MCGC3_PLLS_MASK | (6<<MCGC3_VDIV_BITNUM)  // VDIV = 6 -> multiply by 24 -> 2MHz * 24 = 48MHz
      },
      { // 12 MHz
        (3<<MCGC1_RDIV_BITNUM),                   // RDIV = 3 -> 12MHz/8=1.5 MHz
        MCGC3_PLLS_MASK | (8<<MCGC3_VDIV_BITNUM)  // VDIV = 8 -> multiply by 32 -> 1.5MHz * 32 = 48MHz
      }
};
/*!  Sets up the clock for USB operation
 *
 * MGCOUT = 48MHz, BUS_CLOCK = 24MHz, (PEE mode)
 *
 * Modes: FEI [FLL engaged internal] -> 
 *        FBE [FLL bypassed external] ->
 *        PBE [PLL bypassed external] ->
 *        PEE [PLL engaged external]
 *
 * Refer 12.5.2.1 of MC9S08JM60 Reference
 * 
 * @param clksel - parameters for external crystal used
 * 
 */
static void initCrystalClock(const ClockFactors clockFactors) {
   // Out of reset MCG is in FEI mode
   
   // 1. Switch from FEI (FLL engaged internal) to FBE (FLL bypassed external)
   // 1 a) Set up crystal 
   MCGC2 =                       // BDIV = 0, divide by 1
            MCGC2_HGO_MASK       // oscillator in high gain mode
          | MCGC2_EREFS_MASK     // because crystal is being used
          | MCGC2_RANGE_MASK     // 12 MHz is in high freq range
          | MCGC2_ERCLKEN_MASK;  // activate external reference clock
   // 1 c) Select clock mode
   MCGC1 =   (2<<MCGC1_CLKS_BITNUM) |  // CLKS = 10 -> External reference clock
          // MCGC1_IRCLKEN_MASK|
			 clockFactors.mcgC1;       // RDIV = ?  -> xtal/? -> 2/1.5MHz
   
   // 1 b) Wait for crystal to start up, mode change & ext ref to MCGOUT        
   // 1 d) Wait for mode change
   // 1 e) Wait for MCGOUT indicating that the external reference to be fed to MCGOUT
    while ((MCGSC & (MCGSC_IREFST_MASK|MCGSC_OSCINIT_MASK|    MCGSC_CLKST_MASK)) != 
                    (        0        |MCGSC_OSCINIT_MASK|(2<<MCGSC_CLKST_BITNUM))) {
    }
   // 2. Switch FBE (FLL bypassed external) to PBE (PLL Bypassed External) mode
   // 2 b) Set RDIV for correct range
   // 2 c) Enable the PLL & set VDIV value
   MCGC3 = clockFactors.mcgC3;        // VDIV=? -> multiply by 24/32 -> 2/1.5MHz * 24 = 48MHz */

   // 2 e) Wait until PLLS clock source changes to the PLL
   while (MCGSC_PLLST == 0) {
   }
   // 2 f)  Wait for PLL to acquired lock
   while (MCGSC_LOCK == 0) {
   }
   // 3. PBE mode transitions into PEE mode:
   // 3 a) Set RDIV for correct range and select PLL.FLL clock
   MCGC1 =  //MCGC1_IRCLKEN_MASK |
		    clockFactors.mcgC1;

   // 3 c)  Wait for clock stable
   while (MCGSC_CLKST != 3) {
   }
   // Now MCGOUT=48MHz, BUS_CLOCK=24MHz 
   
   // Enable RTC  ~ 0.5 ms
//   RTCSC = RTCSC_RTIF_MASK|(2<<RTCSC_RTCLKS_BITNUM)|RTCSC_RTIE_MASK|(0xC<<RTCSC_RTCPS_BITNUM);
}

// Clock Trim values in Flash
extern const volatile uint8_t NV_MCGTRM_INIT;  // MSB

#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_FRAME
#pragma NO_RETURN
static void autoInitClock( void ) {
#define MCGC2_VALUE  ((0<<MCGC2_BDIV_BITNUM)|MCGC2_ERCLKEN_MASK|MCGC2_EREFS_MASK|MCGC2_RANGE_MASK|MCGC2_HGO_MASK) 
#define MCGC1_VALUE  ((0<<MCGC1_CLKS_BITNUM)|MCGC1_IREFS_MASK) 
#define MCGSC_MASK   (MCGSC_IREFST_MASK|MCGSC_OSCINIT_MASK|MCGSC_LOCK_MASK|    MCGSC_CLKST_MASK)
#define MCGSC_VALUE  (MCGSC_IREFST_MASK|MCGSC_OSCINIT_MASK|MCGSC_LOCK_MASK|(0<<MCGSC_CLKST_BITNUM))
#define RTCSC_VALUE  ((1<<RTCSC_RTCLKS_BITNUM)|(9<<RTCSC_RTCPS_BITNUM))
#define RTCMOD_VALUE (255)
   asm {
#if CPU == JMxx
      // Trim clock for JMxx, JS16 is factory trimmed
      lda NV_MCGTRM_INIT //  Trim only if value has been programmed to Flash.
      beq dontTrim       
      sta MCGTRM
   dontTrim:
#endif	   
      // Set approx. 16 MHz bus clock assuming 31.25 MHz internal clock
      mov #MCGC2_VALUE,MCGC2 // BDIV=0,RANGE=1,HGO=1,LP=0,EREFS=1,ERCLKEN=1,EREFSTEN=0
      mov #MCGC1_VALUE,MCGC1 // CLKS=00,RDIV=0,IREFS=1,IRCLKEN=0,IREFSTEN=0
      
      // Wait for clock mode change
   clkLoop:
      lda MCGSC
      and #MCGSC_MASK
      cmp #MCGSC_VALUE
      bne clkLoop
      
      // RTC driven by xtal
      // RTC will roll-over as follows
      // xtal       Approx. time
      //  4 MHz      128 ms 
      //  8 MHz       64 ms  (91)
      // 12 MHz       43 ms  (52)
      mov #RTCSC_VALUE,RTCSC
      mov #RTCMOD_VALUE,RTCMOD

      // Count is in 1ms units
      clra 					// A = count in ms
   oloop:
      // ~1 ms
      clrh
      clrx
   loop:
      aix    #1             // 2 cy
      cphx   #2000          // 3 cy
      bne    loop           // 3 cy => 8 cy @16MHz 500 ns per iteration
      inca                  // Count ms
      brclr  7,RTCSC,oloop  // Check RTC roll-over
      clr    RTCSC          // Stop RTC

      // JS16 & JMxx can auto-detect 4MHz without trimming
      ldhx   clockFactors   // clksel_4MHz
      cmp    #93
      bhi    skip
#if CPU == JMxx
      ldx    NV_MCGTRM_INIT //  JMxx cannot reliably detect 8MHz if not trimmed
      beq    is12MHz        //  Assume 12MHz
#endif
      ldhx   clockFactors:2 // clksel_8MHz
      cmp    #52
      bhi    skip
   is12MHz:
      ldhx   clockFactors:4 // clksel_12MHz
   skip:
      jmp    initCrystalClock
   }
}

#if (DEBUG&STACK_DEBUG)
/*! \brief Clear stack to allows probing to determine the used stack area.
 *
 *  Clear the stack before first use.  This allows later probing to determine how much
 *  stack space has been used.
 */
static void clearStack(void) {
extern char far __SEG_START_SSTACK[];  // bottom of stack space

   asm {
      ais   #-2                  // Allows space for value to be pushed
      tsx                        // Current TOS
      sthx  1,sp                 // Save on stack for comparison
      ldhx  #@__SEG_START_SSTACK // Clear from bottom of stack
   loop:    
      clr   ,x                   // Clear next byte
      aix   #1 
      cphx  1,sp                 // Reached TOS?
      bne   loop                 // No - loop
      
      ais   #2                   // Clean up stack
   }
}
#else
#define clearStack() ;
#endif

/*! \brief Initialise the system.
 *
 *  Initialisation of the following:
 *  \li  Default port values
 *  \li  Watchdog (off),
 *  \li  Stack,
 *  \li  BDM interface,
 *  \li  USB interface.
 *  \li  Configure Clock for 48MHz operation
 */
static void init(void) {

   // Default ports to inputs
   PTADD = 0x00;
   PTBDD = 0x00;
#if (CPU==JMxx)
   PTCDD = 0x00;
   PTDDD = 0x00;
   PTEDD = 0x00;
   PTFDD = 0x00;
   PTGDD = 0x00;
#endif

//    Turn off important things
#if ((HW_CAPABILITY & CAP_FLASH) != 0)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif
   VDD_OFF();
   
   // Default to Ports have PUPs
   // Note - this doesn't affect outputs
   PTAPE = 0xFF;
   PTBPE = 0xFF;
#if (CPU==JMxx)
   PTCPE = 0xFF;
   PTDPE = 0xFF;
   PTEPE = 0xFF;
   PTFPE = 0xFF;
   PTGPE = 0xFF;
#endif

   EnableInterrupts;

#if (HW_CAPABILITY&CAP_VDDSENSE)
   SPMSC1_BGBE = 1;                // Enable Bandgap Reference
#endif
   
   LED_INIT();
   clearStack();
   initUSB();  // Assumes clock already done
   
   (void)bdm_init();
   (void)bdm_off();
   
#ifdef VDD_ON_INITIALLY
   // For compatibility with original board s/w
   // The board is powered when initially plugged in
#if (VDD_ON_INITIALLY == 3)
   bdm_option.targetVdd = BDM_TARGET_VDD_3V3;
#elif (VDD_ON_INITIALLY == 5)
   bdm_option.targetVdd = BDM_TARGET_VDD_5;
#else
#error "Illegal VDD_ON_INITIALLY value"   
#endif
   (void)bdm_interfaceOff();
   (void)bdm_setTargetVdd();
#endif
   
#if (DEBUG&SCI_DEBUG) != 0)   
   debugSCIInit();
#endif
}

void initdebugMessageBuffer(void);

/*! \brief Main loop.
 *
 *  Most of the action is interrupt driven.  
 *
 * @return Never exits

*/
void main(void) {
#if !defined(SOPT1_BKGDPE_MASK)// || defined(DISABLE_BKGD)
#undef SOPT1_BKGDPE_MASK
#define SOPT1_BKGDPE_MASK (0) // Only exists on some CPUs or BKGD pin in use as GPIO
#endif
   SOPT1 = SOPT1_STOPE_MASK|SOPT1_BKGDPE_MASK; // Disable COP, enable STOP instr. & BKGD pin
   
   EnableInterrupts; /* enable interrupts */
   
   //TERMIO_Init();
   //initdebugMessageBuffer();
   
   //dprint("main():Starting\r\n");
   
   autoInitClock();
//   initCrystalClock(clockFactors[clksel_12MHz]);

#if (DEBUG&SCI_DEBUG) != 0)   
   debugSCIInit();
   debugPuts("main(): Starting\r");
#endif

   init();
   commandLoop(); // doesn't return
   
//   for(;;) {
//      dprint("main():Waiting\r\n");
//      wait();
//   }
}

#if DEBUG&DEBUG_MESSAGES
static char debugMessageBuffer[450] = {0};

void initdebugMessageBuffer(void) {
int i;
   for (i=0; i<sizeof(debugMessageBuffer)/sizeof(debugMessageBuffer[0]); i++)
      debugMessageBuffer[i] = 0;   
}

void dputs(char *msg) {
static char *msgPtr = debugMessageBuffer;

   if (msgPtr >= debugMessageBuffer+sizeof(debugMessageBuffer)-30) {
      *msgPtr = '\0';
      msgPtr = debugMessageBuffer;
   }
   while (*msg != '\0')
      *msgPtr++ = *msg++;
  *msgPtr++ = ' ';
  *msgPtr++ = ' ';
  *msgPtr++ = ' ';
}

#endif // DEBUG&DEBUG_MESSAGES

