/*! \file
    \brief Contains startup code and main loop.
    
    This file contains the main entry point for the program.
    It initialises the system and then remains in the main idle loop.
   
   The basic program flow:
   \li   All BDM activity is driven by commands received from the USB
   \li   All BDM commands are executed from within the ISRs servicing the USB IRQ
              (i.e. with interrupts disabled)
   \li   Suspend timekeeping is performed in the main loop
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
    \endverbatim
*/

#include <hidef.h> /* for EnableInterrupts macro */
#include "derivative.h" /* include peripheral declarations */
#include "Common.h"
#include "ICP.h"
#include "Configure.h"
#include "Commands.h"
#include "USB.h"
#include "BDM.h"
#include "led.h"
#include "main.h"
#include "BDMCommon.h"

/*! \brief Detect In-Circuit Programming (ICP) mode.
 *
 * ICP mode is indicated by a loopback from BDM to RST pins of the BDM cable.
 * For this to work the interface IC must be powered.
 * This routine is called DIRECTLY from the ICP boot code - minimal C setup (stack only)!
 *
 * @return 0 => ICP not required, \n
 *         1 => ICP required
 */
U8 userDetectICP(void) {

   VDD_OFF(); // Turn off Vdd as early as possible
   return 0;  // ICP not required (rely on boostrap in ICP)
}


/*! \brief Used to detect USB bus suspension.
 *
 * Counts up in the main loop and the BDM is suspended when it reaches a threshold value.
 * The USB interrupt handler resets the count.
 */
#pragma DATA_SEG __SHORT_SEG Z_PAGE
volatile U8 suspend_timer;
#pragma DATA_SEG DEFAULT

#if (DEBUG&STACK_DEBUG)
/*! \brief Clear stack to allows probing to determine the used stack area.
 *
 *  Clear the stack before first use.  This allows later probing to determine how much
 *  stack space has been used.
 */
void clearStack(void) {
extern char far __SEG_START_SSTACK[];  // bottom of stack space

   asm {
      tsx                        // Current TOS
      aix   #-2                  // Allows space for value to be pushed
      pshx                       // Save on stack for comparison
      pshh
      ldhx  #@__SEG_START_SSTACK // Clear from bottom of stack
   loop:    
      clr   ,x                   // Clear next byte
      aix   #1 
      cpx   2,sp                 // Reached TOS?
      bne   loop                 // No - loop
      pshh
      pula
      cmp   1,sp
      bne   loop
      
      ais   #2                   // Clean up stack
   }
}
#else
#define clearStack() ;
#endif

/*! \brief Initialise the system.
 *
 *  Initialisation of the following:
 *  \li  Watchdog (off),
 *  \li  Stack,
 *  \li  BDM interface,
 *  \li  USB interface.
 */
void init(void) {
   /* disable part reset on USB reset, enable STOP instruction & disable COP */
   CONFIG = CONFIG_URSTD_MASK | CONFIG_STOP_MASK | CONFIG_COPD_MASK;

   // Turn off important things
   VPP_OFF();
   VDD_OFF();
   FLASH12V_OFF();
   
   clearStack();   

   // Make unused pins 'safe' (not floating)
   // These settings may be overridden later
   POCR = POCR_PAP_MASK|POCR_PCP_MASK|POCR_PTE20P_MASK;
    
   /* acknowledge IRQ interrupt and disable it */
   ISCR   = ISCR_ACK_MASK | ISCR_IMASK_MASK;   
   
   (void)bdm_init();  // Do early as possible
   
   usb_init();
   EnableInterrupts;
}

/*! \brief Waits ~100us 
 */
void wait100us(void) {
#define CYCLES_IN_100US (BUS_FREQ/10000UL)

   asm {
            LDA   #(CYCLES_IN_100US/3)-4-2-4  /* minus cycles needed for BSR, LDA and RTS */
      Loop: DBNZA Loop  /* 3 cycles per iteration */ 
   }
}

/*! \brief Main loop.
 *
 *  Most of the action is interrupt driven.  This loop only detects USB suspension.
 *
 * @return Never exits

*/
void main(void) {

   init();
   for(;;){
      wait100us();
      suspend_timer++;
      if (suspend_timer>=SUSPEND_TIME) {
         /* host is not sending keepalive signals, time to suspend all operation */
         /* BDM is in idle mode when not communicating, so nothing to do there */
         U16 i;
         /* Acknowledge any pending interrupt and disable KBD interrupts; */
         /* This will prevent RESET activity waking us up out of stop */
         KBSCR = KBSCR_IMASKK_MASK | KBSCR_ACKK_MASK;

         led_state=LED_OFF;  /* switch the LED off */
         GREEN_LED_OFF();    /* do it now, the interrupt which would do it normally is not going to come */

         UIR0_SUSPND=1;      /* suspend USB */
         while (suspend_timer) asm(STOP);  /* go to sleep, wait for USB resume or reset */

         for (i=0;i<RESUME_RECOVERY;i++) wait100us();  /* wait for host to recover */

         led_state=LED_ON;   /* Switch the LED back on */
         (void)bdm_init();    /* Reinitialise the BDM after wake-up as the device might */
                             /* have been disconected in the meantime; assume nothing */
      }
   }
}
