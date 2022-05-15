/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_CF_JS16CWJ - USBDM/CF, JS16 Version \n
    TBLCF + extensions,
             
    @htmlonly <a href="USBDM_CF_JS16CWJ.pdf">schematic</a> @endhtmlonly \n
    Supports Coldfire V2, 3 & 4 and MC56F80xx targets \n

    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//==========================================================================================
// USB Serial Number
#define SERIAL_NO "USBDM-JS16-CF-0001"
#define ProductDescription "USBDM DSC,Coldfire-V2,3,4 BDM"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_RST_IO|CAP_CFVx_HW|CAP_JTAG_HW|CAP_CORE_REGS)
#define TARGET_CAPABILITY (CAP_RST   |CAP_CFVx   |CAP_JTAG   |CAP_DSC |CAP_ARM_JTAG)

#ifndef PLATFORM
#define PLATFORM USBDM   //! Choose BDM emulation
#endif

#define CPU  JS16

//=================================================================================
// Debug pin - used to check timing and hardware sequences etc.
//
#if (DEBUG != 0)
#define DEBUG_PIN_DDR PTBDD_PTBDD1
#define DEBUG_PIN_PER PTBPE_PTBPE1
#define DEBUG_PIN     PTBD_PTBD1
#endif

//=================================================================================
// ICP pin - used to force ICP in bootstrap code
//
#define ICP_PIN_DDR DEBUG_PIN_DDR
#define ICP_PIN_PER DEBUG_PIN_PER
#define ICP_PIN     DEBUG_PIN

//===========================================================================================
// Type of BDM interface chips are supported

#define DRIVER  LVC125  //! Choose driver IC being used

#ifndef DRIVER
#error "Please define DRIVER in Configure.h"
#define DRIVER LVC125 //! Choose driver IC being used
#endif

#if ((DRIVER!=LVC125) && (DRIVER!=LVC45))
#error "Please correctly define DRIVER in Configure.h"
#define DRIVER LVC125 //! Choose driver IC being used
#endif


//=================================================================================
// Port Pin assignments
// Please note: some pin assignments cannot be changed freely
// RX AND TX ROUTINES ARE DEPENDENT ON THIS SPECIFIC ASSIGNMENTS
//

//=================================================================================
// Serial Port Bit Masks
//
#if (HW_CAPABILITY&CAP_CDC)
#define CTS_IN             PTBD_PTBD1
#define DTR_OUT            PTBD_PTBD2
#define DTR_OUT_DDR        // PTB.2 is output only no DDR

#define CTS_IS_HIGH()      (CTS_IN!=0)
#define DTR_INACTIVE()          DTR_OUT = 0
#define DTR_ACTIVE()         DTR_OUT = 1
#endif

//=================================================================================
// Coldfire V2,3,4 BDM Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_CFVx_HW)
// CF_DRV (=JTAG_DRV) 3-state control
#define CF_DRV            
#define CF_DRV_DDR        
#define CF_DRV_PER        

#define CF_DRV_ENABLE()   
#define CF_DRV_DISABLE()  

#define DATA_PORT          PTAD
#define DATA_PORT_DDR      PTADD

// DSO in pin (MISO1)
#define DSO_IN             PTAD_PTAD1
#define DSO_IN_BITNUM      (1)
#define DSO_IN_MASK        (1<<DSO_IN_BITNUM)
#define DSO_IN_DDR         PTADD_PTADD1
#define DSO_IN_PORT        PTAD

// DSI out pin (MOSI1) - DSI 
#define DSI_OUT            PTAD_PTAD2
#define DSI_OUT_BITNUM     (2) 
#define DSI_OUT_MASK       (1<<DSI_OUT_BITNUM)
#define DSI_OUT_PER        PTAPE_PTAPE5
#define DSI_OUT_DDR        PTADD_PTADD5

// DSCLK out pin (SPSCK1) - DSCLK must be on the same port 
#define DSCLK_OUT          PTAD_PTAD3
#define DSCLK_OUT_BITNUM   (3)
#define DSCLK_OUT_MASK     (1<<DSCLK_OUT_BITNUM)
#define DSCLK_OUT_PORT     PTAD
#define DSCLK_OUT_PER      PTAPE_PTAPE6
#define DSCLK_OUT_DDR      PTADD_PTADD6

#define ASM_DSCLK_HIGH     bset  DSCLK_OUT_BITNUM,DSCLK_OUT_PORT
#define ASM_DSCLK_LOW      bclr  DSCLK_OUT_BITNUM,DSCLK_OUT_PORT

// DSCLK 3-state control - not used on JS16
//#define DSCLK_DRV            
//#define DSCLK_DRV_DDR        
//#define DSCLK_DRV_PER        

#define DSCLK_DRV_ENABLE()   {}
#define DSCLK_DRV_DISABLE()  {}

// BKPT out pin
#define BKPT_OUT           PTAD_PTAD5
#define BKPT_OUT_DDR       PTADD_PTADD5
#define BKPT_OUT_PER       PTAPE_PTAPE5

#define BKPT_LOW()         (BKPT_OUT=0,BKPT_OUT_DDR=1)
#define BKPT_HIGH()        (BKPT_OUT=1,BKPT_OUT_DDR=0)

// TA out pin 
#define TA_OUT			   PTAD_PTAD7             
#define TA_OUT_DDR         PTADD_PTADD7
#define TA_OUT_PER         PTAPE_PTAPE7

#define TA_LOW()           (TA_OUT=0,TA_OUT_DDR=1)
#define TA_3STATE()        (TA_OUT=1,TA_OUT_DDR=0)
   
// ALLPST in pin 
#define ALLPST_IN          0

#define ALLPST_IS_HIGH     (ALLPST_IN!=0)

// Masks to make BDM interface idle @ref bdmcf_init
// DSI=0, DSCLK=0 all others undriven (PUPs) 
#define BDMCF_IDLE         (0)
#define BDMCF_IDLE_DDR     (DSI_OUT_MASK|DSCLK_OUT_MASK)
#endif

//=================================================================================
// JTAG Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_JTAG_HW)

// JTAG_DRV 3-state control
#define JTAG_DRV            CF_DRV
#define JTAG_DRV_DDR        CF_DRV_DDR
#define JTAG_DRV_PER        CF_DRV_PER

#define JTAG_DRV_ENABLE()   
#define JTAG_DRV_DISABLE()  

// TDI out pin (MOSI1)
#define TDI_OUT             PTAD_PTAD2
#define TDI_OUT_BITNUM	    (2)
#define TDI_OUT_MASK        (1<<TDI_OUT_BITNUM)
#define TDI_OUT_DDR         PTADD_PTADD2

#define TDI_HIGH()          TDI_OUT = 1    // Assumes direction already set
#define TDI_LOW()           TDI_OUT = 0

// TDO in pin (MISO)
#define TDO_IN              PTAD_PTAD1
#define TDO_IN_BITNUM       (1)
#define TDO_IN_MASK         (1<<TDO_IN_BITNUM)
#define TDO_IN_DDR          PTADD_PTADD1

#define TDO_IS_HIGH         (TDO_IN!=0)

// TCLK out pin (SPICK1)
#define TCLK_OUT            PTAD_PTAD3
#define TCLK_OUT_BITNUM     (3)
#define TCLK_OUT_MASK       (1<<TCLK_OUT_BITNUM)
#define TCLK_OUT_DDR        PTADD_PTADD3

#define TCLK_HIGH()         TCLK_OUT = 1   // Assumes direction already set
#define TCLK_LOW()          TCLK_OUT = 0

// TMS out pin (MISO2)
#define TMS_OUT             PTAD_PTAD5    
#define TMS_OUT_DDR         PTADD_PTADD5

#define TMS_HIGH()          (TMS_OUT = 1)//, TMS_OUT_DDR = 1)
#define TMS_LOW()           (TMS_OUT = 0)//, TMS_OUT_DDR = 1)

// TRST* out pin
#define TRST_OUT            PTAD_PTAD6 
#define TRST_OUT_DDR        PTADD_PTADD6
#define TRST_OUT_PER        PTAPE_PTAEP6

#define TRST_LOW()          TRST_OUT_DDR=1;TRST_OUT = 0;
#define TRST_3STATE()       TRST_OUT_DDR=0;TRST_OUT = 1; // via FET

//! Masks to make JTAG interface idle @ref jtag_init 
//! 
#define JTAG_IDLE           (TCLK_OUT_MASK)
#define JTAG_IDLE_DDR       (TDI_OUT_MASK|TCLK_OUT_MASK)
#endif // CAP_JTAG_HW

//=================================================================================
// HCS08, HCS12, RS08 & Coldfire V1 BDM Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_BDM)

#endif // CAP_BDM
//=================================================================================
// RESET control & sensing
//
#if (HW_CAPABILITY&CAP_RST_IO)

// RESET output pin (bi-directional via FET)
#define RESET_OUT           PTAD_PTAD0
#define RESET_OUT_DDR       PTADD_PTADD0
#define RESET_OUT_PER       PTAPE_PTAPE0
#define RESET_LOW()         (RESET_OUT=0,RESET_OUT_DDR=1)
#define RESET_3STATE()      (RESET_OUT=1,RESET_OUT_DDR=0) // Pull-up on pin

// RESET input pin
#define RESET_IN            PTAD_PTAD0
#define RESET_IN_DDR        PTADD_PTADD0
#define RESET_IN_PER        PTAPE_PTAPE0
#define RESET_IN_NUM        (0)
#define RESET_IN_MASK       (1<<RESET_IN_NUM)

#define RESET_IS_HIGH       (RESET_IN!=0)
#define RESET_IS_LOW        (RESET_IN==0)

#endif // CAP_RST_IO

//=================================================================================
// LED Port bit masks
//
#define GREEN_LED_MASK  (PTBD_PTBD3_MASK)
#define RED_LED_MASK    (0) // No red LED!
#define LED_PORT_DATA   (PTBD)
#define LED_PORT_DDR    (PTBDD)

// LEDs off, LED pins are outputs 
#if 0

#define LED_INIT()         ((LED_PORT_DATA |= RED_LED_MASK|GREEN_LED_MASK), \
                            (LED_PORT_DDR  |= RED_LED_MASK|GREEN_LED_MASK))
#else
#define LED_INIT()         ((LED_PORT_DATA |= RED_LED_MASK), \
                            (LED_PORT_DATA |= GREEN_LED_MASK), \
                            (LED_PORT_DDR  |= RED_LED_MASK), \
                            (LED_PORT_DDR  |= GREEN_LED_MASK))
#endif
#define GREEN_LED_ON()     (LED_PORT_DATA &= (GREEN_LED_MASK^0xFF))			//!
#define GREEN_LED_OFF()    (LED_PORT_DATA |= GREEN_LED_MASK)				//!
#define GREEN_LED_TOGGLE() (LED_PORT_DATA ^= GREEN_LED_MASK)				//!
#if RED_LED_MASK == 0
#define RED_LED_ON()       ; //!
#define RED_LED_OFF()      ; //!
#define RED_LED_TOGGLE()   ; //!
#else
#define RED_LED_ON()       (LED_PORT_DATA &= (RED_LED_MASK^0xFF))
#define RED_LED_OFF()      (LED_PORT_DATA |= RED_LED_MASK)
#define RED_LED_TOGGLE()   (LED_PORT_DATA ^= RED_LED_MASK)
#endif
//=================================================================================
// Flash programming control
//
#if (HW_CAPABILITY&CAP_FLASH)

#else // !CAP_FLASH
// No Flash programming supply
#define FLASH12V_ON()   ; //!
#define FLASH12V_OFF()  ; //!

#define VPP_ON()        ; //!
#define VPP_OFF()       ; //!

#endif //CAP_FLASH

//=================================================================================
// Target Vdd control

#if (HW_CAPABILITY&CAP_VDDCONTROL)

#else // !CAP_VDDCONTROL
// No Vdd control
#define VDD_OFF()       ; // Vdd Off
#define VDD3_ON()       ; // Vdd = 3.3V
#define VDD5_ON()       ; // Vdd = 5V

#endif // CAP_VDDCONTROL

//=================================================================================
// Use of 1k5 resistor on USB D+ line
//
//  The JMxx has a programmable 1k5 pull-up resistor on the USB D+ line.
//
#define USBPUP_ON    (0) // Turn on internal PUP
#define USBPUP_OFF   (1) // Turn off internal PUP

#define USBPUP USBPUP_ON // Internal 1k5 PUP present on D+

#ifndef USBPUP
#error "Please define USBPUP in Configure.h"
#define USBPUP USBPUP_OFF
#endif

#if ((USBPUP != USBPUP_ON) && (USBPUP != USBPUP_OFF))
#error "Please correctly define USBPUP in Configure.h"
#endif


//================================================================================
// Timer Channel use
//   TPMx = TPM
//     TPM-CH1 - Timeouts (Output compare, no pin used)
//

// Timeout TPM.Ch1 : Output Compare (no pin)
#define TIMEOUT_TPMxCnSC_CHF              TPMC1SC_CH1F       // Event Flag
#define TIMEOUT_TPMxCnSC                  TPMC1SC            // TPM Status & Configuration
#define TIMEOUT_TPMxCnSC_OC_MASK          TPMC1SC_MS1A_MASK  // TPMxCnSC value for OC event
#define TIMEOUT_TPMxCnVALUE               TPMC1V             // OC Event time

//================================================================================
// RESET Detection - falling edge using KBI inputs
//
//     KBI     - RESET_IN pin, Reset detection (Keypress falling edge detection)
//
#if (HW_CAPABILITY&CAP_RST_IO)
// Configure RESET change sensing (Falling edges)
#define CONFIGURE_RESET_SENSE()   (KBIES &= ~RESET_IN_MASK)
// Enable & Configure RESET Change interrupts
#define ENABLE_RESET_SENSE_INT()  (KBIPE |= RESET_IN_MASK,  \
                                   KBISC = KBISC_KBACK_MASK,\
                                   KBISC = KBISC_KBIE_MASK )
// Clear Reset Change Event
#define CLEAR_RESET_SENSE_FLAG()  (KBISC = KBISC_KBIE_MASK|KBISC_KBACK_MASK)
// Disable  RESET Change interrupts
#define DISABLE_RESET_SENSE_INT() (KBIPE &= ~RESET_IN_MASK)
#endif

//===================================================================================
// Target Vdd sensing
#if (HW_CAPABILITY&CAP_VDDSENSE)
// Target Vdd Present

#else // !CAP_VDDSENSE

#define VDD_SENSE                 (1) // Assume VDD present
#endif

//===================================================================================
// Target Vdd measuring

// Enable & Configure Vdd Change interrupts (Using Analogue comparator, rising or falling edges)
// NOT AVAILABLE

#endif // _CONFIGURE_H_
