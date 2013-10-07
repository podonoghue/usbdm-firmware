/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM - Universal BDM, JS16 Version \n
    Combined TBDML/OSBDM + extensions, see 
    @htmlonly <a href="USBDM_SER_SWD_JS16CWJ.pdf">schematic</a> @endhtmlonly \n
    Supports HC12, HCS08, RS08 and Coldfire V1 targets \n

    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//==========================================================================================
// USB Serial Number
#define SERIAL_NO          "USBDM-JS16-SWD_SER-0001"
#define ProductDescription "USBDM HCS08,HCS12,CFV1,ARM-SWD BDM"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_RST_IO|CAP_CDC|CAP_BDM|CAP_SWD_HW|CAP_CORE_REGS)
#define TARGET_CAPABILITY (CAP_HCS12 |CAP_CDC|CAP_HCS08|CAP_CFV1|CAP_ARM_SWD)

#ifndef PLATFORM
#define PLATFORM USBDM   //! Choose BDM emulation
#endif

#define CPU  JS16

#define DISABLE_BKGD  // BKGD pin is used as GPIO, comment for enable debug using BDM

#if !defined(LOG) && !defined(DISABLE_BKGD)
#define DISABLE_BKGD  // BKGD pin is used as GPIO
#endif

// Define for automatic WINUSB Driver loading 
//#define MS_COMPATIBLE_ID_FEATURE (1)

//=================================================================================
// Debug pin - used to check timing and hardware sequences etc.
//
#if (DEBUG != 0)
#define DEBUG_PIN_DDR PTBDD_PTBDD4
#define DEBUG_PIN_PER PTBPE_PTBPE4
#define DEBUG_PIN     PTBD_PTBD4
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
//#define CTS_IN             PTBD_PTBD1
//#define DTR_OUT            PTBD_PTBD2
//#define DTR_OUT_DDR        // PTB.2 is output only no DDR
//
//#define CTS_IS_HIGH()      (CTS_IN!=0)
//#define DTR_INACTIVE()          DTR_OUT = 0
#define DTR_ACTIVE()         ;//DTR_OUT = 1
#endif

//=================================================================================
// HCS08, HCS12, RS08 & Coldfire V1 BDM Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_BDM)
#define DATA_PORT         PTAD
#define DATA_PORT_DDR     PTADD
#define DATA_PORT_PER     PTAPE

// BDM data out pin - hard coded in Rx/Tx routines
#define BDM_OUT           PTAD_PTAD2
#define BDM_OUT_BIT       (2)  // Bit number!
#define BDM_OUT_MASK      (1<<BDM_OUT_BIT)
#define BDM_OUT_PER       PTAPE_PTAPE2

// BDM data in pin
#define BDM_IN            PTAD_PTAD0
#define BDM_IN_BIT        (0)  // Bit number!
#define BDM_IN_MASK       (1<<BDM_IN_BIT)

// BDM data direction pin - controls buffer enable/direction
#define BDM_EN            PTAD_PTAD4
#define BDM_EN_BIT        (4)  // Bit number!
#define BDM_EN_MASK       (1<<BDM_EN_BIT)
#define BDM_EN_PER        PTAPE_PTAPE4

#define BDM_EN_RD_MASK    BDM_EN_MASK
#define BDM_EN_WR_MASK    0

// These two ASM macros assume port pin direction is already correct
#define BDM_ENABLE_ASM    BCLR BDM_EN_BIT,DATA_PORT
#define BDM_3STATE_ASM    BSET BDM_EN_BIT,DATA_PORT

//======================================================================
// State     BDM_O   BDM_EN   BDM_I  LVC125  LVC45   Bare Pin  BKGD_PIN
// Low        L        WR       Z     EN,L    Tx,L      L         L
// High       H        WR       Z     EN,H    Tx,H      H         H
// 3-state    Z        RD       Z     DIS,Z   Rx,Z     Z(in)      Z
#define BDM_LOW()    (DATA_PORT      = BDM_EN_WR_MASK|0,            \
					  DATA_PORT_DDR  = BDM_EN_MASK   |BDM_OUT_MASK)
#define BDM_HIGH()   (DATA_PORT      = BDM_EN_WR_MASK|BDM_OUT_MASK, \
					  DATA_PORT_DDR  = BDM_EN_MASK   |BDM_OUT_MASK)
#define BDM_3STATE() (DATA_PORT      = BDM_EN_RD_MASK|BDM_OUT_MASK,  \
					  DATA_PORT_DDR  = BDM_EN_MASK   |0)
#endif // CAP_BDM

//=================================================================================
// ARM SWD Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_SWD_HW)
#ifndef DATA_PORT
#define DATA_PORT          PTAD
#define DATA_PORT_DDR      PTADD
#define DATA_PORT_PER      PTAPE
#endif

// SWD data out pin
#define SWD_OUT             PTAD_PTAD2
#define SWD_OUT_BIT         (2)  // Bit number!
#define SWD_OUT_MASK        (1<<SWD_OUT_BIT)
#define SWD_OUT_PER         PTAPE_PTAPE2
#define SWD_OUT_DDR         PTADD_PTADD2

// SWD data out 3-state control
#define SWD_OUT_EN          PTAD_PTAD4
#define SWD_OUT_EN_BIT      (4)  // Bit number!
#define SWD_OUT_EN_MASK     (1<<SWD_OUT_EN_BIT)
#define SWD_OUT_EN_PER      PTAPE_PTAPE4
#define SWD_OUT_EN_DDR      PTADD_PTADD4
                            
// Following assume pin already configured by SWD_LOW(),SWD_HIGH(),SWD_3STATE()
#define SWD_ENABLE_ASM      BCLR SWD_OUT_EN_BIT,DATA_PORT
#define SWD_3STATE_ASM      BSET SWD_OUT_EN_BIT,DATA_PORT
#define SWD_LOW_ASM         BCLR SWD_OUT_BIT,DATA_PORT 
#define SWD_HIGH_ASM        BSET SWD_OUT_BIT,DATA_PORT
#define SWD_DISABLE_ASM     BCLR SWD_OUT_EN_BIT,DATA_PORT_DDR
#define SWD_ENABLE_DRV_ASM  BSET SWD_OUT_EN_BIT,DATA_PORT_DDR

// Following configure pin & driver
#define SWD_LOW()           (SWD_OUT=0, SWD_OUT_DDR=1, SWD_OUT_EN=0, SWD_OUT_EN_DDR=1)
#define SWD_HIGH()          (SWD_OUT=1, SWD_OUT_DDR=1, SWD_OUT_EN=0, SWD_OUT_EN_DDR=1)
#define SWD_3STATE()        (SWD_OUT_EN=1, SWD_OUT_EN_DDR=1, SWD_OUT_DDR=1)
#define SWD_DISABLE()       (SWD_OUT_EN=1, SWD_OUT_EN_DDR=0, SWD_OUT_DDR=0) // PUPs

// SWD data in pin          
#define SWD_IN              PTAD_PTAD0
#define SWD_IN_BIT          (0)  // Bit number!
#define SWD_IN_MASK         (1<<SWD_IN_BIT)
#define SWD_IN_PER          PTAPE_PTAPE0
#define SWD_IN_DDR          PTADD_PTADD0
                            
// SWCLK out pin
#define SWCLK_OUT           PTAD_PTAD3
#define SWCLK_OUT_BIT       (3)  // Bit number!
#define SWCLK_OUT_MASK      (1<<SWCLK_OUT_BIT)
#define SWCLK_OUT_PER       PTAPE_PTAPE3
#define SWCLK_OUT_DDR       PTADD_PTADD3

// SWCLK enable pin - controls buffer enable
#define SWCLK_OUT_EN        PTBD_PTBD1
#define SWCLK_OUT_EN_BIT    (1)  // Bit number!
#define SWCLK_OUT_EN_MASK   (1<<SWCLK_OUT_EN_BIT)
#define SWCLK_OUT_EN_PER    PTBPE_PTBPE1
#define SWCLK_OUT_EN_DDR    PTBDD_PTBDD1

// Following assume pin direction already set
#define SWCLK_LOW_ASM       BCLR SWCLK_OUT_BIT,DATA_PORT 
#define SWCLK_HIGH_ASM      BSET SWCLK_OUT_BIT,DATA_PORT

// Following configure pin & driver
#define SWCLK_LOW()         (SWCLK_OUT=0, SWCLK_OUT_DDR=1, SWCLK_OUT_EN=0, SWCLK_OUT_EN_DDR=1)
#define SWCLK_HIGH()        (SWCLK_OUT=1, SWCLK_OUT_DDR=1, SWCLK_OUT_EN=0, SWCLK_OUT_EN_DDR=1)
#define SWCLK_3STATE()      (SWCLK_OUT_EN=1, SWCLK_OUT_EN_DDR=1, SWCLK_OUT_DDR=1) // PUPs
#define SWCLK_DISABLE()     (SWCLK_OUT_EN=1, SWCLK_OUT_EN_DDR=0, SWCLK_OUT_DDR=0) // PUPs

#endif // CAP_SWD

//=================================================================================
// RESET control & sensing
//
#if (HW_CAPABILITY&CAP_RST_IO)

// RESET output pin
#define RESET_OUT             PTBD_PTBD3
//#define RESET_OUT_DDR       PTBPE_PTBPE3 // PTE3 is output always
//#define RESET_OUT_PER       PTBPE_PTBPE3  // PTE3 is output always, so no PUP
#define RESET_LOW()         (RESET_OUT=0)
#define RESET_3STATE()      (RESET_OUT=1)
#define RESET_DISABLE()     (RESET_OUT=1) // No separate 3-state control

// RESET input pin
#define RESET_IN            PTAD_PTAD5
#define RESET_IN_DDR        PTADD_PTADD5
#define RESET_IN_PER        PTAPE_PTAPE5
#define RESET_IN_NUM        (5)
#define RESET_IN_MASK       (1<<RESET_IN_NUM)

// RESET input pin
#define RESET_IS_HIGH       (RESET_IN!=0)
#define RESET_IS_LOW        (RESET_IN==0)

#endif // CAP_RST_IO

//=================================================================================
// LED Port bit masks
//
#define GREEN_LED_MASK      (PTBD_PTBD2_MASK)
#define RED_LED_MASK        (0) // No red LED!
#define LED_PORT_DATA       (PTBD)
#define LED_PORT_DDR        (PTBDD)

// LEDs off, LED pins are outputs 
#define LED_INIT()          ((LED_PORT_DATA |= RED_LED_MASK|GREEN_LED_MASK), \
                             (LED_PORT_DDR  |= RED_LED_MASK|GREEN_LED_MASK))
#define GREEN_LED_ON()      (LED_PORT_DATA &= (GREEN_LED_MASK^0xFF))
#define GREEN_LED_OFF()     (LED_PORT_DATA |= GREEN_LED_MASK)
#define GREEN_LED_TOGGLE()  (LED_PORT_DATA ^= GREEN_LED_MASK)
#if RED_LED_MASK == 0      
#define RED_LED_ON()        ; //!
#define RED_LED_OFF()       ; //!
#define RED_LED_TOGGLE()    ; //!
#else
#define RED_LED_ON()        (LED_PORT_DATA &= (RED_LED_MASK^0xFF))
#define RED_LED_OFF()       (LED_PORT_DATA |= RED_LED_MASK)
#define RED_LED_TOGGLE()    (LED_PORT_DATA ^= RED_LED_MASK)
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
//
//     TPM-CH0 - BKGD_I pin, SYNC measuring, ACKN detection (Input Capture)
//     TPM-CH1 - Timeouts (Output compare, no pin used)
//

// SYNC Detection TPM.Ch0 : Input capture  (PTA.0)
#define BKGD_TPMxCnSC_CHF                 TPMC0SC_CH0F       // Event Flag
#define BKGD_TPMxCnSC                     TPMC0SC            // TPM Channel Status & Configuration
#define BKGD_TPMxCnSC_RISING_EDGE_MASK    TPMC0SC_ELS0A_MASK // TPMxCnSC value for rising edge
#define BKGD_TPMxCnSC_FALLING_EDGE_MASK   TPMC0SC_ELS0B_MASK // TPMxCnSC value for falling edge
#define BKGD_TPMxCnVALUE                  TPMC0V             // IC Event time
#define BKGD_TPM_SETUP_ASM                BCLR 7,BKGD_TPMxCnSC 

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
