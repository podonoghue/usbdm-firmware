/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM - Universal BDM, JMxx Version \n
    Combined TBDML/OSBDM + extensions, see 
             
    @htmlonly <a href="USBDM_JMxxCLD.pdf">schematic</a> @endhtmlonly \n
    Supports HC12, HCS08, RS08 and Coldfire V1 targets \n

    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//==========================================================================================
// USB Serial Number
#define SERIAL_NO "USBDM-JMxx-0001"
#define ProductDescription "USBDM RS08,HCS08,HCS12,CF-V1 BDM"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_RST_IO|CAP_BDM  |CAP_FLASH)
#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_HCS12 |CAP_HCS08|CAP_RS08|CAP_CFV1)

#ifndef PLATFORM
#define PLATFORM USBDM   //! Choose BDM emulation
#endif

#define CPU  JMxx        //! Implementation Target

//=================================================================================
// Debug pin - used to check timing and hardware sequences etc.
//
#if (DEBUG != 0)
#define DEBUG_PIN_DDR PTGDD_PTGDD1
#define DEBUG_PIN_PER PTGPE_PTGPE1
#define DEBUG_PIN     PTGD_PTGD1
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
// HCS08, HCS12, RS08 & Coldfire V1 BDM Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_BDM)
#define DATA_PORT          PTED
#define DATA_PORT_DDR      PTEDD
#define DATA_PORT_PER      PTEPE

// BDM data out pin
#define BDM_OUT           PTED_PTED7
#define BDM_OUT_BIT       (7)
#define BDM_OUT_MASK      (1<<BDM_OUT_BIT)
#define BDM_OUT_PER       PTEPE_PTEPE7

// BDM data in pin
#define BDM_IN            PTED_PTED0
#define BDM_IN_BIT        (0)
#define BDM_IN_MASK       (1<<BDM_IN_BIT)

// BDM data direction pin - controls buffer enable/direction
#define BDM_EN            PTED_PTED6
#define BDM_EN_BIT        (6)
#define BDM_EN_MASK       (1<<BDM_EN_BIT)
#define BDM_EN_PER        PTEPE_PTEPE6

// Polarity of BDM buffer enable/direction varies with driver IC
#if (DRIVER == LVC125)
	#define BDM_EN_RD_MASK  BDM_EN_MASK
	#define BDM_EN_WR_MASK  0
    // These two ASM macros assume port pin direction is already correct
	#define BDM_ENABLE_ASM  BCLR BDM_EN_BIT,DATA_PORT
	#define BDM_3STATE_ASM  BSET BDM_EN_BIT,DATA_PORT
#elif (DRIVER == LVC45)
	#define BDM_EN_RD_MASK  0
	#define BDM_EN_WR_MASK  BDM_EN_MASK
    // These two ASM macros assume port pin direction is already correct
	#define BDM_ENABLE_ASM  BSET BDM_EN_BIT,DATA_PORT
	#define BDM_3STATE_ASM  BCLR BDM_EN_BIT,DATA_PORT
#endif
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
// RESET control & sensing
//
#if (HW_CAPABILITY&CAP_RST_IO)

// RESET output pin
#define RESET_OUT           PTED_PTED5
#define RESET_OUT_DDR       PTEDD_PTEDD5
#define RESET_OUT_PER       PTEPE_PTEPE5
#define RESET_LOW()         (RESET_OUT=0,RESET_OUT_DDR=1)
#define RESET_3STATE()      (RESET_OUT=1,RESET_OUT_DDR=0) // Pull-up on pin

// RESET input pin
#define RESET_IN            PTED_PTED2
#define RESET_IN_DDR        PTEDD_PTEDD2
#define RESET_IN_PER        PTEPE_PTEPE2
#define RESET_IN_NUM        (2)
#define RESET_IN_MASK       (1<<RESET_IN_NUM)

#define RESET_IS_HIGH       (RESET_IN!=0)
#define RESET_IS_LOW        (RESET_IN==0)

#endif // CAP_RST_IO

//=================================================================================
// LED Port bit masks
//
#define GREEN_LED_MASK  (PTFD_PTFD4_MASK)
#define RED_LED_MASK    (PTFD_PTFD5_MASK)
#define LED_PORT_DATA   (PTFD)
#define LED_PORT_DDR    (PTFDD)

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

// Flash 12V charge pump control
#define FLASH12_ENn      (PTBD_PTBD1)
#define FLASH12V_DDR     (PTBDD_PTBDD1)

// Drive transistor for VPP on/off
#define VPP_EN           (PTBD_PTBD3)
#define VPP_EN_DDR       (PTBDD_PTBDD3)

// Order of statements within the following macros may be important 
#define FLASH12V_ON()   (FLASH12_ENn=0, FLASH12V_DDR=1)
#define FLASH12V_OFF()  (FLASH12_ENn=1, FLASH12V_DDR=1)

#define VPP_ON()        (VPP_EN=1,  VPP_EN_DDR=1) // VPP Control pin is output, VPP on
#define VPP_OFF()       (VPP_EN=0,  VPP_EN_DDR=1) // VPP Control pin is output, VPP off 

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

//  Vdd on/off
#define VDD3_EN          (PTBD_PTBD0)    // Enables 5V but regulated to 3.3V for target
#define VDD3_EN_DDR      (PTBDD_PTBDD0)
#define VDD5_ENn         (PTBD_PTBD2)
#define VDD5_EN_DDR      (PTBDD_PTBDD2)

#define VDD_OFF()        (VDD3_EN=0, VDD5_ENn=1, VDD3_EN_DDR=1, VDD5_EN_DDR=1) // Vdd Off
#define VDD3_ON()        (VDD3_EN=1, VDD5_ENn=1, VDD3_EN_DDR=1, VDD5_EN_DDR=1) // Vdd = 3.3V
#define VDD5_ON()        (VDD3_EN=0, VDD5_ENn=0, VDD3_EN_DDR=1, VDD5_EN_DDR=1) // Vdd = 5V

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
//   TPMx = TPM1
//     TPM1-CH0 - RST_I pin, Reset detection (IC falling edge detection)
//     TPM1-CH1 - BKGD_I pin, SYNC measuring, ACKN detection (Input Capture)
//     TPM1-CH3 - Timeouts (Output compare, no pin used)
//
// Use TPM1 as Timer TPMx
#define TPMSC              TPM1SC
#define TPMCNT             TPM1CNT
#define TPMSC_CLKSA_MASK   TPM1SC_CLKSA_MASK

// RESET Detection TPM1.Ch0 : Interrupt, Input capture, falling edge on pin 
#define RESET_TPMxCnSC_CHIE               TPM1C0SC_CH0IE      // Mask/enable RESET detection
#define RESET_TPMxCnSC_CHF                TPM1C0SC_CH0F       // Event Interrupt Flag
#define RESET_TPMxCnSC                    TPM1C0SC            // TPM Status & Configuration
#define RESET_TPMxCnSC_FALLING_EDGE_MASK  TPM1C0SC_ELS0B_MASK // Falling-edge input capture  

// BKGD Timing input - SYNC measuring, ACKN detection (IC rising & falling edges)
#define BKGD_TPMxCnSC_CHF                 TPM1C1SC_CH1F       // Event Flag
#define BKGD_TPMxCnSC                     TPM1C1SC            // TPM Channel Status & Configuration
#define BKGD_TPMxCnSC_RISING_EDGE_MASK    TPM1C1SC_ELS1A_MASK // TPMxCnSC value for rising edge
#define BKGD_TPMxCnSC_FALLING_EDGE_MASK   TPM1C1SC_ELS1B_MASK // TPMxCnSC value for falling edge
#define BKGD_TPMxCnVALUE                  TPM1C1V             // IC Event time
#define BKGD_TPM_SETUP_ASM                BCLR 7,BKGD_TPMxCnSC 

// Timeout TPM1.Ch3 : Output Compare (no pin)
#define TIMEOUT_TPMxCnSC_CHF              TPM1C3SC_CH3F       // Event Flag
#define TIMEOUT_TPMxCnSC                  TPM1C3SC            // TPM Status & Configuration
#define TIMEOUT_TPMxCnSC_OC_MASK          TPM1C3SC_MS3A_MASK  // TPMxCnSC value for OC event
#define TIMEOUT_TPMxCnVALUE               TPM1C3V             // OC Event time

//================================================================================
// RESET Detection - falling edge using Timer inputs
//
//     - RESET_IN pin, Reset detection (Falling edge detection)
//
#if (HW_CAPABILITY&CAP_RST_IO)
// Configure RESET change sensing (Falling edges)
#define CONFIGURE_RESET_SENSE()   (RESET_TPMxCnSC = RESET_TPMxCnSC_FALLING_EDGE_MASK)
// Enable & Configure RESET Change interrupts
#define ENABLE_RESET_SENSE_INT()  (RESET_TPMxCnSC_CHIE = 1)
// Clear Reset Change Event
#define CLEAR_RESET_SENSE_FLAG()  (RESET_TPMxCnSC_CHF  = 0)
// Disable  RESET Change interrupts
#define DISABLE_RESET_SENSE_INT() (RESET_TPMxCnSC_CHIE = 0)
#endif

//===================================================================================
// Target Vdd sensing
#if (HW_CAPABILITY&CAP_VDDSENSE)
// Target Vdd Present
#define VDD_SENSE                 (ACMPSC_ACO == 0)

#else // !CAP_VDDSENSE

#define VDD_SENSE                 (1) // Assume VDD present
#endif

//===================================================================================
// Target Vdd measuring

#define VDD_MEASURE_CHANNEL 9
// Enable & Configure Vdd Change interrupts (Using Analogue comparator, rising or falling edges)
#define CONFIGURE_VDD_SENSE()     (ACMPSC = ACMPSC_ACME_MASK|ACMPSC_ACBGS_MASK|ACMPSC_ACMOD1_MASK|ACMPSC_ACMOD0_MASK)
// Enable Vdd Change interrupts
#define ENABLE_VDD_SENSE_INT()    (ACMPSC_ACIE = 1) 
// Clear Vdd Change Event
#define CLEAR_VDD_SENSE_FLAG()    (ACMPSC_ACF = 1)
// Disable Vdd Change interrupts
#define DISABLE_VDD_SENSE_INT()   (ACMPSC_ACIE = 0) 

#endif // _CONFIGURE_H_
