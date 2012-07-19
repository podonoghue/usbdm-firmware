/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_CF_JMxxCLD - Universal BDM/CF, JMxxCLD Version \n
    Combined TBDML/OSBDM/TBLCF + extensions,
             
    @htmlonly <a href="USBDM_CF_JMxxCLD.pdf">schematic</a> @endhtmlonly \n
    Supports HC12, HCS08, RS08 and Coldfire V1, V2, V3 & V4 targets \n

    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//==========================================================================================
// USB Serial Number
#define SERIAL_NO          "USBDM-JMxx-CF-0001"
#define ProductDescription "USBDM RS08,HCS08,HCS12,DSC,Coldfire BDM"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_JTAG_HW|CAP_BDM  |CAP_FLASH|        CAP_RST_IO|CAP_CFVx)
#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_HCS12 |CAP_HCS08|CAP_RS08 |CAP_CFV1|CAP_RST   |CAP_CFVx|CAP_JTAG|CAP_DSC|CAP_ARM_JTAG|CAP_PST)

#ifndef PLATFORM
#define PLATFORM USBDM   //! Choose BDM emulation
#endif

#define CPU  JMxx

//=================================================================================
// Debug pin - used to check timing and hardware sequences etc.
//
#if (DEBUG != 0)
#define DEBUG_PIN_DDR PTGDD_PTGDD2
#define DEBUG_PIN_PER PTGPE_PTGPE2
#define DEBUG_PIN     PTGD_PTGD2
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
#define CTS_IN             // Not used
#define DTR_OUT            
#define DTR_OUT_DDR        

#define CTS_IS_HIGH()      (1)
#define DTR_INACTIVE()          ;
#define DTR_ACTIVE()         ;
#endif

//=================================================================================
// Coldfire V2,3,4 BDM Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_CFVx_HW)
// CF_DRV (=JTAG_DRV) 3-state control
#define CF_DRV            PTCD_PTCD1
#define CF_DRV_DDR        PTCDD_PTCDD1
#define CF_DRV_PER        PTCPE_PTCPE1

#define CF_DRV_ENABLE()   (CF_DRV = 0, CF_DRV_DDR=1)
#define CF_DRV_DISABLE()  (CF_DRV = 1, CF_DRV_DDR=0)

// DSO in pin (MISO1)
#define DSO_IN             PTED_PTED4
#define DSO_IN_BITNUM      (4)
#define DSO_IN_MASK        (1<<DSO_IN_BITNUM)
#define DSO_IN_DDR         PTEDD_PTEDD4
#define DSO_IN_PORT        PTED

// DSI out pin (MOSI1) - DSI, DSCLK must be on the same port 
#define DSI_OUT            PTED_PTED5
#define DSI_OUT_BITNUM     (5) 
#define DSI_OUT_MASK       (1<<DSI_OUT_BITNUM)
#define DSI_OUT_PER        PTEPE_PTEPE5
#define DSI_OUT_DDR        PTEDD_PTEDD5

// DSCLK out pin (SPSCK1) - DSI, DSCLK must be on the same port 
#define DSCLK_OUT          PTED_PTED6
#define DSCLK_OUT_BITNUM   (6)
#define DSCLK_OUT_MASK     (1<<DSCLK_OUT_BITNUM)
#define DSCLK_OUT_PORT     PTED
#define DSCLK_OUT_PER      PTEPE_PTEPE6
#define DSCLK_OUT_DDR      PTEDD_PTEDD6

#define ASM_DSCLK_HIGH     bset  DSCLK_OUT_BITNUM,DSCLK_OUT_PORT
#define ASM_DSCLK_LOW      bclr  DSCLK_OUT_BITNUM,DSCLK_OUT_PORT

// DSCLK 3-state control
#define DSCLK_DRV            PTFD_PTFD5
#define DSCLK_DRV_DDR        PTFDD_PTFDD5
#define DSCLK_DRV_PER        PTFPE_PTFPE5

#define DSCLK_DRV_ENABLE()   (DSCLK_DRV = 0, DSCLK_DRV_DDR=1)
#define DSCLK_DRV_DISABLE()  (DSCLK_DRV = 1, DSCLK_DRV_DDR=0)

// BKPT out pin
#define BKPT_OUT           PTBD_PTBD0
#define BKPT_OUT_DDR       PTBDD_PTBDD0
#define BKPT_OUT_PER       PTBPE_PTBPE0

#define BKPT_LOW()         (BKPT_OUT=0,BKPT_OUT_DDR=1)
#define BKPT_HIGH()        (BKPT_OUT=1,BKPT_OUT_DDR=0)

// TA out pin 
#define TA_OUT             PTFD_PTFD4
#define TA_OUT_DDR         PTFDD_PTFDD4
#define TA_OUT_PER         PTFPE_PTFPE4

#define TA_LOW()           (TA_OUT=0,TA_OUT_DDR=1)
#define TA_3STATE()        (TA_OUT=1,TA_OUT_DDR=0)
   
// ALLPST in pin 
#define ALLPST_IN          PTCD_PTCD2
#define ALLPST_IN_DDR      PTCDD_PTCDD2
#define ALLPST_IN_PER      PTCPE_PTCPE2

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

#define JTAG_DRV_ENABLE()   (JTAG_DRV = 0, JTAG_DRV_DDR=1)
#define JTAG_DRV_DISABLE()  (JTAG_DRV = 1, JTAG_DRV_DDR=0)

// TDI out pin (MOSI1)
#define TDI_OUT             DSI_OUT
#define TDI_OUT_BITNUM	    DSI_OUT_BITNUM
#define TDI_OUT_MASK        DSI_OUT_MASK
#define TDI_OUT_DDR         DSI_OUT_DDR

#define TDI_HIGH()          TDI_OUT = 1    // Assumes direction already set
#define TDI_LOW()           TDI_OUT = 0

// TDO in pin (MISO)
#define TDO_IN              DSO_IN
#define TDO_IN_BITNUM       DSO_IN_BITNUM
#define TDO_IN_MASK         DSO_IN_MASK
#define TDO_IN_DDR          DSO_IN_DDR

#define TDO_IS_HIGH         (TDO_IN!=0)

// TCLK out pin (SPICK1)
#define TCLK_OUT            DSCLK_OUT
#define TCLK_OUT_BITNUM     DSCLK_OUT_BITNUM
#define TCLK_OUT_MASK       DSCLK_OUT_MASK
#define TCLK_OUT_DDR        DSCLK_OUT_DDR

#define TCLK_HIGH()         TCLK_OUT = 1   // Assumes direction already set
#define TCLK_LOW()          TCLK_OUT = 0

// TMS out pin (MISO2)
#define TMS_OUT             BKPT_OUT    
#define TMS_OUT_DDR         BKPT_OUT_DDR

#define TMS_HIGH()          (TMS_OUT = 1)//, TMS_OUT_DDR = 1)
#define TMS_LOW()           (TMS_OUT = 0)//, TMS_OUT_DDR = 1)

// TRST* out pin
#define TRST_OUT            PTED_PTED3
#define TRST_OUT_DDR        PTEDD_PTEDD3
#define TRST_OUT_PER        PTEPE_PTEPE3

#define TRST_LOW()          (TRST_OUT=0,TRST_OUT_DDR=1)
#define TRST_3STATE()       (TRST_OUT=1,TRST_OUT_DDR=0)

//! Masks to make JTAG interface idle @ref jtag_init 
//! TDI held low, TCLK undriven with PUP
#define JTAG_IDLE           (TCLK_OUT_MASK)
#define JTAG_IDLE_DDR       (TDI_OUT_MASK|TCLK_OUT_MASK)
#endif // CAP_JTAG_HW

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
#define BDM_EN            PTED_PTED1
#define BDM_EN_BIT        (1)
#define BDM_EN_MASK       (1<<BDM_EN_BIT)
#define BDM_EN_PER        PTEPE_PTEPE1

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
#define RESET_OUT           PTCD_PTCD4
#define RESET_OUT_DDR       PTCDD_PTCDD4
#define RESET_OUT_PER       PTCPE_PTCPE4
#define RESET_LOW()         (RESET_OUT=0,RESET_OUT_DDR=1)
#define RESET_3STATE()      (RESET_OUT=1,RESET_OUT_DDR=0) // Pull-up on pin

// RESET input pin
#define RESET_IN            PTFD_PTFD1
#define RESET_IN_DDR        PTFDD_PTFDD1
#define RESET_IN_PER        PTFPE_PTFPE1
#define RESET_IN_NUM        (1)
#define RESET_IN_MASK       (1<<RESET_IN_NUM)

#define RESET_IS_HIGH       (RESET_IN!=0)
#define RESET_IS_LOW        (RESET_IN==0)

#endif // CAP_RST_IO

//=================================================================================
// LED Port bit masks
//
#define GREEN_LED_MASK  (PTGD_PTGD0_MASK)
#define RED_LED_MASK    (PTGD_PTGD1_MASK)
#define LED_PORT_DATA   (PTGD)
#define LED_PORT_DDR    (PTGDD)

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
#define FLASH12_ENn      (PTBD_PTBD5)
#define FLASH12V_DDR     (PTBDD_PTBDD5)

// Drive transistor for VPP on/off
#define VPP_EN           (PTFD_PTFD0)
#define VPP_EN_DDR       (PTFDD_PTFDD0)

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
#define VDD3_EN          (PTBD_PTBD1)   // Enables 5V but regulated to 3.3V for target
#define VDD3_EN_DDR      (PTBDD_PTBDD1)
#define VDD5_EN          (PTBD_PTBD4)
#define VDD5_EN_DDR      (PTBDD_PTBDD4)

#define VDD_OFF()        (VDD5_EN=0, VDD5_EN_DDR=1, VDD3_EN=0, VDD3_EN_DDR=1) // Vdd Off
#define VDD3_ON()        (VDD5_EN=0, VDD5_EN_DDR=1, VDD3_EN=1, VDD3_EN_DDR=1) // Vdd = 3.3V
#define VDD5_ON()        (VDD3_EN=0, VDD3_EN_DDR=1, VDD5_EN=1, VDD5_EN_DDR=1) // Vdd = 5V

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
//     TPM1-CH3 - RST_I pin, Reset detection (IC falling edge detection)
//     TPM1-CH0 - BKGD_I pin, SYNC measuring, ACKN detection (Input Capture)
//     TPM1-CH1 - Timeouts (Output compare, no pin used)
//     TPM1-CH2 - Flash VPP_EN pin control
//
// Use TPM1 as Timer TPMx
#define TPMSC              TPM1SC
#define TPMCNT             TPM1CNT
#define TPMCNTH            TPM1CNTH
#define TPMSC_CLKSA_MASK   TPM1SC_CLKSA_MASK

// RESET Detection TPM1.Ch3 : Interrupt, Input capture, falling edge on pin 
#define RESET_TPMxCnSC_CHIE               TPM1C3SC_CH3IE      // Mask/enable RESET detection
#define RESET_TPMxCnSC_CHF                TPM1C3SC_CH3F       // Event Interrupt Flag
#define RESET_TPMxCnSC                    TPM1C3SC            // TPM Status & Configuration
#define RESET_TPMxCnSC_FALLING_EDGE_MASK  TPM1C3SC_ELS3B_MASK // Falling-edge input capture  

// SYNC Detection TPM1.Ch0 : Input capture
#define BKGD_TPMxCnSC_CHF                 TPM1C0SC_CH0F        // Event Flag
#define BKGD_TPMxCnSC                     TPM1C0SC             // TPM Channel Status & Configuration
#define BKGD_TPMxCnSC_RISING_EDGE_MASK    TPM1C0SC_ELS0A_MASK  // TPMxCnSC value for rising edge
#define BKGD_TPMxCnSC_FALLING_EDGE_MASK   TPM1C0SC_ELS0B_MASK  // TPMxCnSC value for falling edge
#define BKGD_TPMxCnVALUE                  TPM1C0V              // IC Event time
#define BKGD_TPM_SETUP_ASM                BCLR 7,BKGD_TPMxCnSC 

// Timeout TPM1.Ch1 : Output Compare (no pin)
#define TIMEOUT_TPMxCnSC_CHF              TPM1C1SC_CH1F       // Event Flag
#define TIMEOUT_TPMxCnSC                  TPM1C1SC            // TPM Status & Configuration
#define TIMEOUT_TPMxCnSC_OC_MASK          TPM1C1SC_MS1A_MASK  // TPMxCnSC value for OC event
#define TIMEOUT_TPMxCnVALUE               TPM1C1V             // OC Event time

#if (HW_CAPABILITY&CAP_FLASH)
// VPP_EN is controlled via a timer channel to prevent conflicts with other pin usage on common port
#define VPP_USES_TIMER
// Vpp_En control : TPM1.Ch1 : Output Compare (PTF.0)
#define VPP_TPMxCnSC_CHF                  TPM1C2SC_CH2F       // Event Flag
#define VPP_TPMxCnSC                      TPM1C2SC            // TPM Status & Configuration
#define VPP_TPMxCnSC_OC_CLR_MASK          (TPM1C2SC_MS2A_MASK|TPM1C2SC_ELS2B_MASK)  // Clear pin
#define VPP_TPMxCnSC_OC_SET_MASK          (TPM1C2SC_MS2A_MASK|TPM1C2SC_ELS2B_MASK|TPM1C2SC_ELS2A_MASK) // Set pin
#define VPP_TPMxCnVALUE                   TPM1C2V             // OC Event time
#endif

//================================================================================
// RESET Detection - falling edge using Timer inputs
//
//     KBI     - RESET_IN pin, Reset detection (Falling edge detection)
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
