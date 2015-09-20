/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_TWR.h - USDBM Common material for TWR boards
            
    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/

#ifndef USBDM_TWR_H_
#define USBDM_TWR_H_
#ifndef PLATFORM
#define PLATFORM USBDM   //! Choose BDM emulation
#endif

#define CPU  JMxx        //! Implementation Target

//=================================================================================
// Debug pin - used to check timing and hardware sequences etc.
//
#if (DEBUG != 0)
#define DEBUG_PIN_DDR PTGDD_PTGDD3
#define DEBUG_PIN_PER PTGPE_PTGPE3
#define DEBUG_PIN     PTGD_PTGD3
#endif

//=================================================================================
// ICP pin - used to force ICP in bootstrap code
// IRQ pin is used directly
#define ICP_USES_IRQ 1 
#define ICP_PIN_DDR  //!  
#define ICP_PIN_PER  //! 
#define ICP_PIN      //! 

//===========================================================================================
// Type of BDM interface chips are supported

#define DRIVER  LVC45  //! Choose driver IC being used

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
#define DTR_OUT            PTGD_PTGD2   
#define DTR_OUT_DDR        PTGDD_PTGDD2

#define CTS_IS_HIGH()      (CTS_IN!=0)
#define DTR_INACTIVE()     (DTR_OUT_DDR = 1, DTR_OUT=0)
#define DTR_ACTIVE()       (DTR_OUT_DDR = 1, DTR_OUT=1)

#define USE_SCI1 (1) // Use SCI1 (default is SCI2)
// TxD = PTE0
// RxD = PTE1
#endif

//=================================================================================
// JTAG Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_JTAG_HW)

// Allow SCLK_OUT to control input to buffer driving TCLK/PSTCLK
// TCLK_EN is never disabled by USBDM (buffer may however be 3-stated)
#define TCLK_EN_DDR         PTEDD_PTEDD3
#define TCLK_ENABLE()       (TCLK_EN_DDR = 0) // Allow TCLK_OUT to control buffer 

// OUT_EN 3-state control (controls TRST/DSCLK,TDI/TDO,TMS/BKPT, RxD, RESET_IN etc)
#define OUT_EN              PTED_PTED7
#define OUT_EN_DDR          PTEDD_PTEDD7
//#define OUT_EN_PER          PTEDD_PTEPE7

#define JTAG_DRV_ENABLE()  (OUT_EN=1, OUT_EN_DDR=1)
#define JTAG_DRV_DISABLE() (OUT_EN=0, OUT_EN_DDR=1)

// TDI out pin (MOSI1)
#define TDI_OUT             PTED_PTED5
#define TDI_OUT_BITNUM	    (5)
#define TDI_OUT_MASK        (1<<TDI_OUT_BITNUM)
#define TDI_OUT_DDR         PTEDD_PTEDD5

#define TDI_HIGH()          TDI_OUT = 1    // Assumes direction already set
#define TDI_LOW()           TDI_OUT = 0

// TDO in pin (MISO)
#define TDO_IN              PTED_PTED4
#define TDO_IN_BITNUM       (4)
#define TDO_IN_MASK         (1<<TDO_IN_BITNUM)
#define TDO_IN_DDR          PTEDD_PTEDD4

#define TDO_IS_HIGH         (TDO_IN!=0)

// TCLK out pin (SPICK1)
#define TCLK_OUT            PTED_PTED6
#define TCLK_OUT_BITNUM     (6)
#define TCLK_OUT_MASK       (1<<TCLK_OUT_BITNUM)
#define TCLK_OUT_DDR        PTEDD_PTEDD6

#define TCLK_HIGH()         TCLK_OUT = 1   // Assumes direction already set
#define TCLK_LOW()          TCLK_OUT = 0

// TMS out pin (MISO2)
#define TMS_OUT             PTBD_PTBD3    
#define TMS_OUT_DDR         PTBDD_PTBDD3

#define TMS_HIGH()          (TMS_OUT = 1)//, TMS_OUT_DDR = 1)
#define TMS_LOW()           (TMS_OUT = 0)//, TMS_OUT_DDR = 1)

// TRST* out pin
// Signal used to control TDSCLK/TRST* (force high/low) otherwise
// when 3-state TDSCLK/TRST* is driven from DSCLK_OUT
// 3-state  - (BDM mode)  DSCLK/TRST* controlled by DSCLK_OUT (DSCLK)
// low      - (JTAG mode) DSCLK/TRST* low (TRST* asserted)
// high     - (JTAG mode) DSCLK/TRST* high (TRST* negated)
#define TRST_OUT            PTED_PTED2
#define TRST_OUT_DDR        PTEDD_PTEDD2
#define TRST_OUT_PER        PTEPE_PTEPE2

#define TRST_LOW()     (TRST_OUT = 0, TRST_OUT_DDR=1) // DSCLK/TRST* = 0
// Can't 3-state so set high
#define TRST_3STATE()  (TRST_OUT = 1, TRST_OUT_DDR=1) // DSCLK/TRST* = 1

//! Masks to make JTAG interface idle @ref jtag_init 
//! TDI held low, TCLK undriven with PUP
#define JTAG_IDLE           (TCLK_OUT_MASK)
#define JTAG_IDLE_DDR       (TDI_OUT_MASK|TCLK_OUT_MASK)
#endif // CAP_JTAG_HW

//=================================================================================
// Coldfire V2,3,4 BDM Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_CFVx_HW)
#if !(HW_CAPABILITY&CAP_JTAG_HW)
#error "CAP_CFVx_HW requires CAP_JTAG_HW"
#endif

// Individual 3-state control for buffer driving TCLK/PSTCLK (CFVx boards only)
// Otherwise buffer is controlled by OUT_EN and enabled with other CFVx BDM signals
#define TCLK_CTL            PTBD_PTBD2
#define TCLK_CTL_DDR        PTBDD_PTBDD2

#define TCLK_CTL_ENABLE()   (TCLK_CTL = 1, TCLK_CTL_DDR=1)
#define TCLK_CTL_DISABLE()  (TCLK_CTL = 0, TCLK_CTL_DDR=1)

// CF_DRV 3-state control
#define CF_DRV            PTED_PTED7
#define CF_DRV_BITNUM     (7)
#define CF_DRV_MASK       (1<<CF_DRV_BITNUM)
#define CF_DRV_DDR        PTEDD_PTEDD7
//#define CF_DRV_PER        PTEPE_PTEPE7

#define CF_DRV_ENABLE()   (CF_DRV = 1, CF_DRV_DDR=1)
#define CF_DRV_DISABLE()  (CF_DRV = 0, CF_DRV_DDR=1)

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
// Actually a connection overriding CLK_OUT - not used in CFVx mode

// Signal used to disable TDSCLK (force high/low) otherwise
// when 3-state TDSCLK is driven from DSCLK_OUT
// Doubles as TRST control
// 3-state  - (BDM mode)  DSCLK/TRST* controlled by DSCLK_OUT (DSCLK)
// low      - (JTAG mode) DSCLK/TRST* low (TRST* asserted)
// high     - (JTAG mode) DSCLK/TRST* high (TRST* negated)
#define DSCLK_DRV            PTED_PTED2
#define DSCLK_DRV_DDR        PTEDD_PTEDD2
#define DSCLK_DRV_PER        PTEPE_PTEPE2

#define DSCLK_DRV_ENABLE()   (DSCLK_DRV = 0, DSCLK_DRV_DDR=0) // DSCLK/TRST*=DSCLK_OUT
#define DSCLK_DRV_DISABLE()  (DSCLK_DRV = 1, DSCLK_DRV_DDR=1) // Force DSCLK/TRST* high

// BKPT out pin
#define BKPT_OUT           PTBD_PTBD3
#define BKPT_OUT_DDR       PTBDD_PTBDD3
#define BKPT_OUT_PER       PTBPE_PTBPE3

#define BKPT_LOW()         (BKPT_OUT=0,BKPT_OUT_DDR=1)
#define BKPT_HIGH()        (BKPT_OUT=1,BKPT_OUT_DDR=0) // Uses pull-up

// TA out pin - not used
//#define TA_OUT             
//#define TA_OUT_DDR         
//#define TA_OUT_PER         

#define TA_LOW()           ;
#define TA_3STATE()        ;
   
// PST in pins
#define PST_IN             PTCD
#define PST_IN_DDR         PTCDD
#define PST_IN_MASK        (0x0F)

#define PST_IN_ENABLE()    (PST_IN_DDR &= ~PST_IN_MASK)
#define ALLPST_IS_HIGH     ((PST_IN&PST_IN_MASK) == PST_IN_MASK)

#define DATA_PORT          PTED
#define DATA_PORT_DDR      PTEDD

// Masks to make BDM interface idle @ref bdmcf_init
// DSI=0, DSCLK=0 all others undriven (PUPs) 
#define BDMCF_IDLE         (CF_DRV_MASK)
#define BDMCF_IDLE_DDR     (DSI_OUT_MASK|DSCLK_OUT_MASK|CF_DRV_MASK)
#endif // (HW_CAPABILITY&CAP_CFVx_HW)

//=================================================================================
// HCS08, HCS12, RS08 & Coldfire V1 BDM Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_BDM)
#define DATA_PORT          PTFD
#define DATA_PORT_DDR      PTFDD
#define DATA_PORT_PER      PTFDE

// BDM data out pin
#define BDM_OUT           PTFD_PTFD4
#define BDM_OUT_BIT       (4)
#define BDM_OUT_MASK      (1<<BDM_OUT_BIT)
#define BDM_OUT_PER       PTFPE_PTFPE4

// BDM data in pin
#define BDM_IN            PTFD_PTFD1
#define BDM_IN_BIT        (1)
#define BDM_IN_MASK       (1<<BDM_IN_BIT)

// BDM data direction pin - controls buffer enable/direction
#define BDM_EN            PTFD_PTFD5
#define BDM_EN_BIT        (5)
#define BDM_EN_MASK       (1<<BDM_EN_BIT)
#define BDM_EN_PER        PTFPE_PTFPE5

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

// RESET output pin (inverted through BJT)
#define RESET_OUT           PTCD_PTCD4
#define RESET_OUT_DDR       PTCDD_PTCDD4
#define RESET_OUT_PER       PTCPE_PTCPE4
#define RESET_LOW()         (RESET_OUT=1,RESET_OUT_DDR=1)
#define RESET_3STATE()      (RESET_OUT=0,RESET_OUT_DDR=1)

// RESET input pin
#define RESET_IN            PTDD_PTDD2
#define RESET_IN_DDR        PTDDD_PTDDD2
#define RESET_IN_PER        PTDPE_PTDPE2
#define RESET_IN_NUM        (2)
#define RESET_IN_MASK       (1<<RESET_IN_NUM)

#define RESET_IS_HIGH       (RESET_IN!=0)
#define RESET_IS_LOW        (RESET_IN==0)

#else
#define RESET_IS_HIGH       (0)
#define RESET_IS_LOW        (1)
#endif // CAP_RST_IO

//=================================================================================
// LED Port bit masks
//
#define GREEN_LED_MASK  (PTDD_PTDD1_MASK) //! D6 (yellow/green)
#define RED_LED_MASK    (PTDD_PTDD0_MASK) //! D5 (amber)
#define LED_PORT_DATA   (PTDD)            //!
#define LED_PORT_DDR    (PTDDD)           //! 

// LEDs off, LED pins are outputs 
#define LED_INIT()         ((LED_PORT_DATA |= RED_LED_MASK|GREEN_LED_MASK), \
                            (LED_PORT_DDR  |= RED_LED_MASK|GREEN_LED_MASK))
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
#define FLASH12_EN       (PTCD_PTCD5)
#define FLASH12V_DDR     (PTCDD_PTCDD5)

// Drive transistor for VPP on/off
#define VPP_EN           (PTFD_PTFD0)
#define VPP_EN_DDR       (PTFDD_PTFDD0)

// Order of statements within the following macros may be important 
#define FLASH12V_ON()   (FLASH12_EN=1, FLASH12V_DDR=1)
#define FLASH12V_OFF()  (FLASH12_EN=0, FLASH12V_DDR=0) // External pull-down

#define VPP_ON()        (VPP_EN=1,  VPP_EN_DDR=1)
#define VPP_OFF()       (VPP_EN=0,  VPP_EN_DDR=0) // External pull-down 

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
#define VDD_ON_INITIALLY 3  // Enable target Vdd=3V on plugin

//  Vdd on/off
#define VDD3_EN          (PTBD_PTBD1)   // Enables 5V but regulated to 3.3V for target
#define VDD3_EN_DDR      (PTBDD_PTBDD1)
#define VDD5x_EN         (PTBD_PTBD0)   // Main supply 5V to Interface
#define VDD5x_EN_DDR     (PTBDD_PTBDD0)

#define VDD_OFF()        (VDD3_EN=0, VDD5x_EN=0, VDD3_EN_DDR=1, VDD5x_EN_DDR=1) // Vdd Off, Vx=Off
#define VDD3_ON()        (VDD3_EN=1, VDD5x_EN=1, VDD3_EN_DDR=1, VDD5x_EN_DDR=1) // Vdd = 3.3V, Vx=5v
#define VDD5_ON()         VDD3_ON() // Treat as 3V

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
//     TPM1-CH3 - BKGD_I pin, SYNC measuring, ACKN detection (Input Capture)
//     TPM1-CH1 - Timeouts (Output compare, no pin used)
//     TPM1-CH2 - Flash VPP_EN pin control
//
// Use TPM1 as Timer TPMx
#define TPMSC              TPM1SC
#define TPMCNT             TPM1CNT
#define TPMCNTH            TPM1CNTH
#define TPMSC_CLKSA_MASK   TPM1SC_CLKSA_MASK

// SYNC Detection TPM1.Ch3 : Input capture (on BKGD_I pin)
#define BKGD_TPMxCnSC_CHF                 TPM1C3SC_CH3F       // Event Flag
#define BKGD_TPMxCnSC                     TPM1C3SC            // TPM Channel Status & Configuration
#define BKGD_TPMxCnSC_RISING_EDGE_MASK    TPM1C3SC_ELS3A_MASK // TPMxCnSC value for rising edge
#define BKGD_TPMxCnSC_FALLING_EDGE_MASK   TPM1C3SC_ELS3B_MASK // TPMxCnSC value for falling edge
#define BKGD_TPMxCnVALUE                  TPM1C3V             // IC Event time
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
#define VDD_SENSE                 (TARGETVDD_IN) // May be unreliable

#else // !CAP_VDDSENSE

#define VDD_SENSE                 (1) // Assume VDD present
#endif

//===================================================================================
// Target Vdd measuring

#define VDD_MEASURE_CHANNEL 5 // ADC channel to use
#define VDD_HAS_DIVIDER     2 // Most boards have VDD input 2:1 voltage divider

// Vdd in pin
#define TARGETVDD_IN            PTBD_PTBD5
#define TARGETVDD_IN_DDR        PTBDD_PTBDD5
#define TARGETVDD_IN_PER        PTBPE_PTBPE5
#define TARGETVDD_IN_NUM        (5)
#define TARGETVDD_IN_MASK       (1<<TARGETVDD_IN_NUM)

// Configure Target VDD change sensing (Falling edges)
#define CONFIGURE_VDD_SENSE()     (KBIES &= ~TARGETVDD_IN_MASK)
// Enable Vdd Change interrupts
#define ENABLE_VDD_SENSE_INT()    (KBIPE |= TARGETVDD_IN_MASK,  \
                                   KBISC = KBISC_KBACK_MASK,\
                                   KBISC = KBISC_KBIE_MASK ) 
// Clear Vdd Change Event
#define CLEAR_VDD_SENSE_FLAG()    (KBISC = KBISC_KBIE_MASK|KBISC_KBACK_MASK)
// Disable Vdd Change interrupts
#define DISABLE_VDD_SENSE_INT()   (KBIPE &= ~TARGETVDD_IN_MASK)

#endif /* USBDM_TWR_H_ */
