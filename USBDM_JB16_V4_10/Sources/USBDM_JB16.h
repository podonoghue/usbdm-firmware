/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM - Universal BDM, JB16 Version \n
    Combined TBDML/OSBDM + extensions hardware, see 
    @htmlonly <a href="USBDM_JB16_SOIC.pdf">schematic</a> @endhtmlonly \n
    Supports HC12, HCS08, RS08 and Coldfire V1 targets \n

    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropropriately.
*/
//================================================================================
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

#ifndef PLATFORM
#define PLATFORM USBDM   // Choose BDM emulation
#endif

// USB Serial Number
#define SERIAL_NO "USBDM-JB16-0001"
// USB Product description string - should include VERSION_STR
#define ProductDescription "USBDM - JB16, Version " VERSION_STR
//=================================================================================
// Debug pin - used to check timing and hardware sequences etc.
//
#if (DEBUG != 0)
#define DEBUG_PIN_DDR DDRE_DDRE0
#define DEBUG_PIN     PTE_PTE0
#endif

#define ICP_PIN            (1<<0)  // ICP Test input A.0
#define ENABLE_ICP_PIN()   (POCR_PAP = 1, DDRA &= ~ICP_PIN)
#define DISABLE_ICP_PIN()  (POCR_PAP = 0)
#define TEST_ICP_PIN()     (~PTA & ICP_PIN)

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_RST_IO|CAP_BDM  |CAP_FLASH)
#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_HCS12|CAP_HCS08|CAP_RS08|CAP_CFV1)

//===========================================================================================
// Type of BDM interface chips are supported

#define DRIVER  LVC125  // Choose driver IC being used

#ifndef DRIVER
#error "Please define DRIVER in Configure.h"
#define DRIVER LVC125
#endif

#if ((DRIVER!=LVC125) && (DRIVER!=LVC45))
#error "Please correctly define DRIVER in Configure.h"
#define DRIVER LVC125
#endif







//=================================================================================
// Rx & Tx Pin assignments
// Please note: pin assignments cannot be changed freely
// RX AND TX ROUTINES ARE DEPENDENT ON THIS SPECIFIC ASSIGNMENTS

//=================================================================================
// Data Port Bit masks - hard coded in Rx/Tx routines
//
#if (HW_CAPABILITY&CAP_BDM)
#define DATA_PORT         PTA
#define DATA_PORT_DDR     DDRA

// BDM data out pin - hard coded in Rx/Tx routines
#define BDM_OUT           PTA_PTA7
#define BDM_OUT_BIT       (7)  // Bit number!
#define BDM_OUT_MASK      (1<<BDM_OUT_BIT)

// BDM data in pin
#define BDM_IN            PTA_PTA0
#define BDM_IN_BIT        (0)  // Bit number!
#define BDM_IN_MASK       (1<<BDM_IN_BIT)

// Primary BDM data direction pin - only disabled during Tx routines
#define BDM_DIR_Rx        PTA_PTA6
#define BDM_DIR_Rx_BIT    (6)  // Bit number!
#define BDM_DIR_Rx_MASK   (1<<BDM_DIR_Rx_BIT)

// Polarity of BDM buffer enable/direction varies with driver IC
#if (DRIVER == LVC125)
	#define BDM_DIR_Rx_RD     BDM_DIR_Rx_MASK
	#define BDM_DIR_Rx_WR     0
#elif (DRIVER == LVC45)
	#define BDM_DIR_Rx_RD     0
	#define BDM_DIR_Rx_WR     BDM_DIR_Rx_MASK
#endif

//=================================================================================
// Direction Port bit masks
//
#define DIR_PORT          PTC
#define DIR_PORT_DDR      DDRC

// Secondary BDM data direction pin - only enabled during Tx routines
#define BDM_DIR           PTC_PTC1
#define BDM_DIR_BIT       (1) // Bit #
#define BDM_DIR_MASK      (1<<BDM_DIR_BIT)

#define BDM_DIR_DDR       DDRC_DDRC1

// Polarity of BDM buffer enable/direction varies with driver IC
// Following MACROs only used by Tx related routines
// i.e. when BKGD is controlled through DIR_PORT
#if (DRIVER == LVC125)
	#define BDM_DIR_RD        BDM_DIR_MASK
	#define BDM_DIR_WR        0
	#define BDM_ENABLE_TX()   (BDM_DIR=0)
	#define BDM_3STATE_TX()   (BDM_DIR=1)
	#define BDM_ENABLE_ASM_TX BCLR BDM_DIR_BIT,DIR_PORT
	#define BDM_3STATE_ASM_TX BSET BDM_DIR_BIT,DIR_PORT
#elif (DRIVER == LVC45)
	#define BDM_DIR_RD        0
	#define BDM_DIR_WR        BDM_DIR_MASK
	#define BDM_ENABLE_TX()   (BDM_DIR=1)
	#define BDM_3STATE_TX()   (BDM_DIR=0)
	#define BDM_ENABLE_ASM_TX BSET BDM_DIR_BIT,DIR_PORT
	#define BDM_3STATE_ASM_TX BCLR BDM_DIR_BIT,DIR_PORT
#endif

#endif // CAP_BDM
//=================================================================================
// RESET control & sensing
//
#if (HW_CAPABILITY&CAP_RST_IO)

// RESET output pin
#define RESET_OUT         PTA_PTA4
#define RESET_OUT_DDR     DDRA_DDRA4
#define RESET_OUT_BIT     (4)
#define RESET_OUT_MASK    (1<<RESET_OUT_BIT)

// RESET input pin
#define RESET_IN          PTA_PTA5
#define RESET_IN_BIT      (5)
#define RESET_IN_MASK     (1<<RESET_IN_BIT)
// RESET_IN_DDR is not used as implicitly controlled when using shared DATA_PORT
//#define RESET_IN_DDR      DDRA_DDRA5

#define RESET_IS_HIGH       (RESET_IN!=0)
#define RESET_IS_LOW        (RESET_IN==0)

// RESET buffer enable/direction control varies with driver
#define RESET_LOW()       (RESET_OUT=0,RESET_OUT_DDR=1)
#define RESET_3STATE()    (RESET_OUT_DDR=0)

#define CONFIGURE_RESET_SENSE()   (KBSCR  = 0)               // Capture RESET falling edges
#define CLEAR_RESET_SENSE_FLAG()  (KBSCR_ACKK = 1)           // Clear RESET (KBD)Event
#define ENABLE_RESET_SENSE_INT()  (KBIER |= RESET_IN_MASK)   // Enable RESET (KBD) Interrupts
#define DISABLE_RESET_SENSE_INT() (KBIER &= ~RESET_IN_MASK)  // Disable RESET (KBD) Interrupts
#endif // CAP_RST_IO

//=================================================================================
// LED Port bit masks
// JB16 has built in current limits on PTD.2-5 so no resistors are needed.
// A resistor is needed if PTD.0-1 is used (original TBDML).
//
#define GREEN_LED_MASK  (PTD_PTD3_MASK)
#define RED_LED_MASK    (PTD_PTD4_MASK)
#define LED_PORT_DATA   (PTD)
#define LED_PORT_DDR    (DDRD)

/* LEDs off, some LED pins are outputs with Io limit */  
#define LED_INIT()         ((LED_PORT_DATA |= RED_LED_MASK|GREEN_LED_MASK), \
                            (POCR_PTDLDD = 1),\
                            (LED_PORT_DDR  |= RED_LED_MASK|GREEN_LED_MASK))
#define GREEN_LED_ON()     (LED_PORT_DATA &=~GREEN_LED_MASK)
#define GREEN_LED_OFF()    (LED_PORT_DATA |= GREEN_LED_MASK)
#define GREEN_LED_TOGGLE() (LED_PORT_DATA ^= GREEN_LED_MASK)
#define RED_LED_ON()       (LED_PORT_DATA &=~RED_LED_MASK)
#define RED_LED_OFF()      (LED_PORT_DATA |= RED_LED_MASK)
#define RED_LED_TOGGLE()   (LED_PORT_DATA ^= RED_LED_MASK)

//=================================================================================
// Flash programming control
//
#if (HW_CAPABILITY&CAP_FLASH)

// Flash 12V charge pump control
#define FLASH12_ENn      (PTD_PTD2)
#define FLASH12V_DDR     (DDRD_DDRD2)

// Drive transistor for VPP on/off
#define VPP_EN           (PTD_PTD5)
#define VPP_EN_DDR       (DDRD_DDRD5)

// Order of statements within the following macros may be important 
#define FLASH12V_ON()   (FLASH12_ENn=0, FLASH12V_DDR=1)
#define FLASH12V_OFF()  (FLASH12_ENn=1, FLASH12V_DDR=1)

#define VPP_ON()        (VPP_EN=1,  VPP_EN_DDR=1) // VPP Control pin is output, VPP on
#define VPP_OFF()       (VPP_EN=0,  VPP_EN_DDR=1) // VPP Control pin is output, VPP off 

#else // !CAP_FLASH
// No Flash programming supply
#define FLASH12V_ON()   ;
#define FLASH12V_OFF()  ;

#define VPP_ON()        ;
#define VPP_OFF()       ;

#endif //CAP_FLASH

//=================================================================================
// Target Vdd control

#if (HW_CAPABILITY&CAP_VDDCONTROL)

//  Vdd on/off
#define VDD3_EN          (PTD_PTD1)
#define VDD3_EN_DDR      (DDRD_DDRD1)
#define VDD5_ENn         (PTD_PTD0)
#define VDD5_EN_DDR      (DDRD_DDRD0)

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
// Choice of input capture pins for SYNC & ACK timing
// This timer is also used for timing delays
//
#define IC_T1CH01 (1) // PTE1/T1CH01 pin
#define IC_T2CH01 (2) // PTE2/T2CH01 pin

#define IC_BDM_TIMING_CHANNEL IC_T2CH01

#if (IC_BDM_TIMING_CHANNEL == IC_T1CH01)
#define IC_BDM_TIMING_TMOD      T1MOD
#define IC_BDM_TIMING_TSC0      T1SC0
#define IC_BDM_TIMING_TSC       T1SC
#define IC_BDM_TIMING_TSC_TOF   T1SC_TOF
#define IC_BDM_TIMING_TCH0      T1CH0
#define IC_BDM_TIMING_TSC0_CH0F T1SC0_CH0F
#elif (IC_BDM_TIMING_CHANNEL == IC_T2CH01)
#define IC_BDM_TIMING_TMOD      T2MOD
#define IC_BDM_TIMING_TSC0      T2SC0
#define IC_BDM_TIMING_TSC       T2SC
#define IC_BDM_TIMING_TSC_TOF   T2SC_TOF
#define IC_BDM_TIMING_TCH0      T2CH0
#define IC_BDM_TIMING_TSC0_CH0F T2SC0_CH0F
#else
#error "Please define timer channel (IC_BDM_TIMING_CHANNEL) in the Configuration file"
#endif

//=================================================================================
//
#if (HW_CAPABILITY&CAP_VDDSENSE)

#define VDD_SENSE       (PTE_PTE1==0)
#define VDD_SENSE_DDR   (DDRE_DDRE1)

#define IC_VDD_SENSING_CHANNEL IC_T1CH01

// Choice of input capture pins for Target Vdd Sensing
#if (IC_VDD_SENSING_CHANNEL == IC_T1CH01)
#define IC_VDD_SENSING_TMOD      T1MOD
#define IC_VDD_SENSING_TSC0      T1SC0
#define IC_VDD_SENSING_TSC       T1SC
#define IC_VDD_SENSING_TCH0      T1CH0
#define IC_VDD_SENSING_TSC_TOF   T1SC_TOF
#define IC_VDD_SENSING_TSC0_CH0F T1SC0_CH0F
#elif (IC_VDD_SENSING_CHANNEL == IC_T2CH01)
#define IC_VDD_SENSING_TMOD      T2MOD
#define IC_VDD_SENSING_TSC0      T2SC0
#define IC_VDD_SENSING_TSC       T2SC
#define IC_VDD_SENSING_TCH0      T2CH0
#define IC_VDD_SENSING_TSC_TOF   T2SC_TOF
#define IC_VDD_SENSING_TSC0_CH0F T2SC0_CH0F
#else
#error "Please define timer channel (IC_VDD_SENSING_CHANNEL) in the Configuration file"
#endif

// Enable & Configure Vdd Change interrupts
#define CONFIGURE_VDD_SENSE()  (                            /* Capture Vdd rising & falling edges */ \
   IC_VDD_SENSING_TMOD = 0xFFFF,                            /* No timer timeout                   */ \
   IC_VDD_SENSING_TSC  = 0,                                 /* Start counting at bus rate         */ \
   IC_VDD_SENSING_TSC0 = T1SC0_ELS0B_MASK|T1SC0_ELS0A_MASK) /* IC event on rising or falling edge */
// Enable Vdd Change interrupts
#define ENABLE_VDD_SENSE_INT() (IC_VDD_SENSING_TSC0 |= T1SC0_CH0IE_MASK) // Enable Vdd Change (IC) interrupts
// Clear Vdd Change Event
#define CLEAR_VDD_SENSE_FLAG() (IC_VDD_SENSING_TSC0_CH0F = 0)            // Clear Vdd Change (IC) Event
// Disable Vdd Change interrupts
#define DISABLE_VDD_SENSE_INT() (IC_VDD_SENSING_TSC0 &= ~T1SC0_CH0IE_MASK) // Enable Vdd Change (IC) interrupts
#endif // CAP_VDDSENSE

#endif
