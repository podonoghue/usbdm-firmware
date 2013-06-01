/*! @file
    @brief This file contains hardware specific information and configuration for the "HCS08-OSBDM+E".
    
    USBDM - Universal BDM, JB16 Version \n
    Configuration for HCS08-OSBDM+E 30/12/2006, see
    @htmlonly <a href="OSBDMe.pdf">schematic</a> @endhtmlonly \n
    Supports HCS08, RS08 and Coldfire V1 targets \n

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
#define ProductDescription "USBDM - JB16,OSBDME, Version " VERSION_STR
//=================================================================================
// Debug pin - used to check timing and hardware sequences etc.
//
#if (DEBUG != 0)
#define DEBUG_PIN_DDR DDRE_DDRE0
#define DEBUG_PIN     PTE_PTE0
#endif

#define ICP_PIN            (1<<0)  // ICP Test input
#define ENABLE_ICP_PIN()   (POCR_PAP = 1, DDRA &= ~ICP_PIN)
#define DISABLE_ICP_PIN()  (POCR_PAP = 0)
#define TEST_ICP_PIN()     (~PTA & ICP_PIN)

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_BDM|CAP_FLASH)
#define TARGET_CAPABILITY (CAP_HCS08|CAP_RS08|CAP_CFV1)

//===========================================================================================
// Type of BDM interface chips are supported

#define DRIVER  LVC45  // Choose driver IC being used

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
#define BDM_IN            PTA_PTA6
#define BDM_IN_BIT        (6)  // Bit number!
#define BDM_IN_MASK       (1<<BDM_IN_BIT)

// Primary BDM data direction pin - only disabled during Tx routines
#define BDM_DIR_Rx        PTA_PTA4
#define BDM_DIR_Rx_BIT    (4)  // Bit number!
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
#define BDM_DIR           PTC_PTC0
#define BDM_DIR_BIT       (0) // Bit #
#define BDM_DIR_MASK      (1<<BDM_DIR_BIT)

#define BDM_DIR_DDR       DDRC_DDRC0

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
// NOTE: RESET sensing & Control not supported
//
#if (HW_CAPABILITY&CAP_RST_IO)

#error "RESET detection & control not supported on this hardware"

#else // !(CAPABILITY&CAP_RESET)

#define RESET_LOW()    ;
#define RESET_3STATE()    ;
#define RESET_IS_HIGH       (RESET_IN!=0)

#define CONFIGURE_RESET_SENSE()   ;              // Capture RESET falling edges
#define CLEAR_RESET_SENSE_FLAG()  ;          // Clear RESET (KBD)Event
#define ENABLE_RESET_SENSE_INT()  ;   // Enable RESET (KBD) Interrupts
#define DISABLE_RESET_SENSE_INT() ;  // Disable RESET (KBD) Interrupts
#endif // CAP_RST_IO

//=================================================================================
// LED Port bit masks
// JB16 has built in current limits on PTD.2-5 so no resistors are needed.
// A resistor is needed if PTD.0-1 is used (original TBDML).
//
#define GREEN_LED_MASK  (PTD_PTD0_MASK)
#define RED_LED_MASK    (PTD_PTD1_MASK)
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
// Why are there two signals? Only one is needed surely?
#define VPP_12V         (PTE_PTE2)
#define VPP_12V_DDR     (DDRE_DDRE2)
#define VPP_6V7         (PTE_PTE0)
#define VPP_6V7_DDR     (DDRE_DDRE0)

// Drive transistor for VPP on/off
#define VPP_EN       (PTD_PTD4)
#define VPP_EN_DDR   (DDRD_DDRD4)

// Order of statements within the following macros may be important 
#define VPP_SET_6V7()  (VPP_6V7=0, VPP_12V=0, VPP_6V7_DDR=1, VPP_12V_DDR=0)
#define VPP_SET_12V()  (VPP_6V7=0, VPP_12V=0, VPP_12V_DDR=1, VPP_6V7_DDR=0) 

#define FLASH12V_ON() VPP_SET_12V();
#define FLASH12V_OFF() ;               // Can't turn off Flash inverter

#define VPP_ON()        (VPP_EN=0, VPP_EN_DDR=1) // VPP Control pin is output, VPP on 
#define VPP_OFF()       (VPP_EN=1, VPP_EN_DDR=1) // VPP Control pin is output, VPP off

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

#error "Target Vdd control not supported on this hardware"

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

#define IC_BDM_TIMING_CHANNEL IC_T1CH01

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

#endif // CAP_VDDSENSE

#endif
