/*! @file
    @brief This file contains hardware specific information and configuration for the "Combined TBDML/OSBDM + extensions".
    
    USBSPYDER - 
    
    @htmlonly <a href=" ">schematic</a> @endhtmlonly \n
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
#define ProductDescription "USBDM - JB16, USBSPYDER - broken!, Version " VERSION_STR
//=================================================================================
// Debug pin - used to check timing and hardware sequences etc.
//
#if (DEBUG != 0)
#define DEBUG_PIN_DDR DDRA_DDRA0
#define DEBUG_PIN     PTA_PTA0
#endif

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY (CAP_RST_IO|CAP_FLASH|CAP_BDM|CAP_VDDCONTROL)
#define TARGET_CAPABILITY (CAP_HCS12|CAP_HCS08|CAP_RS08|CAP_CFV1)

//===========================================================================================
// Type of BDM interface chips are supported

#define DRIVER  FETDRV  // Choose driver IC being used

#ifndef DRIVER
#error "Please define DRIVER in Configure.h"
#define DRIVER LVC125
#endif

#if ((DRIVER!=LVC125) && (DRIVER!=LVC45) && (DRIVER!=FETDRV))
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
#define DATA_PORT         PTE
#define DATA_PORT_DDR     DDRE

// BDM data out pin - hard coded in Rx/Tx routines
#define BDM_OUT           PTE_PTE1
#define BDM_OUT_BIT       (1)  // Bit number!
#define BDM_OUT_MASK      (1<<BDM_OUT_BIT)

// BDM data in pin
#define BDM_IN            PTE_PTE1
#define BDM_IN_BIT        (1)  // Bit number!
#define BDM_IN_MASK       (1<<BDM_IN_BIT)

// Primary BDM data direction pin - only disabled during Tx routines
#define BDM_DIR_Rx        PTE_PTE0 // Dummy - 
#define BDM_DIR_Rx_BIT    (0)  // Bit number! - Dummy
#define BDM_DIR_Rx_MASK   (1<<BDM_DIR_Rx_BIT)

// Polarity of BDM buffer enable/direction varies with driver IC
#if (DRIVER == LVC125)
#define BDM_DIR_Rx_RD     0
#define BDM_DIR_Rx_WR     0

#elif (DRIVER == LVC45)
#define BDM_DIR_Rx_RD     0
#define BDM_DIR_Rx_WR     0

#elif (DRIVER == FETDRV)
#define BDM_DIR_Rx_RD     0
#define BDM_DIR_Rx_WR     0
#endif

//=================================================================================
// Direction Port bit masks
//
#define DIR_PORT          PTD
#define DIR_PORT_DDR      DDRD

// Secondary BDM data direction pin - only enabled during Tx routines
#define BDM_DIR           PTC_PTC1
#define BDM_DIR_BIT       (1) // Bit #
#define BDM_DIR_MASK      (1<<BDM_DIR_BIT)

#define BDM_DIR_DDR       DDRC_DDRC1

// Polarity of BDM buffer enable/direction varies with driver
// Following MACROs only used by Tx related routines
#if (DRIVER == FETDRV)
#define BDM_DIR_RD        0
#define BDM_DIR_WR        0
#define BDM_ENABLE_TX()   
#define BDM_3STATE_TX()   
#define BDM_ENABLE_ASM_TX NOP
#define BDM_3STATE_ASM_TX NOP
#endif

#endif // CAP_BDM
//=================================================================================
// RESET control & sensing
//
#if (HW_CAPABILITY&CAP_RST_IO)

// RESET output pin
#define RESET_OUT         PTD_PTD0
#define RESET_OUT_DDR     DDRD_DDRD0

// RESET input pin
#define RESET_IN          PTD_PTD0
#define RESET_IS_HIGH       (RESET_IN!=0)
#define RESET_IS_LOW        (RESET_IN==0)

// RESET interrupt sensing not available
#define KBIER_RESET_MASK  0

#define RESET_LOW()       (RESET_OUT=0,RESET_OUT_DDR=1) // RESET IN/OUT driven low
#define RESET_3STATE()    (            RESET_OUT_DDR=0) // RESET IN/OUT is now in
#define CONFIGURE_RESET_SENSE()                  // Capture RESET falling edges
#define CLEAR_RESET_SENSE_FLAG()             // Clear RESET (KBD)Event
#define ENABLE_RESET_SENSE_INT()     // Enable RESET (KBD) Interrupts
#define DISABLE_RESET_SENSE_INT()   // Disable RESET (KBD) Interrupts
#endif // CAP_RST_IO

//=================================================================================
// LED Port bit masks
// JB16 has built in current limits on PTD.2-5 so no resistors are needed.
// A resistor is needed if PTD.0-1 is used (original TBDML hardware or JB16JDWE).
//
#define GREEN_LED_MASK  0
#define RED_LED_MASK    0
#define LED_PORT_DATA   
#define LED_PORT_DDR   

/* LEDs off, some LED pins are outputs with Io limit */  
#define LED_INIT()         ;
#define GREEN_LED_ON()     ;
#define GREEN_LED_OFF()  ;
#define GREEN_LED_TOGGLE()		;
#define RED_LED_ON()      ;
#define RED_LED_OFF()     ;
#define RED_LED_TOGGLE()   ;

//=================================================================================
// Flash programming control
//
#if (HW_CAPABILITY&CAP_FLASH)

// Flash 12V charge pump control
#define FLASH12_ENn     
#define FLASH12V_DDR     

// Drive transistor for VPP on/off
#define VPP_EN           (PTA_PTA2)
#define VPP_EN_DDR       (DDRA_DDRA2)

// Order of statements within the following macros may be important 
#define FLASH12V_ON()   
#define FLASH12V_OFF()  

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
#define VDD3_EN          (PTA_PTA3)
#define VDD3_EN_DDR      (DDRA_DDRA3)
#define VDD5_ENn       
#define VDD5_EN_DDR     

#define VDD_OFF()        (VDD3_EN=0, VDD3_EN_DDR=1) // Vdd Off
#define VDD3_ON()        (VDD3_EN=1, VDD3_EN_DDR=1) // Vdd = 3.3V
#define VDD5_ON()        (VDD3_EN=0, VDD3_EN_DDR=1) // Vdd Off

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
