/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_MK20D5 - USBDM-SWD for MK20 chip
             
    Supports Kinetis targets \n

    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

#define __LITTLE_ENDIAN__

// Used to create port register names
//--------------------------------------------------------
#define CONCAT2_(x,y) x ## y
#define CONCAT3_(x,y,z) x ## y ## z
#define CONCAT4_(w,x,y,z) w ## x ## y ## z

#define PCR(reg,num)   CONCAT4_(PORT,reg,_PCR,num)
#define PDOR(reg)      CONCAT3_(GPIO,reg,_PDOR)
#define PSOR(reg)      CONCAT3_(GPIO,reg,_PSOR)
#define PCOR(reg)      CONCAT3_(GPIO,reg,_PCOR)
#define PTOR(reg)      CONCAT3_(GPIO,reg,_PTOR)
#define PDIR(reg)      CONCAT3_(GPIO,reg,_PDIR)
#define PDDR(reg)      CONCAT3_(GPIO,reg,_PDDR)

//==========================================================================================
// USB Serial Number
#define SERIAL_NO "USBDM-MK20D5-SWD-0001"
#define ProductDescription "USBDM ARM-SWD for MK20D5"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
// CAP_CDC
//#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_JTAG_HW|CAP_BDM  |CAP_FLASH|        CAP_RST_IO|CAP_CFVx)
//#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_HCS12 |CAP_HCS08|CAP_RS08 |CAP_CFV1|CAP_RST   |CAP_CFVx|CAP_JTAG|CAP_DSC|CAP_ARM_JTAG|CAP_PST)
#define HW_CAPABILITY       (CAP_RST_IO|CAP_SWD_HW|CAP_CDC)
#define TARGET_CAPABILITY   (CAP_RST   |CAP_ARM_SWD|CAP_CDC)

#ifndef PLATFORM
#define PLATFORM USBDM   //! Choose BDM emulation
#endif

#define CPU  MK20D5

// Define for automatic WINUSB Driver loading 
//#define MS_COMPATIBLE_ID_FEATURE (1)

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

//=================================================================================
// Port Pin assignments
// Please note: some pin assignments cannot be changed freely
// RX AND TX ROUTINES ARE DEPENDENT ON THIS SPECIFIC ASSIGNMENTS
//

//=================================================================================
// Serial Port Bit Masks
//
#if (HW_CAPABILITY&CAP_CDC)
#define CTS_IN             
#define DTR_OUT            
#define DTR_OUT_DDR        

#define CTS_IS_HIGH()      (1)
#define DTR_LOW()          ;
#define DTR_HIGH()         ;
#define DTR_ACTIVE()       ;

// UART Tx Pin = B17 ALT3
#define TX_ALT_FN             (3)
#define TX_OUT_EN_NUM         17
#define TX_OUT_EN_REG         B
#define TX_OUT_EN_MASK        (1<<TX_OUT_EN_NUM)
#define TX_OUT_EN_PCR         PCR(TX_OUT_EN_REG,TX_OUT_EN_NUM)
#define TX_OUT_EN_PDOR        PDOR(TX_OUT_EN_REG)
#define TX_OUT_EN_PSOR        PSOR(TX_OUT_EN_REG)  // Data set 
#define TX_OUT_EN_PCOR        PCOR(TX_OUT_EN_REG)  // Data clear
#define TX_OUT_EN_PTOR        PTOR(TX_OUT_EN_REG)  // Data toggle
#define TX_OUT_EN_PDIR        PDIR(TX_OUT_EN_REG)  // Data input
#define TX_OUT_EN_PDDR        PDDR(TX_OUT_EN_REG)  // Data direction

// UART Rx Pin = B16 ALT3
#define RX_ALT_FN             (3)
#define RX_OUT_EN_NUM         16
#define RX_OUT_EN_REG         B
#define RX_OUT_EN_MASK        (1<<RX_OUT_EN_NUM)
#define RX_OUT_EN_PCR         PCR(RX_OUT_EN_REG,RX_OUT_EN_NUM)
#define RX_OUT_EN_PDOR        PDOR(RX_OUT_EN_REG)
#define RX_OUT_EN_PSOR        PSOR(RX_OUT_EN_REG)  // Data set 
#define RX_OUT_EN_PCOR        PCOR(RX_OUT_EN_REG)  // Data clear
#define RX_OUT_EN_PTOR        PTOR(RX_OUT_EN_REG)  // Data toggle
#define RX_OUT_EN_PDIR        PDIR(RX_OUT_EN_REG)  // Data input
#define RX_OUT_EN_PDDR        PDDR(RX_OUT_EN_REG)  // Data direction

#define UART_NUM              0

#endif

//=================================================================================
// ARM SWD Mode Port Bit masks
//
#if (HW_CAPABILITY&CAP_SWD_HW)

#define SWD_DISABLE_ALT_FN 0
#define SWD_GPIO_ALT_FN 1
#define SWD_SPI_ALT_FN 2

#define SWCLK_DISABLE_ALT_FN 0
#define SWCLK_GPIO_ALT_FN 1
#define SWCLK_SPI_ALT_FN 2

// Note:
//    The SWD_O is connected to SPI_SOUT pin
//    The SWD_DRV 3-state buffer enable is connected to SPI_PCS0 pin

// SWD data out pin (SPI_O)
#define SWD_OUT_NUM            6
#define SWD_OUT_REG            C
#define SWD_OUT_MASK           (1<<SWD_OUT_NUM)
#define SWD_OUT_PCR            PCR(SWD_OUT_REG,SWD_OUT_NUM)
#define SWD_OUT_PDOR           PDOR(SWD_OUT_REG)
#define SWD_OUT_PSOR           PSOR(SWD_OUT_REG)  // Data set 
#define SWD_OUT_PCOR           PCOR(SWD_OUT_REG)  // Data clear
#define SWD_OUT_PTOR           PTOR(SWD_OUT_REG)  // Data toggle
#define SWD_OUT_PDIR           PDIR(SWD_OUT_REG)  // Data input
#define SWD_OUT_PDDR           PDDR(SWD_OUT_REG)  // Data direction

// SWD data out enable pin (SPI_SS, 3-state buffer control)
#define SWD_OUT_EN_NUM         4 //0
#define SWD_OUT_EN_REG         C //B
#define SWD_OUT_EN_MASK        (1<<SWD_OUT_EN_NUM)
#define SWD_OUT_EN_PCR         PCR(SWD_OUT_EN_REG,SWD_OUT_EN_NUM)
#define SWD_OUT_EN_PDOR        PDOR(SWD_OUT_EN_REG)
#define SWD_OUT_EN_PSOR        PSOR(SWD_OUT_EN_REG)  // Data set 
#define SWD_OUT_EN_PCOR        PCOR(SWD_OUT_EN_REG)  // Data clear
#define SWD_OUT_EN_PTOR        PTOR(SWD_OUT_EN_REG)  // Data toggle
#define SWD_OUT_EN_PDIR        PDIR(SWD_OUT_EN_REG)  // Data input
#define SWD_OUT_EN_PDDR        PDDR(SWD_OUT_EN_REG)  // Data direction
//
// Following configure pin & driver
//
//! SWCLK 3-state (assumes pin controlled)
#define SWD_3STATE()        ;
//! SWCLK enable (either high/low)
#define SWD_ENABLE()        ;
//! SWD_OUT Initialisation before first use (SWD_OUT=SPI, SWD_EN=SPI_CS)
#define SWD_OUT_INIT()      (SWD_OUT_PCR   =PORT_PCR_MUX(SWD_SPI_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK, \
                             SWD_OUT_EN_PCR=PORT_PCR_MUX(SWD_SPI_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK)
//! SWD_OUT Finalisation after last use
#define SWD_OUT_FINI()      (SWD_OUT_PCR   =PORT_PCR_MUX(SWD_DISABLE_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK, \
                             SWD_OUT_EN_PCR=PORT_PCR_MUX(SWD_DISABLE_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK)

//---------------------------------------------------------------------------------------

// SWD data in pin
// Note:
//    The SWD_I pin is connected to SPI_SIN
#define SWD_IN_NUM         7
#define SWD_IN_REG         C
#define SWD_IN_MASK        (1<<SWD_IN_NUM)
#define SWD_IN_PCR         PCR(SWD_IN_REG,SWD_IN_NUM)
#define SWD_IN_PDIR        PDIR(SWD_IN_REG)  // Data input
#define SWD_IN_PDDR        PDDR(SWD_IN_REG)  // Data direction

//! SWD_IN Initialisation before first use (SWD_IN=SPI)
#define SWD_IN_INIT()     (SWD_IN_PCR=PORT_PCR_MUX(SWD_SPI_ALT_FN)|PORT_PCR_PFE_MASK|PORT_PCR_PE_MASK)
//! SWD Finalisation after last use
#define SWD_IN_FINI()     (SWD_IN_PCR=PORT_PCR_MUX(SWD_DISABLE_ALT_FN)|PORT_PCR_PFE_MASK|PORT_PCR_PE_MASK)

//---------------------------------------------------------------------------------------
                            
// Note:
//    The SWDCLK_O pin is connected to SPI_SCK pin
//    The SWDCLK_DRV 3-state buffer enable is connected to SPI_PCS1 pin

// SWCLK out pin
#define SWCLK_OUT_NUM            5
#define SWCLK_OUT_REG            C
#define SWCLK_OUT_MASK           (1<<SWCLK_OUT_NUM)
#define SWCLK_OUT_PCR            PCR(SWCLK_OUT_REG,SWCLK_OUT_NUM)
#define SWCLK_OUT_PDOR           PDOR(SWCLK_OUT_REG)
#define SWCLK_OUT_PSOR           PSOR(SWCLK_OUT_REG)  // Data set 
#define SWCLK_OUT_PCOR           PCOR(SWCLK_OUT_REG)  // Data clear
#define SWCLK_OUT_PTOR           PTOR(SWCLK_OUT_REG)  // Data toggle
#define SWCLK_OUT_PDIR           PDIR(SWCLK_OUT_REG)  // Data input
#define SWCLK_OUT_PDDR           PDDR(SWCLK_OUT_REG)  // Data direction

// SWCLK output enable pin (3-state buffer control)
#define SWCLK_OUT_EN_NUM         3
#define SWCLK_OUT_EN_REG         C
#define SWCLK_OUT_EN_MASK        (1<<SWCLK_OUT_EN_NUM)
#define SWCLK_OUT_EN_PCR         PCR(SWCLK_OUT_EN_REG,SWCLK_OUT_EN_NUM)
#define SWCLK_OUT_EN_PDOR        PDOR(SWCLK_OUT_EN_REG)
#define SWCLK_OUT_EN_PSOR        PSOR(SWCLK_OUT_EN_REG)  // Data set 
#define SWCLK_OUT_EN_PCOR        PCOR(SWCLK_OUT_EN_REG)  // Data clear
#define SWCLK_OUT_EN_PTOR        PTOR(SWCLK_OUT_EN_REG)  // Data toggle
#define SWCLK_OUT_EN_PDIR        PDIR(SWCLK_OUT_EN_REG)  // Data input
#define SWCLK_OUT_EN_PDDR        PDDR(SWCLK_OUT_EN_REG)  // Data direction
//
// Following configure pin & driver
//
//! SWCLK 3-state (assumes pin controlled)
#define SWCLK_3STATE()     (SWCLK_OUT_EN_PSOR=SWCLK_OUT_EN_MASK)
//! SWCLK enable (either high/low)
#define SWCLK_ENABLE()     (SWCLK_OUT_EN_PCOR=SWCLK_OUT_EN_MASK)
#if 0
//! SWCLK Initialisation before first use
#define SWCLK_OUT_INIT()   (SWCLK_ENABLE(),                                                                                           \
                            SWCLK_OUT_EN_PDDR |= SWCLK_OUT_EN_MASK,                                                                   \
                            SWCLK_OUT_PCR      = PORT_PCR_MUX(SWD_SPI_ALT_FN) |PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK,   \
                            SWCLK_OUT_EN_PCR   = PORT_PCR_MUX(SWD_GPIO_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK)
//! SWCLK Finalisation after last use
#define SWCLK_OUT_FINI()   (SWCLK_OUT_PCR    = PORT_PCR_MUX(SWD_DISABLE_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK, \
                            SWCLK_OUT_EN_PCR = PORT_PCR_MUX(SWD_DISABLE_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK)
#else
//
// Following configure pin & driver - Requires small value PUP on SWCLK
//
//! SWCLK_OUT Initialisation before first use
#define SWCLK_OUT_INIT()    (SWCLK_OUT_PCR   =PORT_PCR_MUX(SWCLK_SPI_ALT_FN) |PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK, \
                               SWCLK_OUT_EN_PCR=PORT_PCR_MUX(SWCLK_SPI_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK)
//! SWCLK_OUT Finalisation after last use
#define SWCLK_OUT_FINI()    (SWCLK_OUT_PCR   =PORT_PCR_MUX(SWCLK_DISABLE_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK, \
                               SWCLK_OUT_EN_PCR=PORT_PCR_MUX(SWCLK_DISABLE_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK)
#endif
//---------------------------------------------------------------------------------------

// SPI used                            
#define SPI_NUM 0 // Modify to change which SPI is used

#if (SPI_NUM==0)
   #define SPI_CLK_ENABLE()   (SIM_SCGC6 |= SIM_SCGC6_SPI0_MASK)
#else
   #error "Unknown SPI number"
#endif

#endif // CAP_SWD

//=================================================================================
// RESET control & sensing
//
#if (HW_CAPABILITY&CAP_RST_IO)

// RESET out pin
#define RESET_OUT_NUM         1
#define RESET_OUT_REG         C
#define RESET_OUT_MASK        (1<<RESET_OUT_NUM)
#define RESET_OUT_PCR         PCR(RESET_OUT_REG,RESET_OUT_NUM)
#define RESET_OUT_PDOR        PDOR(RESET_OUT_REG)
#define RESET_OUT_PSOR        PSOR(RESET_OUT_REG)  // Data set 
#define RESET_OUT_PCOR        PCOR(RESET_OUT_REG)  // Data clear
#define RESET_OUT_PTOR        PTOR(RESET_OUT_REG)  // Data toggle
#define RESET_OUT_PDIR        PDIR(RESET_OUT_REG)  // Data input
#define RESET_OUT_PDDR        PDDR(RESET_OUT_REG)  // Data direction

// RESET output pin control
#define RESET_LOW()          (RESET_OUT_PDDR |=  RESET_OUT_MASK)
#define RESET_3STATE()       (RESET_OUT_PDDR &= ~RESET_OUT_MASK)
#define RESET_OUT_INIT()     (RESET_3STATE(), \
                              RESET_OUT_PCOR = RESET_OUT_MASK, \
                              RESET_OUT_PCR  = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK)
#define RESET_OUT_FINI()     RESET_3STATE() //(RESET_OUT_PCR=PORT_PCR_MUX(0)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK)

//---------------------------------------------------------------------------------------

// RESET in pin
#define RESET_IN_NUM         3
#define RESET_IN_REG         B
#define RESET_IN_MASK        (1<<RESET_IN_NUM)
#define RESET_IN_PCR         PCR(RESET_IN_REG,RESET_IN_NUM)
#define RESET_IN_PDIR        PDIR(RESET_IN_REG)  // Data input
#define RESET_IN_PDDR        PDDR(RESET_IN_REG)  // Data direction
#define RESET_IN_FINI()      (RESET_IN_PCR=PORT_PCR_MUX(0)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK)
#define RESET_IN_INIT()      (RESET_IN_PCR=PORT_PCR_MUX(1)|PORT_PCR_PFE_MASK|PORT_PCR_PE_MASK, \
                              RESET_IN_PDDR &= ~RESET_IN_MASK)

#define RESET_IN             (RESET_IN_PDIR&RESET_IN_MASK)

// RESET input pin status
#define RESET_IS_HIGH()      (RESET_IN!=0)
#define RESET_IS_LOW()       (RESET_IN==0)

#endif // CAP_RST_IO

//=================================================================================
// LED Port bit masks
//
#define GREEN_LED_NUM         0
#define GREEN_LED_REG         E
#define GREEN_LED_MASK        (1<<GREEN_LED_NUM)
#define GREEN_LED_PCR         PCR(GREEN_LED_REG,GREEN_LED_NUM)  
#define GREEN_LED_PDOR        PDOR(GREEN_LED_REG)
#define GREEN_LED_PSOR        PSOR(GREEN_LED_REG)  // Data set 
#define GREEN_LED_PCOR        PCOR(GREEN_LED_REG)  // Data clear
#define GREEN_LED_PTOR        PTOR(GREEN_LED_REG)  // Data toggle
#define GREEN_LED_PDIR        PDIR(GREEN_LED_REG)  // Data input
#define GREEN_LED_PDDR        PDDR(GREEN_LED_REG)  // Data direction

//! Enable green LED
__inline 
static void greenLedEnable(void) {
   GREEN_LED_PDDR |= GREEN_LED_MASK;
   GREEN_LED_PCR   = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
}
//! Turn on green LED
__inline 
static void greenLedOn(void) {
   GREEN_LED_PSOR = GREEN_LED_MASK;
}
//! Turn off green LED
__inline 
static void greenLedOff(void) {
   GREEN_LED_PCOR = GREEN_LED_MASK;
}
//! Toggle green LED
__inline 
static void greenLedToggle(void) {
   GREEN_LED_PTOR = GREEN_LED_MASK;
}

#define RED_LED_NUM           1
#define RED_LED_REG           E
#define RED_LED_MASK          (1<<RED_LED_NUM)
#define RED_LED_PCR           PCR(RED_LED_REG,RED_LED_NUM)  
#define RED_LED_PDOR          PDOR(RED_LED_REG)
#define RED_LED_PSOR          PSOR(RED_LED_REG)  // Data set 
#define RED_LED_PCOR          PCOR(RED_LED_REG)  // Data clear
#define RED_LED_PTOR          PTOR(RED_LED_REG)  // Data toggle
#define RED_LED_PDIR          PDIR(RED_LED_REG)  // Data input
#define RED_LED_PDDR          PDDR(RED_LED_REG)  // Data direction

//! Enable red LED
__inline 
static void redLedEnable(void) {
   RED_LED_PDDR |= RED_LED_MASK;
   RED_LED_PCR   = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
}
//! Turn on red LED
__inline 
static void redLedOn(void) {
   RED_LED_PSOR = RED_LED_MASK;
}
//! Turn off red LED
__inline 
static void redLedOff(void) {
   RED_LED_PCOR = RED_LED_MASK;
}
//! Toggle red LED
__inline 
static void redLedToggle(void) {
   RED_LED_PTOR = RED_LED_MASK;
}
//! Initialise LEDs
//! Pins are outputs, off
//!
__inline 
static void ledInit(void) {
   greenLedOff();
   redLedOff();
   greenLedEnable();
   redLedEnable();
}

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
#define VDD3_EN          (PTBD_PTBD1)
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
//
//     TPM1-CH3 - RST_I pin, Reset detection (IC falling edge detection)
//     TPM1-CH0 - BKGD_I pin, SYNC measuring, ACKN detection (Input Capture)
//     TPM1-CH2 - Timeouts (Output compare, no pin used)
//
// Use TPM0 as Timer
#define TPMSC              TPM0_SC
#define TPMCNTH            TPM0_CNT
#define TPMSC_CLKSA_MASK   TPM_SC_CLKSA_MASK

// RESET Detection TPM1.Ch3 : Interrupt, Input capture, falling edge on pin 
#define RESET_TPMxCnSC_CHIE               TPM1C3SC_CH3IE      // Mask/enable RESET detection
#define RESET_TPMxCnSC_CHF                TPM1C3SC_CH3F       // Event Interrupt Flag
#define RESET_TPMxCnSC                    TPM1C3SC            // TPM Status & Configuration
#define RESET_TPMxCnSC_FALLING_EDGE_MASK  TPM1C3SC_ELS3B_MASK // Falling-edge input capture  

// SYNC Detection TPM1.Ch0 : Input capture
#define SYNC_TPMxCnSC_CHF                 TPM1C0SC_CH0F       // Event Flag
#define SYNC_TPMxCnSC                     TPM1C0SC            // TPM Channel Status & Configuration
#define SYNC_TPMxCnSC_RISING_EDGE_MASK    TPM1C0SC_ELS0A_MASK // TPMxCnSC value for rising edge
#define SYNC_TPMxCnSC_FALLING_EDGE_MASK   TPM1C0SC_ELS0B_MASK // TPMxCnSC value for falling edge
#define SYNC_TPMxCnVALUE                  TPM1C0V             // IC Event time

// ACKN uses same channel as SYNC
#define ACKN_TPMxCnSC_CHF                 SYNC_TPMxCnSC_CHF
#define ACKN_TPMxCnSC                     SYNC_TPMxCnSC
#define ACKN_TPMxCnSC_RISING_EDGE_MASK    SYNC_TPMxCnSC_RISING_EDGE_MASK
#define ACKN_TPMxCnSC_FALLING_EDGE_MASK   SYNC_TPMxCnSC_FALLING_EDGE_MASK
#define ACKN_SETUP_ASM                    BCLR 7,ACKN_TPMxCnSC

// Timeout TPM1.Ch2 : Output Compare (no pin)
#define TIMEOUT_TPMxCnSC_CHF              TPM0_C2SC_CH2F       // Event Flag
#define TIMEOUT_TPMxCnSC                  TPM1C2SC            // TPM Status & Configuration
#define TIMEOUT_TPMxCnSC_OC_MASK          TPM1C2SC_MS2A_MASK  // TIMEOUT_TPMxCnSC value for OC event
#define TIMEOUT_TPMxCnVALUE               TPM1C2V             // OC Event time

#if (HW_CAPABILITY&CAP_VDDSENSE)
// Target Vdd Present
#define VDD_SENSE                 (ACMPSC_ACO == 0)

#else // !CAP_VDDSENSE

#define VDD_SENSE                 (1) // Assume VDD present
#endif

//================================================================================
// RESET Detection - falling edge using KBI inputs
//
//     KBI     - RESET_IN pin, Reset detection (Keypress falling edge detection)
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
// Enable & Configure Vdd Change interrupts (Using Analogue comparator, rising or falling edges)
#define CONFIGURE_VDD_SENSE()     (ACMPSC = ACMPSC_ACME_MASK|ACMPSC_ACBGS_MASK|ACMPSC_ACMOD1_MASK|ACMPSC_ACMOD0_MASK)
// Enable Vdd Change interrupts
#define ENABLE_VDD_SENSE_INT()    (ACMPSC_ACIE = 1) 
// Clear Vdd Change Event
#define CLEAR_VDD_SENSE_FLAG()    (ACMPSC_ACF = 1)
// Disable Vdd Change interrupts
#define DISABLE_VDD_SENSE_INT()   (ACMPSC_ACIE = 0) 

#endif // _CONFIGURE_H_

