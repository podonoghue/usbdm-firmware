/*! @file

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

#define CPU  MK20D5

//=================================================================================
// Port Pin assignments
//

//=================================================================================
// LED Port bit masks
//
#define GREEN_LED_NUM         4
#define GREEN_LED_REG         D
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
//! Initialise LEDs
//! Pins are outputs, off
//!
__inline 
static void ledInit(void) {
   greenLedOff();
   greenLedEnable();
}

#endif // _CONFIGURE_H_

