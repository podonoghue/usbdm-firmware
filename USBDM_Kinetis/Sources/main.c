#include <stdio.h>

#include "derivative.h" /* include peripheral declarations */
#include "Configure.h"
#include "USB.h"
#include "Clock.h"
#include "CmdProcessing.h"
#include "BDMCommon.h"

void initPorts(void) {
   // Enable all port clocks
   SIM_SCGC5 |=   SIM_SCGC5_PORTA_MASK
                | SIM_SCGC5_PORTB_MASK
                | SIM_SCGC5_PORTC_MASK
                | SIM_SCGC5_PORTD_MASK
                | SIM_SCGC5_PORTE_MASK;
   ledInit();
}

void delay(void) {
   int i;
   for(i=0; i<800000; i++) {
      asm("nop");
   }
}

//#define DEBUG_SPEED
#ifdef DEBUG_SPEED
#define LED_NUM           3
#define LED_REG           C
#define LED_MASK          (1<<LED_NUM)
#define LED_PCR           PCR(LED_REG,LED_NUM)
#define LED_PDOR          PDOR(LED_REG)
#define LED_PSOR          PSOR(LED_REG)  // Data set
#define LED_PCOR          PCOR(LED_REG)  // Data clear
#define LED_PTOR          PTOR(LED_REG)  // Data toggle
#define LED_PDIR          PDIR(LED_REG)  // Data input
#define LED_PDDR          PDDR(LED_REG)  // Data direction

//! Enable red LED
__inline
static void ledEnable(void) {
   LED_PDDR |= LED_MASK;
   LED_PCR   = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
}
//! Turn on red LED
__inline
static void ledOn(void) {
   LED_PSOR = LED_MASK;
}
//! Turn off red LED
__inline
static void ledOff(void) {
   LED_PCOR = LED_MASK;
}
//! Toggle red LED
__inline
static void ledToggle(void) {
   LED_PTOR = LED_MASK;
}

static void ledToggle2(void) {
   // LED_PTOR = LED_MASK;
   asm("movw r3,#0xf080");
   asm("movt r3,#0x400f");
   asm("mov r2,#0x8");
   asm("str r2,[r3,#12]");
   asm("cpy sp,r7");
}
#endif

void init() {
   initPorts();

#ifdef DEBUG_SPEED
   initUSB();
   initTimers();
   ledEnable();
   for(;;) {
      WAIT_WITH_TIMEOUT_US(100,0);
      ledToggle2();
   }
#endif
   
#if 0
   RESET_OUT_INIT();
   initUSB();
   //   initTimers();
   greenLedEnable();
   for(;;) { 
      greenLedToggle();
      RESET_LOW();
      RESET_3STATE();
      delay();
   }
#else

   initUSB();
   initTimers();
   bdm_interfaceOff();

#ifdef VDD_ON_INITIALLY
   // For compatibility with original board s/w
   // The board is powered when initially plugged in
#if (VDD_ON_INITIALLY == 3)
   bdm_option.targetVdd = BDM_TARGET_VDD_3V3;
#elif (VDD_ON_INITIALLY == 5)
   bdm_option.targetVdd = BDM_TARGET_VDD_5;
#else
   bdm_option.targetVdd = BDM_TARGET_VDD_OFF;
#endif
   bdm_setTargetVdd();
   RESET_LOW();
   WAIT_MS(100);
   RESET_3STATE();
#endif
}

int main(void) {
   initClock();
   init();
   commandLoop();

   //   SWD_OUT_ENABLE();
   for(;;) {	   
      //	  SWD_OUT_HIGH();
      //	  WAIT_WITH_TIMEOUT_MS(100,0);
      //	  WAIT_WITH_TIMEOUT_US(100,0);
      //	  WAIT_US(100);
      //	  WAIT_MS(10);
      //	  millisecondTimerWait(10);
      //	  fastTimerWait(TIMER_MICROSECOND(1000));
      //	  SWD_OUT_LOW();
      //	  WAIT_WITH_TIMEOUT_MS(100,0);
      //	  WAIT_WITH_TIMEOUT_US(100,0);
      //	  WAIT_US(100);
      //	  WAIT_MS(10);
      //	  millisecondTimerWait(10);
      //	  fastTimerWait(TIMER_MICROSECOND(1000));
   }
#endif
   //   return 0;
}
