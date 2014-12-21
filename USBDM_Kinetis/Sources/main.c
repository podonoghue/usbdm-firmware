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
      __asm__("nop");
   }
}

#define DEBUG_TEST_LOOP
#ifdef DEBUG_TEST_LOOP
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

// PTD5 - Test pin on external pin
#define TEST_NUM           5
#define TEST_REG           D
#define TEST_MASK          (1<<TEST_NUM)
#define TEST_PCR           PCR(TEST_REG,TEST_NUM)
#define TEST_PDOR          PDOR(TEST_REG)
#define TEST_PSOR          PSOR(TEST_REG)  // Data set
#define TEST_PCOR          PCOR(TEST_REG)  // Data clear
#define TEST_PTOR          PTOR(TEST_REG)  // Data toggle
#define TEST_PDIR          PDIR(TEST_REG)  // Data input
#define TEST_PDDR          PDDR(TEST_REG)  // Data direction

// PTB1 - Target reset switch/output
#define BOOT_NUM           1
#define BOOT_REG           B
#define BOOT_MASK          (1<<BOOT_NUM)
#define BOOT_PCR           PCR(BOOT_REG,BOOT_NUM)
#define BOOT_PDOR          PDOR(BOOT_REG)
#define BOOT_PSOR          PSOR(BOOT_REG)  // Data set
#define BOOT_PCOR          PCOR(BOOT_REG)  // Data clear
#define BOOT_PTOR          PTOR(BOOT_REG)  // Data toggle
#define BOOT_PDIR          PDIR(BOOT_REG)  // Data input
#define BOOT_PDDR          PDDR(BOOT_REG)  // Data direction

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


//! Enable red LED
__inline
static void testEnable(void) {
   TEST_PDDR |= TEST_MASK;
   TEST_PCR   = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
}
//! Turn on red LED
__inline
static void testOn(void) {
   TEST_PSOR = TEST_MASK;
}
//! Turn off red LED
__inline
static void testOff(void) {
   TEST_PCOR = TEST_MASK;
}
//! Toggle red LED
__inline
static void testToggle(void) {
   TEST_PTOR = TEST_MASK;
}

__inline
static void bootInputEnable(void) {
   BOOT_PDDR &= ~TEST_MASK;
   BOOT_PCR   = PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
}

__inline
static int bootInputActive(void) {
   return (BOOT_PDIR & BOOT_MASK) != 0;
}

void reboot() {
   for(;;) {
      WDOG_UNLOCK = 0;
   }
}

static void testMode() {
//   __enable_irq();
//   SystemInitLowLevel();
//   SystemInit();
   initPorts();
   initTimers();
   bdm_interfaceOff();
   initUSB();
   ledEnable();
   testEnable();
   bootInputEnable();
   for(;;) {
      WAIT_MS(100);
      ledToggle();
      testToggle();
      if (bootInputActive() == 0) {
         reboot();
      }
   }
}

static void testResetToggle() {
   RESET_OUT_INIT();
//   initUSB();
//   initTimers();
   greenLedEnable();
   for(;;) {
      greenLedToggle();
      RESET_LOW();
      RESET_3STATE();
      delay();
   }
}

static void testSWDToggle() {
   SWD_OUT_ENABLE();
   for(;;) {
      SWD_OUT_HIGH();
      WAIT_WITH_TIMEOUT_MS(100,0);
      WAIT_WITH_TIMEOUT_US(100,0);
      WAIT_US(100);
      WAIT_MS(10);
      millisecondTimerWait(10);
      fastTimerWait(TIMER_MICROSECOND(1000));
      SWD_OUT_LOW();
      WAIT_WITH_TIMEOUT_MS(100,0);
      WAIT_WITH_TIMEOUT_US(100,0);
      WAIT_US(100);
      WAIT_MS(10);
      millisecondTimerWait(10);
      fastTimerWait(TIMER_MICROSECOND(1000));
   }
}

#endif

void init() {
   initClock();
   initPorts();
   initTimers();
   bdm_interfaceOff();
   initUSB();

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
#ifdef DEBUG_TEST_LOOP
   testMode();
#endif
   init();
   commandLoop();
   
   return 0;
}
