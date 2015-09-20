/*
 *  Vectors-mke.c
 *
 *  Generic vectors and security for Kinetis MKExx
 *
 *  Created on: 07/12/2012
 *      Author: podonoghue
 */
#include <stdint.h>
#include <string.h>
#include "derivative.h"

#define DEVICE_SUBFAMILY_CortexM0

/*
 * Security information
 */
typedef struct {
   uint8_t  backdoorKey[8];
   uint32_t reseved;
   uint8_t  eeprot;
   uint8_t  fprot;
   uint8_t  fsec;
   uint8_t  fopt;
} SecurityInfo;

__attribute__ ((section(".security_information")))
const SecurityInfo securityInfo = {
    /* backdoor */ {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
    /* reseved  */ 0xFFFFFFFF,
    /* eeprot   */ 0xFF,
    /* fprot    */ 0xFF,
    /* fsec     */ 0xFE,
    /* fopt     */ 0xFF,
};

/*
 * Vector table related
 */
typedef void( *const intfunc )( void );

#define WEAK_DEFAULT_HANDLER __attribute__ ((__weak__, alias("Default_Handler")))

#ifndef SCB_ICSR
#define SCB_ICSR (*(volatile uint32_t*)(0xE000ED04))
#endif

/**
 * Default handler for interrupts
 *
 * Most of the vector table is initialised to point at this handler.
 *
 * If you end up here it probably means:
 *   - You have accidently enabled an interrupt source in a peripheral
 *   - Enabled the wrong interrupt source
 *   - Failed to install or create a handler for an interrupt you intended using e.g. mis-spelled the name.
 *     Compare your handler (C function) name to that used in the vector table.
 *
 * You can check 'vectorNum' below to determine the interrupt source.  Look this up in the vector table below.
 */
__attribute__((__interrupt__))
void Default_Handler(void) {

   __attribute__((unused))
   volatile uint32_t vectorNum = (SCB_ICSR&SCB_ICSR_VECTACTIVE_Msk)>>SCB_ICSR_VECTACTIVE_Pos;

   while (1) {
      __BKPT(0);
   }
}

typedef struct {
   unsigned int r0;
   unsigned int r1;
   unsigned int r2;
   unsigned int r3;
   unsigned int r12;
   void       (*lr)();
   void       (*pc)();
   unsigned int psr;
} ExceptionFrame;

/*  Low-level exception handler
 *
 *  Interface from asm to C.
 *  Passes address of exception handler to C-level handler
 *
 *  See http://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html
 */
__attribute__((__naked__, __weak__, __interrupt__))
void HardFault_Handler(void) {
   /*
    * Determines the active stack pointer and loads it into r0
    * This is used as the 1st argument to _HardFault_Handler(volatile ExceptionFrame *exceptionFrame)
    * and allows access to the saved processor state.
    * Other registers are unchanged and available in the usual register view
    */
   __asm__ volatile (
          "       mov r0,lr                                     \n"
          "       mov r1,#4                                     \n"
          "       and r0,r1                                     \n"
          "       bne skip1                                     \n"
          "       mrs r0,msp                                    \n"
          "       b   skip2                                     \n"
          "skip1:                                               \n"
          "       mrs r0,psp                                    \n"
          "skip2:                                               \n"
          "       nop                                           \n"
          "       ldr r2, handler_addr_const                    \n"
          "       bx r2                                         \n"
          "       handler_addr_const: .word _HardFault_Handler  \n"
      );
}

/******************************************************************************/
/* Hard fault handler in C with stack frame location as input parameter
 *
 * Assumed exception frame without floating-point storage
 *
 * @param exceptionFrame address of exception frame
 *
 * If you end up here you have probably done one of the following:
 *   - Accessed illegal/unimplemented memory e.g. gone off the end of an array
 *   - Accessed a disabled peripheral - Check you have enabled the clock
 *   - Accessed unaligned memory - unlikely I guess
 *
 */
__attribute__((__naked__))
void _HardFault_Handler(volatile ExceptionFrame *exceptionFrame __attribute__((__unused__))) {
   while (1) {
      // Stop here for debugger
      __BKPT(0);
   }
}

void __HardReset(void) __attribute__((__interrupt__));
extern uint32_t __StackTop;

/*
 * Each vector is assigned an unique name.  This is then 'weakly' assigned to the
 * default handler.
 * To install a handler, create a function with the name shown and it will override
 * the weak default.
 */
void NMI_Handler(void)                        WEAK_DEFAULT_HANDLER;
void MemManage_Handler(void)                  WEAK_DEFAULT_HANDLER;
void BusFault_Handler(void)                   WEAK_DEFAULT_HANDLER;
void UsageFault_Handler(void)                 WEAK_DEFAULT_HANDLER;
void SVC_Handler(void)                        WEAK_DEFAULT_HANDLER;
void DebugMon_Handler(void)                   WEAK_DEFAULT_HANDLER;
void PendSV_Handler(void)                     WEAK_DEFAULT_HANDLER;
void SysTick_Handler(void)                    WEAK_DEFAULT_HANDLER;
void FTMRH_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PMC_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void IRQ_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void I2C0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void SPI0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void SPI1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void UART0_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void UART1_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void UART2_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void ADC0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void ACMP0_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void FTM0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void FTM1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void FTM2_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void RTC_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void ACMP1_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PIT_Ch0_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void PIT_Ch1_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void KBI0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void KBI1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void ICS_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void WDOG_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;

typedef struct {
   uint32_t *initialSP;
   intfunc  handlers[];
} VectorTable;

__attribute__ ((section(".interrupt_vectors")))
VectorTable const __vector_table = {
                                     /*  Exc# Irq# */
   &__StackTop,                      /*    0   -16  Initial stack pointer                                                            */
   {
      __HardReset,                   /*    1   -15  Reset Handler                                                                    */
      NMI_Handler,                   /*    2,  -14  Non maskable Interrupt, cannot be stopped or preempted                           */
      HardFault_Handler,             /*    3,  -13  Hard Fault, all classes of Fault                                                 */
      MemManage_Handler,             /*    4,  -12  Memory Management, MPU mismatch, including Access Violation and No Match         */
      BusFault_Handler,              /*    5,  -11  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault   */
      UsageFault_Handler,            /*    6,  -10  Usage Fault, i.e. Undef Instruction, Illegal State Transition                    */
      0,                             /*    7,   -9                                                                                   */
      0,                             /*    8,   -8                                                                                   */
      0,                             /*    9,   -7                                                                                   */
      0,                             /*   10,   -6                                                                                   */
      SVC_Handler,                   /*   11,   -5  System Service Call via SVC instruction                                          */
      DebugMon_Handler,              /*   12,   -4  Debug Monitor                                                                    */
      0,                             /*   13,   -3                                                                                   */
      PendSV_Handler,                /*   14,   -2  Pendable request for system service                                              */
      SysTick_Handler,               /*   15,   -1  System Tick Timer                                                                */

                                     /* External Interrupts */
      Default_Handler,               /*   16,    0                                                                                   */
      Default_Handler,               /*   17,    1                                                                                   */
      Default_Handler,               /*   18,    2                                                                                   */
      Default_Handler,               /*   19,    3                                                                                   */
      Default_Handler,               /*   20,    4                                                                                   */
      FTMRH_IRQHandler,              /*   21,    5  FTMRH Command complete or error                                                  */
      PMC_IRQHandler,                /*   22,    6  PMC Low-voltage detect, low-voltage warning                                      */
      IRQ_IRQHandler,                /*   23,    7  External Interrupt                                                               */
      I2C0_IRQHandler,               /*   24,    8  I2C Interface 0                                                                  */
      Default_Handler,               /*   25,    9                                                                                   */
      SPI0_IRQHandler,               /*   26,   10  Serial Peripheral Interface 0                                                    */
      SPI1_IRQHandler,               /*   27,   11  Serial Peripheral Interface 1                                                    */
      UART0_IRQHandler,              /*   28,   12  UART0 Status and error                                                           */
      UART1_IRQHandler,              /*   29,   13  UART1 Status and error                                                           */
      UART2_IRQHandler,              /*   30,   14  UART2 Status and error                                                           */
      ADC0_IRQHandler,               /*   31,   15  Analogue to Digital Converter 0                                                  */
      ACMP0_IRQHandler,              /*   32,   16  Analogue comparator 0                                                            */
      FTM0_IRQHandler,               /*   33,   17  Flexible Timer Module 0                                                          */
      FTM1_IRQHandler,               /*   34,   18  Flexible Timer Module 1                                                          */
      FTM2_IRQHandler,               /*   35,   19  Flexible Timer Module 2                                                          */
      RTC_IRQHandler,                /*   36,   20  Real Time Clock overflow                                                         */
      ACMP1_IRQHandler,              /*   37,   21  Analogue comparator 0                                                            */
      PIT_Ch0_IRQHandler,            /*   38,   22  Programmable Interrupt Timer Channel 0                                           */
      PIT_Ch1_IRQHandler,            /*   39,   23  Programmable Interrupt Timer Channel 1                                           */
      KBI0_IRQHandler,               /*   40,   24  Keyboard Interrupt 0                                                             */
      KBI1_IRQHandler,               /*   41,   25  Keyboard Interrupt 1                                                             */
      Default_Handler,               /*   42,   26                                                                                   */
      ICS_IRQHandler,                /*   43,   27  ICS                                                                              */
      WDOG_IRQHandler,               /*   44,   28  Watch dog                                                                        */
   }
};



