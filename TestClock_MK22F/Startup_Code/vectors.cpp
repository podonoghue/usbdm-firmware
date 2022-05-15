/*
 *  @file vectors.cpp  (180.ARM_Peripherals/Startup_Code/vectors-cm4.cpp)
 *
 *  Generated from  vectors-cm4.cpp
 *
 *  Generic vectors Kinetis Cortex-M4 devices
 *
 *  Based on Keil Application Note 209
 *
 *  Created on: 22/05/2017
 *      Author: podonoghue
 */
#include <stdint.h>
#include <string.h>
#include "derivative.h"
#include "hardware.h"



/*
 * Vector table related
 */
typedef void( *const intfunc )( void );

#define WEAK_DEFAULT_HANDLER __attribute__ ((__nothrow__, __weak__, alias("Default_Handler")))
/**
 * Default handler for interrupts
 *
 * Most of the vector table is initialised to point at this handler.
 *
 * If you end up here it probably means:
 *   - Failed to enable the interrupt handler in the USBDM configuration (Configure.usbdmProject)
 *   - You have accidently enabled an interrupt source in a peripheral
 *   - Enabled the wrong interrupt source
 *   - Failed to install or create a handler for an interrupt you intended using e.g. mis-spelled the name.
 *     Compare your handler (C function) name to that used in the vector table.
 *
 * You can check 'vectorNum' below to determine the interrupt source.  Look this up in the vector table below.
 */
extern "C" {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
__attribute__((__interrupt__))
void Default_Handler(void) {

#ifdef SCB
   __attribute__((unused))
   volatile uint32_t vectorNum = (SCB->ICSR&SCB_ICSR_VECTACTIVE_Msk)>>SCB_ICSR_VECTACTIVE_Pos;
#endif

   while (1) {
      __asm__("bkpt");
   }
}
#pragma GCC diagnostic pop
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
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
     __asm__ volatile ( "  tst   lr, #4              \n");  // Check mode
     __asm__ volatile ( "  ite   eq                  \n");  // Get active SP in r0
     __asm__ volatile ( "  mrseq r0, msp             \n");
     __asm__ volatile ( "  mrsne r0, psp             \n");
     __asm__ volatile ( "  mov   r1, lr              \n");  // Get LR=EXC_RETURN in r1
     __asm__ volatile ( "  b     _HardFault_Handler  \n");  // Go to C handler
}
#pragma GCC diagnostic pop

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
extern "C" {
__attribute__((__naked__))
void _HardFault_Handler(
      volatile ExceptionFrame *exceptionFrame __attribute__((__unused__)),
      uint32_t execReturn                     __attribute__((__unused__)) ) {

#if defined(DEBUG_BUILD) && USE_CONSOLE
   using namespace USBDM;

   console.setPadding(Padding_LeadingZeroes);
   console.setWidth(8);
   console.writeln("\n[Hardfault]\n - Stack frame:\n");
   console.writeln("R0  = 0x", exceptionFrame->r0,  Radix_16);
   console.writeln("R1  = 0x", exceptionFrame->r1,  Radix_16);
   console.writeln("R2  = 0x", exceptionFrame->r2,  Radix_16);
   console.writeln("R3  = 0x", exceptionFrame->r3,  Radix_16);
   console.writeln("R12 = 0x", exceptionFrame->r12, Radix_16);
   console.writeln("LR  = 0x", (void*)(exceptionFrame->lr),  Radix_16);
   console.writeln("PC  = 0x", (void*)(exceptionFrame->pc),  Radix_16);
   console.writeln("PSR = 0x", exceptionFrame->psr, Radix_16);
   console.writeln("- FSR/FAR:");
   uint32_t cfsr = SCB->CFSR;
   console.writeln("CFSR = 0x", cfsr);
   console.writeln("HFSR = 0x", SCB->HFSR, Radix_16);
   console.writeln("DFSR = 0x", SCB->DFSR, Radix_16);
   console.writeln("AFSR = 0x", SCB->AFSR, Radix_16);
   if (cfsr & 0x0080) console.writeln("MMFAR = 0", SCB->MMFAR, Radix_16);
   if (cfsr & 0x8000) console.writeln("BFAR = 0x", SCB->BFAR,  Radix_16);
   console.writeln("- Misc");
   console.write("LR/EXC_RETURN= 0x", execReturn,  Radix_16);
#endif

   while (1) {
      // Stop here for debugger
      __asm__("bkpt");
   }
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
void Reset_Handler(void) __attribute__((__interrupt__));
#pragma GCC diagnostic pop

extern uint32_t __StackTop;
}

/*
 * Each vector is assigned an unique name.  This is then 'weakly' assigned to the
 * default handler.
 * To install a handler, create a C linkage function with the name shown and it will override
 * the weak default.
 */
 #ifdef __cplusplus
extern "C" {
#endif
// Reset handler must have C linkage
void Reset_Handler(void);
#ifdef __cplusplus
}
#endif
void NMI_Handler(void)                        WEAK_DEFAULT_HANDLER;
void MemManage_Handler(void)                  WEAK_DEFAULT_HANDLER;
void BusFault_Handler(void)                   WEAK_DEFAULT_HANDLER;
void UsageFault_Handler(void)                 WEAK_DEFAULT_HANDLER;
void SVC_Handler(void)                        WEAK_DEFAULT_HANDLER;
void DebugMon_Handler(void)                   WEAK_DEFAULT_HANDLER;
void PendSV_Handler(void)                     WEAK_DEFAULT_HANDLER;
void SysTick_Handler(void)                    WEAK_DEFAULT_HANDLER;
void DMA0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA2_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA3_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA4_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA5_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA6_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA7_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA8_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA9_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA10_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void DMA11_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void DMA12_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void DMA13_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void DMA14_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void DMA15_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void DMA_Error_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
void MCM_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void FTF_Command_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void FTF_ReadCollision_IRQHandler(void)       WEAK_DEFAULT_HANDLER;
void PMC_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void LLWU_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void WDOG_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void RNGA_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void I2C0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void I2C1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void SPI0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void SPI1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void I2S0_Tx_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void I2S0_Rx_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void LPUART0_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void UART0_RxTx_IRQHandler(void)              WEAK_DEFAULT_HANDLER;
void UART0_Error_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void UART1_RxTx_IRQHandler(void)              WEAK_DEFAULT_HANDLER;
void UART1_Error_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void UART2_RxTx_IRQHandler(void)              WEAK_DEFAULT_HANDLER;
void UART2_Error_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void ADC0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void CMP0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void CMP1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void FTM0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void FTM1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void FTM2_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void RTC_Alarm_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
void RTC_Seconds_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void PIT0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void PIT1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void PIT2_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void PIT3_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void PDB0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void USB0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DAC0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void MCG_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void LPTMR0_IRQHandler(void)                  WEAK_DEFAULT_HANDLER;
void PORTA_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PORTB_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PORTC_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PORTD_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PORTE_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void SWI_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void FTM3_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DAC1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void ADC1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;

typedef struct {
   uint32_t *initialSP;
   intfunc  handlers[89];
} VectorTable;

extern VectorTable const __vector_table;

__attribute__ ((section(".interrupt_vectors")))
VectorTable const __vector_table = {
                                               /*  Exc# Irq# */
   &__StackTop,                                /*    0   -16  Initial stack pointer                                                            */
   {
      Reset_Handler,                           /*    1,  -15  Reset Vector, invoked on Power up and warm reset                                 */
      NMI_Handler,                             /*    2,  -14  Non maskable Interrupt, cannot be stopped or preempted                           */
      HardFault_Handler,                       /*    3,  -13  Hard Fault, all classes of Fault                                                 */
      MemManage_Handler,                       /*    4,  -12  Memory Management, MPU mismatch, including Access Violation and No Match         */
      BusFault_Handler,                        /*    5,  -11  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault   */
      UsageFault_Handler,                      /*    6,  -10  Usage Fault, i.e. Undef Instruction, Illegal State Transition                    */
      0,                                       /*    7,   -9                                                                                   */
      0,                                       /*    8,   -8                                                                                   */
      0,                                       /*    9,   -7                                                                                   */
      0,                                       /*   10,   -6                                                                                   */
      SVC_Handler,                             /*   11,   -5  System Service Call via SVC instruction                                          */
      DebugMon_Handler,                        /*   12,   -4  Debug Monitor                                                                    */
      0,                                       /*   13,   -3                                                                                   */
      PendSV_Handler,                          /*   14,   -2  Pendable request for system service                                              */
      SysTick_Handler,                         /*   15,   -1  System Tick Timer                                                                */

                                               /* External Interrupts */
      DMA0_IRQHandler,                         /*   16,    0  Direct memory access controller                                                  */
      DMA1_IRQHandler,                         /*   17,    1  Direct memory access controller                                                  */
      DMA2_IRQHandler,                         /*   18,    2  Direct memory access controller                                                  */
      DMA3_IRQHandler,                         /*   19,    3  Direct memory access controller                                                  */
      DMA4_IRQHandler,                         /*   20,    4  Direct memory access controller                                                  */
      DMA5_IRQHandler,                         /*   21,    5  Direct memory access controller                                                  */
      DMA6_IRQHandler,                         /*   22,    6  Direct memory access controller                                                  */
      DMA7_IRQHandler,                         /*   23,    7  Direct memory access controller                                                  */
      DMA8_IRQHandler,                         /*   24,    8  Direct memory access controller                                                  */
      DMA9_IRQHandler,                         /*   25,    9  Direct memory access controller                                                  */
      DMA10_IRQHandler,                        /*   26,   10  Direct memory access controller                                                  */
      DMA11_IRQHandler,                        /*   27,   11  Direct memory access controller                                                  */
      DMA12_IRQHandler,                        /*   28,   12  Direct memory access controller                                                  */
      DMA13_IRQHandler,                        /*   29,   13  Direct memory access controller                                                  */
      DMA14_IRQHandler,                        /*   30,   14  Direct memory access controller                                                  */
      DMA15_IRQHandler,                        /*   31,   15  Direct memory access controller                                                  */
      DMA_Error_IRQHandler,                    /*   32,   16  Direct memory access controller                                                  */
      MCM_IRQHandler,                          /*   33,   17  Miscellaneous Control Module                                                     */
      FTF_Command_IRQHandler,                  /*   34,   18  Flash Memory Interface                                                           */
      FTF_ReadCollision_IRQHandler,            /*   35,   19  Flash Memory Interface                                                           */
      PMC_IRQHandler,                          /*   36,   20  Power Management Controller                                                      */
      LLWU_IRQHandler,                         /*   37,   21  Low Leakage Wakeup                                                               */
      WDOG_IRQHandler,                         /*   38,   22  External Watchdog Monitor                                                        */
      RNGA_IRQHandler,                         /*   39,   23  Random Number Generator Accelerator                                              */
      I2C0_IRQHandler,                         /*   40,   24  Inter-Integrated Circuit                                                         */
      I2C1_IRQHandler,                         /*   41,   25  Inter-Integrated Circuit                                                         */
      SPI0_IRQHandler,                         /*   42,   26  Serial Peripheral Interface                                                      */
      SPI1_IRQHandler,                         /*   43,   27  Serial Peripheral Interface                                                      */
      I2S0_Tx_IRQHandler,                      /*   44,   28  Synchronous Serial Interface                                                     */
      I2S0_Rx_IRQHandler,                      /*   45,   29  Synchronous Serial Interface                                                     */
      LPUART0_IRQHandler,                      /*   46,   30  Serial Communication Interface                                                   */
      UART0_RxTx_IRQHandler,                   /*   47,   31  Serial Communication Interface                                                   */
      UART0_Error_IRQHandler,                  /*   48,   32  Serial Communication Interface                                                   */
      UART1_RxTx_IRQHandler,                   /*   49,   33  Serial Communication Interface                                                   */
      UART1_Error_IRQHandler,                  /*   50,   34  Serial Communication Interface                                                   */
      UART2_RxTx_IRQHandler,                   /*   51,   35  Serial Communication Interface                                                   */
      UART2_Error_IRQHandler,                  /*   52,   36  Serial Communication Interface                                                   */
      Default_Handler,                         /*   53,   37                                                                                   */
      Default_Handler,                         /*   54,   38                                                                                   */
      ADC0_IRQHandler,                         /*   55,   39  Analogue to Digital Converter                                                    */
      CMP0_IRQHandler,                         /*   56,   40  High-Speed Comparator                                                            */
      CMP1_IRQHandler,                         /*   57,   41  High-Speed Comparator                                                            */
      FTM0_IRQHandler,                         /*   58,   42  FlexTimer Module                                                                 */
      FTM1_IRQHandler,                         /*   59,   43  FlexTimer Module                                                                 */
      FTM2_IRQHandler,                         /*   60,   44  FlexTimer Module                                                                 */
      Default_Handler,                         /*   61,   45                                                                                   */
      RTC_Alarm_IRQHandler,                    /*   62,   46  Real Time Clock                                                                  */
      RTC_Seconds_IRQHandler,                  /*   63,   47  Real Time Clock                                                                  */
      PIT0_IRQHandler,                         /*   64,   48  Periodic Interrupt Timer                                                         */
      PIT1_IRQHandler,                         /*   65,   49  Periodic Interrupt Timer                                                         */
      PIT2_IRQHandler,                         /*   66,   50  Periodic Interrupt Timer                                                         */
      PIT3_IRQHandler,                         /*   67,   51  Periodic Interrupt Timer                                                         */
      PDB0_IRQHandler,                         /*   68,   52  Programmable Delay Block                                                         */
      USB0_IRQHandler,                         /*   69,   53  Universal Serial Bus                                                             */
      Default_Handler,                         /*   70,   54                                                                                   */
      Default_Handler,                         /*   71,   55                                                                                   */
      DAC0_IRQHandler,                         /*   72,   56  Digital to Analogue Converter                                                    */
      MCG_IRQHandler,                          /*   73,   57  Multipurpose Clock Generator                                                     */
      LPTMR0_IRQHandler,                       /*   74,   58  Low Power Timer                                                                  */
      PORTA_IRQHandler,                        /*   75,   59  General Purpose Input/Output                                                     */
      PORTB_IRQHandler,                        /*   76,   60  General Purpose Input/Output                                                     */
      PORTC_IRQHandler,                        /*   77,   61  General Purpose Input/Output                                                     */
      PORTD_IRQHandler,                        /*   78,   62  General Purpose Input/Output                                                     */
      PORTE_IRQHandler,                        /*   79,   63  General Purpose Input/Output                                                     */
      SWI_IRQHandler,                          /*   80,   64  Software interrupt                                                               */
      Default_Handler,                         /*   81,   65                                                                                   */
      Default_Handler,                         /*   82,   66                                                                                   */
      Default_Handler,                         /*   83,   67                                                                                   */
      Default_Handler,                         /*   84,   68                                                                                   */
      Default_Handler,                         /*   85,   69                                                                                   */
      Default_Handler,                         /*   86,   70                                                                                   */
      FTM3_IRQHandler,                         /*   87,   71  FlexTimer Module                                                                 */
      DAC1_IRQHandler,                         /*   88,   72  Digital to Analogue Converter                                                    */
      ADC1_IRQHandler,                         /*   89,   73  Analogue to Digital Converter                                                    */
   }
};




