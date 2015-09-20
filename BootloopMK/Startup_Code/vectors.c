/*
 *  Vectors-mk.c
 *
 *  Generic vectors and security for Kinetis
 *
 *  Created on: 07/12/2012
 *      Author: podonoghue
 */
#include <stdint.h>
#include <string.h>
#include "derivative.h"

#define DEVICE_SUBFAMILY_CortexM4

/*
 * Security information
 */
typedef struct {
    uint8_t  backdoorKey[8];
    uint32_t fprot;
    uint8_t  fsec;
    uint8_t  fopt;
    uint8_t  feprot;
    uint8_t  fdprot;
} SecurityInfo;

__attribute__ ((section(".security_information")))
const SecurityInfo securityInfo = {
    /* backdoor */ {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
    /* fprot    */ 0xFFFFFFFF,
    /* fsec     */ 0xFE,
    /* fopt     */ 0xFF,
    /* feprot   */ 0xFF,
    /* fdprot   */ 0xFF,
};


#ifdef NV_FOPT_BOOTPIN_OPT_MASK
/*
 * Security information
 */
typedef struct {
    char     magic[4];           // Magic number indicating valid configuration - 'kcfg'
    uint8_t  reserved[12];       // Reserved
    uint8_t  enabledPeripherals; // 0:LPUART, 1:I2C, 2:SPI, 4:USB
    uint8_t  i2cAddress;         // If not 0xFF, used as the 7-bit I2C slave address.
    uint16_t peripheralTimeout;  // Timeout in milliseconds for active peripheral detection
    uint16_t usbVid;             // Sets the USB Vendor ID reported by the device during enumeration.
    uint16_t usbPid;             // Sets the USB Product ID reported by the device during enumeration.
    uint32_t usbStringsPointer;  // Sets the USB Strings reported by the device during enumeration.
    uint8_t  clockFlags;         // See Table 13-4, clockFlags Configuration Field
    uint8_t  clockDivider;       // Divider to use for core and bus clocks when in high speed mode
} BootloaderConfiguration;

__attribute__ ((section(".bootloader_configuration")))
const BootloaderConfiguration bootloaderConfiguration = {
    /* magic               */ "kcfg",
    /* reserved            */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* enabledPeripherals  */ 0xFF, /* all peripherals */
    /* i2cAddress          */ 0x11,
    /* peripheralTimeout   */ 1000, /* ms */
    /* usbVid              */ 0,
    /* usbPid              */ 0,
    /* usbStringsPointer   */ 0,
    /* clockFlags          */ 0,
    /* clockDivider        */ 0,    /* 0 => high-speed 48MHz */
};
#endif

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
     __asm__ volatile ( "  tst   lr, #4              \n");  // Check mode
     __asm__ volatile ( "  ite   eq                  \n");  // Get active SP in r0
     __asm__ volatile ( "  mrseq r0, msp             \n");
     __asm__ volatile ( "  mrsne r0, psp             \n");
     __asm__ volatile ( "  b     _HardFault_Handler  \n");  // Go to C handler
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
void DMA0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA2_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA3_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA_Error_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
void FTFL_Command_IRQHandler(void)            WEAK_DEFAULT_HANDLER;
void FTFL_Collision_IRQHandler(void)          WEAK_DEFAULT_HANDLER;
void PMC_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void LLWU_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void WDOG_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void I2C0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void SPI0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void I2S0_Tx_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void I2S0_Rx_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void UART0_LON_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
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
void CMT_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void RTC_Alarm_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
void RTC_Seconds_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void PIT_Ch0_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void PIT_Ch1_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void PIT_Ch2_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void PIT_Ch3_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void PDB0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void USBOTG_IRQHandler(void)                  WEAK_DEFAULT_HANDLER;
void USBDCD_IRQHandler(void)                  WEAK_DEFAULT_HANDLER;
void TSI0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void MCG_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void LPTMR0_IRQHandler(void)                  WEAK_DEFAULT_HANDLER;
void PORTA_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PORTB_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PORTC_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PORTD_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PORTE_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void SWI_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;

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
      DMA0_IRQHandler,               /*   16,    0  DMA channel 0 transfer complete interrupt                                        */
      DMA1_IRQHandler,               /*   17,    1  DMA channel 1 transfer complete interrupt                                        */
      DMA2_IRQHandler,               /*   18,    2  DMA channel 2 transfer complete interrupt                                        */
      DMA3_IRQHandler,               /*   19,    3  DMA channel 3 transfer complete interrupt                                        */
      DMA_Error_IRQHandler,          /*   20,    4  DMA error interrupt                                                              */
      Default_Handler,               /*   21,    5                                                                                   */
      FTFL_Command_IRQHandler,       /*   22,    6  FTFL interrupt                                                                   */
      FTFL_Collision_IRQHandler,     /*   23,    7  FTFL Read collision interrupt                                                    */
      PMC_IRQHandler,                /*   24,    8  PMC Low Voltage Detect, Low Voltage Warning                                      */
      LLWU_IRQHandler,               /*   25,    9  LLWU Low Leakage Wakeup                                                          */
      WDOG_IRQHandler,               /*   26,   10  WDOG interrupt                                                                   */
      I2C0_IRQHandler,               /*   27,   11  I2C0 interrupt                                                                   */
      SPI0_IRQHandler,               /*   28,   12  SPI0 interrupt                                                                   */
      I2S0_Tx_IRQHandler,            /*   29,   13  I2S0 transmit interrupt                                                          */
      I2S0_Rx_IRQHandler,            /*   30,   14  I2S0 receive interrupt                                                           */
      UART0_LON_IRQHandler,          /*   31,   15  UART0 LON interrupt                                                              */
      UART0_RxTx_IRQHandler,         /*   32,   16  UART0 receive/transmit interrupt                                                 */
      UART0_Error_IRQHandler,        /*   33,   17  UART0 error interrupt                                                            */
      UART1_RxTx_IRQHandler,         /*   34,   18  UART1 receive/transmit interrupt                                                 */
      UART1_Error_IRQHandler,        /*   35,   19  UART1 error interrupt                                                            */
      UART2_RxTx_IRQHandler,         /*   36,   20  UART2 receive/transmit interrupt                                                 */
      UART2_Error_IRQHandler,        /*   37,   21  UART0 error interrupt                                                            */
      ADC0_IRQHandler,               /*   38,   22  ADC0 interrupt                                                                   */
      CMP0_IRQHandler,               /*   39,   23  CMP0 interrupt                                                                   */
      CMP1_IRQHandler,               /*   40,   24  CMP1 interrupt                                                                   */
      FTM0_IRQHandler,               /*   41,   25  FTM0 fault, overflow and channels interrupt                                      */
      FTM1_IRQHandler,               /*   42,   26  FTM1 fault, overflow and channels interrupt                                      */
      CMT_IRQHandler,                /*   43,   27  CMT interrupt                                                                    */
      RTC_Alarm_IRQHandler,          /*   44,   28  RTC interrupt                                                                    */
      RTC_Seconds_IRQHandler,        /*   45,   29  RTC seconds interrupt                                                            */
      PIT_Ch0_IRQHandler,            /*   46,   30  PIT timer channel 0 interrupt                                                    */
      PIT_Ch1_IRQHandler,            /*   47,   31  PIT timer channel 1 interrupt                                                    */
      PIT_Ch2_IRQHandler,            /*   48,   32  PIT timer channel 2 interrupt                                                    */
      PIT_Ch3_IRQHandler,            /*   49,   33  PIT timer channel 3 interrupt                                                    */
      PDB0_IRQHandler,               /*   50,   34  PDB0 Programmable Delay Block interrupt                                          */
      USBOTG_IRQHandler,             /*   51,   35  USB0 OTG interrupt                                                               */
      USBDCD_IRQHandler,             /*   52,   36  USBDCD interrupt                                                                 */
      TSI0_IRQHandler,               /*   53,   37  TSI0 interrupt                                                                   */
      MCG_IRQHandler,                /*   54,   38  MCG interrupt                                                                    */
      LPTMR0_IRQHandler,             /*   55,   39  LPTMR Low Power Timer interrupt                                                  */
      PORTA_IRQHandler,              /*   56,   40  Port A interrupt                                                                 */
      PORTB_IRQHandler,              /*   57,   41  Port B interrupt                                                                 */
      PORTC_IRQHandler,              /*   58,   42  Port C interrupt                                                                 */
      PORTD_IRQHandler,              /*   59,   43  Port D interrupt                                                                 */
      PORTE_IRQHandler,              /*   60,   44  Port E interrupt                                                                 */
      SWI_IRQHandler,                /*   61,   45  Software interrupt                                                               */
   }
};



