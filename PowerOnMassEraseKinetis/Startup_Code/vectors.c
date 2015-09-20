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

#define MK20D5

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
   volatile uint32_t vectorNum = (SCB_ICSR&SCB_ICSR_VECTACTIVE_MASK)>>SCB_ICSR_VECTACTIVE_SHIFT;

   while (1) {
      __asm__("bkpt");
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
      __asm__("bkpt");
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
void NMI_Handler(void)                 WEAK_DEFAULT_HANDLER;
void MemManage_Handler(void)           WEAK_DEFAULT_HANDLER;
void BusFault_Handler(void)            WEAK_DEFAULT_HANDLER;
void UsageFault_Handler(void)          WEAK_DEFAULT_HANDLER;
void SVC_Handler(void)                 WEAK_DEFAULT_HANDLER;
void DebugMon_Handler(void)            WEAK_DEFAULT_HANDLER;
void PendSV_Handler(void)              WEAK_DEFAULT_HANDLER;
void SysTick_Handler(void)             WEAK_DEFAULT_HANDLER;

void DMA0_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void DMA1_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void DMA2_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void DMA3_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void DMA_Error_Handler(void)           WEAK_DEFAULT_HANDLER;
void DMA_Other_IRQHandler(void)        WEAK_DEFAULT_HANDLER;
void FTFL_Command_IRQHandler(void)     WEAK_DEFAULT_HANDLER;
void FTFL_Collision_IRQHandler(void)   WEAK_DEFAULT_HANDLER;
void PMC_IRQHandler(void)              WEAK_DEFAULT_HANDLER;
void LLWU_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void WDOG_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void I2C0_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void SPI0_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void I2S0_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void I2S1_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void UART0_LON_IRQHandler(void)        WEAK_DEFAULT_HANDLER;
void UART0_Status_IRQHandler(void)     WEAK_DEFAULT_HANDLER;
void UART0_Error_IRQHandler(void)      WEAK_DEFAULT_HANDLER;
void UART1_Status_IRQHandler(void)     WEAK_DEFAULT_HANDLER;
void UART1_Error_IRQHandler(void)      WEAK_DEFAULT_HANDLER;
void UART2_Status_IRQHandler(void)     WEAK_DEFAULT_HANDLER;
void UART2_Error_IRQHandler(void)      WEAK_DEFAULT_HANDLER;
void ADC0_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void CMP0_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void CMP1_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void FTM0_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void FTM1_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void FTM2_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void RTC_Alarm_IRQHandler(void)        WEAK_DEFAULT_HANDLER;
void RTC_Seconds_IRQHandler(void)      WEAK_DEFAULT_HANDLER;
void PIT_Ch0_IRQHandler(void)          WEAK_DEFAULT_HANDLER;
void PIT_Ch1_IRQHandler(void)          WEAK_DEFAULT_HANDLER;
void PIT_Ch2_IRQHandler(void)          WEAK_DEFAULT_HANDLER;
void PIT_Ch3_IRQHandler(void)          WEAK_DEFAULT_HANDLER;
void USBOTG_IRQHandler(void)           WEAK_DEFAULT_HANDLER;
void USB0_Charge_IRQHandler(void)      WEAK_DEFAULT_HANDLER;
void TSI0_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void MCG_IRQHandler(void)              WEAK_DEFAULT_HANDLER;
void LPTMR0_IRQHandler(void)           WEAK_DEFAULT_HANDLER;
void PORTA_IRQHandler(void)            WEAK_DEFAULT_HANDLER;
void PORTB_IRQHandler(void)            WEAK_DEFAULT_HANDLER;
void PORTC_IRQHandler(void)            WEAK_DEFAULT_HANDLER;
void PORTD_IRQHandler(void)            WEAK_DEFAULT_HANDLER;
void PORTE_IRQHandler(void)            WEAK_DEFAULT_HANDLER;
void SWI_IRQHandler(void)              WEAK_DEFAULT_HANDLER;

typedef struct {
   uint32_t *initialSP;
   intfunc  handlers[];
} VectorTable;

__attribute__ ((section(".interrupt_vectors")))
VectorTable const __vector_table = {
    &__StackTop,                   /* Vec #0   Initial stack pointer                        */
    {
          __HardReset,              /* Vec #1   Reset Handler                                */
          NMI_Handler,              /* Vec #2   NMI Handler                                  */
(intfunc) HardFault_Handler,        /* Vec #3   Hard Fault Handler                           */
          MemManage_Handler,        /* Vec #4   MPU Fault Handler                            */
          BusFault_Handler,         /* Vec #5   Bus Fault Handler                            */
          UsageFault_Handler,       /* Vec #6   Usage Fault Handler                          */
          Default_Handler,          /* Vec #7   Reserved                                     */
          Default_Handler,          /* Vec #8   Reserved                                     */
          Default_Handler,          /* Vec #9   Reserved                                     */
          Default_Handler,          /* Vec #10  Reserved                                     */
          SVC_Handler,              /* Vec #11  SVCall Handler                               */
          DebugMon_Handler,         /* Vec #12  Debug Monitor Handler                        */
          Default_Handler,          /* Vec #13  Reserved                                     */
          PendSV_Handler,           /* Vec #14  PendSV Handler                               */
          SysTick_Handler,          /* Vec #15  SysTick Handler                              */

                                    /* External Interrupts */
          DMA0_IRQHandler,          /* Int #0   DMA Channel 0 Transfer Complete and Error    */
          DMA1_IRQHandler,          /* Int #1   DMA Channel 1 Transfer Complete and Error    */
          DMA2_IRQHandler,          /* Int #2   DMA Channel 2 Transfer Complete and Error    */
          DMA3_IRQHandler,          /* Int #3   DMA Channel 3 Transfer Complete and Error    */
          DMA_Error_Handler,        /* Int #4   DMA error interrupt channel                  */
          DMA_Other_IRQHandler,     /* Int #5   DMA ?     */
          FTFL_Command_IRQHandler,  /* Int #6   FTFL Flash command complete                  */
          FTFL_Collision_IRQHandler,/* Int #7   FTFL Flash read collision                    */
          PMC_IRQHandler,           /* Int #8   PMC Low-voltage detect, low-voltage warning  */
          LLWU_IRQHandler,          /* Int #9   LLWU Low Leakage Wake-up                     */
          WDOG_IRQHandler,          /* Int #10  EWM and WDOG interrupt                       */
          I2C0_IRQHandler,          /* Int #11  I2C0 interrupt                               */
          SPI0_IRQHandler,          /* Int #12  SPI0 Interrupt                               */
          I2S0_IRQHandler,          /* Int #13  I2S0 interrupt                               */
          I2S1_IRQHandler,          /* Int #14  I2S1 interrupt                               */
          UART0_LON_IRQHandler,     /* Int #15  UART0 LON interrupt                          */
          UART0_Status_IRQHandler,  /* Int #16  UART0 Status Interrupt                       */
          UART0_Error_IRQHandler,   /* Int #17  UART0 Error interrupt                        */
          UART1_Status_IRQHandler,  /* Int #18  UART1 Status Interrupt                       */
          UART1_Error_IRQHandler,   /* Int #19  UART1 Error interrupt                        */
          UART2_Status_IRQHandler,  /* Int #20  UART2 Status Interrupt                       */
          UART2_Error_IRQHandler,   /* Int #21  UART2 Error interrupt                        */
          ADC0_IRQHandler,          /* Int #22  ADC0 interrupt                               */
          CMP0_IRQHandler,          /* Int #23  CMP0 interrupt                               */
          CMP1_IRQHandler,          /* Int #24  CMP1 interrupt                               */
          FTM0_IRQHandler,          /* Int #25  FTM0 interrupt                               */
          FTM1_IRQHandler,          /* Int #26  FTM1 interrupt                               */
          FTM2_IRQHandler,          /* Int #27  CMT  interrupt                               */
          RTC_Alarm_IRQHandler,     /* Int #28  RTC Alarm interrupt                          */
          RTC_Seconds_IRQHandler,   /* Int #29  RTC Seconds interrupt                        */
          PIT_Ch0_IRQHandler,       /* Int #30  PIT Channel 0 interrupt                      */
          PIT_Ch1_IRQHandler,       /* Int #31  PIT Channel 1 interrupt                      */
          PIT_Ch2_IRQHandler,       /* Int #32  PIT Channel 2 interrupt                      */
          PIT_Ch3_IRQHandler,       /* Int #33  PIT Channel 3 interrupt                      */
          Default_Handler,          /* Int #34  PDB interrupt                                */
          USBOTG_IRQHandler,        /* Int #35  USB OTG interrupt                            */
          USB0_Charge_IRQHandler,   /* Int #36  USB Charger detect interrupt                 */
          TSI0_IRQHandler,          /* Int #37  TSI0 Interrupt                               */
          MCG_IRQHandler,           /* Int #38  MCG Interrupt                                */
          LPTMR0_IRQHandler,        /* Int #39  LPTMR0 interrupt                             */
          PORTA_IRQHandler,         /* Int #40  Port A interrupt                             */
          PORTB_IRQHandler,         /* Int #41  Port B interrupt                             */
          PORTC_IRQHandler,         /* Int #42  Port C interrupt                             */
          PORTD_IRQHandler,         /* Int #43  Port D interrupt                             */
          PORTE_IRQHandler,         /* Int #44  Port E interrupt                             */
          SWI_IRQHandler,           /* Int #45  Port E interrupt                             */
    }
};
