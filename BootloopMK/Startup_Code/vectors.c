/*
 *  Vectors-mk.c
 *
 *  Generic vectors and security for Kinetis MKxxx
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

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------

/*
<h> Flash security value (NV_FTFA_FSEC)
   <o0> Backdoor Key Security Access Enable (FSEC.KEYEN)
      <i> Controls use of Backdoor Key access to unsecure device
      <0=> 0: Access disabled
      <1=> 1: Access disabled (preferred disabled value)
      <2=> 2: Access enabled
      <3=> 3: Access disabled
   <o1> Mass Erase Enable Bits (FSEC.MEEN)
      <i> Controls mass erase capability of the flash memory module.
      <i> Only relevant when FSEC.SEC is set to secure.
      <0=> 0: Mass erase enabled
      <1=> 1: Mass erase enabled
      <2=> 2: Mass erase disabled
      <3=> 3: Mass erase enabled
   <o2> Freescale Failure Analysis Access (FSEC.FSLACC)
      <i> Controls access to the flash memory contents during returned part failure analysis
      <0=> 0: Factory access granted
      <1=> 1: Factory access denied
      <2=> 2: Factory access denied
      <3=> 3: Factory access granted
   <o3> Flash Security (FSEC.SEC)
      <i> Defines the security state of the MCU. 
      <i> In the secure state, the MCU limits access to flash memory module resources. 
      <i> If the flash memory module is unsecured using backdoor key access, SEC is forced to 10b.
      <0=> 0: Secured
      <1=> 1: Secured
      <2=> 2: Unsecured
      <3=> 3: Secured
</h>
*/
#define FSEC_VALUE ((3<<NV_FSEC_KEYEN_SHIFT)|(3<<NV_FSEC_MEEN_SHIFT)|(3<<NV_FSEC_FSLACC_SHIFT)|(2<<NV_FSEC_SEC_SHIFT))
#if ((FSEC_VALUE&NV_FSEC_MEEN_MASK) == (2<<NV_FSEC_MEEN_SHIFT)) && ((FSEC_VALUE&NV_FSEC_SEC_MASK) != (2<<NV_FSEC_SEC_SHIFT))
// Change to warning if your really, really want to do this!
#error "The security values selected will prevent the device from being unsecured using external methods"
#endif

/*
Control extended Boot features on these devices
<h> Flash boot options (NV_FTFA_FOPT)
   <q0.2> NMI pin control (FOPT.NMI_DIS)
      <i> Enables or disables the NMI function
      <0=> NMI interrupts are always blocked.
      <1=> NMI_b interrupts default to enabled
   <q0.1> EZPORT pin control (FOPT.EZPORT_DIS)
      <i> Enables or disables EzPort function
      <i> Disabling EZPORT function avoids inadvertent resets into EzPort mode 
      <i> if the EZP_CS/NMI pin is used for its NMI function 
      <0=> EZP_CSn pin is disabled on reset
      <1=> EZP_CSn pin is enabled on reset
   <q0.0> Low power boot control (FOPT.LPBOOT)
      <i> Controls the reset value of SIM_CLKDIV1.OUTDIVx (clock dividers)
      <i> Allows power consumption during reset to be reduced
      <0=> CLKDIV1,2 = /8, CLKDIV3,4 = /16
      <1=> CLKDIV1,2 = /1, CLKDIV3,4 = /2
</h>
 */
#define FOPT_VALUE (0x7|0xF8)

__attribute__ ((section(".security_information")))
const SecurityInfo securityInfo = {
    /* backdoor */ {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
    /* fprot    */ 0xFFFFFFFF,
    /* fsec     */ FSEC_VALUE,
    /* fopt     */ FOPT_VALUE,
    /* feprot   */ 0xFF,
    /* fdprot   */ 0xFF,
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
void LVD_LVW_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void LLW_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void Watchdog_IRQHandler(void)                WEAK_DEFAULT_HANDLER;
void I2C0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void SPI0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void I2S0_Tx_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void I2S0_Rx_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void UART0_LON_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
void UART0_RX_TX_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void UART0_ERR_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
void UART1_RX_TX_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void UART1_ERR_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
void UART2_RX_TX_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void UART2_ERR_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
void ADC0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void CMP0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void CMP1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void FTM0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void FTM1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void CMT_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void RTC_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void RTC_Seconds_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void PIT0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void PIT1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void PIT2_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void PIT3_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void PDB0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void USB0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
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
      LVD_LVW_IRQHandler,            /*   24,    8  PMC Low Voltage Detect, Low Voltage Warning                                      */
      LLW_IRQHandler,                /*   25,    9  LLW Low Leakage Wakeup                                                           */
      Watchdog_IRQHandler,           /*   26,   10  WDOG interrupt                                                                   */
      I2C0_IRQHandler,               /*   27,   11  I2C0 interrupt                                                                   */
      SPI0_IRQHandler,               /*   28,   12  SPI0 interrupt                                                                   */
      I2S0_Tx_IRQHandler,            /*   29,   13  I2S0 transmit interrupt                                                          */
      I2S0_Rx_IRQHandler,            /*   30,   14  I2S0 receive interrupt                                                           */
      UART0_LON_IRQHandler,          /*   31,   15  UART0 LON interrupt                                                              */
      UART0_RX_TX_IRQHandler,        /*   32,   16  UART0 receive/transmit interrupt                                                 */
      UART0_ERR_IRQHandler,          /*   33,   17  UART0 error interrupt                                                            */
      UART1_RX_TX_IRQHandler,        /*   34,   18  UART1 receive/transmit interrupt                                                 */
      UART1_ERR_IRQHandler,          /*   35,   19  UART1 error interrupt                                                            */
      UART2_RX_TX_IRQHandler,        /*   36,   20  UART2 receive/transmit interrupt                                                 */
      UART2_ERR_IRQHandler,          /*   37,   21  UART0 error interrupt                                                            */
      ADC0_IRQHandler,               /*   38,   22  ADC0 interrupt                                                                   */
      CMP0_IRQHandler,               /*   39,   23  CMP0 interrupt                                                                   */
      CMP1_IRQHandler,               /*   40,   24  CMP1 interrupt                                                                   */
      FTM0_IRQHandler,               /*   41,   25  FTM0 fault, overflow and channels interrupt                                      */
      FTM1_IRQHandler,               /*   42,   26  FTM1 fault, overflow and channels interrupt                                      */
      CMT_IRQHandler,                /*   43,   27  CMT interrupt                                                                    */
      RTC_IRQHandler,                /*   44,   28  RTC interrupt                                                                    */
      RTC_Seconds_IRQHandler,        /*   45,   29  RTC seconds interrupt                                                            */
      PIT0_IRQHandler,               /*   46,   30  PIT timer channel 0 interrupt                                                    */
      PIT1_IRQHandler,               /*   47,   31  PIT timer channel 1 interrupt                                                    */
      PIT2_IRQHandler,               /*   48,   32  PIT timer channel 2 interrupt                                                    */
      PIT3_IRQHandler,               /*   49,   33  PIT timer channel 3 interrupt                                                    */
      PDB0_IRQHandler,               /*   50,   34  PDB0 Programmable Delay Block interrupt                                          */
      USB0_IRQHandler,               /*   51,   35  USB0 OTG interrupt                                                               */
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



