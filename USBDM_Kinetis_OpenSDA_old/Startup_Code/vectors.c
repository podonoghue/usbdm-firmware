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

#if !defined(OPEN_SDA_V1)
/*
 * OpenSDA doesn't use the security area
 */
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
  <h> Flash Configuration
  <i> 16-byte flash configuration field that stores default protection settings (loaded on reset)
  <i> and security information that allows the MCU to restrict access to the FTFL module.
  
  <h> Backdoor Comparison Key
  <i> The Verify Backdoor Access Key command releases security if user-supplied keys
  <i> matches the Backdoor Comparison Key bytes
    <o0>  Backdoor Comparison Key 0.  <0x0-0xFF>
    <o1>  Backdoor Comparison Key 1.  <0x0-0xFF>
    <o2>  Backdoor Comparison Key 2.  <0x0-0xFF>
    <o3>  Backdoor Comparison Key 3.  <0x0-0xFF>
    <o4>  Backdoor Comparison Key 4.  <0x0-0xFF>
    <o5>  Backdoor Comparison Key 5.  <0x0-0xFF>
    <o6>  Backdoor Comparison Key 6.  <0x0-0xFF>
    <o7>  Backdoor Comparison Key 7.  <0x0-0xFF>
  </h>
 */
#define BACKDOOR_VALUE {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, }
/*
   <h> Program Flash Region Protect (NV_FPROT0-3)
      <i> Each program flash region can be protected from program and erase operation by clearing the associated PROT bit.
      <i> Each bit protects a 1/32 region of the program flash memory.
	   <q.31>   FPROT0.0	<0=>protected  <1=>unprotected   <info>lowest 1/32 block
	   <q.30>   FPROT0.1	<0=>protected  <1=>unprotected
	   <q.29>   FPROT0.2	<0=>protected  <1=>unprotected
	   <q.28>   FPROT0.3	<0=>protected  <1=>unprotected
	   <q.27>   FPROT0.4	<0=>protected  <1=>unprotected
	   <q.26>   FPROT0.5	<0=>protected  <1=>unprotected
	   <q.25>   FPROT0.6	<0=>protected  <1=>unprotected
	   <q.24>   FPROT0.7	<0=>protected  <1=>unprotected
	   <q.23>   FPROT1.0	<0=>protected  <1=>unprotected
	   <q.22>   FPROT1.1	<0=>protected  <1=>unprotected
	   <q.21>   FPROT1.2	<0=>protected  <1=>unprotected
	   <q.20>   FPROT1.3	<0=>protected  <1=>unprotected
	   <q.19>   FPROT1.4	<0=>protected  <1=>unprotected
	   <q.18>   FPROT1.5	<0=>protected  <1=>unprotected
	   <q.17>   FPROT1.6	<0=>protected  <1=>unprotected
	   <q.16>   FPROT1.7	<0=>protected  <1=>unprotected
	   <q.15>   FPROT2.0	<0=>protected  <1=>unprotected
	   <q.14>   FPROT2.1	<0=>protected  <1=>unprotected
	   <q.13>   FPROT2.2	<0=>protected  <1=>unprotected
	   <q.12>   FPROT2.3	<0=>protected  <1=>unprotected
	   <q.11>   FPROT2.4	<0=>protected  <1=>unprotected
	   <q.10>   FPROT2.5	<0=>protected  <1=>unprotected
	   <q.9>    FPROT2.6	<0=>protected  <1=>unprotected
	   <q.8>    FPROT2.7	<0=>protected  <1=>unprotected
	   <q.7>    FPROT3.0	<0=>protected  <1=>unprotected
	   <q.6>    FPROT3.1	<0=>protected  <1=>unprotected
	   <q.5>    FPROT3.2	<0=>protected  <1=>unprotected
	   <q.4>    FPROT3.3	<0=>protected  <1=>unprotected
	   <q.3>    FPROT3.4	<0=>protected  <1=>unprotected
	   <q.2>    FPROT3.5	<0=>protected  <1=>unprotected
	   <q.1>    FPROT3.6	<0=>protected  <1=>unprotected
	   <q.0>    FPROT3.7	<0=>protected  <1=>unprotected   <info> highest 1/32 block
   </h>
*/
#define FPROT_VALUE 0xFFFFFFFF
/*
   <h> EEPROM Region Protect (NV_FEPROT)
      <i> Each bit protects a 1/8 region of the EEPROM memory.
      <i> (FlexNVM devices only)
      <q.0>   FEPROT.0	<0=>protected  <1=>unprotected   <info> lowest 1/8 block
      <q.1>   FEPROT.1  <0=>protected  <1=>unprotected
      <q.2>   FEPROT.2  <0=>protected  <1=>unprotected
      <q.3>   FEPROT.3  <0=>protected  <1=>unprotected
      <q.4>   FEPROT.4  <0=>protected  <1=>unprotected
      <q.5>   FEPROT.5  <0=>protected  <1=>unprotected
      <q.6>   FEPROT.6  <0=>protected  <1=>unprotected
      <q.7>   FEPROT.7	<0=>protected  <1=>unprotected   <info> highest 1/8 block
   </h>
*/
#define FEPROT_VALUE 0xFF
/*
   <h> Data Flash Region Protect (NV_FDPROT)
      <i> Each bit protects a 1/8 region of the data flash memory.
      <i> (Device with Data flash only)
      <q.0>   FDPROT.0	<0=>protected  <1=>unprotected   <info> lowest 1/8 block
      <q.1>   FDPROT.1  <0=>protected  <1=>unprotected
      <q.2>   FDPROT.2  <0=>protected  <1=>unprotected
      <q.3>   FDPROT.3  <0=>protected  <1=>unprotected
      <q.4>   FDPROT.4  <0=>protected  <1=>unprotected
      <q.5>   FDPROT.5  <0=>protected  <1=>unprotected
      <q.6>   FDPROT.6  <0=>protected  <1=>unprotected
      <q.7>   FDPROT.7	<0=>protected  <1=>unprotected   <info> highest 1/8 block
   </h>
*/
#define FDPROT_VALUE 0xFF

/*
<h> Flash security value (NV_FSEC)
   <o0> Backdoor Key Security Access Enable (FSEC.KEYEN)
      <i> Controls use of Backdoor Key access to unsecure device
      <info>KEYEN
      <2=> 2: Access enabled
      <3=> 3: Access disabled
   <o1> Mass Erase Enable Bits (FSEC.MEEN)
      <i> Controls mass erase capability of the flash memory module.
      <i> Only relevant when FSEC.SEC is set to secure.
      <info>MEEN
      <2=> 2: Mass erase disabled
      <3=> 3: Mass erase enabled
   <o2> Freescale Failure Analysis Access (FSEC.FSLACC)
      <i> Controls access to the flash memory contents during returned part failure analysis
      <info>FSLACC
      <2=> 2: Factory access denied
      <3=> 3: Factory access granted
   <o3> Flash Security (FSEC.SEC)
      <i> Defines the security state of the MCU. 
      <i> In the secure state, the MCU limits access to flash memory module resources. 
      <i> If the flash memory module is unsecured using backdoor key access, SEC is forced to 10b.
      <info>SEC
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
<h> Flash boot options (NV_FOPT)
   <q0.2> NMI pin control (FOPT.NMI_DIS)
      <i> Enables or disables the NMI function
      <info>NMI_DIS
      <0=> NMI interrupts are always blocked.
      <1=> NMI interrupts default to enabled
   <q0.1> EZPORT pin control (FOPT.EZPORT_DIS)
      <i> Enables or disables EzPort function
      <i> Disabling EZPORT function avoids inadvertent resets into EzPort mode 
      <i> if the EZP_CS/NMI pin is used for its NMI function 
      <info>EZPORT_DIS
      <0=> EZP_CSn pin is disabled on reset
      <1=> EZP_CSn pin is enabled on reset
   <q0.0> Low power boot control (FOPT.LPBOOT)
      <i> Controls the reset value of SIM_CLKDIV1.OUTDIVx (clock dividers)
      <i> Allows power consumption during reset to be reduced
      <info>LPBOOT
      <0=> Low Power - CLKDIV1,2 = /8, CLKDIV3,4 = /16
      <1=> Normal - CLKDIV1,2 = /1, CLKDIV3,4 = /2
</h>
 */
#define FOPT_VALUE (0x7|0xF8)

__attribute__ ((section(".security_information")))
const SecurityInfo securityInfo = {
    /* backdoor */ BACKDOOR_VALUE,
    /* fprot    */ FPROT_VALUE,
    /* fsec     */ FSEC_VALUE,
    /* fopt     */ FOPT_VALUE,
    /* feprot   */ FEPROT_VALUE,
    /* fdprot   */ FDPROT_VALUE,
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
void FTF_Command_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void FTF_ReadCollision_IRQHandler(void)       WEAK_DEFAULT_HANDLER;
void PMC_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void LLWU_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void WDOG_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
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
void RTC_Alarm_IRQHandler(void)               WEAK_DEFAULT_HANDLER;
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
      DMA0_IRQHandler,               /*   16,    0  Direct memory access controller                                                  */
      DMA1_IRQHandler,               /*   17,    1  Direct memory access controller                                                  */
      DMA2_IRQHandler,               /*   18,    2  Direct memory access controller                                                  */
      DMA3_IRQHandler,               /*   19,    3  Direct memory access controller                                                  */
      DMA_Error_IRQHandler,          /*   20,    4  DMA error interrupt                                                              */
      Default_Handler,               /*   21,    5                                                                                   */
      FTF_Command_IRQHandler,        /*   22,    6  Flash Memory Interface                                                           */
      FTF_ReadCollision_IRQHandler,  /*   23,    7  Flash Memory Interface                                                           */
      PMC_IRQHandler,                /*   24,    8  Power Management Controller                                                      */
      LLWU_IRQHandler,               /*   25,    9  Low Leakage Wakeup                                                               */
      WDOG_IRQHandler,               /*   26,   10  External Watchdog Monitor                                                        */
      I2C0_IRQHandler,               /*   27,   11  Inter-Integrated Circuit                                                         */
      SPI0_IRQHandler,               /*   28,   12  Serial Peripheral Interface                                                      */
      I2S0_Tx_IRQHandler,            /*   29,   13  Synchronous Serial Interface                                                     */
      I2S0_Rx_IRQHandler,            /*   30,   14  Synchronous Serial Interface                                                     */
      UART0_LON_IRQHandler,          /*   31,   15  Serial Communication Interface                                                   */
      UART0_RX_TX_IRQHandler,        /*   32,   16  Serial Communication Interface                                                   */
      UART0_ERR_IRQHandler,          /*   33,   17  Serial Communication Interface                                                   */
      UART1_RX_TX_IRQHandler,        /*   34,   18  Serial Communication Interface                                                   */
      UART1_ERR_IRQHandler,          /*   35,   19  Serial Communication Interface                                                   */
      UART2_RX_TX_IRQHandler,        /*   36,   20  Serial Communication Interface                                                   */
      UART2_ERR_IRQHandler,          /*   37,   21  Serial Communication Interface                                                   */
      ADC0_IRQHandler,               /*   38,   22  Analogue to Digital Converter                                                    */
      CMP0_IRQHandler,               /*   39,   23  High-Speed Comparator                                                            */
      CMP1_IRQHandler,               /*   40,   24  High-Speed Comparator                                                            */
      FTM0_IRQHandler,               /*   41,   25  FlexTimer Module                                                                 */
      FTM1_IRQHandler,               /*   42,   26  FlexTimer Module                                                                 */
      CMT_IRQHandler,                /*   43,   27  Carrier Modulator Transmitter                                                    */
      RTC_Alarm_IRQHandler,          /*   44,   28  Real Time Clock                                                                  */
      RTC_Seconds_IRQHandler,        /*   45,   29  Real Time Clock                                                                  */
      PIT0_IRQHandler,               /*   46,   30  Periodic Interrupt Timer                                                         */
      PIT1_IRQHandler,               /*   47,   31  Periodic Interrupt Timer                                                         */
      PIT2_IRQHandler,               /*   48,   32  Periodic Interrupt Timer                                                         */
      PIT3_IRQHandler,               /*   49,   33  Periodic Interrupt Timer                                                         */
      PDB0_IRQHandler,               /*   50,   34  Programmable Delay Block                                                         */
      USB0_IRQHandler,               /*   51,   35  Universal Serial Bus                                                             */
      USBDCD_IRQHandler,             /*   52,   36  USB Device Charger Detection                                                     */
      TSI0_IRQHandler,               /*   53,   37  Touch Sense Interface                                                            */
      MCG_IRQHandler,                /*   54,   38  MCG interrupt                                                                    */
      LPTMR0_IRQHandler,             /*   55,   39  Low Power Timer                                                                  */
      PORTA_IRQHandler,              /*   56,   40  General Purpose Input/Output                                                     */
      PORTB_IRQHandler,              /*   57,   41  General Purpose Input/Output                                                     */
      PORTC_IRQHandler,              /*   58,   42  General Purpose Input/Output                                                     */
      PORTD_IRQHandler,              /*   59,   43  General Purpose Input/Output                                                     */
      PORTE_IRQHandler,              /*   60,   44  General Purpose Input/Output                                                     */
      SWI_IRQHandler,                /*   61,   45  Software interrupt                                                               */
   }
};



