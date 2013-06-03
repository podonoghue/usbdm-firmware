/*
 *    kinetis_sysinit.c - Default init routines for P0
 *                         Kinetis ARM systems
 *    Copyright © 2012 Freescale semiConductor Inc. All Rights Reserved.
 */
#include <stdint.h>
#include <string.h>
#include "kinetis_sysinit.h"
#include "derivative.h"

/**
 **===========================================================================
 **  External declarations
 **===========================================================================
 */
#if __cplusplus
extern "C" {
#endif
extern uint32_t __vector_table[];
extern unsigned long _estack;
extern void __thumb_startup(void);
#if __cplusplus
}
#endif


typedef struct {
   unsigned int r0;
   unsigned int r1;
   unsigned int r2;
   unsigned int r3;
   unsigned int r12;
   unsigned int lr;
   unsigned int pc;
   unsigned int psr;
} ExceptionFrame;

typedef struct {
   unsigned int scb_hfsr; // Hard Fault Status Register
   unsigned int scb_cfsr; // Configurable Fault Status Register
   unsigned int scb_bfar; // BusFault Address Register
} ExceptionInfo;

/******************************************************************************/
/* Exception frame without floating-point storage
 * hard fault handler in C,
 * with stack frame location as input parameter
 */
__attribute__((weak,__interrupt__))
void HardFault_Handler(ExceptionFrame *exceptionFrame) {
#ifdef SCB_HFSR
   char reason[200] = "";
   ExceptionInfo exceptionInfo = {0};
   exceptionInfo.scb_hfsr = SCB_HFSR;
   (void)exceptionInfo.scb_hfsr;
   if ((exceptionInfo.scb_hfsr&SCB_HFSR_FORCED_MASK) != 0) {
      // Forced
      exceptionInfo.scb_cfsr = SCB_CFSR;

      if (SCB_CFSR&SCB_CFSR_BFARVALID_MASK) {
         exceptionInfo.scb_bfar = SCB_BFAR;
      }
      /* CFSR Bit Fields */
      if (SCB_CFSR&SCB_CFSR_DIVBYZERO_MASK  ) { strcat(reason, "Divide by zero,"); }
      if (SCB_CFSR&SCB_CFSR_UNALIGNED_MASK  ) { strcat(reason, "Unaligned access,"); }
      if (SCB_CFSR&SCB_CFSR_NOCP_MASK       ) { strcat(reason, "No co-processor"); }
      if (SCB_CFSR&SCB_CFSR_INVPC_MASK      ) { strcat(reason, "Invalid PC (on return),"); }
      if (SCB_CFSR&SCB_CFSR_INVSTATE_MASK   ) { strcat(reason, "Invalid state (EPSR.T/IT,"); }
      if (SCB_CFSR&SCB_CFSR_UNDEFINSTR_MASK ) { strcat(reason, "Undefined Instruction,"); }
      if (SCB_CFSR&SCB_CFSR_BFARVALID_MASK  ) { strcat(reason, "BFAR contents valid,"); }
      if (SCB_CFSR&SCB_CFSR_LSPERR_MASK     ) { strcat(reason, "Bus fault on FP state save,"); }
      if (SCB_CFSR&SCB_CFSR_STKERR_MASK     ) { strcat(reason, "Bus fault on exception entry,"); }
      if (SCB_CFSR&SCB_CFSR_UNSTKERR_MASK   ) { strcat(reason, "Bus fault on exception return,"); }
      if (SCB_CFSR&SCB_CFSR_IMPRECISERR_MASK) { strcat(reason, "Imprecise data access error,"); }
      if (SCB_CFSR&SCB_CFSR_PRECISERR_MASK  ) { strcat(reason, "Precise data access error,"); }
      if (SCB_CFSR&SCB_CFSR_IBUSERR_MASK    ) { strcat(reason, "Bus fault on instruction pre-fetch,"); }
      if (SCB_CFSR&SCB_CFSR_MMARVALID_MASK  ) { strcat(reason, "MMAR contents valid,"); }
      if (SCB_CFSR&SCB_CFSR_MLSPERR_MASK    ) { strcat(reason, "MemManage fault on FP state save,"); }
      if (SCB_CFSR&SCB_CFSR_MSTKERR_MASK    ) { strcat(reason, "MemManage fault on exception entry,"); }
      if (SCB_CFSR&SCB_CFSR_MUNSTKERR_MASK  ) { strcat(reason, "MemManage fault on exception return,"); }
      if (SCB_CFSR&SCB_CFSR_DACCVIOL_MASK   ) { strcat(reason, "MemManage access violation on data access,"); }
      if (SCB_CFSR&SCB_CFSR_IACCVIOL_MASK   ) { strcat(reason, "MemManage access violation on instruction fetch,"); }
   }
#endif
   while (1) {
      asm("bkpt #0");
   }
}


/**
 **===========================================================================
 **  Default interrupt handler
 **===========================================================================
 */
void Default_Handler() {

	uint32_t vectorNum = SCB_ICSR;

	(void)vectorNum;

	while (1) {
		asm("bkpt #0");
	}
}

/**
 **===========================================================================
 **  Reset handler
 **===========================================================================
 */
void __init_hardware()
{
   SCB_VTOR = (uint32_t)__vector_table; /* Set the interrupt vector table position */
   /*
      Disable the Watchdog because it may reset the core before entering main().
      There are 2 unlock words which shall be provided in sequence before
      accessing the control register.
   */
   if (WDOG_STCTRLH & WDOG_STCTRLH_WDOGEN_MASK) {
      WDOG_UNLOCK  = KINETIS_WDOG_UNLOCK_SEQ_1;
      WDOG_UNLOCK  = KINETIS_WDOG_UNLOCK_SEQ_2;
      WDOG_STCTRLH = KINETIS_WDOG_DISABLED_CTRL;
   }
}

/* Weak definitions of handlers point to Default_Handler if not implemented */
void NMI_Handler() __attribute__ ((weak, alias("Default_Handler")));
//void HardFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler() __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler() __attribute__ ((weak, alias("Default_Handler")));
void DebugMonitor_Handler() __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler() __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler() __attribute__ ((weak, alias("Default_Handler")));

void DMA0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void DMA1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void DMA2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void DMA3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void DMA_Error_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void FTFL_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LVD_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LLW_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void WDOG_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void SPI0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void I2S0Tx_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void I2S0Rx_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART0_LON_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART0_Error_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART1_Error_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART2_Error_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void ADC0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void CMP0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void CMP1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void FTM0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void FTM1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void CMT_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void RTC_Seconds_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PIT_Channel0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PIT_Channel1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PIT_Channel2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PIT_Channel3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PDB0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USB0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USBDCD_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void TSI0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void MCG_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LPTimer_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PORTA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PORTB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PORTC_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PORTD_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PORTE_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void SWI_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));

/* The Interrupt Vector Table */
void (* const InterruptVector[256])() __attribute__ ((section(".vectortable"))) = {
    /* Processor exceptions */
    (void(*)(void)) &_estack,     /*!< Initial stack pointer */
    __thumb_startup,              /*!< Initial program counter */
    NMI_Handler,                  /*!< Non-maskable interrupt */
    HardFault_Handler,            /*!< Hard fault exception */
    MemManage_Handler,            /*!< Memory Manage Fault */
    BusFault_Handler,             /*!< Bus fault exception */
    UsageFault_Handler,           /*!< Usage fault exception */
    0,                            /*!< Reserved interrupt 7 */
    0,                            /*!< Reserved interrupt 8 */
    0,                            /*!< Reserved interrupt 9 */
    0,                            /*!< Reserved interrupt 10 */
    SVC_Handler,                  /*!< A supervisor call exception */
    DebugMonitor_Handler,         /*!< Debug Monitor */
    0,                            /*!< Reserved interrupt 13 */
    PendSV_Handler,               /*!< PendSV exception - request for system level service */
    SysTick_Handler,              /*!< SysTick interrupt */
                                  
    /* Interrupts */              
    DMA0_IRQHandler,              /*!< DMA channel 0 transfer complete interrupt */
    DMA1_IRQHandler,              /*!< DMA channel 1 transfer complete interrupt */
    DMA2_IRQHandler,              /*!< DMA channel 2 transfer complete interrupt */
    DMA3_IRQHandler,              /*!< DMA channel 3 transfer complete interrupt */
    DMA_Error_IRQHandler,         /*!< DMA error interrupt */
    Default_Handler,              /*!< Reserved interrupt 21 */
    FTFL_IRQHandler,              /*!< FTFL interrupt */
    Default_Handler,              /*!< Read collision interrupt */
    LVD_IRQHandler,               /*!< Low Voltage Detect, Low Voltage Warning */
    LLW_IRQHandler,               /*!< Low Leakage Wakeup */
    WDOG_IRQHandler,              /*!< WDOG interrupt */
    I2C0_IRQHandler,              /*!< I2C0 interrupt */
    SPI0_IRQHandler,              /*!< SPI0 interrupt */
    I2S0Tx_IRQHandler,            /*!< I2S0 transmit interrupt */
    I2S0Rx_IRQHandler,            /*!< I2S0 receive interrupt */
    UART0_LON_IRQHandler,         /*!< UART0 LON interrupt */
    UART0_IRQHandler,             /*!< UART0 receive/transmit interrupt */
    UART0_Error_IRQHandler,       /*!< UART0 error interrupt */
    UART1_IRQHandler,             /*!< UART1 receive/transmit interrupt */
    UART1_Error_IRQHandler,       /*!< UART1 error interrupt */
    UART2_IRQHandler,             /*!< UART2 receive/transmit interrupt */
    UART2_Error_IRQHandler,       /*!< UART2 error interrupt */
    ADC0_IRQHandler,              /*!< ADC0 interrupt */
    CMP0_IRQHandler,              /*!< CMP0 interrupt */
    CMP1_IRQHandler,              /*!< CMP1 interrupt */
    FTM0_IRQHandler,              /*!< FTM0 fault, overflow and channels interrupt */
    FTM1_IRQHandler,              /*!< FTM1 fault, overflow and channels interrupt */
    CMT_IRQHandler,               /*!< CMT interrupt */
    RTC_Alarm_IRQHandler,         /*!< RTC interrupt */
    RTC_Seconds_IRQHandler,       /*!< RTC seconds interrupt */
    PIT_Channel0_IRQHandler,      /*!< PIT timer channel 0 interrupt */
    PIT_Channel1_IRQHandler,      /*!< PIT timer channel 1 interrupt */
    PIT_Channel2_IRQHandler,      /*!< PIT timer channel 2 interrupt */
    PIT_Channel3_IRQHandler,      /*!< PIT timer channel 3 interrupt */
    PDB0_IRQHandler,              /*!< PDB0 interrupt */
    USB0_IRQHandler,              /*!< USB0 interrupt */
    USBDCD_IRQHandler,            /*!< USBDCD interrupt */
    TSI0_IRQHandler,              /*!< TSI0 interrupt */
    MCG_IRQHandler,               /*!< MCG interrupt */
    LPTimer_IRQHandler,           /*!< LPTimer interrupt */
    PORTA_IRQHandler,             /*!< Port A interrupt */
    PORTB_IRQHandler,             /*!< Port B interrupt */
    PORTC_IRQHandler,             /*!< Port C interrupt */
    PORTD_IRQHandler,             /*!< Port D interrupt */
    PORTE_IRQHandler,             /*!< Port E interrupt */
    SWI_IRQHandler,               /*!< Software interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
    Default_Handler,              /*!< Unused interrupt */
};
