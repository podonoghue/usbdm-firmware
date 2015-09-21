/*
 *  Vectors-mkl.c
 *
 *  Generic vectors and security for Kinetis MKLxx
 *
 *  Created on: 07/12/2012
 *      Author: podonoghue
 */
#include <stdint.h>
#include <string.h>
#include "derivative.h"

#define MKL25Z4

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
   <e0> Boot ROM Options
      <i> Only available on devices with internal ROM
      <i> Currently: KL03,KL17,KL27,KL43
      <0=> Disabled
      <1=> Enabled
   <o1> Boot Source Selection (FOPT.BOOTSRC_SEL)
      <i> These bits select the boot sources if FOPT.BOOTPIN_OPT = 1
      <0=> 0: Boot from Flash
      <64=> 1: Reserved
      <128=> 2: Boot from ROM
      <192=> 3: Boot from ROM
   <q2.1> External pin selects boot options (FOPT.BOOTPIN_OPT)
      <i> Enables or disables the RESET pin dedicated operation
      <i> Note: RESET pin must be enabled if BOOTCFG0 is used.
      <0=> Boot from ROM if BOOTCFG0 (NMI pin) asserted. 
      <1=> Boot source controlled by BOOTSRC_SEL
   </e>

   <q2.5> Flash initialisation speed (FOPT.FAST_INIT)
      <i> Selects initialization speed on POR, VLLSx, and system reset.
      <0=> Slower (reduced average power)
      <1=> Faster (higher average power)
   <q2.3> RESET pin control (FOPT.RESET_PIN_CFG)
      <i> Enables or disables the RESET pin dedicated operation
      <0=> Disabled (available as port pin)
      <1=> Enabled (PUP, open-drain, filtered)
   <q2.2> NMI pin control (FOPT.NMI_DIS)
      <i> Enables or disables control for the NMI function
      <0=> NMI interrupts are always blocked.
      <1=> NMI_b interrupts default to enabled
   <o3> Low power boot control (FOPT.LPBOOT)
      <i> Controls the reset value of SIM_CLKDIV1.OUTDIV1 (clock divider) and
      <i> SMC_PMCTRL.RUNM (processor run mode)
      <i> Note: VLPR is only used on KL03,KL17,KL27,KL43
      <0=> OUTDIV1 = /8, RUNM = VLPR
      <1=> OUTDIV1 = /4, RUNM = VLPR
      <16=> OUTDIV1 = /2, RUNM = RUN
      <17=> OUTDIV1 = /1, RUNM = RUN
</h>
 */
#define BOOT_ENABLE  (0)
#define FOPT_BOOTSRC (0xC0)
#define FOPT_MISC    (0x2E)
#define FOPT_LPBOOT  (0x11)

#if defined(NV_FOPT_BOOTSRC_SEL) && BOOT_ENABLE
#define FOPT_VALUE (FOPT_BOOTSRC|FOPT_MISC|FOPT_LPBOOT)
#else
#define FOPT_VALUE (0xC2|FOPT_MISC|FOPT_LPBOOT)
#endif

__attribute__ ((section(".security_information")))
const SecurityInfo securityInfo = {
    /* backdoor */ {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
    /* fprot    */ 0xFFFFFFFF,
    /* fsec     */ FSEC_VALUE,
    /* fopt     */ FOPT_VALUE,
    /* feprot   */ 0xFF,
    /* fdprot   */ 0xFF,
};

#if defined(NV_FOPT_BOOTPIN_OPT_MASK)
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
void SVC_Handler(void)                        WEAK_DEFAULT_HANDLER;
void PendSV_Handler(void)                     WEAK_DEFAULT_HANDLER;
void SysTick_Handler(void)                    WEAK_DEFAULT_HANDLER;
void DMA0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA2_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DMA3_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void FTFA_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void LVD_LVW_IRQHandler(void)                 WEAK_DEFAULT_HANDLER;
void LLWU_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void I2C0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void I2C1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void SPI0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void SPI1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void UART0_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void UART1_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void UART2_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void ADC0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void CMP0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void TPM0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void TPM1_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void TPM2_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void RTC_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void RTC_Seconds_IRQHandler(void)             WEAK_DEFAULT_HANDLER;
void PIT_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void USB0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void DAC0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void TSI0_IRQHandler(void)                    WEAK_DEFAULT_HANDLER;
void MCG_IRQHandler(void)                     WEAK_DEFAULT_HANDLER;
void LPTMR0_IRQHandler(void)                  WEAK_DEFAULT_HANDLER;
void PORTA_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;
void PORTD_IRQHandler(void)                   WEAK_DEFAULT_HANDLER;

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
      0,                             /*    4,  -12                                                                                   */
      0,                             /*    5,  -11                                                                                   */
      0,                             /*    6,  -10                                                                                   */
      0,                             /*    7,   -9                                                                                   */
      0,                             /*    8,   -8                                                                                   */
      0,                             /*    9,   -7                                                                                   */
      0,                             /*   10,   -6                                                                                   */
      SVC_Handler,                   /*   11,   -5  System Service Call via SVC instruction                                          */
      0,                             /*   12,   -4                                                                                   */
      0,                             /*   13,   -3                                                                                   */
      PendSV_Handler,                /*   14,   -2  Pendable request for system service                                              */
      SysTick_Handler,               /*   15,   -1  System Tick Timer                                                                */

                                     /* External Interrupts */
      DMA0_IRQHandler,               /*   16,    0  DMA0 Transfer complete or error                                                  */
      DMA1_IRQHandler,               /*   17,    1  DMA1 Transfer complete or error                                                  */
      DMA2_IRQHandler,               /*   18,    2  DMA2 Transfer complete or error                                                  */
      DMA3_IRQHandler,               /*   19,    3  DMA3 Transfer complete or error                                                  */
      Default_Handler,               /*   20,    4                                                                                   */
      FTFA_IRQHandler,               /*   21,    5  FTFA Command complete or error                                                   */
      LVD_LVW_IRQHandler,            /*   22,    6  PMC Low-voltage detect, low-voltage warning                                      */
      LLWU_IRQHandler,               /*   23,    7  Low Leakage Wakeup                                                               */
      I2C0_IRQHandler,               /*   24,    8  I2C Interface 0                                                                  */
      I2C1_IRQHandler,               /*   25,    9  I2C Interface 1                                                                  */
      SPI0_IRQHandler,               /*   26,   10  Serial Peripheral Interface 0                                                    */
      SPI1_IRQHandler,               /*   27,   11  Serial Peripheral Interface 1                                                    */
      UART0_IRQHandler,              /*   28,   12  UART0 Status and error                                                           */
      UART1_IRQHandler,              /*   29,   13  UART1 Status and error                                                           */
      UART2_IRQHandler,              /*   30,   14  UART2 Status and error                                                           */
      ADC0_IRQHandler,               /*   31,   15  Analogue to Digital Converter 0                                                  */
      CMP0_IRQHandler,               /*   32,   16  Comparator 0                                                                     */
      TPM0_IRQHandler,               /*   33,   17  Timer/PWM Module 0                                                               */
      TPM1_IRQHandler,               /*   34,   18  Timer/PWM Module 1                                                               */
      TPM2_IRQHandler,               /*   35,   19  Timer/PWM Module 2                                                               */
      RTC_IRQHandler,                /*   36,   20  Real Time Clock Alarm                                                            */
      RTC_Seconds_IRQHandler,        /*   37,   21  Real Time Clock Seconds                                                          */
      PIT_IRQHandler,                /*   38,   22  Programmable Interrupt Timer (All channels)                                      */
      Default_Handler,               /*   39,   23                                                                                   */
      USB0_IRQHandler,               /*   40,   24  USBB On The Go                                                                   */
      DAC0_IRQHandler,               /*   41,   25  Digital to Analogue Converter                                                    */
      TSI0_IRQHandler,               /*   42,   26  Touch Sense Input                                                                */
      MCG_IRQHandler,                /*   43,   27  Clock interrupt                                                                  */
      LPTMR0_IRQHandler,             /*   44,   28  Low Power Timer                                                                  */
      Default_Handler,               /*   45,   29                                                                                   */
      PORTA_IRQHandler,              /*   46,   30  Port A                                                                           */
      PORTD_IRQHandler,              /*   47,   31  Port D                                                                           */
   }
};



