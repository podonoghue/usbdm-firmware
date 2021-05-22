/*
 *  @file security.c
 *  Derived from security-mk.c
 *
 *  Security and NV options for flash
 *  Reference K20P64M50SF0RM, K20P100M72SF1RM, K20P144M100SF2V2RM, K20P144M120SF3RM(no NMI_DIS),
 *            K22P121M120SF7RM(FAST_INIT)
 *  Created on: 20/5/2017
 *  Devices: Most early MK devices
 */
#include <stdint.h>
#include <string.h>
#include "derivative.h"

/*
 * Security information structure in flash memory
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
  <h> Flash Configuration - Security and Protection
     <i> 16-byte flash configuration field that stores default protection settings (loaded on reset)
     <i> and security information that allows the MCU to restrict access to the Flash module.
  </h>
*/
/*
   <h> Program Flash Region Protect
      <i> Each program flash region can be protected from program and erase operation by clearing the associated PROT bit.
      <i> Each bit protects a 1/32 region of the program flash memory.
      <info>NV_FPROT0-3
      <q.0>   FPROT3.0 <0=>protected  <1=>unprotected   <info>lowest 1/32 block or 1K min
      <q.1>   FPROT3.1 <0=>protected  <1=>unprotected
      <q.2>   FPROT3.2 <0=>protected  <1=>unprotected
      <q.3>   FPROT3.3 <0=>protected  <1=>unprotected
      <q.4>   FPROT3.4 <0=>protected  <1=>unprotected
      <q.5>   FPROT3.5 <0=>protected  <1=>unprotected
      <q.6>   FPROT3.6 <0=>protected  <1=>unprotected
      <q.7>   FPROT3.7 <0=>protected  <1=>unprotected
      <q.8>   FPROT2.0 <0=>protected  <1=>unprotected
      <q.9>   FPROT2.1 <0=>protected  <1=>unprotected
      <q.10>  FPROT2.2 <0=>protected  <1=>unprotected
      <q.11>  FPROT2.3 <0=>protected  <1=>unprotected
      <q.12>  FPROT2.4 <0=>protected  <1=>unprotected
      <q.13>  FPROT2.5 <0=>protected  <1=>unprotected
      <q.14>  FPROT2.6 <0=>protected  <1=>unprotected
      <q.15>  FPROT2.7 <0=>protected  <1=>unprotected
      <q.16>  FPROT1.0 <0=>protected  <1=>unprotected
      <q.17>  FPROT1.1 <0=>protected  <1=>unprotected
      <q.18>  FPROT1.2 <0=>protected  <1=>unprotected
      <q.19>  FPROT1.3 <0=>protected  <1=>unprotected
      <q.20>  FPROT1.4 <0=>protected  <1=>unprotected
      <q.21>  FPROT1.5 <0=>protected  <1=>unprotected
      <q.22>  FPROT1.6 <0=>protected  <1=>unprotected
      <q.23>  FPROT1.7 <0=>protected  <1=>unprotected
      <q.24>  FPROT0.0 <0=>protected  <1=>unprotected
      <q.25>  FPROT0.1 <0=>protected  <1=>unprotected
      <q.26>  FPROT0.2 <0=>protected  <1=>unprotected
      <q.27>  FPROT0.3 <0=>protected  <1=>unprotected
      <q.28>  FPROT0.4 <0=>protected  <1=>unprotected
      <q.29>  FPROT0.5 <0=>protected  <1=>unprotected
      <q.30>  FPROT0.6 <0=>protected  <1=>unprotected
      <q.31>  FPROT0.7 <0=>protected  <1=>unprotected   <info> highest 1/32 block or 1K min
   </h>
*/
#define FPROT_VALUE 0xFFFFFFFF
/*
   <h> EEPROM Region Protect
      <i> Each bit protects a 1/8 region of the EEPROM memory.
      <i> (FlexNVM devices only)
	  <i> Only available on devices with FlexNVM
      <info>NV_FEPROT
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
   <h> Data Flash Region Protect
      <i> Each bit protects a 1/8 region of the flash memory.
      <i> (Device with Data flash only)
	  <i> Only available on devices with FlexNVM
      <info>NV_FDPROT
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
<h> Flash security value
   <info>NV_FSEC
   <o0> Backdoor Key Security Access Enable
      <i> Controls use of Backdoor Key access to unsecure device
      <info>[7,6] KEYEN
      <2=> 2: Access enabled
      <3=> 3: Access disabled
   <o1> Mass Erase Enable Bits
      <i> Controls mass erase capability of the flash memory module.
      <i> Only relevant when FSEC.SEC is set to secure.
      <info>[5,4] MEEN
      <2=> 2: Mass erase disabled
      <3=> 3: Mass erase enabled
   <o2> Freescale Failure Analysis Access
      <i> Controls access to the flash memory contents during returned part failure analysis
      <info>[3,2] FSLACC
      <2=> 2: Factory access denied
      <3=> 3: Factory access granted
   <o3> Flash Security
      <i> Defines the security state of the MCU.
      <i> In the secure state, the MCU limits access to flash memory module resources.
      <i> If the flash memory module is unsecured using backdoor key access, SEC is forced to 10b.
      <info>[1,0] SEC
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
<h> Flash boot options
   <info>NV_FOPT

   <q2.5> Fast initialisation control
      <i> Selects initialization speed on POR, VLLSx, and system reset.
	  <i> Not all devices have this feature
	  <info>[5] FAST_INIT
      <0=> Slow - Slower initialization and reduced average current.
      <1=> Fast - Faster initialization and higher average current.
   <q2.2> NMI pin control
      <i> Enables or disables the NMI function
	  <i> Not all devices have this feature
      <info>[2] NMI_DIS
      <0=> NMI interrupts are always blocked.
      <1=> NMI interrupts default to enabled
   <q2.1> EZPORT pin control
      <i> Enables or disables EzPort function
      <i> Disabling EZPORT function avoids inadvertent resets into EzPort mode 
      <i> if the EZP_CS/NMI pin is used for its NMI function 
      <info>[1] EZPORT_DIS
      <0=> EZP_CSn pin is disabled on reset
      <1=> EZP_CSn pin is enabled on reset
   <q2.0> Low power boot control
      <i> Controls the reset value of SIM_CLKDIV1.OUTDIVx (clock dividers)
      <i> Allows power consumption during reset to be reduced
      <info>[0] LPBOOT
      <0=> Low Power
      <1=> Normal
</h>
 */
#define RESERVED1      (0)      // -0
#define RESERVED2      (0)      // -1
#define FOPT_MISC      (0x27)   // q2
#define FOPT_RESERVED  (0b11011000)   // Bits not controlled by above

#define FOPT_VALUE (FOPT_RESERVED|FOPT_MISC)

/*
  <h> Backdoor Comparison Key
  <i> The Verify Backdoor Access Key command releases security if user-supplied keys
  <i> matches the Backdoor Comparison Key bytes
  <info>NV_BACKDOOR
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
#define BACKDOOR_VALUE { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, }

__attribute__ ((section(".security_information")))
const SecurityInfo securityInfo = {
    /* backdoor */ BACKDOOR_VALUE,
    /* fprot    */ FPROT_VALUE,
    /* fsec     */ FSEC_VALUE,
    /* fopt     */ FOPT_VALUE,
    /* feprot   */ FEPROT_VALUE,
    /* fdprot   */ FDPROT_VALUE,
};

