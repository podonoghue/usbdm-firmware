/*
 *  @file security.c
 *  Derived from  security-mke02.c
 *
 *  Security and NV options for flash with boot options
 *  Reference: MKE02P64M40SF0RM
 *  Created on: 20/5/2017
 *  Devices: MKE02
 */
#include <stdint.h>
#include <string.h>
#include "derivative.h"

/*
 * Security information structure in flash memory
 */
typedef struct {
    uint8_t  backdoorKey[8];
    uint32_t reserved;
    uint8_t  eeprot;
    uint8_t  fprot;
    uint8_t  fsec;
    uint8_t  fopt;
} SecurityInfo;

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
/*
  <h> Flash Configuration - Security and Protection
     <i> 16-byte flash configuration field that stores default protection settings (loaded on reset)
     <i> and security information that allows the MCU to restrict access to the Flash module.
  </h>
*/
/*
   <!e0.7> EEPROM memory protection
      <i> Enables EEPROM protection
      <info>NV_EEPROT.DPOPEN
         <0=>Protected
         <1=>Unprotected
      <o1>   EEPROM Protection Size
      <i>Determines the size of the protected area in the EEPROM memory.
      <info>NV_EEPROT.DPS
        <0=>0: 32 bytes
        <1=>1: 64 bytes
        <2=>2: 96 bytes
        <3=>3: 128 bytes
        <4=>4: 160 bytes
        <5=>5: 192 bytes
        <6=>6: 224 bytes
        <7=>7: 256 bytes
   </h>
*/
#define EEPROT_VALUE (0x80|0x7|0x78)

/*
   <h> Program Flash Region Protect
      <i>Allows ranges of the flash to be protected
      <info>NV_FPROT
	  <q0.7>Flash Protection Operation Select
         <i>Determines the protection function.
         <i>The ranges may represent protected or unprotected ranges
         <info>FPOPEN
         <0=> 0: FPHDIS+FPHS and FPLDIS+FPLS define unprotected ranges
         <1=> 1: FPHDIS+FPHS and FPLDIS+FPLS define protected ranges
	  <o1>Flash Protection Higher Address Size
	     <i>Determines the size of the protected/unprotected area in
	     <i>high flash memory ending at global address 0x7FFF
	     <info>FPHDIS+FPHS
         <0x38=> 0 KB - Disabled
         <0x00=> 1 KB
         <0x08=> 2 KB
         <0x10=> 4 KB
         <0x18=> 8 KB
	  <o2>Flash Protection Lower Address Size
	     <i>Determines the size of the protected/unprotected area in
	     <i>low flash memory starting at global address 0x0000
	     <info>FPLDIS+FPLS
         <0x7=> 0 KB - Disabled
         <0x0=> 2 KB
         <0x1=> 4 KB
         <0x2=> 8 KB
         <0x3=> 16 KB
   </h>
*/
#define FPROT_VALUE (0x80|0x38|0x7|0x40)

/*
<h> Flash security value
   <info>NV_FSEC
   <o0> Backdoor Key Security Access Enable
      <i> Controls use of Backdoor Key access to unsecure device
      <info>KEYEN
      <2=> 2: Access enabled
      <3=> 3: Access disabled
   <o1> Mass Erase Enable Bits
      <i> Controls mass erase capability of the flash memory module.
      <i> Only relevant when FSEC.SEC is set to secure.
      <info>MEEN
      <2=> 2: Mass erase disabled
      <3=> 3: Mass erase enabled
   <o2> Freescale Failure Analysis Access
      <i> Controls access to the flash memory contents during returned part failure analysis
      <info>FSLACC
      <2=> 2: Factory access denied
      <3=> 3: Factory access granted
   <o3> Flash Security
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
   <o> Non-volatile byte<0x0-0xFF>
   <i> Flash non-volatile byte
   <i> This byte is copied to the FOPT register in  the flash controller during reset
   <info>FOPT
 */
#define FOPT_VALUE (0xFF)

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
    /* -        */ 0xFFFFFFFFUL,
    /* eeprot   */ EEPROT_VALUE,
    /* fprot    */ FPROT_VALUE,
    /* fsec     */ FSEC_VALUE,
    /* fopt     */ FOPT_VALUE,
};
