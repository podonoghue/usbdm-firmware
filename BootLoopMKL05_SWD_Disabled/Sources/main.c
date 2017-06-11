/*
 ============================================================================
 * main.c
 *
 *  Created on: 04/12/2012
 *      Author: podonoghue
 ============================================================================
 */
#include <stdio.h>
#include "system.h"
#include "derivative.h"
#include "utilities.h"
#include "leds.h"
#include "stdbool.h"
#include "console.h"

/*
 * Use PTB13 (= A5 on FRDM board) as self erase pin
 */
#define SELF_ERASE_PORT             B                    //!< Pin register
#define SELF_ERASE_NUM              13                   //!< Pin number
#define SELF_ERASE_MASK             (1<<SELF_ERASE_NUM)  //!< Mask for pin

// These error numbers are just for debugging
typedef enum {
     FLASH_ERR_OK                = (0),
     FLASH_ERR_LOCKED            = (1),  // Flash is still locked
     FLASH_ERR_ILLEGAL_PARAMS    = (2),  // Parameters illegal
     FLASH_ERR_PROG_FAILED       = (3),  // STM - Programming operation failed - general
     FLASH_ERR_PROG_WPROT        = (4),  // STM - Programming operation failed - write protected
     FLASH_ERR_VERIFY_FAILED     = (5),  // Verify failed
     FLASH_ERR_ERASE_FAILED      = (6),  // Erase or Blank Check failed
     FLASH_ERR_TRAP              = (7),  // Program trapped (illegal instruction/location etc.)
     FLASH_ERR_PROG_ACCERR       = (8),  // Kinetis/CFVx - Programming operation failed - ACCERR
     FLASH_ERR_PROG_FPVIOL       = (9),  // Kinetis/CFVx - Programming operation failed - FPVIOL
     FLASH_ERR_PROG_MGSTAT0      = (10), // Kinetis - Programming operation failed - MGSTAT0
     FLASH_ERR_CLKDIV            = (11), // CFVx - Clock divider not set
     FLASH_ERR_ILLEGAL_SECURITY  = (12), // Kinetis/CFV1+ - Illegal value for security location
     FLASH_ERR_UNKNOWN           = (13)  // Unspecified error
} FlashDriverError_t;

#pragma pack(push,1)
typedef struct {
   uint8_t  FSTAT;
   uint8_t  FCNFG;
   uint8_t  FSEC;
   uint8_t  FOPT;
   uint32_t FCCOB0_3;
   uint32_t FCCOB4_7;
   uint32_t FCCOB8_B;
   uint32_t FPROT0_3;
   uint8_t  FEPROT;
   uint8_t  FDPROT;
} FlashController;
#pragma pack(pop)

#define flashController ((FlashController *)FTFA_BasePtr)

#define F_ERASE_ALL_BLOCKS (0x44)

bool     doMassErase = false;
uint32_t errorCode   = 0;

/*
 * Simple delay - not for real programs!
 */
void delay(void) {
   volatile unsigned long i;
   for (i=400000; i>0; i--) {
      __asm__("nop");
      if ((GPIOB->PDIR&SELF_ERASE_MASK) == 0) {
         doMassErase = true;
      }
   }
}

void setErrorCode(uint32_t code) {
   errorCode = code;
   for(;;) {
      printf("Error code = %ld\n", errorCode);
   }
}

__attribute__ ((section(".ram_code")))
/**
 * Execute a flash command
 *
 * @note this executes in RAM
 */
void executeCommand() {
   // Clear any existing errors
   flashController->FSTAT = FTFA_FSTAT_ACCERR_MASK|FTFA_FSTAT_FPVIOL_MASK;

   // Launch command
   flashController->FSTAT = FTFA_FSTAT_CCIF_MASK;

   // Wait for command complete
   while ((flashController->FSTAT & FTFA_FSTAT_CCIF_MASK) == 0) {
   }
   // Handle any errors
   if ((flashController->FSTAT & FTFA_FSTAT_FPVIOL_MASK ) != 0) {
      setErrorCode(FLASH_ERR_PROG_FPVIOL);
   }
   if ((flashController->FSTAT & FTFA_FSTAT_ACCERR_MASK ) != 0) {
      setErrorCode(FLASH_ERR_PROG_ACCERR);
   }
   if ((flashController->FSTAT & FTFA_FSTAT_MGSTAT0_MASK ) != 0) {
      setErrorCode(FLASH_ERR_PROG_MGSTAT0);
   }
}

/**
 * Mass erase chip
 *
 * @note this executes in RAM and doesn't return as the caller will have gone away!
 */
__attribute__ ((section(".ram_code")))
void massErase() {
   // Un-protect flash
   flashController->FPROT0_3 = 0xFFFFFFFF;
   flashController->FCCOB0_3 = (F_ERASE_ALL_BLOCKS << 24) | 0;
   executeCommand();
   for(;;) {
   }
}

static const char bannerMessage[] =
   "The target chip on this board has been set up to disable it's SWD interface\n"
   "This is necessary to allow programming of an external device\n"
   "This disables programming of the target chip!!!\n"
   "To restore programming the chip must be erased\n"
   "\n\nPress 'y' or pull PTB13 (=A5) low to erase chip...\n\n\n";

int main(void) {

   led_initialise();
   greenLedOn();

   // Use PTB13 (= A5 on FRDM board) as self erase pin
   SIM->SCGC5                             |=  PORT_CLOCK_MASK(SELF_ERASE_PORT);
   PCR(SELF_ERASE_PORT, SELF_ERASE_NUM)    = PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
   PDDR(SELF_ERASE_PORT)                  &= ~SELF_ERASE_MASK;

   printf(bannerMessage);

   // Wait a while for user to select erase (pull A5 low)
   for(int i=0; i<20; i++) {
      greenLedToggle();
      delay();
      if (console_rxChar() == 'y') {
         doMassErase = true;
      }
      if (doMassErase) {
         printf("\n\nDoing mass erase\n");
         printf("Wait a while and then remove and replace board to complete unlock\n");
         massErase();
      }
   }

   greenLedOff();

   printf("Disabling SWD\n");
   // Disable SWD
   PORTA->PCR[0] = PORT_PCR_MUX(0); // Disable SWD_CLK
   PORTA->PCR[1] = PORT_PCR_MUX(0); // Disable RESET
   PORTA->PCR[2] = PORT_PCR_MUX(0); // Disable SWD_DIO
   printf("Disabled SWD\n");
   for(;;) {
   }
   return 0;
}
