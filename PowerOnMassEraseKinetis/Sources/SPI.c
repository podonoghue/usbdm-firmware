/*
 * SPI.c
 *
 *  Created on: 07/08/2012
 *      Author: podonoghue
 */
#include "Configure.h"
#include "Commands.h"
#include "Common.h"
#include "SPI.h"
#include "BDM.h"

#if (HW_CAPABILITY&(CAP_CFVx_HW|CAP_JTAG_HW|CAP_SWD_HW))

volatile uint8_t bitDelay;    //!< Required software delay used with SPI base Tx/Rx

static uint32_t ctar0Value = 0;
static uint32_t ctar1Value = 0;

uint32_t spiBaudValue  = 0;

//! Initialise SPI
//!
//! @param default ctar0 value for SPI.CTAR0 register
//! @param default ctar1 value for SPI.CTAR1 register
//!
//! @note a default frequency is used.
//!
void spi_init(uint32_t ctar0, uint32_t ctar1) {
   ctar0Value = ctar0;
   ctar1Value = ctar1;   
   // Configure SPI
   (void)spi_setSpeed(0);
   SPI0_MCR   = SPI_MCR_CLR_RXF_MASK|SPI_MCR_ROOE_MASK|SPI_MCR_CLR_TXF_MASK|SPI_MCR_PCSIS((1<<0)|(1<<1))|
                SPI_MCR_MSTR_MASK|SPI_MCR_FRZ_MASK|SPI_MCR_DCONF(0)|SPI_MCR_SMPL_PT(0);
}

typedef struct {
   uint32_t freq; 
   uint32_t ctarBaud; 
} Scidata;

static const Scidata spiBaudTable[] = {
     //  freq(kHz)                        ctarBaud
 /*  0 */ {12000, SPI_CTAR_PBR(0)|SPI_CTAR_BR(0)|SPI_CTAR_PCSSCK(0)|SPI_CTAR_CSSCK(0)},
 /*  1 */ { 8000, SPI_CTAR_PBR(1)|SPI_CTAR_BR(0)|SPI_CTAR_PCSSCK(0)|SPI_CTAR_CSSCK(1)},
 /*  2 */ { 6000, SPI_CTAR_PBR(0)|SPI_CTAR_BR(1)|SPI_CTAR_PCSSCK(0)|SPI_CTAR_CSSCK(1)},
 /*  3 */ { 4800, SPI_CTAR_PBR(2)|SPI_CTAR_BR(0)|SPI_CTAR_PCSSCK(1)|SPI_CTAR_CSSCK(0)},
 /*  4 */ { 4000, SPI_CTAR_PBR(1)|SPI_CTAR_BR(1)|SPI_CTAR_PCSSCK(1)|SPI_CTAR_CSSCK(0)},
 /*  5 */ { 3000, SPI_CTAR_PBR(0)|SPI_CTAR_BR(3)|SPI_CTAR_PCSSCK(0)|SPI_CTAR_CSSCK(2)},
 /*  6 */ { 2667, SPI_CTAR_PBR(1)|SPI_CTAR_BR(2)|SPI_CTAR_PCSSCK(2)|SPI_CTAR_CSSCK(0)},
 /*  7 */ { 2400, SPI_CTAR_PBR(2)|SPI_CTAR_BR(1)|SPI_CTAR_PCSSCK(2)|SPI_CTAR_CSSCK(0)},
 /*  8 */ { 2000, SPI_CTAR_PBR(1)|SPI_CTAR_BR(3)|SPI_CTAR_PCSSCK(1)|SPI_CTAR_CSSCK(1)},
 /*  9 */ { 1500, SPI_CTAR_PBR(0)|SPI_CTAR_BR(4)|SPI_CTAR_PCSSCK(0)|SPI_CTAR_CSSCK(3)},
 /* 10 */ { 1200, SPI_CTAR_PBR(2)|SPI_CTAR_BR(3)|SPI_CTAR_PCSSCK(2)|SPI_CTAR_CSSCK(1)},
 /* 11 */ { 1000, SPI_CTAR_PBR(1)|SPI_CTAR_BR(4)|SPI_CTAR_PCSSCK(1)|SPI_CTAR_CSSCK(2)},
 /* 12 */ {  857, SPI_CTAR_PBR(3)|SPI_CTAR_BR(3)|SPI_CTAR_PCSSCK(1)|SPI_CTAR_CSSCK(2)},
 /* 13 */ {  750, SPI_CTAR_PBR(0)|SPI_CTAR_BR(5)|SPI_CTAR_PCSSCK(2)|SPI_CTAR_CSSCK(2)},
 /* 14 */ {  600, SPI_CTAR_PBR(2)|SPI_CTAR_BR(4)|SPI_CTAR_PCSSCK(2)|SPI_CTAR_CSSCK(2)},
 /* 15 */ {  500, SPI_CTAR_PBR(1)|SPI_CTAR_BR(5)|SPI_CTAR_PCSSCK(1)|SPI_CTAR_CSSCK(3)},
 /* 16 */ {  429, SPI_CTAR_PBR(3)|SPI_CTAR_BR(4)|SPI_CTAR_PCSSCK(3)|SPI_CTAR_CSSCK(2)},
 /* 17 */ {  250, SPI_CTAR_PBR(1)|SPI_CTAR_BR(6)|SPI_CTAR_PCSSCK(1)|SPI_CTAR_CSSCK(4)},
 /* 18 */ {  125, SPI_CTAR_PBR(1)|SPI_CTAR_BR(7)|SPI_CTAR_PCSSCK(1)|SPI_CTAR_CSSCK(5)},
 /* 19 */ {  107, SPI_CTAR_PBR(3)|SPI_CTAR_BR(6)|SPI_CTAR_PCSSCK(3)|SPI_CTAR_CSSCK(4)},
};                                                                                 

//! Sets Communication speed for SWD, CF V2, 3 & 4 targets
//!
//! @param freq => Frequency in kHz (0 => use default value)
//!
//! @return  \ref BDM_RC_OK              => Success                 \n
//!          \ref BDM_RC_ILLEGAL_PARAMS  => Speed is not supported
//!
uint8_t spi_setSpeed(uint16_t freq) {
   if (freq == 0) {
      freq = DEFAULT_SPI_FREQUENCY;
   }
   unsigned index;
   for(index=0; index<sizeof(spiBaudTable)/sizeof(spiBaudTable[0]); index++) {
      if (spiBaudTable[index].freq <= freq) {
         break;
      }
   }
   if (index>=sizeof(spiBaudTable)/sizeof(spiBaudTable[0])) {
      return BDM_RC_ILLEGAL_PARAMS;
   }
   // Enable SPI module clock
   SPI_CLK_ENABLE();

   bitDelay                 = 0;
   spiBaudValue             = spiBaudTable[index].ctarBaud;

   spi_setCTAR0(ctar0Value);
   spi_setCTAR1(ctar1Value);

   return BDM_RC_OK;
}

#endif
