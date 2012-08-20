/*
 * SPI.c
 *
 *  Created on: 07/08/2012
 *      Author: podonoghue
 */
#include "Configure.h"
#include "Common.h"
#include "SPI.h"
#include "BDM.h"
#include "Commands.h"
#include "CmdProcessing.h"

#pragma DATA_SEG __SHORT_SEG Z_PAGE
// MUST be placed into the direct segment (assumed in ASM code).
volatile U8 bitDelay;    //!< Required software delay used with SPI base Tx/Rx
volatile U8 rxTiming1;   //!< bdm_Rx timing constant #1
volatile U8 txTiming1;   //!< bdm_Tx timing constant #1
#pragma DATA_SEG DEFAULT

#if (HW_CAPABILITY&(CAP_CFVx_HW|CAP_JTAG_HW|CAP_SWD_HW))

//! Structure to relate BDM Communication speed to SPI configuration value
typedef struct {
   U16 freq;         //!< Freq in kHz
   U8  spiValue;     //!< SPI baud value 
   U8  delayCount;   //!< count for start bit time
} SPISpeed;

//!  Table relating BDM Communication speed to SPI configuration value
//!  @note Assumes SPI Clock input is 24 MHz
static const SPISpeed SPISpeedValues[ ] = {
   // Freq in kHz           SPI Value                            // Divide by (SPPR+1) * 2**(SPR+1)
    { 12000, ((0<<SPIxBR_SPPR_BITNUM)|(0<<SPIxBR_SPR_BITNUM)),  1},  // = (0+1) * (2**(0+1)) =  /2 => 12   MHz
    {  6000, ((0<<SPIxBR_SPPR_BITNUM)|(1<<SPIxBR_SPR_BITNUM)),  1},  // = (0+1) * (2**(1+1)) =  /4 =>  6   MHz
    {  4000, ((2<<SPIxBR_SPPR_BITNUM)|(0<<SPIxBR_SPR_BITNUM)),  1},  // = (2+1) * (2**(0+1)) =  /6 =>  4   MHz
    {  3000, ((0<<SPIxBR_SPPR_BITNUM)|(2<<SPIxBR_SPR_BITNUM)),  1},  // = (0+1) * (2**(2+1)) =  /8 =>  3   MHz
    {  2000, ((2<<SPIxBR_SPPR_BITNUM)|(1<<SPIxBR_SPR_BITNUM)),  1},  // = (2+1) * (2**(1+1)) = /12 =>  2   MHz
    {  1500, ((0<<SPIxBR_SPPR_BITNUM)|(3<<SPIxBR_SPR_BITNUM)),  1},  // = (0+1) * (2**(3+1)) = /16 =>  1.5 MHz
    {  1000, ((2<<SPIxBR_SPPR_BITNUM)|(2<<SPIxBR_SPR_BITNUM)),  2},  // = (2+1) * (2**(2+1)) = /24 =>  1   MHz
    {   750, ((0<<SPIxBR_SPPR_BITNUM)|(4<<SPIxBR_SPR_BITNUM)),  3},  // = (0+1) * (2**(4+1)) = /32 =>  750 kHz
    {   500, ((2<<SPIxBR_SPPR_BITNUM)|(3<<SPIxBR_SPR_BITNUM)),  6},  // = (2+1) * (2**(3+1)) = /48 =>  500 kHz
    {   250, ((2<<SPIxBR_SPPR_BITNUM)|(4<<SPIxBR_SPR_BITNUM)),  10}, // = (2+1) * (2**(4+1)) = /96 =>  250 KHz
};

//! Sets Communication speed for SWD, CF V2, 3 & 4 targets
//!
//! @param freq => Frequency on kHz (0 => use default value)
//!
//! @return  \ref BDM_RC_OK              => Success                 \n
//!          \ref BDM_RC_ILLEGAL_PARAMS  => Speed is not supported
//!
U8 spi_setSpeed(U16 freq) {
int sub;

   if (freq == 0) {
      freq = DEFAULT_SPI_FREQUENCY;
   }
   for (sub = 0; sub<sizeof(SPISpeedValues)/sizeof(SPISpeedValues[0]); sub++) {
      if (SPISpeedValues[sub].freq <= freq) {
         SPIxBR                   = SPISpeedValues[sub].spiValue;
         bitDelay                 = SPISpeedValues[sub].delayCount;
         cable_status.sync_length = SPISpeedValues[sub].freq;
         return BDM_RC_OK;
      }
   }
   return BDM_RC_ILLEGAL_PARAMS;
}
#endif
