/*
 * SPI.h
 *
 *  Created on: 07/08/2012
 *      Author: podonoghue
 */

#ifndef SPI_H_
#define SPI_H_

#if (CPU==JMxx)
#define SPIxC1_MSTR_MASK      SPI1C1_MSTR_MASK
#define SPIxC1                SPI1C1
#define SPIxC1_SPE_MASK       SPI1C1_SPE_MASK
#define SPIxC1_CPOL_MASK      SPI1C1_CPOL_MASK
#define SPIxC1_CPHA_MASK      SPI1C1_CPHA_MASK
#define SPIxC1_SSOE_MASK      SPI1C1_SSOE_MASK
#define SPIxC1_LSBFE_MASK     SPI1C1_LSBFE_MASK
#define SPIxC2_SPIMODE_MASK   SPI1C2_SPIMODE_MASK
#define SPIxC2_MODFEN_MASK    SPI1C2_MODFEN_MASK
#define SPIxS                 SPI1S
#define SPIxD                 SPI1D
#define SPIxDH                SPI1DH
#define SPIxDL                SPI1DL
#define SPIxD16               SPI1D16
#define SPIxC2                SPI1C2

#define SPIxBR_SPPR_BITNUM 	  SPI1BR_SPPR_BITNUM
#define SPIxBR_SPR_BITNUM  	  SPI1BR_SPR_BITNUM
#define SPIxBR 				  SPI1BR
#define SPIxD16 			  SPI1D16

#elif (CPU==JS16)
#define SPIxC1_MSTR_MASK      SPIC1_MSTR_MASK
#define SPIxC1                SPIC1
#define SPIxC1_SPE_MASK       SPIC1_SPE_MASK
#define SPIxC1_CPOL_MASK      SPIC1_CPOL_MASK
#define SPIxC1_CPHA_MASK      SPIC1_CPHA_MASK
#define SPIxC1_SSOE_MASK      SPIC1_SSOE_MASK
#define SPIxC1_LSBFE_MASK     SPIC1_LSBFE_MASK
#define SPIxC2_SPIMODE_MASK   SPIC2_SPIMODE_MASK
#define SPIxC2_MODFEN_MASK    SPIC2_MODFEN_MASK
#define SPIxS                 SPIS 
#define SPIxD                 SPID
#define SPIxDH                SPIDH
#define SPIxDL                SPIDL
#define SPIxD16               SPID16
#define SPIxC2                SPIC2

#define SPIxBR_SPPR_BITNUM 	  SPIBR_SPPR_BITNUM
#define SPIxBR_SPR_BITNUM  	  SPIBR_SPR_BITNUM
#define SPIxBR 				  SPIBR
#define SPIxD16 			  SPID16
#endif

#define SPIxC1_OFF   (SPIxC1_MSTR_MASK)  //!< SPI Masks - Mask to disable SPI

#define SPIS_SPTEF_BIT (5)
#define SPIS_SPRF_BIT  (7)

#pragma DATA_SEG __SHORT_SEG Z_PAGE
// MUST be placed into the direct segment (assumed in ASM code).
extern volatile U8 bitDelay;  //!< Required software delay used with SPI base Tx/Rx
extern volatile U8 rxTiming1; //!< bdm_Rx timing constant #1
extern volatile U8 txTiming1; //!< bdm_Tx timing constant #1
#pragma DATA_SEG DEFAULT

#define DEFAULT_SPI_FREQUENCY  (1000)    // 1000 kHz

U8 spi_setSpeed(U16 freq);

#endif /* SPI_H_ */
