/*! \file
    \brief Target processor definitions.
    
    This file contains the definitions of registers and bit masks for the various target processors.
    The usual header files cannot be used as the "target" processor isn't the one being compiled for!
*/    

#ifndef _TARGET_DEFINES_H_
#define _TARGET_DEFINES_H_

//=======================================================================
// HCS08
//=======================================================================

// HCS08 Registers addresses
//===============================
//#define HCS08_SRS            (0x1800) //!< HCS08 SRS address
#define HCS08_SBDFR_DEFAULT  (0x1801) //!< Default HCS08 SBDFR address

// HCS08 Register bit masks
//===============================
#define HCS_SBDFR_BDFR (0x01) //!< HCS08 SBDFR BDFR mask

// HC08 BDCSCR register masks 
//===============================
#define HC08_BDCSCR_ENBDM  (0x80) //!< BDCSCR Enable BDM mask
#define HC08_BDCSCR_BDMACT (0x40) //!< BDCSCR BDM Active (target halted) mask
#define HC08_BDCSCR_BKPTEN (0x20) //!< BDCSCR Breakpoint enable
#define HC08_BDCSCR_FTS    (0x10) //!< BDCSCR Breakpoint tag instruction
#define HC08_BDCSCR_CLKSW  (0x08) //!< BDCSCR BDM Clock select mask
#define HC08_BDCSCR_WS     (0x04) //!< BDCSCR CPU in wait/stop (or was before BACKGROUND cmd)
#define HC08_BDCSCR_WSF    (0x04) //!< BDCSCR Memory access failed due to Wait or Stop entry
#define HC08_BDCSCR_DVF    (0x04) //!< BDCSCR Data Valid Failure - memory access failed due to slow memory access conflict 

//=======================================================================
// RS08
//=======================================================================

// RS08 Flash page size
//=======================================================================
#define RS08_FLASH_PAGE_SIZE (64)

// RS08 BDCSCR register masks
#define RS08_BDCSCR_ENBDM  (0x80) //!< BDCSCR Enable BDM mask
#define RS08_BDCSCR_BDMACT (0x40) //!< BDCSCR BDM Active (target halted) mask
#define RS08_BDCSCR_BKPTEN (0x20) //!< BDCSCR Breakpoint enable
#define RS08_BDCSCR_FTS    (0x10) //!< BDCSCR Breakpoint tag instruction
#define RS08_BDCSCR_CLKSW  (0x08) //!< BDCSCR BDM Clock select mask
#define RS08_BDCSCR_WS     (0x04) //!< BDCSCR
#define RS08_BDCSCR_WSF    (0x02) //!< BDCSCR

// RS08 FLCR register masks
//===============================
#define RS08_FLCR_PGM     (1<<0)
#define RS08_FLCR_MASS    (1<<2)
#define RS08_FLCR_HVEN    (1<<3)

// RS08 SOPT register masks
//===============================
#define RS08_SOPT_COPE    (1<<7) //!< SOPT COP Enable
#define RS08_SOPT_COPT    (1<<6) //!< SOPT COP Time
#define RS08_SOPT_STOPE   (1<<5) //!< SOPT STOP instruction enable
#define RS08_SOPT_BKGDPE  (1<<1) //!< SOPT COP BKGD Pin enable
#define RS08_SOPT_RSTPE   (1<<0) //!< SOPT COP RESET Pin enable

// RS08 Common Registers addresses
//===============================
#define RS_PAGESEL         (0x001F) //!< Page select
#define RS_PAGEWIN         (0x00C0) //!< Paging Window (0xC0-0xFF)
#define RS_FAST_RAM        (0x00)   //!< Fast scratch RAM (0x00-0x0D)
#define RS_TRIM_FLASH      (0x3ffa) //!< ICS Trim value in flash (copied to RS_ICSTRM)
#define RS_TRIM_FLASH2     (0x3ffb) //!< ICS Trim value LSB in flash (copied to RS_ICSSC)

// RS08KA2/KA1 Registers addresses
//===============================
#define RS08KA2_DIR_RAM     (0x20)   //!< RS08KA2/1 Direct page RAM (0x20...)
#define RS08KA2_DIR_RAM_END (0x4F)   //!< RS08KA2/1 Direct page RAM (0x20...)
#define RS08KA2_ICSC1       (0x0014) //!< RS08KA2/1 Clock select
#define RS08KA2_ICSC2       (0x0015) //!< RS08KA2/1 Clock select
#define RS08KA2_ICSTRM      (0x0016) //!< RS08KA2/1 ICS Trim Value
#define RS08KA2_ICSSC       (0x0017) //!< RS08KA2/1 ICS Trim LSB
#define RS08KA2_FLCR        (0x0211) //!< RS08KA2/1 Flash Control Register
#define RS08KA2_SDID        (0x0206) //!< RS08KA2 System Device ID Reg address
#define RS08KA2_SDIDValue   (0x0800) //!< RS08KA2 System Device ID value
#define RS08KA2_SOPT        (0x0201) //!< RS08KA2 System Option reg address

// RS08KA8/4 Registers addresses
//===============================
#define RS08KA8_DIR_RAM     (0x30)   //!< RS08KA8/4 Direct page RAM (0x30...)
#define RS08KA8_DIR_RAM_END (0x9F)   //!< RS08KA8/4 Direct page RAM (0x20...)
#define RS08KA8_ICSC1       (0x023C) //!< RS08KA8/4 Bus Freq. & Low power select
#define RS08KA8_ICSC2       (0x023D) //!< RS08KA8/4 Bus Freq. & Low power select
#define RS08KA8_ICSTRM      (0x023E) //!< RS08KA8/4 ICS Trim Value
#define RS08KA8_ICSSC       (0x023F) //!< RS08KA8/4 ICS Trim LSB
#define RS08KA8_FLCR        (0x0211) //!< RS08KA8/4 Flash Control Register
#define RS08KA8_SDID        (0x0206) //!< RS08KA8/4 System Device ID Reg address
#define RS08KA8_SDIDValue   (0x0803) //!< RS08KA8/4 System Device ID value
#define RS08KA8_SOPT        (0x0201) //!< RS08KA8/4 System Option reg address

// RS08LA8 Registers addresses
//===============================
#define RS08LA8_DIR_RAM     (0x50)   //!< RS08LA8 Direct page RAM (0x50...)
#define RS08LA8_DIR_RAM_END (0xBF)   //!< RS08LA8 Direct page RAM (0x50...)
#define RS08LA8_ICSC1       (0x022C) //!< RS08LA8 Bus Freq. & Low power select
#define RS08LA8_ICSC2       (0x022D) //!< RS08LA8 Bus Freq. & Low power select
#define RS08LA8_ICSTRM      (0x022E) //!< RS08LA8 ICS Trim Value
#define RS08LA8_ICSSC       (0x022F) //!< RS08LA8 ICS Trim LSB
#define RS08LA8_FLCR        (0x0251) //!< RS08LA8 Flash Control Register
#define RS08LA8_SDID        (0x0216) //!< RS08LA8 System Device ID Reg address
#define RS08LA8_SDIDValue   (0x0804) //!< RS08LA8 System Device ID value
#define RS08LA8_SOPT        (0x0019) //!< RS08LA8 System Option reg address

// RS08LE4 Registers addresses
//===============================
#define RS08LE4_DIR_RAM     (0x50)   //!< RS08LE4 Direct page RAM (0x50...)
#define RS08LE4_DIR_RAM_END (0xBF)   //!< RS08LE4 Direct page RAM (0x50...)
#define RS08LE4_ICSC1       (0x022C) //!< RS08LE4 Bus Freq. & Low power select
#define RS08LE4_ICSC2       (0x022D) //!< RS08LE4 Bus Freq. & Low power select
#define RS08LE4_ICSTRM      (0x022E) //!< RS08LE4 ICS Trim Value
#define RS08LE4_ICSSC       (0x022F) //!< RS08LE4 ICS Trim LSB
#define RS08LE4_FLCR        (0x023D) //!< RS08LE4 Flash Control Register
#define RS08LE4_SDID        (0x021A) //!< RS08LE4 System Device ID Reg address
#define RS08LE4_SDIDValue   (0x0805) //!< RS08LE4 System Device ID value
#define RS08LE4_SOPT        (0x0019) //!< RS08LE4 System Option reg address

// ICSC1
#define RS08KA8_ICSC1_IREFS  (1<<2) //!< RS08 ICSC1 IREFS mask

// ICSC2
#define RS08_ICSV2_BDIV2  (1<<6) //!< RS08 ICSV2 Clock divide select mask

// RS08 Macros
//=================
#define RS08_PAGENO(addr)   (((addr)>>6) & 0xFF)       //!< Value for PAGEWINDOW from address
#define RS08_WIN_ADDR(addr) (RS_PAGEWIN+(addr & 0x3F)) //!< Value for offset within window, includes PAGEWIN base address

// RS08 Register bit masks
//===============================
// FLCR
#define RS_HVEN (0x08)  //!< RS08 FLCR High voltage enable mask
#define RS_MASS (0x04)  //!< RS08 FLCR Mass Erase mask
#define RS_PGM  (0x01)  //!< RS08 FLCR Program mask

//=======================================================================
// HC12
//=======================================================================

// HC12 Register addresses (BDM Space)
#define HC12_BDMSTS     0xFF01	//!< Address of HC12 BDM Status register
#define HC12_BDMCCR     0xFF06	//!< Address of CCR register in BDM memory space
#define HC12_BDMPPR     0xFF08	//!< Address of HC12 BDM Status register

// HC12 BDMPPR register masks 
#define HC12_BDMPPR_BPAE   (0x80) //!< enable BDMPPR function

// HC12 BDMSTS register masks 
#define HC12_BDMSTS_ENBDM  (0x80) //!< HC12 BDMSTS Enable BDM mask
#define HC12_BDMSTS_BDMACT (0x40) //!< HC12 BDMSTS BDM Active (target hakted) mask
#define HC12_BDMSTS_CLKSW  (0x04) //!< HC12 BDMSTS BDM Clock select mask
#define HC12_BDMSTS_UNSEC  (0x02) //!< HC12 BDMSTS BDM Clock select mask

#define HCS12_PARTID       (0x001A) //!< HCS12 PARTID register address

//=======================================================================
// Coldfire V1
//=======================================================================

// Coldfire V1 XCSR
#define CFV1_XCSR_RUNSTATE      (0xC0) //!< CFv1 XCSR Halt&Stop masks
#define CFV1_XCSR_HALT          (0x80) //!< CFv1 XCSR Halt mask
#define CFV1_XCSR_STOP          (0x40) //!< CFv1 XCSR Stop mask
#define CFV1_XCSR_CLKSW         (0x04) //!< CFv1 XCSR BDM Clock select
#define CFV1_XCSR_SEC           (0x02) //!< CFv1 XCSR Device secured
#define CFV1_XCSR_ENBDM         (0x01) //!< CFv1 XCSR Enable BDM mask
#define CFV1_XCSR_CSTAT         (7<<3) //!< CFv1 XCSR Status mask (all bits)
#define CFV1_XCSR_CSTAT_OK      (0<<3) //!< CFv1 XCSR Status - OK
#define CFV1_XCSR_CSTAT_INVALID (1<<3) //!< CFv1 XCSR Status - Invalid 
#define CFV1_XCSR_CSTAT_ILLEGAL (2<<3) //!< CFv1 XCSR Status - Illegal instruction
#define CFV1_XCSR_CSTAT_OVERRUN (4<<3) //!< CFv1 XCSR Status - Command Overrun

// Coldfire V1 CSR2 (byte read/write)
#define CFV1_CSR2_COPHR          (0x20) //!< CV1 CSR2 bit mask
#define CFV1_CSR2_IOPHR          (0x10) //!< CV1 IOPHR bit mask
#define CFV1_CSR2_IADHR          (0x08) //!< CV1 IADHR bit mask
#define CFV1_CSR2_BFHBR          (0x02) //!< CV1 BFHBR bit mask
#define CFV1_CSR2_BDFR           (0x01) //!< CV1 BDFR bit mask

// Coldfire Debug Registers (32-bit read/write)
#define CFV1_CSR                 (0x00)         //!< Debug register #
#define CFV1_CSR_SSM             (0x00000010UL) //!< Single Step mode
#define CFV1_CSR_VBD             (0x00020000UL) //!< Visibility Bus Disable

#define CFV1_SIM_SRS_ADDR        (0xFF8100)  // Address of SRS register

//=======================================================================
// Coldfire V2, 3 & 4
//=======================================================================

#define CFVx_CSR_SSM             (1UL<<4)  //!< Single Step mode
#define CFVx_CSR_FOF             (1UL<<27) //!< Fault on Fault
#define CFVx_CSR_TRG             (1UL<<26) //!< Breakpoint triggered
#define CFVx_CSR_HALT            (1UL<<25) //!< HALT excuted
#define CFVx_CSR_BKPT            (1UL<<24) //!< BKPT pin asserted

#endif //  _TARGET_DEFINES_H_
