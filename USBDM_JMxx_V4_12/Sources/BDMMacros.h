/*! \file
    \brief Header file for BDM MACROS

   Change History
+================================================================================================
| 18 Jul 2014 | Added HCS12ZVM support                                             - pgo V4.10.6.170
+================================================================================================
\endverbatim
 */
#ifndef _BDMMACROS_H_
#define _BDMMACROS_H_

#ifdef __HC08__
#pragma DATA_SEG __SHORT_SEG Z_PAGE
#endif // __HC08__

extern uint8_t   (*bdm_rx_ptr)(void); //! Pointers to BDM Rx routines
extern void (*bdm_tx_ptr)(uint8_t);   //! Pointers to BDM Tx routines
#pragma DATA_SEG DEFAULT

//! Wrapper for current BDM rx routine
//!
//! Points to the bdm_Rx routine for the current communication speed.
#define bdm_rx()      (*bdm_rx_ptr)()

//! Wrapper for current BDM tx routine
//!
//! Points to the bdm_Tx routine for the current communication speed.
#define bdmTx(data)  (*bdm_tx_ptr)(data)

extern void bdmTx16(uint16_t data); // Tx 16-bit value

// This is a very large size improvement for no performance cost!
extern uint8_t   doACKN_WAIT64(void);   //! Wait for 64 bit times or ACKN
extern uint8_t   doACKN_WAIT150(void);  //! Wait for 150 bit times or ACKN
extern void doWAIT16(void);        //! Wait fot 16 bit times

// Hardware commands
#define _BDM_BACKGROUND           (0x90) 
#define _BDM_ACK_ENABLE           (0xD5)
#define _BDM_ACK_DISABLE          (0xD6)
#define _BDM_READ_BYTE            (0xE0)
#define _BDM_WRITE_BYTE           (0xC0)
//#define _BDM_WRITE_BLOCK          (0x88)

// HC/S12(x) hardware commands
#define _BDM12_READ_BD_BYTE       (0xE4)
#define _BDM12_READ_BD_WORD       (0xEC)
#define _BDM12_READ_WORD          (0xE8)
#define _BDM12_WRITE_BD_BYTE      (0xC4)
#define _BDM12_WRITE_BD_WORD      (0xCC)
#define _BDM12_WRITE_WORD         (0xC8)

// HCS08 'hardware' non-intrusive commands
#define _BDM08_READ_STATUS        (0xE4)
#define _BDM08_WRITE_CONTROL      (0xC4)
#define _BDM08_READ_BYTE_WS       (0xE1)
#define _BDM08_READ_LAST          (0xE8)
#define _BDM08_WRITE_BYTE_WS      (0xC1)
#define _BDM08_READ_BKPT          (0xE2)
#define _BDM08_WRITE_BKPT         (0xC2)

// Firmware commands
#define _BDM_GO                   (0x08)
#define _BDM_TRACE1               (0x10)
#define _BDM_TAGGO                (0x18)

// HCS08 'firmware' (active background mode commands)
#define _BDM08_READ_REG           (0x60) // Add reg #
#define _BDM08_READ_A             (0x68)
#define _BDM08_READ_CCR           (0x69)
#define _BDM08_READ_PC            (0x6B)
#define _BDM08_READ_HX            (0x6C)
#define _BDM08_READ_SP            (0x6F)
#define _BDM08_READ_NEXT          (0x70)
#define _BDM08_READ_NEXT_WS       (0x71)
#define _BDM08_WRITE_REG          (0x40) // Add reg #
#define _BDM08_WRITE_A            (0x48)
#define _BDM08_WRITE_CCR          (0x49)
#define _BDM08_WRITE_PC           (0x4B)
#define _BDM08_WRITE_HX           (0x4C)
#define _BDM08_WRITE_SP           (0x4F)
#define _BDM08_WRITE_NEXT         (0x50)
#define _BDM08_NEXT_WS            (0x51)

#define _BDMRS08_WRITE_SPC        (0x4F)

// HC/S12(x) firmware commands
#define _BDM12_READ_NEXT          (0x62)
#define _BDM12_READ_REG           (0x60) // Add reg #
#define _BDM12_READ_PC            (0x63)
#define _BDM12_READ_D             (0x64)
#define _BDM12_READ_X             (0x65)
#define _BDM12_READ_Y             (0x66)
#define _BDM12_READ_SP            (0x67)
#define _BDM12_WRITE_NEXT         (0x42)
#define _BDM12_WRITE_REG          (0x40) // Add reg #
#define _BDM12_WRITE_PC           (0x43)
#define _BDM12_WRITE_D            (0x44)
#define _BDM12_WRITE_X            (0x45)
#define _BDM12_WRITE_Y            (0x46)
#define _BDM12_WRITE_SP           (0x47)
#define _BDM12_GO_UNTIL           (0x0C)

// RS08 BDM Commands
//=======================================
#define RS_BDC_RESET (0x18)

// 9S12ZV  Commands
//=======================================
#define _BDMZ12_ACK_DISABLE        (0x03)
#define _BDMZ12_ACK_ENABLE         (0x02)
#define _BDMZ12_BACKGROUND         (0x04)
#define _BDMZ12_DUMP_MEM           (0x32)
#define _BDMZ12_DUMP_MEM_WS        (0x33)
#define _BDMZ12_FILL_MEM           (0x12)
#define _BDMZ12_FILL_MEM_WS        (0x13)
#define _BDMZ12_GO                 (0x08)
#define _BDMZ12_GO_UNTIL           (0x0C)
#define _BDMZ12_NOP                (0x00)
#define _BDMZ12_READ_Rn            (0x60)
#define _BDMZ12_READ_MEM           (0x30)
#define _BDMZ12_READ_MEM_WS        (0x31)
#define _BDMZ12_READ_DBGTB         (0x07)
#define _BDMZ12_READ_SAME          (0x54)
#define _BDMZ12_READ_SAME_WS       (0x55)
#define _BDMZ12_READ_BDCCSR        (0x2D)
#define _BDMZ12_SYNC_PC            (0x01)
#define _BDMZ12_WRITE_MEM          (0x10)
#define _BDMZ12_WRITE_MEM_WS       (0x11)
#define _BDMZ12_WRITE_Rn           (0x40)
#define _BDMZ12_WRITE_BDCCSR       (0x0D)
#define _BDMZ12_ERASE_FLASH        (0x95)
#define _BDMZ12_TRACE1             (0x09)

#define _BDMZ12_SZ_BYTE            (0x0<<2)
#define _BDMZ12_SZ_WORD            (0x1<<2)
#define _BDMZ12_SZ_LONG            (0x2<<2)

// Coldfire V1  Commands
//=======================================
#define _BDMCF_ACK_DISABLE        (0x03)
#define _BDMCF_ACK_ENABLE         (0x02)
#define _BDMCF_BACKGROUND         (0x04)
#define _BDMCF_DUMP_MEM           (0x32)
#define _BDMCF_DUMP_MEM_WS        (0x33)
#define _BDMCF_FILL_MEM           (0x12)
#define _BDMCF_FILL_MEM_WS        (0x13)
#define _BDMCF_GO                 (0x08)
#define _BDMCF_NOP                (0x00)
#define _BDMCF_READ_CREG          (0xE0)
#define _BDMCF_READ_DREG          (0xA0)
#define _BDMCF_READ_MEM           (0x30)
#define _BDMCF_READ_MEM_WS        (0x31)
#define _BDMCF_READ_PSTB          (0x50)
#define _BDMCF_READ_Rn            (0x60)
#define _BDMCF_READ_XCSR_BYTE     (0x2D)
#define _BDMCF_READ_CSR2_BYTE     (0x2E)
#define _BDMCF_READ_CSR3_BYTE     (0x2F)
#define _BDMCF_SYNC_PC            (0x01)
#define _BDMCF_WRITE_CREG         (0xC0)
#define _BDMCF_WRITE_DREG         (0x80)
#define _BDMCF_WRITE_MEM          (0x10)
#define _BDMCF_WRITE_MEM_WS       (0x11)
#define _BDMCF_WRITE_Rn           (0x40)
#define _BDMCF_WRITE_XCSR_BYTE    (0x0D)
#define _BDMCF_WRITE_CSR2_BYTE    (0x0E)
#define _BDMCF_WRITE_CSR3_BYTE    (0x0F)

#define _BDMCF_SZ_BYTE            (0x0<<2)
#define _BDMCF_SZ_WORD            (0x1<<2)
#define _BDMCF_SZ_LONG            (0x2<<2)

/*
** The following macros perform individual BDM command
** Naming convention: bdm_cmd_x_y,
**     where x in number of input parameters,
**           y is number of output paramreters
** suffxes indicate Word or Byte width
** HC/S12(X) wait times (in case ACKN is not used) are longer than those required for HCS08,
** so HCS08 can use the same macros
** HCS08 should support ACKN anyway
*/

//============================================================
// The following commands DO NOT expect an ACK & do not delay
// They leave the interface in the Tx condition (ready to drive)
// They leave interrupts disabled
//

//  Write command byte, truncated sequence
extern void BDM_CMD_0_0_T(uint8_t cmd);

//  Special for Software Reset HCS08, truncated sequence
extern void BDM_CMD_1W1B_0_T(uint8_t cmd, uint16_t parameter1, uint8_t parameter2);

//  Special truncated sequence
extern void BDM_CMD_1B_0_T(uint8_t cmd, uint8_t parameter);

//============================================================
// The following commands DO NOT expect an ACK & do not delay
//

// Write cmd without ACK (HCS08/RS08/CFV1)
extern void BDM_CMD_0_0_NOACK(uint8_t cmd);

// Write cmd & read byte without ACK (HCS08)
extern void BDM_CMD_0_1B_NOACK(uint8_t cmd, uint8_t *result);

// Write cmd & byte without ACK (HCS08)
extern void BDM_CMD_1B_0_NOACK(uint8_t cmd, uint8_t parameter);

// Write cmd & read word without ACK
extern void BDM_CMD_0_1W_NOACK(uint8_t cmd, uint16_t *parameter);

// Write cmd & word without ACK
extern void BDM_CMD_1W_0_NOACK(uint8_t cmd, uint16_t parameter);

//! Write cmd, word, byte & read data(status) without ACK
extern void BDM_CMD_1W1B_1B_NOACK(uint8_t cmd, uint16_t parameter, uint8_t value, uint8_t *status);

//! Write cmd, word & read word (status/data) without ACK
extern void BDM_CMD_1W_1W_NOACK(uint8_t cmd, uint16_t parameter, uint16_t *result);

//====================================================================
// The following DO expect an ACK or wait at end of the command phase

// Write command - No parameter
extern uint8_t BDM_CMD_0_0(uint8_t cmd);

// Write command + word
extern uint8_t BDM_CMD_1W_0(uint8_t cmd, uint16_t parameter);

// Read word
extern uint8_t BDM_CMD_0_1W(uint8_t cmd, uint16_t *parameter);

// Write longword
extern uint8_t BDM_CMD_1L_0(uint8_t cmd, uint32_t parameter);

// Read longword
extern uint8_t BDM_CMD_0_1L(uint8_t cmd, uint32_t *parameter);

// Write word & read byte (read word but return byte - HC/S12(x)) */
extern uint8_t BDM_CMD_1W_1WB(uint8_t cmd, uint16_t parameter, uint8_t *result);

// Write 2 words
extern uint8_t BDM_CMD_2W_0(uint8_t cmd, uint16_t parameter1, uint16_t parameter2);

// Write word and a byte (sends 2 words, the byte in both high and low byte of the 16-bit value) */
extern uint8_t BDM_CMD_2WB_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2);

// Write word & read word
extern uint8_t BDM_CMD_1W_1W(uint8_t cmd, uint16_t parameter, uint16_t *result) ;

// Read word HCS08
extern uint8_t BDM_CMD_0_1B(uint8_t cmd, uint8_t *result);

// Write byte HCS08
extern uint8_t BDM_CMD_1B_0(uint8_t cmd, uint8_t parameter);

// Write word & read byte (read word but return byte - HCS08
extern uint8_t BDM_CMD_1W_1B(uint8_t cmd, uint16_t parameter, uint8_t *result);

// Write 1 word then delay then 1 byte HCS08
extern uint8_t BDM_CMD_1W1B_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2);

// Write 1 24-bit value then 1 byte 
extern uint8_t BDM_CMD_1A1B_0(uint8_t cmd, uint32_t addr, uint8_t value);

// Write 1 24-bit value then 1 word 
extern uint8_t BDM_CMD_1A1W_0(uint8_t cmd, uint32_t addr, uint16_t value);

// Write 1 24-bit value then 1 longword 
extern uint8_t BDM_CMD_1A1L_0(uint8_t cmd, uint32_t addr, uint32_t *value);

// Write 24-bit address & read byte
extern uint8_t BDM_CMD_1A_1B(uint8_t cmd, uint32_t addr, uint8_t *result);

// Write 24-bit address, check status & read byte
extern uint8_t BDM_CMD_1A_CS_1B(uint8_t cmd, uint32_t addr, uint8_t *result);

// Write 24-bit address & read word
extern uint8_t BDM_CMD_1A_1W(uint8_t cmd, uint32_t addr, uint16_t *result);

// Write 24-bit address & read longword
extern uint8_t BDM_CMD_1A_1L(uint8_t cmd, uint32_t addr, uint32_t *result);

/* Definitions of the actual commands - these should be used in the C-code
 *
 * All BDM commands need to be called with interrupts disabled (either by hand or, for example, from within an ISR)
 *    value   is 8 or 16 bit value,
 *    addr    is 16 bit address,
 *    addr24  is 24-bit address
 *    value_p is pointer to 8 or 16 bit variable
 */

// FIRMWARE Commands
#define BDM_CMD_GO()                         BDM_CMD_0_0(_BDM_GO)            //!< Target Go (HC12,HCS08,RS08)
#define BDM12_CMD_GO_UNTIL()                 BDM_CMD_0_0(_BDM12_GO_UNTIL)    //!< Target Go until (HC12)
#define BDM_CMD_TRACE1()                     BDM_CMD_0_0(_BDM_TRACE1)        //!< Target Trace a single instruction (HC12,HCS08,RS08)
#define BDM_CMD_TAGGO()                      BDM_CMD_0_0(_BDM_TAGGO)         //!<  Target Go with tagging (HC12?)

// Write memory using X as a pointer with automatic pre-increment (HC12)
#define BDM12_CMD_WRITE_NEXT(value)          BDM_CMD_1W_0(_BDM12_WRITE_NEXT,value)

// Write register commands
#define BDM12_CMD_WRITE_REG(reg,value)       BDM_CMD_1W_0((_BDM12_WRITE_REG+(reg)),value) //!< Wite REG (HC12)
#define BDM12_CMD_WRITE_PC(value)            BDM_CMD_1W_0(_BDM12_WRITE_PC,value)          //!< Wite PC (HC12)
#define BDM12_CMD_WRITE_D(value)             BDM_CMD_1W_0(_BDM12_WRITE_D,value)           //!< Write D(HC12)
#define BDM12_CMD_WRITE_X(value)             BDM_CMD_1W_0(_BDM12_WRITE_X,value)           //!< Write X (HC12)
#define BDM12_CMD_WRITE_Y(value)             BDM_CMD_1W_0(_BDM12_WRITE_Y,value)           //!< Write Y (HC12)
#define BDM12_CMD_WRITE_SP(value)            BDM_CMD_1W_0(_BDM12_WRITE_SP,value)          //!< Write SP (HC12)

//! Read memory using X as a pointer with automatic pre-increment (HC12)
#define BDM12_CMD_READ_NEXT(value_p)         BDM_CMD_0_1W(_BDM12_READ_NEXT,value_p)

// Read register commands
#define BDM12_CMD_READ_REG(reg,value_p)      BDM_CMD_0_1W((_BDM12_READ_REG+(reg)),value_p)   //!< Read REG (HC12)
#define BDM12_CMD_READ_PC(value_p)           BDM_CMD_0_1W(_BDM12_READ_PC,value_p)            //!< Read PC (HC12)
#define BDM12_CMD_READ_D(value_p)            BDM_CMD_0_1W(_BDM12_READ_D,value_p)             //!< Read D (HC12)
#define BDM12_CMD_READ_X(value_p)            BDM_CMD_0_1W(_BDM12_READ_X,value_p)             //!< Read X (HC12)
#define BDM12_CMD_READ_Y(value_p)            BDM_CMD_0_1W(_BDM12_READ_Y,value_p)             //!< Read Y (HC12)
#define BDM12_CMD_READ_SP(value_p)           BDM_CMD_0_1W(_BDM12_READ_SP,value_p)            //!< Read SP (HC12)

// Hardware commands
#define BDM_CMD_ACK_ENABLE()                 BDM_CMD_0_0(_BDM_ACK_ENABLE)     //!< Enable ACKN (HC12,HCS08,RS08)
#define BDM_CMD_ACK_DISABLE()                BDM_CMD_0_0(_BDM_ACK_DISABLE)    //!< Disable ACKN  (HC12,HCS08) - Does timeout on ACKN
#define BDM_CMD_BACKGROUND()                 BDM_CMD_0_0(_BDM_BACKGROUND)     //!< Halt Target (HC12,HCS08,RS08)

// Read and write commands
#define BDM12_CMD_WRITEW(addr,value)         BDM_CMD_2W_0(_BDM12_WRITE_WORD,addr,value)   //!< Write 16-bit value (HC12)
#define BDM12_CMD_WRITEB(addr,value)         BDM_CMD_2WB_0(_BDM_WRITE_BYTE,addr,value)    //!< Write 8-bit value (HC12)
#define BDM12_CMD_READW(addr,value_p)        BDM_CMD_1W_1W(_BDM12_READ_WORD,addr,value_p) //!< Read 16-bit value (HC12)
#define BDM12_CMD_READB(addr,value_p)        BDM_CMD_1W_1WB(_BDM_READ_BYTE,addr,value_p)  //!< Read 8-bit value (HC12)

// Read and writes from/to the BDM memory space
#define BDM12_CMD_BDWRITEW(addr,value)       BDM_CMD_2W_0(_BDM12_WRITE_BD_WORD,addr,value)    //!< Write 16-bit value  BDM address space (HC12)
#define BDM12_CMD_BDWRITEB(addr,value)       BDM_CMD_2WB_0(_BDM12_WRITE_BD_BYTE,addr,value)   //!< Write 8-bit value  BDM address space (HC12)
#define BDM12_CMD_BDREADW(addr,value_p)      BDM_CMD_1W_1W(_BDM12_READ_BD_WORD,addr,value_p)  //!< Read 16-bit value  BDM address space (HC12)
#define BDM12_CMD_BDREADB(addr,value_p)      BDM_CMD_1W_1WB(_BDM12_READ_BD_BYTE,addr,value_p) //!< Read 8-bit value  BDM address space(HC12)

// Read register commands
#define BDM08_CMD_READSTATUS(value_p)        BDM_CMD_0_1B_NOACK(_BDM08_READ_STATUS,value_p)  //!< Read Status Register(HCS08,RS08)
#define BDM08_CMD_WRITECONTROL(value)        BDM_CMD_1B_0_NOACK(_BDM08_WRITE_CONTROL,value)  //!< Write Control Register (HCS08,RS08)

// Memory read/write
#define BDM08_CMD_READB(addr,value_p)          BDM_CMD_1W_1B(_BDM_READ_BYTE,addr,value_p)       //!< Read 8-bit memory value (HCS08,RS08)
#define BDM08_CMD_WRITEB(addr,value)           BDM_CMD_1W1B_0(_BDM_WRITE_BYTE,addr,value)       //!< Write 8-bit memory value (HCS08,RS08)
#define BDM08_CMD_WRITE_NEXT(value)            BDM_CMD_1B_0(_BDM08_WRITE_NEXT,value)            //!< Write memory using ++H:X as a pointer
#define BDM08_CMD_READ_NEXT(value_p)           BDM_CMD_0_1B(_BDM08_READ_NEXT,value_p)           //!< Read memory using ++H:X as a pointer
#define BDM08_CMD_READB_WS(addr,val_stat_p)    BDM_CMD_1W_1W_NOACK(_BDM08_READ_BYTE_WS,addr,val_stat_p)    //!< Read 8-bit memory value with status (HCS08)
#define BDM08_CMD_WRITEB_WS(addr,val,stat_p)   BDM_CMD_1W1B_1B_NOACK(_BDM08_WRITE_BYTE_WS,addr,val,stat_p) //!< Write 8-bit memory value with status (HCS08)

#define BDM08_CMD_READ_LAST(val_stat_p)        BDM_CMD_0_1W_NOACK(_BDM08_READ_LAST,val_stat_p)    //!< Read last 8-bit memory location accessed with status (HCS08)

#define BDM08_CMD_RESET(addr,val)              BDM_CMD_1W1B_0_T(_BDM_WRITE_BYTE, addr, val) //!< Reset Target (HCS08)
//#define BDM08_CMD_WRITEBLOCK(addr,value)     BDM_CMD_1W1B_0(_BDM_WRITE_BLOCK,addr,value)     //!< Write Block (HCS08)

#define BDMRS08_CMD_RESET()                  BDM_CMD_0_0_T(RS_BDC_RESET)                     //!< Reset Target (RS08)

// Read register commands
#define BDM08_CMD_READ_PC(value_p)           BDM_CMD_0_1W(_BDM08_READ_PC,value_p)         //!< Read PC (HC08)
#define BDM08_CMD_READ_SP(value_p)           BDM_CMD_0_1W(_BDM08_READ_SP,value_p)         //!< Read SP (HC08)
#define BDM08_CMD_READ_HX(value_p)           BDM_CMD_0_1W(_BDM08_READ_HX,value_p)         //!< Read HX (HC08)
#define BDM08_CMD_READ_A(value_p)            BDM_CMD_0_1B(_BDM08_READ_A,value_p)          //!< Read A (HC08)
#define BDM08_CMD_READ_CCR(value_p)          BDM_CMD_0_1B(_BDM08_READ_CCR,value_p)        //!< Read CCR (HC08)
#define BDM08_CMD_READ_BKPT(value_p)         BDM_CMD_0_1W_NOACK(_BDM08_READ_BKPT,value_p) //!< Read BKPT (HC08) - No ACK fix - pgo

// Write register commands
#define BDM08_CMD_WRITE_PC(value)            BDM_CMD_1W_0(_BDM08_WRITE_PC,value)           //!< Write PC (HC08)
#define BDM08_CMD_WRITE_SP(value)            BDM_CMD_1W_0(_BDM08_WRITE_SP,value)           //!< Write SP (HC08)
#define BDM08_CMD_WRITE_HX(value)            BDM_CMD_1W_0(_BDM08_WRITE_HX,value)           //!< Write HX (HC08)
#define BDM08_CMD_WRITE_A(value)             BDM_CMD_1B_0(_BDM08_WRITE_A,value)            //!< Write A (HC08)
#define BDM08_CMD_WRITE_CCR(value)           BDM_CMD_1B_0(_BDM08_WRITE_CCR,value)          //!< Write CCR (HC08)
#define BDM08_CMD_WRITE_BKPT(value)          BDM_CMD_1W_0_NOACK(_BDM08_WRITE_BKPT,value)   //!< Write Breakpoint (HC08) - No ACK fix - pgo
#define BDMRS08_CMD_WRITE_SPC(value)         BDM_CMD_1W_0(_BDMRS08_WRITE_SPC,value)        //!< Write Shadow PC (RS08)

/*
 * 
 */
#define BDMZ12_CMD_READ_BDCCSR(value_p)      BDM_CMD_0_1W_NOACK(_BDMZ12_READ_BDCCSR,value_p)  //!< Read BDCCSR - No ACK
#define BDMZ12_CMD_WRITE_BDCCSR(value_p)     BDM_CMD_1W_0_NOACK(_BDMZ12_WRITE_BDCCSR,value_p) //!< Read BDCCSR - No ACK
#define BDMZ12_CMD_TRACE1()                  BDM_CMD_0_0(_BDMZ12_TRACE1)                      //!< Trace a single instruction

// Coldfire V1 Commands
#define BDMCF_CMD_ACK_ENABLE()               BDM_CMD_0_0(_BDMCF_ACK_ENABLE)     //!< Enable ACKN (CFv1)
#define BDMCF_CMD_ACK_DISABLE()              BDM_CMD_0_0(_BDMCF_ACK_DISABLE)    //!< Disable ACKN (Expects ACK but times out) (CFv1)
#define BDMCF_CMD_BACKGROUND()               BDM_CMD_0_0(_BDMCF_BACKGROUND)     //!< Halt Target (CFv1)
#define BDMCF_CMD_GO()                       BDM_CMD_0_0(_BDMCF_GO)             //!< Target Go (CFv1)
#define BDMCF_CMD_NOP()                      BDM_CMD_0_0(_BDMCF_NOP)            //!< No Operation (CFv1)
#define BDMCF_CMD_SYNC_PC()                  BDM_CMD_0_0(_BDMCF_SYNC_PC)        //!< Sync PC (CFv1)

#define BDMCF_CMD_READ_REG(regNo,value_p)    BDM_CMD_0_1L(_BDMCF_READ_Rn|(regNo),value_p)    //!< Read Register (CFv1)
#define BDMCF_CMD_READ_CREG(regNo,value_p)   BDM_CMD_0_1L(_BDMCF_READ_CREG|(regNo),value_p)  //!< Read Control Register (CFv1)
#define BDMCF_CMD_READ_DREG(regNo,value_p)   BDM_CMD_0_1L(_BDMCF_READ_DREG|(regNo),value_p)  //!< Read Debug Register (CFv1)
#define BDMCF_CMD_READ_PSTBe(regNo,value_p)  BDM_CMD_0_1L(_BDMCF_READ_PSTB|(regNo),value_p)  //!< Read Trace buffer (CFv1)
#define BDMCF_CMD_READ_DREG(regNo,value_p)   BDM_CMD_0_1L(_BDMCF_READ_DREG|(regNo),value_p)  //!< Read Debug Register (CFv1)

#define BDMCF_CMD_READ_XCSR(value_p)         BDM_CMD_0_1B_NOACK(_BDMCF_READ_XCSR_BYTE,value_p) //!< Read XCSR.msb (CFv1) - No ACK
#define BDMCF_CMD_READ_CSR2(value_p)         BDM_CMD_0_1B_NOACK(_BDMCF_READ_CSR2_BYTE,value_p) //!< Read CSR2.msb (CFv1) - No ACK
#define BDMCF_CMD_READ_CSR3(value_p)         BDM_CMD_0_1B_NOACK(_BDMCF_READ_CSR3_BYTE,value_p) //!< Read CSR3.msb (CFv1) - No ACK

#define BDMCF_CMD_WRITE_REG(regNo,value)     BDM_CMD_1L_0(_BDMCF_WRITE_Rn|(regNo),value)      //!< Write Register (CFv1)
#define BDMCF_CMD_WRITE_CREG(regNo,value)    BDM_CMD_1L_0(_BDMCF_WRITE_CREG|(regNo),value)    //!< Write Control Register (CFv1)
#define BDMCF_CMD_WRITE_DREG(regNo,value)    BDM_CMD_1L_0(_BDMCF_WRITE_DREG|(regNo),value)    //!< Write Debug Register (CFv1)

#define BDMCF_CMD_WRITE_XCSR(value)          BDM_CMD_1B_0_NOACK(_BDMCF_WRITE_XCSR_BYTE,value) //!< Write XCSR.msb (CFv1) - No ACK
#define BDMCF_CMD_WRITE_CSR2(value)          BDM_CMD_1B_0_NOACK(_BDMCF_WRITE_CSR2_BYTE,value) //!< Write CSR2.msb (CFv1) - No ACK
#define BDMCF_CMD_WRITE_CSR3(value)          BDM_CMD_1B_0_NOACK(_BDMCF_WRITE_CSR3_BYTE,value) //!< Write CSR3.msb (CFv1) - No ACK

#define BDMCF_CMD_READ_MEM_S(addr24,value_p) BDM_CMD_1A_CS_1B(_BDMCF_READ_MEM_WS|_BDMCF_SZ_BYTE,addr24,value_p) //!< Read 8-bit memory value with return status (CFv1)

#define BDMCF_CMD_READ_MEM_B(addr24,value_p) BDM_CMD_1A_1B(_BDMCF_READ_MEM|_BDMCF_SZ_BYTE,addr24,value_p) //!< Read 8-bit memory value (CFv1)
#define BDMCF_CMD_READ_MEM_W(addr24,value_p) BDM_CMD_1A_1W(_BDMCF_READ_MEM|_BDMCF_SZ_WORD,addr24,value_p) //!< Read 16-bit memory value (CFv1)
#define BDMCF_CMD_READ_MEM_L(addr24,value_p) BDM_CMD_1A_1L(_BDMCF_READ_MEM|_BDMCF_SZ_LONG,addr24,value_p) //!< Read 32-bit memory value (CFv1)

#define BDMCF_CMD_DUMP_MEM_B(value_p)        BDM_CMD_0_1B(_BDMCF_DUMP_MEM|_BDMCF_SZ_BYTE,value_p) //!< Read consecutive 8-bit memory value (CFv1)
#define BDMCF_CMD_DUMP_MEM_W(value_p)        BDM_CMD_0_1W(_BDMCF_DUMP_MEM|_BDMCF_SZ_WORD,value_p) //!< Read consecutive 16-bit memory value (CFv1)
#define BDMCF_CMD_DUMP_MEM_L(value_p)        BDM_CMD_0_1L(_BDMCF_DUMP_MEM|_BDMCF_SZ_LONG,value_p) //!< Read consecutive 32-bit memory value (CFv1)

#define BDMCF_CMD_WRITE_MEM_B(addr24,value)  BDM_CMD_1A1B_0(_BDMCF_WRITE_MEM|_BDMCF_SZ_BYTE,addr24,value) //!< Write 8-bit memory value (CFv1)
#define BDMCF_CMD_WRITE_MEM_W(addr24,value)  BDM_CMD_1A1W_0(_BDMCF_WRITE_MEM|_BDMCF_SZ_WORD,addr24,value) //!< Write 16-bit memory value (CFv1)
#define BDMCF_CMD_WRITE_MEM_L(addr24,value)  BDM_CMD_1A1L_0(_BDMCF_WRITE_MEM|_BDMCF_SZ_LONG,addr24,value) //!< Write 32-bit memory value (CFv1)

#define BDMCF_CMD_FILL_MEM_B(value)          BDM_CMD_1B_0(_BDMCF_FILL_MEM|_BDMCF_SZ_BYTE,value) //!< Write consecutive 8-bit memory value (CFv1)
#define BDMCF_CMD_FILL_MEM_W(value)          BDM_CMD_1W_0(_BDMCF_FILL_MEM|_BDMCF_SZ_WORD,value) //!< Write consecutive 16-bit memory value (CFv1)
#define BDMCF_CMD_FILL_MEM_L(value)          BDM_CMD_1L_0(_BDMCF_FILL_MEM|_BDMCF_SZ_LONG,value) //!< Write consecutive 32-bit memory value (CFv1)

#endif // _BDMMACROS_H_
