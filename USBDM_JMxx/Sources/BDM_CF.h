/*! \file
    \brief Header file for BDM routines.

 */

#ifndef _BDMCF_H_
#define _BDMCF_H_


#if (HW_CAPABILITY&CAP_CFVx_HW)

//==============================================================
// Shared interface
void  bdmcf_init(void);

void bdm_suspend(void);
void bdmCF_off(void);
uint16_t  bdm_targetVddMeasure(void);
void bdmcf_interfaceIdle(void);
uint8_t   bdmCF_powerOnReset(void);
void bdmCF_suspend(void);
void bdmcf_interfaceIdle(void);

//===============================================================
// Coldfire BDM mode interface
uint8_t   bdmcf_resync(void);
uint8_t   bdmcf_halt(void);
uint8_t   bdmcf_reset(uint8_t bkpt);
uint8_t   bdmcf_ta(uint8_t time_10us);
uint8_t   bdmcf_tx_msg(uint16_t data);
uint8_t   bdmcf_rx_msg(uint16_t *data);
uint8_t   bdmcf_txrx_msg(uint16_t *data);
void bdmcf_tx(uint8_t count, uint8_t *data);
uint8_t   bdmcf_complete_chk(uint16_t next_cmd);
uint8_t   bdmcf_complete_chk_rx(void);
uint8_t   bdmcf_tx_msg_half_rx(uint16_t data);
uint8_t   bdmcf_rx(uint8_t count, uint8_t *data);
uint8_t   bdmcf_rxtx(uint8_t count, uint8_t *data, uint16_t next_cmd);
uint16_t  bdmcf_txRx16(uint16_t data);

// Prototypes for the Rx and Tx functions
void bdmcf_tx8_1(uint8_t data);
uint8_t   bdmcf_rx8_1(void);
uint8_t   bdmcf_txrx8_1(uint8_t data);
uint8_t   bdmcf_txrx_start(void);
#endif

#if (HW_CAPABILITY&CAP_CFVx_HW) || (HW_CAPABILITY&CAP_JTAG_HW)
//=================================================================
// JTAG mode interface
uint8_t   spi_setSpeed(uint16_t freq);
void jtag_interfaceIdle(void);
void jtag_init(void);
void jtag_off(void);
void jtag_transition_reset(void);
void jtag_transition_shift(uint8_t mode);
void jtag_write(uint8_t tap_transition, uint8_t bit_count, const uint8_t *writePtr);
void jtag_read(uint8_t tap_transition, uint8_t bit_count, uint8_t *readPtr);
void jtag_read_write(uint8_t transitionToIdle, uint8_t bitCount, const uint8_t *writePtr, uint8_t *readPtr);
void jtag_set_hdr(uint16_t value);
void jtag_set_hir(uint16_t value);
void jtag_set_tdr(uint16_t value);
void jtag_set_tir(uint16_t value);
#endif 

#if (HW_CAPABILITY&(CAP_CFVx_HW|CAP_SWD_HW))
uint8_t   spi_setSpeed(uint16_t freq);
#endif

#endif // _BDMCF_H_

