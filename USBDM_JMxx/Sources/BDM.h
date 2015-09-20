/*! \file
    \brief Header file for BDM routines.
    
 */
#ifndef _BDM_H_
#define _BDM_H_

#include "Common.h"
#include "Commands.h"

//===================================================================================
// Shared interface
void  bdmHCS_init(void);
void  bdmHCS_interfaceIdle(void);
void  bdmHCS_suspend(void);
void  bdmHCS_off( void );
uint8_t    bdmHCS_powerOnReset(void);

//=============================================================
// BDM mode interface
uint8_t    bdm_RxTxSelect(void);
void  bdm_txPrepare(void);
//void  bdm_txFinish(void);
void  bdm_ackn(void);
void  bdm_wait64(void);
//void  bdm_wait150(void);
void  bdm_acknInit(void);
uint8_t    bdm_hardwareReset(uint8_t mode);
uint8_t    bdm_targetReset( uint8_t mode);
uint8_t    bdm_syncMeasure(void);
uint8_t    bdm_softwareReset(uint8_t mode);
uint8_t    bdm_connect(void);
uint8_t    bdm_physicalConnect(void);
uint8_t    bdm_enableBDM(void);
uint8_t    bdm_readBDMStatus(uint8_t *status);
uint8_t    bdm_writeBDMControl(uint8_t bdm_sts);
void  bdm_checkTiming(void);
void  bdm_checkWaitTiming(void);

uint8_t    bdmHC12_confirmSpeed(uint16_t syncValue);
uint8_t    bdm_makeActiveIfStopped(void);
uint8_t    bdm_halt(void);
uint8_t    bdm_go(void);
uint8_t    bdm_step(void);
uint8_t    bdm_testTx(uint8_t);
uint8_t    bdm_setInterfaceLevel(uint8_t level);

void  bdm_WaitForResetRise(void);

uint8_t    convertColdfireStatusByte(uint8_t bdmStatus);

//! MACRO to calculate a SYNC value from a given frequency in Hz
#define SYNC_MULTIPLE(x)  (uint16_t)((2*128*(60000000UL/10))/(x/10))

#pragma MESSAGE DISABLE C1106 // Disable warnings about Non-standard bitfield types
//! Target status
typedef struct {
   TargetType_t        target_type:8;  //!< Target type \ref TargetType_t
   AcknMode_t          ackn:8;         //!< Target supports ACKN see \ref AcknMode_t
   ResetMode_t         reset:8;        //!< Target has been reset, see \ref ResetMode_T
   SpeedMode_t         speed:8;        //!< Target speed determination method, see \ref SpeedMode_t
   TargetVddState_t    power:8;        //!< Target Vdd state
   TargetVppSelect_t   flashState:8;   //!< State of RS08 Flash programming,  see \ref FlashState_t
   uint16_t            sync_length;    //!< Length of the target SYNC pulse in 60MHz ticks
   uint16_t            wait150_cnt;    //!< Time for 150 BDM cycles in bus cycles of the MCU divided by N
   uint16_t            wait64_cnt;     //!< Time for 64 BDM cycles in bus cycles of the MCU divided by N
   uint8_t             bdmpprValue;    //!< BDMPPR value for HCS12
} CableStatus_t;

//! Target interface options
typedef struct {
   uint8_t  targetVdd;                //!< Target Vdd (off, 3.3V or 5V)
   uint8_t  cycleVddOnReset;          //!< Cycle target Power  when resetting
   uint8_t  cycleVddOnConnect;        //!< Cycle target Power if connection problems (when resetting?)
   uint8_t  leaveTargetPowered;       //!< Leave target power on exit
   uint8_t  autoReconnect;            //!< Automatically re-connect to target (for speed change)
   uint8_t  guessSpeed;               //!< Guess speed for target w/o ACKN
   uint8_t  useAltBDMClock;           //!< Use alternative BDM clock source in target (HCS08)
   uint8_t  useResetSignal;           //!< Use RESET signal on BDM interface
   uint8_t  reserved[3];
} BDM_Option_t;

#pragma MESSAGE DEFAULT C1106 // Restore warnings about Non-standard bitfield types

extern void bdm_txEmpty(uint8_t data);
extern uint8_t   bdm_rxEmpty(void);

#ifdef __HC08__
#pragma DATA_SEG __SHORT_SEG Z_PAGE
#endif // __HC08__
extern uint8_t   (*bdm_rx_ptr)(void); // Pointer to BDM Rx routines
extern void (*bdm_tx_ptr)(uint8_t);   // Pointer to BDM Tx routines

//! Returns true if BDM Rx & Tx routines have been set for the current communication speed
#define BDM_TXRX_SET ((bdm_rx_ptr != bdm_rxEmpty) && (bdm_tx_ptr != bdm_txEmpty))

#ifdef __HC08__
#pragma DATA_SEG DEFAULT
#endif // __HC08__

#define bdm_rx()      (*bdm_rx_ptr)() 
#define bdmTx(data)  (*bdm_tx_ptr)(data) 

#endif // _BDM_H_
