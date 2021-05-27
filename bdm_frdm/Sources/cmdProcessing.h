/*! \file
    \brief Main command procedure for executing BDM commands received over the USB.
*/
#ifndef _CMDPROCESSING_H_
#define _CMDPROCESSING_H_
#include <stdint.h>
#include "commands.h"

/** Buffer for USB command in, result out */
extern uint8_t   commandBuffer[MAX_COMMAND_SIZE+4];

/** Size of command return result */
extern int returnSize;

/**
 * Process commands from USB device
 *
 *   @note : Command                                    \n
 *       commandBuffer[0]    = Size of command (N)      \n
 *       commandBuffer[1]    = Command                  \n
 *       commandBuffer[2..N] = Parameters/data
 *
 *   @note : Response                                   \n
 *       commandBuffer[0]    = Error code               \n
 *       commandBuffer[1..N] = Response/data
 */
extern void commandLoop(void);

/**
 *   Optionally re-connects with target
 *
 *  @param when Indicates situation in which the routine is being called \n
 *        AUTOCONNECT_STATUS - being called during status query \n
 *        AUTOCONNECT_ALWAYS - being called before command execution
 *
 *  @return BDM_RC_OK    Success
 *  @return != BDM_RC_OK Error
 */
extern USBDM_ErrorCode optionalReconnect(AutoConnect_t when);

/** Target status */
struct CableStatus_t {
   TargetType_t        target_type:8;  //!< Target type TargetType_t
   AcknMode_t          ackn:8;         //!< Target supports ACKN see AcknMode_t
   ResetMode_t         xxxxx:8;        //!< Not used
   SpeedMode_t         speed:8;        //!< Target speed determination method, see SpeedMode_t
   TargetVddState_t    yyyyy:8;        //!< Not used
   TargetVppSelect_t   flashState:8;   //!< State of RS08 Flash programming,  see FlashState_t
   uint16_t            sync_length;    //!< Length of the target SYNC pulse in 60MHz ticks
   uint16_t            wait150_cnt;    //!< Time for 150 BDM cycles in bus cycles of the MCU divided by N
   uint16_t            wait64_cnt;     //!< Time for 64 BDM cycles in bus cycles of the MCU divided by N
   uint8_t             bdmpprValue;    //!< BDMPPR value for HCS12
} ;

/** Target interface options */
struct BDM_Option_t {
   bool               cycleVddOnReset:1;      //!< Cycle target Power  when resetting
   bool               cycleVddOnConnect:1;    //!< Cycle target Power if connection problems (when resetting?)
   bool               leaveTargetPowered:1;   //!< Leave target power on exit
   bool               guessSpeed:1;           //!< Guess speed for target w/o ACKN
   bool               useResetSignal:1;       //!< Use RESET signal on BDM interface
   TargetVddSelect_t  targetVdd;              //!< Selected target Vdd (off, 3.3V or 5V)
   ClkSwValues_t      useAltBDMClock:8;       //!< Use alternative BDM clock source in target (HCS08)
   AutoConnect_t      autoReconnect:8;        //!< Automatically re-connect method (for speed change)
   uint16_t           SBDFRaddress;           //!< Address of HCS08_SBDFR register
   uint8_t            reserved[3];
} ;

/** Status of the BDM interface */
extern CableStatus_t cable_status;

/** Options for cable operation */
extern BDM_Option_t  bdm_option;

#endif
