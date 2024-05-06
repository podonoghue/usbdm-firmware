/*
 * cmdProcessing.cpp
 *
 *  Created on: 26 Jul 2016
 *      Author: podonoghue
 */

#include <interfaceCommon.h>
#include <string.h>
#include "configure.h"
#include "targetVddInterface.h"
#include "delay.h"
#include "cmdProcessing.h"
#include "cmdProcessingSWD.h"
#include "resetInterface.h"
#include "usb.h"
#include "swd.h"

#include "Names.h"

using namespace USBDM;

/** Buffer for USB command in, result out */
uint8_t commandBuffer[MAX_COMMAND_SIZE+4];

/** Size of command return result */
int returnSize;

/**
 *  Status of the BDM
 */
CableStatus_t cable_status;

/**
 *  Options for the BDM
 */
BDM_Option_t bdm_option = {
      /* cycleVddOnReset    */   false,                //!< Cycle target Power when resetting
      /* cycleVddOnConnect  */   false,                //!< Cycle target Power if connection problems (when resetting?)
      /* leaveTargetPowered */   false,                //!< Leave target power on when exiting
      /* guessSpeed         */   true,                 //!< Guess speed for target w/o ACKN
      /* useResetSignal     */   false,                //!< Use RESET signal on BDM interface
      /* targetVdd          */   BDM_TARGET_VDD_OFF,   //!< Target Vdd (off, 3.3V or 5V)
      /* useAltBDMClock     */   CS_DEFAULT,           //!< Use alternative BDM clock source in target
      /* autoReconnect      */   AUTOCONNECT_STATUS,   //!< Automatically re-connect to target (for speed change)
      /* SBDFRaddress       */   0,                    //!< Not used
      /* reserved           */   {0}                   //   Reserved
};

/** Error code from last/current command */
static USBDM_ErrorCode commandStatus;

/**
 *  Creates status byte
 *
 *  @return 16-bit status byte \ref StatusBitMasks_t
 */
uint16_t makeStatusWord(void) {
   uint16_t status = 0;

#if (HW_CAPABILITY&CAP_RST_IN)
   if (ResetInterface::read()) {
      status |= S_RESET_STATE;       // The RSTO pin is currently high (inactive)
      if (ResetInterface::resetActivity()) {
         status |= S_RESET_DETECT;  // The target was recently reset
      }
   }
   else {
      // Reset currently  low (active)
      status |= S_RESET_DETECT;  // The target was recently reset externally
   }
#endif // (HW_CAPABILITY&CAP_RST_IN)
#if (HW_CAPABILITY&CAP_VDDCONTROL)
   switch (TargetVddInterface::checkVddState()) {    // Target has power ?
      case VddState_None       : break;
      case VddState_Overloaded : status |= S_POWER_ERR;  break;
      case VddState_Internal   : status |= S_POWER_INT;  break;
      case VddState_External   : status |= S_POWER_EXT;  break;
   }
#else
   // Assume power present
   status |= S_POWER_EXT;
#endif
   return status;
}

/**
 *   Optionally re-connects with target
 *
 *  @param when Indicates situation in which the routine is being called\n
 *        AUTOCONNECT_STATUS - being called during status query
 *        AUTOCONNECT_ALWAYS - being called before command execution
 *
 *  @return
 *     == \ref BDM_RC_OK => success
 *     != \ref BDM_RC_OK => error
 */
USBDM_ErrorCode optionalReconnect(AutoConnect_t when) {
   USBDM_ErrorCode rc = BDM_RC_OK;
   (void)when; // remove warning
   return rc;
}

/**
 *  Dummy routine for unused slots in command tables
 *
 *  @return
 *     BDM_RC_ILLEGAL_COMMAND => illegal command
 */
USBDM_ErrorCode f_CMD_ILLEGAL(void) {
   return BDM_RC_ILLEGAL_COMMAND;
}

/**
 *  Return status from last command received
 *
 *  @return
 *    status from last command
 */
USBDM_ErrorCode f_CMD_GET_COMMAND_STATUS(void) {
   return commandStatus;
}

/**
 *  Various debugging & testing commands
 *
 *  @note
 *    commandBuffer \n
 *     - [1..N] = various commands
 *
 *  @return
 *     error code
 */
USBDM_ErrorCode f_CMD_DEBUG(void) {

   DebugSubCommands subCommand = (DebugSubCommands)commandBuffer[2];

   switch ((uint8_t)subCommand) {
#if (HW_CAPABILITY & CAP_VDDCONTROL)
      case BDM_DBG_VDD_OFF: // Target Vdd voltage off
         TargetVddInterface::vddOff();
         return BDM_RC_OK;

      case BDM_DBG_VDD3_ON: // Target Vdd voltage on
         TargetVddInterface::vdd3V3On();
         return BDM_RC_OK;

      case BDM_DBG_VDD5_ON: // Target Vdd voltage on
         TargetVddInterface::vdd5VOn();
         return BDM_RC_OK;

      case BDM_DBG_CYCLE_POWER: // Cycle power to target
         return cycleTargetVdd(RESET_SPECIAL);
#endif
#if (HW_CAPABILITY & (CAP_VDDSENSE|CAP_VDDCONTROL))
      case BDM_DBG_MEASURE_VDD: // Measure Target Vdd
      {
         uint16_t voltage = targetVddMeasure(); // return the value
         commandBuffer[1] = (uint8_t)(voltage>>8);
         commandBuffer[2] = (uint8_t)voltage;
      }
      returnSize = 3;
      return BDM_RC_OK;
#endif

      case BDM_DBG_TESTALTSPEED:
         return BDM_RC_OK;

      case BDM_DBG_SWD_ERASE_LOOP:  //!< - Mass erase on reset capture
         return Swd::kinetisMassErase();

      case BDM_DBG_SWD:  //!< - Test ARM-SWD functions
         return Swd::connect();
      case BDM_DBG_SERIAL_ON:
         Usb0::setDiscardCharacters(false);
         return BDM_RC_OK;

      case BDM_DBG_SERIAL_OFF:
         Usb0::setDiscardCharacters(true);
         return BDM_RC_OK;

   } // switch
   return BDM_RC_ILLEGAL_PARAMS;
}

/**
 *  Set various options
 *
 *  @note
 *    commandBuffer\n
 *    - [2..N] = options (image of \ref BDM_Option_t)
 *
 *   @return
 *     BDM_RC_OK => success
 */
USBDM_ErrorCode f_CMD_SET_OPTIONS(void) {
   USBDM_ErrorCode rc = BDM_RC_OK;
   // Save BDM Options
   int sub=2;
   uint8_t value = commandBuffer[sub++];
   bdm_option.cycleVddOnReset    = value&(1<<0);
   bdm_option.cycleVddOnConnect  = value&(1<<1);
   bdm_option.leaveTargetPowered = value&(1<<2);
   bdm_option.guessSpeed         = value&(1<<3);
   bdm_option.useResetSignal     = value&(1<<4);
   bdm_option.targetVdd          = (TargetVddSelect_t)commandBuffer[sub++];
   bdm_option.useAltBDMClock     = (ClkSwValues_t)commandBuffer[sub++];
   bdm_option.autoReconnect      = (AutoConnect_t)commandBuffer[sub++];
   return rc;
}

/**
 * BDM Capabilities
 */
static const uint8_t capabilities[] = {
      // Inversion is hidden by driver!
      // Note: CAP_HCS08 & CAP_CFV1 are returned inverted for backwards compatibility
      (uint8_t)((TARGET_CAPABILITY^(CAP_HCS08|CAP_CFV1))>>8),  // Returns 16-bit value
      (uint8_t)((TARGET_CAPABILITY^(CAP_HCS08|CAP_CFV1))&0xFF),
      (uint8_t)(MAX_COMMAND_SIZE>>8),
      (uint8_t)MAX_COMMAND_SIZE,
      VERSION_MAJOR,             // Extended firmware version number nn.nn.nn
      VERSION_MINOR,
      VERSION_MICRO,
};

/**
 *  Returns capability vector for hardware
 *
 *  @return
 *   commandBuffer                                                \n
 *    - [1..2] = BDM capability, see \ref HardwareCapabilities_t  \n
 *    - [3..4] = Maximum command buffer size
 */
USBDM_ErrorCode f_CMD_GET_CAPABILITIES(void) {
   // Copy BDM Options
   (void)memcpy(commandBuffer+1, capabilities, sizeof(capabilities));
   returnSize = sizeof(capabilities) + 1;
   return BDM_RC_OK;
}

/**
 *  Return status of BDM communication
 *
 *  @note
 *    commandBuffer\n
 *   - [1..2] = BDM status
 */
USBDM_ErrorCode f_CMD_GET_BDM_STATUS(void) {
   uint16_t word;
   word = makeStatusWord();
   unpack16BE(word, commandBuffer+1);
   returnSize  = 3;

   return BDM_RC_OK;
}

/**
 *  Get pin status
 *
 *  @note
 *    commandBuffer\n
 *   - [1..2] = BDM status
 */
void getPinStatus(void) {
   PinLevelMasks_t status = ResetInterface::isHigh()?PIN_RESET_LOW:PIN_RESET_HIGH;
   status = status | Swd::getPinState();

   unpack16BE(status, commandBuffer+1);
   returnSize  = 3;
}

/*
 *  Directly control pins
 *
 *  @note
 *    commandBuffer\n
 *      Entry: [2..3] = control value\n
 *      Exit:  [1..2] = pin values (MSB unused) - not yet implemented
 */
USBDM_ErrorCode f_CMD_CONTROL_PINS(void) {

   PinLevelMasks_t control = (PinLevelMasks_t)((commandBuffer[2]<<8)|commandBuffer[3]);

   // Set up for OK return
   commandBuffer[1] = 0;
   commandBuffer[2] = 0;
   returnSize  = 3;

   if (control == PIN_NOCHANGE) {
      getPinStatus();
      return BDM_RC_OK;
   }
   if (control == PIN_RELEASE) {
      switch (cable_status.target_type) {
#if (HW_CAPABILITY&CAP_SWD_HW)
         case T_ARM_SWD :
            Swd::interfaceIdle();
            break;
#endif
         case T_OFF :
         default:
            return BDM_RC_ILLEGAL_COMMAND;
      }
      getPinStatus();
      return BDM_RC_OK;
   }
   // Restrict control to mode specific active pins
   switch (cable_status.target_type) {
#if (HW_CAPABILITY&CAP_SWD_HW)
      case T_ARM_SWD :
         if (control & ~(PIN_SWD_MASK|PIN_SWCLK_MASK|PIN_RESET_MASK))
            return BDM_RC_ILLEGAL_PARAMS;
         break;
#endif
      case T_OFF :
      default:
         return BDM_RC_ILLEGAL_PARAMS;
   }

#if (HW_CAPABILITY & CAP_RST_OUT)
   switch (control & PIN_RESET_MASK) {
      case PIN_RESET_3STATE :
         ResetInterface::highZ();
#if (HW_CAPABILITY & CAP_RST_IN)
         if (!USBDM::waitMS(200, ResetInterface::read)) {
            return(BDM_RC_RESET_TIMEOUT_RISE);
         }
#endif // (HW_CAPABILITY&CAP_RST_IN)
         break;
      case PIN_RESET_LOW :
         ResetInterface::low();
         break;
   }
#endif // (HW_CAPABILITY&CAP_RST_OUT)

#if (HW_CAPABILITY & CAP_SWD_HW)
   Swd::setPinState(control);
#endif

   getPinStatus();
   return BDM_RC_OK;
}

/**
 *  Directly control Target Vdd
 *
 *  @note
 *    commandBuffer\n
 *      Entry: [2..3] = control value (MSB unused)\n
 *      Exit:  none
 */
USBDM_ErrorCode f_CMD_SET_VDD(void) {

#if (HW_CAPABILITY&CAP_VDDCONTROL)
   TargetVddSelect_t vddSelect = (TargetVddSelect_t)commandBuffer[3];
   switch(vddSelect) {
      case BDM_TARGET_VDD_DISABLE:
      case BDM_TARGET_VDD_ENABLE:
         // Retain current option
         return setTargetVdd(vddSelect);

      case BDM_TARGET_VDD_OFF:
      case BDM_TARGET_VDD_3V3:
      case BDM_TARGET_VDD_5V:
         // Change option
         bdm_option.targetVdd = (TargetVddSelect_t)commandBuffer[3];
         return enableTargetVdd();
      default:
         return BDM_RC_ILLEGAL_PARAMS;
   }
#else
   return checkTargetVdd();
#endif
}

//=================================================
// Command Dispatch code
//=================================================

/**  Pointer to command function */
typedef USBDM_ErrorCode (*FunctionPtr)(void);

/**  Structure representing a set of function ptrs */
typedef struct {
   uint8_t firstCommand;          //!< First command value accepted
   uint8_t size;                  //!< Size of command structure
   const FunctionPtr *functions;  //!< Pointer to commands
} FunctionPtrs;

extern USBDM_ErrorCode f_CMD_SET_TARGET(void);

/** Command functions shared by all targets */
static const FunctionPtr commonFunctionPtrs[] = {

      // Common to all targets
      f_CMD_GET_COMMAND_STATUS         ,//= 0,  CMD_USBDM_GET_COMMAND_STATUS
      f_CMD_SET_TARGET                 ,//= 1,  CMD_USBDM_SET_TARGET
      f_CMD_SET_VDD                    ,//= 2,  CMD_USBDM_SET_VDD
      f_CMD_DEBUG                      ,//= 3,  CMD_USBDM_DEBUG
      f_CMD_GET_BDM_STATUS             ,//= 4,  CMD_USBDM_GET_BDM_STATUS
      f_CMD_GET_CAPABILITIES           ,//= 5,  CMD_USBDM_GET_CAPABILITIES
      f_CMD_SET_OPTIONS                ,//= 6,  CMD_USBDM_SET_OPTIONS
      f_CMD_ILLEGAL                    ,//= 7,  Reserved
      f_CMD_CONTROL_PINS               ,//= 8,  CMD_USBDM_CONTROL_PINS
      //   f_CMD_ILLEGAL                    ,//= 9,  Reserved
      //   f_CMD_ILLEGAL                    ,//= 10, Reserved
      //   f_CMD_ILLEGAL                    ,//= 11, Reserved
      //   f_CMD_ILLEGAL                    ,//= 12, CMD_USBDM_GET_VER (handled by EP0)
      //   f_CMD_ILLEGAL                    ,//= 13, Reserved
      //   f_CMD_ILLEGAL                    ,//= 14, CMD_USBDM_ICP_BOOT (handled by EP0)
};

#if (TARGET_CAPABILITY&CAP_ARM_SWD)
   /** Command functions for ARM-SWD targets */
   static const FunctionPtr SWDfunctionPtrs[] = {
         /**
          * SWD Target specific versions
          */
         Swd::f_CMD_CONNECT                ,//= 15, CMD_USBDM_CONNECT
         Swd::f_CMD_SET_SPEED              ,//= 16, CMD_USBDM_SET_SPEED
         Swd::f_CMD_GET_SPEED              ,//= 17, CMD_USBDM_GET_SPEED
         f_CMD_ILLEGAL                     ,//= 18, CMD_CUSTOM_COMMAND
         f_CMD_ILLEGAL                     ,//= 19, RESERVED
         f_CMD_ILLEGAL                     ,//= 20, CMD_USBDM_READ_STATUS_REG
         f_CMD_ILLEGAL                     ,//= 21, CMD_USBDM_WRITE_CONTROL_REG
         f_CMD_ILLEGAL                     ,//= 22, CMD_USBDM_TARGET_RESET
         Swd::f_CMD_TARGET_STEP            ,//= 23, CMD_USBDM_TARGET_STEP
         Swd::f_CMD_TARGET_GO              ,//= 24, CMD_USBDM_TARGET_GO
         Swd::f_CMD_TARGET_HALT            ,//= 25, CMD_USBDM_TARGET_HALT
         Swd::f_CMD_WRITE_REG              ,//= 26, CMD_USBDM_WRITE_REG    - Write ARM-SWD core register
         Swd::f_CMD_READ_REG               ,//= 27  CMD_USBDM_READ_REG     - Read ARM-SWD core register
         Swd::f_CMD_WRITE_CREG             ,//= 28  CMD_USBDM_WRITE_CREG   - Write to AP register (sets AP_SELECT & APACC)
         Swd::f_CMD_READ_CREG              ,//= 29  CMD_USBDM_READ_CREG    - Read to AP register (sets AP_SELECT & APACC)
         Swd::f_CMD_WRITE_DREG             ,//= 30  CMD_USBDM_WRITE_DREG   - Write SWD DP register
         Swd::f_CMD_READ_DREG              ,//= 31  CMD_USBDM_READ_DREG    - Read SWD DP register;
         Swd::f_CMD_WRITE_MEM              ,//= 32  CMD_USBDM_WRITE_MEM
         Swd::f_CMD_READ_MEM               ,//= 33  CMD_USBDM_READ_MEM
#if HW_CAPABILITY&CAP_CORE_REGS
         Swd::f_CMD_READ_ALL_CORE_REGS     ,//= 34  CMD_USBDM_READ_ALL_REGS - Block read ARM-SWD core registers
#endif
   };
   /** Information about command functions for ARM-SWD targets */
   static const FunctionPtrs SWDFunctionPointers   = {CMD_USBDM_CONNECT,
         (uint8_t)(sizeof(SWDfunctionPtrs)/sizeof(FunctionPtr)),
         SWDfunctionPtrs};
#endif

//  Pointer to function table for current target type
static const FunctionPtrs *currentFunctions = nullptr; // default to none

/*
 *  Set target type
 *  Initialise interface for given target
 *  @note \n
 *    commandBuffer[2] = target type
 */
USBDM_ErrorCode f_CMD_SET_TARGET(void) {

   TargetType_t target = TargetType_t(commandBuffer[2]);

   // Always ARM SWD
   currentFunctions = &SWDFunctionPointers;

   if ((target != T_OFF) && (target != T_ARM)&& (target != T_ARM_SWD)) {
      target = T_ILLEGAL;
   }
   return setTarget(target);
}

/*
 *   Processes all commands received over USB
 *
 *   The command is expected to be in commandBuffer[1..N]
 *
 *   @return \n
 *      Number of bytes left in commandBuffer to be sent back as response.\n
 *      commandBuffer[0]    = result code, BDM_RC_OK => success, else failure error code\n
 *      commandBuffer[1..N] = command results
 */
static void commandExec(void) {
//   Debug::high();

   BDMCommands command    = BDMCommands(commandBuffer[1]);  // Command is 1st byte
   FunctionPtr commandPtr = f_CMD_ILLEGAL;                  // Default to illegal command

//   USBDM::console.WRITE("Command = ").WRITELN(getCommandName(command));

   if (((uint8_t)command >= CMD_USBDM_CONTROL_PINS) && (currentFunctions == nullptr)) {
      // Command greater than this require the interface to have been set up i.e.
      // target selected, so leave as illegal command
   }
   else if ((uint8_t)command < sizeofArray(commonFunctionPtrs)) {
      // Modeless command
      commandPtr = commonFunctionPtrs[(uint8_t)command];
   }
   else {
      // Target specific command
      // Earlier check means we don't need to check if target set here
      int commandIndex = (uint8_t)command - currentFunctions->firstCommand;
      if ((commandIndex >= 0) && (commandIndex < currentFunctions->size))
         commandPtr = currentFunctions->functions[commandIndex];
   }
   // Execute the command
   // Note: returnSize & commandBuffer may be updated by command
   //       returnSize has a default value of 1
   //       commandStatus has a default value of BDM_RC_OK
   //       On error, returnSize is forced to 1 (error code return only)
   returnSize    = 1;
   commandStatus = BDM_RC_OK;
   if (command >= CMD_USBDM_READ_STATUS_REG) {
      // Check if re-connect needed before most commands (always)
      commandStatus = optionalReconnect(AUTOCONNECT_ALWAYS);
   }
   if (commandStatus == BDM_RC_OK) {
      commandStatus = commandPtr();   // Execute command & update command status
   }
   commandBuffer[0] = commandStatus;  // Return command status
   if (commandStatus != BDM_RC_OK) {
      returnSize = 1;  // Return a single byte error code
      // Always do
      // Changed guard V4.10.6
      if ((uint8_t)command > sizeof(commonFunctionPtrs)/sizeof(FunctionPtr)) {
         // Modeless command
         // Do any common error recovery or cleanup here
#if (TARGET_CAPABILITY&CAP_ARM_SWD)
         if (cable_status.target_type == T_ARM_SWD) {
            // Re-connect in case synchronisation lost
            (void)Swd::connect();
            if (commandStatus == BDM_RC_ACK_TIMEOUT) {
               // Abort AP transactions as they are the usual cause of WAIT timeouts
               (void)Swd::abortAP();
            }
            // Clear sticky bits since already reporting error
            (void)Swd::clearStickyBits();
         }
#endif
      }
   }
//   Debug::low();
}

/**
 * Process commands from USB device
 *
 *   @note : Command                                    \n
 *       commandBuffer[0]    = size of command (N)      \n
 *       commandBuffer[1]    = command                  \n
 *       commandBuffer[2..N] = parameters/data
 *
 *   @note : Response                                   \n
 *       commandBuffer[0]    = error code               \n
 *       commandBuffer[1..N] = response/data
 */
void commandLoop() {
   static uint8_t commandSequence = 0;

   for(;;) {
      int receivedSize = USBDM::UsbImplementation::receiveBulkData(MAX_COMMAND_SIZE, commandBuffer);
      if (receivedSize <= 0) {
         continue;
      }
      commandSequence = commandBuffer[1] & 0xC0;
      commandBuffer[1] &= 0x3F;
      commandExec();
      commandBuffer[0] |= commandSequence;
      USBDM::UsbImplementation::sendBulkData(returnSize, commandBuffer);
   }
}
