/*
 * cmdProcessing.cpp
 *
 *  Created on: 26 Jul 2016
 *      Author: podonoghue
 */

#include <string.h>
#include "targetVddInterface.h"
#include "delay.h"
#include "configure.h"
#include "bdmCommon.h"
#include "cmdProcessing.h"
#include "cmdProcessingSWD.h"
#if HW_CAPABILITY & CAP_BDM
#include "cmdProcessingHCS.h"
#endif
#include "resetInterface.h"
#include "usb.h"
#include "swd.h"
#if HW_CAPABILITY & CAP_BDM
#include "targetDefines.h"
#include "bdm.h"
#endif

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
#ifdef HCS08_SBDFR_DEFAULT
      /* SBDFRaddress       */   HCS08_SBDFR_DEFAULT,  //!< Default HCS08_SBDFR address
#else
      /* SBDFRaddress       */   0,                    //!< Not used
#endif
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

   // Target specific checks
   switch (cable_status.target_type) {
#if (HW_CAPABILITY&CAP_CFVx_HW) && (TARGET_CAPABILITY&CAP_PST)
      case T_CFVx :
         if (ALLPST_IS_HIGH) {
            status |= S_HALT;  // Target is halted
         }
         break;
#endif
#if HW_CAPABILITY&CAP_BDM
      case T_HC12:
#if TARGET_CAPABILITY & CAP_S12Z
      case T_S12Z  :
#endif
      case T_HCS08:
      case T_RS08:
      case T_CFV1:
         if (cable_status.ackn==ACKN) {  // Target supports ACKN and the feature is enabled ?
            status |= S_ACKN;
         }
         switch (cable_status.speed) {
            case SPEED_NO_INFO       : status |= S_NOT_CONNECTED;  break;
            case SPEED_USER_SUPPLIED : status |= S_USER_DONE;      break;
            case SPEED_SYNC          : status |= S_SYNC_DONE;      break;
            case SPEED_GUESSED       : status |= S_GUESS_DONE;     break;
         }
         break;
#endif
            default:
               break;
   }
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
#if (HW_CAPABILITY&CAP_VDDSENSE)
   switch (TargetVddInterface::checkVddState()) {    // Target has power ?
      case VddState_None     : break;
      case VddState_Error    : status |= S_POWER_ERR;  break;
      case VddState_Internal : status |= S_POWER_INT;  break;
      case VddState_External : status |= S_POWER_EXT;  break;
   }
#else
   // Assume power present
   status |= S_POWER_EXT;
#endif
#if (HW_CAPABILITY&CAP_FLASH)
   switch (cable_status.flashState) {
      //    case BDM_TARGET_VPP_OFF     : status |= S_VPP_OFF;      break;
      case BDM_TARGET_VPP_STANDBY : status |= S_VPP_STANDBY;  break;
      case BDM_TARGET_VPP_ON      : status |= S_VPP_ON;       break;
      case BDM_TARGET_VPP_ERROR   : status |= S_VPP_ERR;      break;
   }
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
#if HW_CAPABILITY&CAP_BDM
   switch (cable_status.target_type) {
      case T_HC12:
         if (cable_status.speed == SPEED_USER_SUPPLIED) {
            //   User has specified speed
            break;
         }
         /* Falls through - for GCC */
         // no break - for eclipse
      case T_RS08:
      case T_HCS08:
      case T_CFV1:
         if (bdm_option.autoReconnect == when) {
            // If auto re-connect enabled at this time then ...
            //    ...make sure of connection
            rc = Bdm::physicalConnect();
         }
         break;
      default: ;
   }
#endif
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

#if (TARGET_CAPABILITY & CAP_ARM_SWD) && defined(ERASE_KINETIS)

#define F_ERSBLK                        0x08

int executeCommand(void) {
   // Clear any existing errors
   FTFL_FSTAT = FTFL_FSTAT_ACCERR_MASK|FTFL_FSTAT_FPVIOL_MASK;

   // Start command
   FTFL_FSTAT = FTFL_FSTAT_CCIF_MASK;

   // Wait for command complete
   while ((FTFL_FSTAT & FTFL_FSTAT_CCIF_MASK) == 0) {
   }
   // Convert error codes
   if ((FTFL_FSTAT & FTFL_FSTAT_FPVIOL_MASK ) != 0) {
      return BDM_RC_FAIL;
   }
   if ((FTFL_FSTAT & FTFL_FSTAT_ACCERR_MASK ) != 0) {
      return BDM_RC_FAIL;
   }
   if ((FTFL_FSTAT & FTFL_FSTAT_MGSTAT0_MASK ) != 0) {
      return BDM_RC_FAIL;
   }
   return BDM_RC_OK;
}

#define FTFL_FCCOB3_0 (*(uint32_t*)&FTFL_FCCOB3)

/*
 * Erase entire flash block
 */
USBDM_ErrorCode eraseFlashBlock(uint32_t address) {
   FTFL_FCCOB3_0 = (F_ERSBLK << 24) | address;
   return executeCommand();
}

USBDM_ErrorCode eraseKinetisSecurity(void) {
   // Unprotect flash
   FTFL_FPROT0 = 0xFF;
   FTFL_FPROT1 = 0xFF;
   FTFL_FPROT2 = 0xFF;
   FTFL_FPROT3 = 0xFF;
   FTFL_FDPROT = 0xFF;

   // Disable flash caching
   //   FMC_PFB0CR  = 0x00000000;

   // Erase security sector
   return eraseFlashBlock(NV_BACKKEY3);
}
#endif

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
#if (HW_CAPABILITY&CAP_BDM)
      case BDM_DBG_ACKN: // try the ACKN feature
         Bdm::enableACKNMode();
         commandBuffer[1] = (uint8_t) makeStatusWord(); // return the status byte
         returnSize = 2;
         return BDM_RC_OK;

      case BDM_DBG_SYNC: { // try the sync feature
         USBDM_ErrorCode rc = Bdm::sync();
         if (rc != BDM_RC_OK) {
            return rc;
         }
         commandBuffer[1] = (uint8_t)makeStatusWord(); // return the status byte
         (*(uint16_t*)(commandBuffer+2)) = cable_status.sync_length;
         returnSize = 4;
      }
      return BDM_RC_OK;
#endif
#if (HW_CAPABILITY & CAP_FLASH)
      case BDM_DBG_VPP_OFF: // Programming voltage off
         return bdmSetVpp(BDM_TARGET_VPP_OFF );

      case BDM_DBG_VPP_ON:
         // Programming voltage on (requires FLASH12V on first)
         return bdmSetVpp(BDM_TARGET_VPP_ON );

      case BDM_DBG_FLASH12V_OFF: // 12V charge pump off
         return bdmSetVpp(BDM_TARGET_VPP_OFF );

      case BDM_DBG_FLASH12V_ON:  // 12V charge pump on
         return bdmSetVpp(BDM_TARGET_VPP_STANDBY );
#endif
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
#if (HW_CAPABILITY & CAP_VDDSENSE)
      case BDM_DBG_MEASURE_VDD: // Measure Target Vdd
      {
         uint16_t voltage = targetVddMeasure(); // return the value
         commandBuffer[1] = (uint8_t)(voltage>>8);
         commandBuffer[2] = (uint8_t)voltage;
      }
      returnSize = 3;
      return BDM_RC_OK;
#endif
#if defined(INLINE_ACKN)
      case BDM_DBG_TESTWAITS:
         bdm_checkWaitTiming();
         return BDM_RC_OK;
#endif

      case BDM_DBG_TESTALTSPEED:
         return BDM_RC_OK;

#if (DEBUG&STACK_DEBUG)
      case BDM_DBG_STACKSIZE: // Measure stack size
      {
         extern char  __SEG_START_SSTACK[];     // Bottom of stack space
         extern char  __SEG_END_SSTACK[];       // Top of stack space
         char *stackProbe = __SEG_START_SSTACK; // Probe for stack RAM
         uint32_t size;

         while (*++stackProbe == 0) { // Find 1st used (non-zero) byte on stack
         }
         size =  (uint32_t) __SEG_END_SSTACK - (uint32_t) stackProbe;
         commandBuffer[1] = (uint8_t) (size>>24);
         commandBuffer[2] = (uint8_t) (size>>16);
         commandBuffer[3] = (uint8_t) (size>>8);
         commandBuffer[4] = (uint8_t) (size);
         returnSize = 5;
         return BDM_RC_OK;
      }
#endif // (DEBUG&STACK_DEBUG)
#if (HW_CAPABILITY&CAP_BDM)
      case BDM_DBG_TESTBDMTX: // Test BDM Tx routine
         return BDM_RC_ILLEGAL_COMMAND;//bdm_testTx(commandBuffer[3]);
#endif
#if TARGET_CAPABILITY & CAP_ARM_SWD
      case   BDM_DBG_SWD_ERASE_LOOP:  //!< - Mass erase on reset capture
         return Swd::kinetisMassErase();

      case   BDM_DBG_SWD:  //!< - Test ARM-SWD functions
         return Swd::connect();
#endif
#if (TARGET_CAPABILITY & CAP_ARM_SWD) && defined(ERASE_KINETIS)

      case   BDM_DBG_SWD+10:  //!< - Erase Kinetis Security region
      return eraseKinetisSecurity();
#endif
#if TARGET_CAPABILITY & CAP_ARM_JTAG
      case   BDM_DBG_ARM:  //!< - Test ARM-JTAG functions
         return arm_test();
#endif
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
   Swd::getPinState(status);
#if HW_CAPABILITY & CAP_BDM
   Bdm::getPinState(status);
#endif

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
#if HW_CAPABILITY&CAP_BDM
         case T_HC12 :
#if TARGET_CAPABILITY & CAP_S12Z
         case T_S12Z  :
#endif
         case T_HCS08 :
         case T_RS08 :
         case T_CFV1 :
            Bdm::initialise();
            break;
#endif
#if (HW_CAPABILITY&CAP_CFVx_HW)
         case T_CFVx :
            bdmcf_interfaceIdle();
            break;
#endif
#if (HW_CAPABILITY&CAP_JTAG_HW)
         case T_JTAG :
         case T_ARM_JTAG :
         case T_MC56F80xx :
            jtag_interfaceIdle();
            break;
#endif
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
#if (HW_CAPABILITY&CAP_BDM)
#if TARGET_CAPABILITY & CAP_S12Z
      case T_S12Z  :
#endif
      case T_HC12 :
      case T_HCS08 :
      case T_RS08 :
      case T_CFV1 :
         if (control & ~(PIN_BKGD_MASK|PIN_RESET_MASK))
            return BDM_RC_ILLEGAL_PARAMS;
         break;
#endif
#if (HW_CAPABILITY&CAP_CFVx_HW)
      case T_CFVx :
         if (control & ~(PIN_TA|PIN_RESET|PIN_BKPT))
            return BDM_RC_ILLEGAL_PARAMS;
         break;
#endif
#if (HW_CAPABILITY&CAP_JTAG_HW)
      case T_MC56F80xx :
         if (control & ~(PIN_TRST|PIN_RESET|PIN_DE))
            return BDM_RC_ILLEGAL_PARAMS;
         break;
      case T_JTAG :
      case T_ARM_JTAG :
         if (control & ~(PIN_TRST|PIN_RESET))
            return BDM_RC_ILLEGAL_PARAMS;
         break;
#endif
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

#if (HW_CAPABILITY & CAP_BDM)
   Bdm::setPinState(control);
#endif

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

#if (HW_CAPABILITY&CAP_JTAG_HW)
   switch(control&PIN_TRST) {
      case PIN_TRST_3STATE:
#ifdef TRST_3STATE
         TRST_3STATE();
#endif
         break;
      case PIN_TRST_LOW:
         TRST_LOW();
         break;
   }
#endif
#if (HW_CAPABILITY&CAP_CFVx_HW)
   switch (control & PIN_TA) {
      case PIN_TA_3STATE :
         TA_3STATE();
         break;
      case PIN_TA_LOW :
         TA_LOW();
         break;
   }
   switch(control&PIN_BKPT) {
      case PIN_BKPT_3STATE:
         BKPT_HIGH(); // should be 3-state!
         break;
      case PIN_BKPT_LOW:
         BKPT_LOW();
         break;
   }
#endif

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

/**  Structure representing a s.et of function ptrs */
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
      //   f_CMD_ILLEGAL                    ,//= 12, CMD_USBDM_GET_VER (EP0)
      //   f_CMD_ILLEGAL                    ,//= 13, Reserved
      //   f_CMD_ILLEGAL                    ,//= 14, CMD_USBDM_ICP_BOOT (EP0)
};

#if (TARGET_CAPABILITY&CAP_HCS12)
static const FunctionPtr HCS12functionPtrs[] = {
      // Target specific versions
      Hcs::f_CMD_CONNECT               ,//= 15, CMD_USBDM_CONNECT
      Hcs::f_CMD_SET_SPEED             ,//= 16, CMD_USBDM_SET_SPEED
      Hcs::f_CMD_GET_SPEED             ,//= 17, CMD_USBDM_GET_SPEED

      f_CMD_ILLEGAL                    ,//= 18, CMD_CUSTOM_COMMAND
      f_CMD_ILLEGAL                    ,//= 19, RESERVED

      Hcs::f_CMD_READ_STATUS_REG       ,//= 20, CMD_USBDM_READ_STATUS_REG
      Hcs::f_CMD_WRITE_CONTROL_REG     ,//= 21, CMD_USBDM_WRITE_CONTROL_REG

      Hcs::f_CMD_RESET                 ,//= 22, CMD_USBDM_TARGET_RESET
      Hcs::f_CMD_STEP                  ,//= 23, CMD_USBDM_TARGET_STEP
      Hcs::f_CMD_GO                    ,//= 24, CMD_USBDM_TARGET_GO
      Hcs::f_CMD_HALT                  ,//= 25, CMD_USBDM_TARGET_HALT

      Hcs12::f_CMD_WRITE_REG           ,//= 26, CMD_USBDM_WRITE_REG
      Hcs12::f_CMD_READ_REG            ,//= 27, CMD_USBDM_READ_REG

      f_CMD_ILLEGAL                    ,//= 28, CMD_USBDM_WRITE_CREG
      f_CMD_ILLEGAL                    ,//= 29, CMD_USBDM_READ_CREG

      Hcs12::f_CMD_WRITE_BD            ,//= 30, CMD_USBDM_WRITE_DREG
      Hcs12::f_CMD_READ_BD             ,//= 31, CMD_USBDM_READ_DREG

      Hcs12::f_CMD_WRITE_MEM           ,//= 32, CMD_USBDM_WRITE_MEM
      Hcs12::f_CMD_READ_MEM            ,//= 33, CMD_USBDM_READ_MEM
};
static const FunctionPtrs HCS12FunctionPointers = {CMD_USBDM_CONNECT,
      (uint8_t)(sizeof(HCS12functionPtrs)/sizeof(FunctionPtr)),
      HCS12functionPtrs};
#endif

#if (TARGET_CAPABILITY&CAP_S12Z)
   static const FunctionPtr S12ZfunctionPtrs[] = {
         // Target specific versions
         Hcs::f_CMD_CONNECT            ,//= 15, CMD_USBDM_CONNECT
         Hcs::f_CMD_SET_SPEED          ,//= 16, CMD_USBDM_SET_SPEED
         Hcs::f_CMD_GET_SPEED          ,//= 17, CMD_USBDM_GET_SPEED

         S12z::f_CMD_CUSTOM_COMMAND    ,//= 18, CMD_CUSTOM_COMMAND
         f_CMD_ILLEGAL                 ,//= 19, RESERVED

         Hcs::f_CMD_READ_STATUS_REG    ,//= 20, CMD_USBDM_READ_STATUS_REG
         Hcs::f_CMD_WRITE_CONTROL_REG  ,//= 21, CMD_USBDM_WRITE_CONTROL_REG

         Hcs::f_CMD_RESET              ,//= 22, CMD_USBDM_TARGET_RESET
         Hcs::f_CMD_STEP               ,//= 23, CMD_USBDM_TARGET_STEP
         Hcs::f_CMD_GO                 ,//= 24, CMD_USBDM_TARGET_GO
         Hcs::f_CMD_HALT               ,//= 25, CMD_USBDM_TARGET_HALT

         Cfv1::f_CMD_WRITE_REG         ,//= 26, CMD_USBDM_WRITE_REG
         Cfv1::f_CMD_READ_REG          ,//= 27  CMD_USBDM_READ_REG

         f_CMD_ILLEGAL                 ,//= 28, CMD_USBDM_WRITE_CREG
         f_CMD_ILLEGAL                 ,//= 29, CMD_USBDM_READ_CREG

         f_CMD_ILLEGAL                 ,//= 30, CMD_USBDM_WRITE_DREG
         f_CMD_ILLEGAL                 ,//= 31, CMD_USBDM_READ_DREG

         Cfv1::f_CMD_WRITE_MEM         ,//= 32, CMD_USBDM_WRITE_MEM
         Cfv1::f_CMD_READ_MEM          ,//= 33, CMD_USBDM_READ_MEM
   };
   static const FunctionPtrs S12ZFunctionPointers = {CMD_USBDM_CONNECT,
         (uint8_t)(sizeof(S12ZfunctionPtrs)/sizeof(FunctionPtr)),
         S12ZfunctionPtrs};
#endif

#if (TARGET_CAPABILITY&(CAP_HCS08|CAP_RS08))
   static const FunctionPtr HCS08functionPtrs[] = {
         /**
          * HCS08 Target specific versions
          */
         Hcs::f_CMD_CONNECT            ,//= 15, CMD_USBDM_CONNECT
         Hcs::f_CMD_SET_SPEED          ,//= 16, CMD_USBDM_SET_SPEED
         Hcs::f_CMD_GET_SPEED          ,//= 17, CMD_USBDM_GET_SPEED

         f_CMD_ILLEGAL                 ,//= 18, CMD_CUSTOM_COMMAND
         f_CMD_ILLEGAL                 ,//= 19, RESERVED

         Hcs::f_CMD_READ_STATUS_REG    ,//= 20, CMD_USBDM_READ_STATUS_REG
         Hcs::f_CMD_WRITE_CONTROL_REG  ,//= 21, CMD_USBDM_WRITE_CONTROL_REG

         Hcs::f_CMD_RESET              ,//= 22, CMD_USBDM_TARGET_RESET
         Hcs::f_CMD_STEP               ,//= 23, CMD_USBDM_TARGET_STEP
         Hcs::f_CMD_GO                 ,//= 24, CMD_USBDM_TARGET_GO
         Hcs::f_CMD_HALT               ,//= 25, CMD_USBDM_TARGET_HALT

         Hcs08::f_CMD_WRITE_REG        ,//= 26, CMD_USBDM_WRITE_REG
         Hcs08::f_CMD_READ_REG         ,//= 27, CMD_USBDM_READ_REG

         f_CMD_ILLEGAL                 ,//= 28, CMD_USBDM_WRITE_CREG
         f_CMD_ILLEGAL                 ,//= 29, CMD_USBDM_READ_CREG

         Hcs08::f_CMD_WRITE_BKPT       ,//= 30, CMD_USBDM_WRITE_DREG
         Hcs08::f_CMD_READ_BKPT        ,//= 31, CMD_USBDM_READ_DREG

         Hcs08::f_CMD_WRITE_MEM        ,//= 32, CMD_USBDM_WRITE_MEM
         Hcs08::f_CMD_READ_MEM         ,//= 33, CMD_USBDM_READ_MEM

#if (TARGET_CAPABILITY & CAP_RS08)
         f_CMD_ILLEGAL                    ,//= 34, CMD_USBDM_TRIM_CLOCK - obsolete
         f_CMD_ILLEGAL                    ,//= 35, CMD_USBDM_RS08_FLASH_ENABLE - obsolete
         f_CMD_ILLEGAL                    ,//= 36, CMD_USBDM_RS08_FLASH_STATUS - obsolete
         f_CMD_ILLEGAL                    ,//= 37, CMD_USBDM_RS08_FLASH_DISABLE - obsolete
         f_CMD_ILLEGAL                    ,//= 38, CMD_USBDM_JTAG_GOTORESET
         f_CMD_ILLEGAL                    ,//= 39, CMD_USBDM_JTAG_GOTOSHIFT
         f_CMD_ILLEGAL                    ,//= 40, CMD_USBDM_JTAG_WRITE
         f_CMD_ILLEGAL                    ,//= 41, CMD_USBDM_JTAG_READ
         f_CMD_SET_VPP                    ,//= 42, CMD_USBDM_SET_VPP
#endif
   };
   static const FunctionPtrs HCS08FunctionPointers = {CMD_USBDM_CONNECT,
         (uint8_t)(sizeof(HCS08functionPtrs)/sizeof(FunctionPtr)),
         HCS08functionPtrs};
#endif

#if (TARGET_CAPABILITY&CAP_CFV1)
   static const FunctionPtr CFV1functionPtrs[] = {
         // Target specific versions
         Hcs::f_CMD_CONNECT              ,//= 15, CMD_USBDM_CONNECT
         Hcs::f_CMD_SET_SPEED            ,//= 16, CMD_USBDM_SET_SPEED
         Hcs::f_CMD_GET_SPEED            ,//= 17, CMD_USBDM_GET_SPEED

         f_CMD_ILLEGAL                   ,//= 18, CMD_CUSTOM_COMMAND
         f_CMD_ILLEGAL                   ,//= 19, RESERVED

         Hcs::f_CMD_READ_STATUS_REG      ,//= 20, CMD_USBDM_READ_STATUS_REG
         Hcs::f_CMD_WRITE_CONTROL_REG    ,//= 21, CMD_USBDM_WRITE_CONTROL_REG

         Hcs::f_CMD_RESET                ,//= 22, CMD_USBDM_TARGET_RESET
         Hcs::f_CMD_STEP                 ,//= 23, CMD_USBDM_TARGET_STEP
         Hcs::f_CMD_GO                   ,//= 24, CMD_USBDM_TARGET_GO
         Hcs::f_CMD_HALT                 ,//= 25, CMD_USBDM_TARGET_HALT

         Cfv1::f_CMD_WRITE_REG           ,//= 26, CMD_USBDM_WRITE_REG
         Cfv1::f_CMD_READ_REG            ,//= 27  CMD_USBDM_READ_REG

         Cfv1::f_CMD_WRITE_CREG          ,//= 28  CMD_USBDM_WRITE_CREG
         Cfv1::f_CMD_READ_CREG           ,//= 29  CMD_USBDM_READ_CREG

         Cfv1::f_CMD_WRITE_DREG          ,//= 30  CMD_USBDM_WRITE_DREG
         Cfv1::f_CMD_READ_DREG           ,//= 31  CMD_USBDM_READ_DREG

         Cfv1::f_CMD_WRITE_MEM           ,//= 32  CMD_USBDM_WRITE_MEM
         Cfv1::f_CMD_READ_MEM            ,//= 33  CMD_USBDM_READ_MEM
#if HW_CAPABILITY&CAP_CORE_REGS
         Cfv1::f_CMD_READ_ALL_CORE_REGS  ,//= 34  CMD_USBDM_READ_ALL_REGS
#else
         f_CMD_ILLEGAL                   ,//= 34, CMD_USBDM_READ_ALL_REGS
#endif
   };
   static const FunctionPtrs CFV1FunctionPointers  = {CMD_USBDM_CONNECT,
         (uint8_t)(sizeof(CFV1functionPtrs)/sizeof(FunctionPtr)),
         CFV1functionPtrs};
#endif

#if (TARGET_CAPABILITY&CAP_CFVx)
   static const FunctionPtr CFVxfunctionPtrs[] = {
         // Target specific versions
         f_CMD_CFVx_RESYNC                ,//= 15, CMD_USBDM_CONNECT
         f_CMD_SPI_SET_SPEED              ,//= 16, CMD_USBDM_SET_SPEED
         f_CMD_SPI_GET_SPEED              ,//= 17, CMD_USBDM_GET_SPEED

         f_CMD_ILLEGAL                    ,//= 18, CMD_CUSTOM_COMMAND
         f_CMD_ILLEGAL                    ,//= 19, RESERVED

         f_CMD_CFVx_READ_STATUS_REG       ,//= 20, CMD_USBDM_READ_STATUS_REG
         f_CMD_ILLEGAL                    ,//= 21, CMD_USBDM_WRITE_CONTROL_REG

         f_CMD_CFVx_RESET                 ,//= 22, CMD_USBDM_TARGET_RESET
         f_CMD_CFVx_STEP                  ,//= 23, CMD_USBDM_TARGET_STEP
         f_CMD_CFVx_GO                    ,//= 24, CMD_USBDM_TARGET_GO
         f_CMD_CFVx_HALT                  ,//= 25, CMD_USBDM_TARGET_HALT

         f_CMD_CFVx_WRITE_REG             ,//= 26, CMD_USBDM_WRITE_REG
         f_CMD_CFVx_READ_REG              ,//= 27  CMD_USBDM_READ_REG

         f_CMD_CFVx_WRITE_CREG            ,//= 28  CMD_USBDM_WRITE_CREG
         f_CMD_CFVx_READ_CREG             ,//= 29  CMD_USBDM_READ_CREG

         f_CMD_CFVx_WRITE_DREG            ,//= 30  CMD_USBDM_WRITE_DREG
         f_CMD_CFVx_READ_DREG             ,//= 31  CMD_USBDM_READ_DREG

         f_CMD_CFVx_WRITE_MEM             ,//= 32  CMD_USBDM_WRITE_MEM
         f_CMD_CFVx_READ_MEM              ,//= 33  CMD_USBDM_READ_MEM
#if HW_CAPABILITY&CAP_CORE_REGS
         f_CMD_CFVx_READ_ALL_CORE_REGS    ,//= 34  CMD_USBDM_READ_ALL_REGS
         //#else
         //   f_CMD_ILLEGAL                    ,//= 34, CMD_USBDM_READ_ALL_REGS
#endif
   };
   static const FunctionPtrs CFVxFunctionPointers  = {CMD_USBDM_CONNECT,
         sizeof(CFVxfunctionPtrs)/sizeof(FunctionPtr),
         CFVxfunctionPtrs};
#endif
#if (TARGET_CAPABILITY&CAP_ARM_JTAG)
   // Combined JTAG/ARM_JTAG Table
   static const FunctionPtr JTAGfunctionPtrs[] = {
         // Target specific versions
         f_CMD_ARM_CONNECT                ,//= 15, CMD_USBDM_CONNECT
         f_CMD_SPI_SET_SPEED              ,//= 16, CMD_USBDM_SET_SPEED
         f_CMD_SPI_GET_SPEED              ,//= 17, CMD_USBDM_GET_SPEED
         f_CMD_ILLEGAL                    ,//= 18, CMD_CUSTOM_COMMAND
         f_CMD_ILLEGAL                    ,//= 19, RESERVED
         f_CMD_ILLEGAL                    ,//= 20, CMD_USBDM_READ_STATUS_REG
         f_CMD_ILLEGAL                    ,//= 21, CMD_USBDM_WRITE_CONTROL_REG
         f_CMD_JTAG_RESET                 ,//= 22, CMD_USBDM_TARGET_RESET
         f_CMD_ARM_TARGET_STEP            ,//= 23, CMD_USBDM_TARGET_STEP
         f_CMD_ARM_TARGET_GO              ,//= 24, CMD_USBDM_TARGET_GO
         f_CMD_ARM_TARGET_HALT            ,//= 25, CMD_USBDM_TARGET_HALT
         f_CMD_ARM_WRITE_REG              ,//= 26, CMD_USBDM_WRITE_REG
         f_CMD_ARM_READ_REG               ,//= 27  CMD_USBDM_READ_REG
         f_CMD_ARM_WRITE_CREG             ,//= 28  CMD_USBDM_WRITE_CREG
         f_CMD_ARM_READ_CREG              ,//= 29  CMD_USBDM_READ_CREG
         f_CMD_ARM_WRITE_DREG             ,//= 30  CMD_USBDM_WRITE_DREG
         f_CMD_ARM_READ_DREG              ,//= 31  CMD_USBDM_READ_DREG
         f_CMD_ARM_WRITE_MEM              ,//= 32  CMD_USBDM_WRITE_MEM
         f_CMD_ARM_READ_MEM               ,//= 33  CMD_USBDM_READ_MEM
#if HW_CAPABILITY&CAP_CORE_REGS
         f_CMD_ARM_READ_ALL_CORE_REGS     ,//= 34  CMD_USBDM_READ_ALL_REGS
#else
         f_CMD_ILLEGAL                    ,//= 34, CMD_USBDM_READ_ALL_REGS
#endif
         f_CMD_ILLEGAL                    ,//= 35, CMD_USBDM_RS08_FLASH_ENABLE
         f_CMD_ILLEGAL                    ,//= 36, CMD_USBDM_RS08_FLASH_STATUS
         f_CMD_ILLEGAL                    ,//= 37, CMD_USBDM_RS08_FLASH_DISABLE
         f_CMD_JTAG_GOTORESET             ,//= 38, CMD_USBDM_JTAG_GOTORESET
         f_CMD_JTAG_GOTOSHIFT             ,//= 39, CMD_USBDM_JTAG_GOTOSHIFT
         f_CMD_JTAG_WRITE                 ,//= 40, CMD_USBDM_JTAG_WRITE
         f_CMD_JTAG_READ                  ,//= 41, CMD_USBDM_JTAG_READ
         f_CMD_ILLEGAL                    ,//= 42, CMD_USBDM_SET_VPP
         f_CMD_JTAG_READ_WRITE            ,//= 43, CMD_USBDM_JTAG_READ_WRITE
         f_CMD_JTAG_EXECUTE_SEQUENCE      ,//= 44, CMD_JTAG_EXECUTE_SEQUENCE
   };
   static const FunctionPtrs JTAGFunctionPointers   = {CMD_USBDM_CONNECT,
         sizeof(JTAGfunctionPtrs)/sizeof(FunctionPtr),
         JTAGfunctionPtrs};
#elif (TARGET_CAPABILITY&(CAP_DSC|CAP_JTAG))
      // Table for JTAG w/o JTAG_ARM
   static const FunctionPtr JTAGfunctionPtrs[] = {
         // Target specific versions
         f_CMD_ILLEGAL                    ,//= 15, CMD_USBDM_CONNECT
         f_CMD_SPI_SET_SPEED              ,//= 16, CMD_USBDM_SET_SPEED
         f_CMD_SPI_GET_SPEED              ,//= 17, CMD_USBDM_GET_SPEED
         f_CMD_ILLEGAL                    ,//= 18, CMD_CUSTOM_COMMAND
         f_CMD_ILLEGAL                    ,//= 19, RESERVED
         f_CMD_ILLEGAL                    ,//= 20, CMD_USBDM_READ_STATUS_REG
         f_CMD_ILLEGAL                    ,//= 21, CMD_USBDM_WRITE_CONTROL_REG
         f_CMD_JTAG_RESET                 ,//= 22, CMD_USBDM_TARGET_RESET
         f_CMD_ILLEGAL                    ,//= 23, CMD_USBDM_TARGET_STEP
         f_CMD_ILLEGAL                    ,//= 24, CMD_USBDM_TARGET_GO
         f_CMD_ILLEGAL                    ,//= 25, CMD_USBDM_TARGET_HALT
         f_CMD_ILLEGAL                    ,//= 26, CMD_USBDM_WRITE_REG
         f_CMD_ILLEGAL                    ,//= 27  CMD_USBDM_READ_REG
         f_CMD_ILLEGAL                    ,//= 28  CMD_USBDM_WRITE_CREG
         f_CMD_ILLEGAL                    ,//= 29  CMD_USBDM_READ_CREG
         f_CMD_ILLEGAL                    ,//= 30  CMD_USBDM_WRITE_DREG
         f_CMD_ILLEGAL                    ,//= 31  CMD_USBDM_READ_DREG
         f_CMD_ILLEGAL                    ,//= 32  CMD_USBDM_WRITE_MEM
         f_CMD_ILLEGAL                    ,//= 33  CMD_USBDM_READ_MEM
         f_CMD_ILLEGAL                    ,//= 34, CMD_USBDM_TRIM_CLOCK
         f_CMD_ILLEGAL                    ,//= 35, CMD_USBDM_RS08_FLASH_ENABLE
         f_CMD_ILLEGAL                    ,//= 36, CMD_USBDM_RS08_FLASH_STATUS
         f_CMD_ILLEGAL                    ,//= 37, CMD_USBDM_RS08_FLASH_DISABLE
         f_CMD_JTAG_GOTORESET             ,//= 38, CMD_USBDM_JTAG_GOTORESET
         f_CMD_JTAG_GOTOSHIFT             ,//= 39, CMD_USBDM_JTAG_GOTOSHIFT
         f_CMD_JTAG_WRITE                 ,//= 40, CMD_USBDM_JTAG_WRITE
         f_CMD_JTAG_READ                  ,//= 41, CMD_USBDM_JTAG_READ
         f_CMD_ILLEGAL                    ,//= 42, CMD_USBDM_SET_VPP
         f_CMD_JTAG_READ_WRITE            ,//= 43, CMD_USBDM_JTAG_READ_WRITE
         f_CMD_JTAG_EXECUTE_SEQUENCE      ,//= 44, CMD_JTAG_EXECUTE_SEQUENCE
   };
   static const FunctionPtrs JTAGFunctionPointers   = {CMD_USBDM_CONNECT,
         sizeof(JTAGfunctionPtrs)/sizeof(FunctionPtr),
         JTAGfunctionPtrs};
#endif

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
         Swd::f_CMD_WRITE_REG              ,//= 26, CMD_USBDM_WRITE_REG
         Swd::f_CMD_READ_REG               ,//= 27  CMD_USBDM_READ_REG
         Swd::f_CMD_WRITE_CREG             ,//= 28  CMD_USBDM_WRITE_CREG
         Swd::f_CMD_READ_CREG              ,//= 29  CMD_USBDM_READ_CREG
         Swd::f_CMD_WRITE_DREG             ,//= 30  CMD_USBDM_WRITE_DREG
         Swd::f_CMD_READ_DREG              ,//= 31  CMD_USBDM_READ_DREG
         Swd::f_CMD_WRITE_MEM              ,//= 32  CMD_USBDM_WRITE_MEM
         Swd::f_CMD_READ_MEM               ,//= 33  CMD_USBDM_READ_MEM
#if HW_CAPABILITY&CAP_CORE_REGS
         Swd::f_CMD_READ_ALL_CORE_REGS     ,//= 34  CMD_USBDM_READ_ALL_REGS
#endif
   };
   /** Information about command functions for ARM-SWD targets */
   static const FunctionPtrs SWDFunctionPointers   = {CMD_USBDM_CONNECT,
         (uint8_t)(sizeof(SWDfunctionPtrs)/sizeof(FunctionPtr)),
         SWDfunctionPtrs};
#endif

//  Pointer to function table for current target type
static const FunctionPtrs *currentFunctions = NULL; // default to none

static const FunctionPtrs *const functionsPtrs[] = {
#if (TARGET_CAPABILITY&CAP_HCS12)
      /* T_HC12 */ &HCS12FunctionPointers,
#else
      NULL,
#endif
#if (TARGET_CAPABILITY&CAP_HCS08)
      /* T_HCS08 */ &HCS08FunctionPointers,
#else
      NULL,
#endif
#if (TARGET_CAPABILITY & CAP_RS08)
      /* T_RS08 */  &HCS08FunctionPointers,
#else
      NULL,
#endif
#if (TARGET_CAPABILITY & CAP_CFV1)
      /* T_CFV1 */ &CFV1FunctionPointers,
#else
      NULL,
#endif
#if (TARGET_CAPABILITY&CAP_CFVx)
      /* T_CFVx */ &CFVxFunctionPointers,
#else
      NULL,
#endif
#if (TARGET_CAPABILITY&CAP_JTAG)
      /* T_JTAG */ &JTAGFunctionPointers,
#else
      NULL,
#endif
      /* T_EZFLASH */ NULL,
#if (TARGET_CAPABILITY&CAP_DSC)
      /* T_MC56F80xx */ &JTAGFunctionPointers,
#else
      NULL,
#endif
#if (TARGET_CAPABILITY&CAP_ARM_JTAG)
      /* T_ARM_JTAG */ &JTAGFunctionPointers,
#else
      NULL,
#endif
#if (TARGET_CAPABILITY&CAP_ARM_SWD)
      /* T_ARM_SWD */ &SWDFunctionPointers,
#else
      NULL,
#endif
      /* T_ARM     */  NULL,
#if (TARGET_CAPABILITY&CAP_S12Z)
      /* T_HC12ZVM */ &S12ZFunctionPointers,
#else
      NULL,
#endif
};

/*
 *  Set target type
 *  Initialise interface for given target
 *  @note \n
 *    commandBuffer[2] = target type
 */
USBDM_ErrorCode f_CMD_SET_TARGET(void) {
   TargetType_t target = (TargetType_t) commandBuffer[2];

   if (target >= (sizeof(functionsPtrs)/sizeof(functionsPtrs[0]))) {
      currentFunctions = NULL;
   }
   else {
      currentFunctions = functionsPtrs[target];
   }
   if ((target != T_OFF) && (currentFunctions == NULL)) {
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

   BDMCommands command    = (BDMCommands)commandBuffer[1];  // Command is 1st byte
   FunctionPtr commandPtr = f_CMD_ILLEGAL;     // Default to illegal command

   USBDM::console.WRITE("Command = ").WRITELN(command);

   // Check if modeless command
   if ((uint8_t)command < sizeof(commonFunctionPtrs)/sizeof(FunctionPtr)) {
      // Modeless command
      commandPtr = commonFunctionPtrs[(uint8_t)command];
   }
   else {
      // Target specific command
      if (currentFunctions != NULL) {
         int commandIndex = (uint8_t)command - currentFunctions->firstCommand;
         if ((commandIndex >= 0) && (commandIndex < currentFunctions->size))
            commandPtr = currentFunctions->functions[commandIndex];
      }
   }
   // Execute the command
   // Note: returnSize & commandBuffer may be updated by command
   //       returnSize has a default value of 1
   //       commandStatus has a default value of BDM_RC_OK
   //       On error, returnSize is forced to 1 (error code return only)
   returnSize       = 1;
   commandStatus = BDM_RC_OK;
   if (command >= CMD_USBDM_READ_STATUS_REG) {
      // Check if re-connect needed before most commands (always)
      commandStatus = optionalReconnect(AUTOCONNECT_ALWAYS);
   }
   if (commandStatus == BDM_RC_OK) {
      commandStatus = commandPtr();      // Execute command & update command status
   }
   commandBuffer[0] = commandStatus;  // return command status
   if (commandStatus != BDM_RC_OK) {
      returnSize = 1;  // Return a single byte error code
      // Always do
      // Changed guard V4.10.6
      if ((uint8_t)command > sizeof(commonFunctionPtrs)/sizeof(FunctionPtr)) {
         // Modeless command
         // Do any common error recovery or cleanup here
#if (TARGET_CAPABILITY&CAP_CFVx)
         if (cable_status.target_type == T_CFVx) {
            (void)bdmcf_complete_chk_rx(); //  Send at least 2 NOPs to purge the BDM
            (void)bdmcf_complete_chk_rx(); //  of the offending command
         }
#endif
#if (TARGET_CAPABILITY&CAP_ARM_JTAG)
         if (cable_status.target_type == T_ARM_JTAG) {
            // Re-connect in case synchronisation lost
            if (commandStatus == BDM_RC_ACK_TIMEOUT) {
               // Abort AP transactions as they are the usual cause of WAIT timeouts
               (void)arm_abortAP();
            }
            // Clear sticky bits since already reporting error
            (void)arm_CheckStickyUnpipelined();
         }
#endif
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
      (void)USBDM::UsbImplementation::receiveBulkData(MAX_COMMAND_SIZE, commandBuffer);
      commandSequence = commandBuffer[1] & 0xC0;
      commandBuffer[1] &= 0x3F;
      commandExec();
      commandBuffer[0] |= commandSequence;
      USBDM::UsbImplementation::sendBulkData(returnSize, commandBuffer);
   }
}
