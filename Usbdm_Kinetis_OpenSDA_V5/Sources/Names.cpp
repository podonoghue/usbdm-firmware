/*
 * Names.c
 *
 *  Created on: 16/02/2010
 *      Author: podonoghue
 */

/** \file
 *   \brief Debugging message file
 *
 *   This file provides mappings from various code numbers to strings.\n
 *   It is mostly used for debugging messages.
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#if defined(DEBUG_BUILD)

/**
 * \brief Maps a command code to a string
 *
 * @param command Command number
 *
 * @return pointer to static string describing the command
 */
const char *getCommandName(unsigned char command) {
   //! Command String from Command #
   static const char *const newCommandTable[]= {
      "CMD_USBDM_GET_COMMAND_RESPONSE"          , // 0
      "CMD_USBDM_SET_TARGET"                    , // 1
      "CMD_USBDM_SET_VDD"                       , // 2
      "CMD_USBDM_DEBUG"                         , // 3
      "CMD_USBDM_GET_BDM_STATUS"                , // 4
      "CMD_USBDM_GET_CAPABILITIES"              , // 5
      "CMD_USBDM_SET_OPTIONS"                   , // 6
      NULL                                      , // 7
      "CMD_USBDM_CONTROL_PINS"                  , // 8
      NULL                                      , // 9
      NULL                                      , // 10
      NULL                                      , // 11
      "CMD_USBDM_GET_VER"                       , // 12
      NULL                                      , // 13
      "CMD_USBDM_ICP_BOOT"                      , // 14

      "CMD_USBDM_CONNECT"                       , // 15
      "CMD_USBDM_SET_SPEED"                     , // 16
      "CMD_USBDM_GET_SPEED"                     , // 17

      "CMD_USBDM_CONTROL_INTERFACE"             , // 18
      NULL                                      , // 19

      "CMD_USBDM_READ_STATUS_REG"               , // 20
      "CMD_USBDM_WRITE_CONROL_REG"              , // 21

      "CMD_USBDM_TARGET_RESET"                  , // 22
      "CMD_USBDM_TARGET_STEP"                   , // 23
      "CMD_USBDM_TARGET_GO"                     , // 24
      "CMD_USBDM_TARGET_HALT"                   , // 25

      "CMD_USBDM_WRITE_REG"                     , // 26
      "CMD_USBDM_READ_REG"                      , // 27

      "CMD_USBDM_WRITE_CREG"                    , // 28
      "CMD_USBDM_READ_CREG"                     , // 29

      "CMD_USBDM_WRITE_DREG"                    , // 30
      "CMD_USBDM_READ_DREG"                     , // 31

      "CMD_USBDM_WRITE_MEM"                     , // 32
      "CMD_USBDM_READ_MEM"                      , // 33

      "CMD_USBDM_READ_ALL_CORE_REGS"            , // 34
      "CMD_USBDM_RS08_FLASH_ENABLE - removed"   , // 35
      "CMD_USBDM_RS08_FLASH_STATUS - removed"   , // 36
      "CMD_USBDM_RS08_FLASH_DISABLE - removed"  , // 37

      "CMD_USBDM_JTAG_GOTORESET"                , // 38
      "CMD_USBDM_JTAG_GOTOSHIFT"                , // 39
      "CMD_USBDM_JTAG_WRITE"                    , // 40
      "CMD_USBDM_JTAG_READ"                     , // 41
      "CMD_USBDM_SET_VPP"                       , // 42,
      "CMD_USBDM_JTAG_READ_WRITE"               , // 43,
      "CMD_USBDM_JTAG_EXECUTE_SEQUENCE"         , // 44,
   };

   char const *commandName = NULL;

   command &= ~0x80;

   if (command < sizeof(newCommandTable)/sizeof(newCommandTable[0])) {
         commandName = newCommandTable[command];
   }
   if (commandName == NULL) {
      commandName = "UNKNOWN";
   }
   return commandName;
}

/**
 * \brief Maps a Debug Command # to a string
 *
 * @param cmd Debug command number
 *
 * @return pointer to static string describing the command
 */
const char *getDebugCommandName(unsigned char cmd) {
   //! Debug command string from code
   static const char *const debugCommands[] = {
      "ACKN",                        // 0
      "SYNC",                        // 1
      "Test Port",                   // 2
      "USB Disconnect",              // 3
      "Find Stack size",             // 4
      "Vpp Off",                     // 5
      "Vpp On",                      // 6
      "Flash 12V Off",               // 7
      "Flash 12V On",                // 8
      "Vdd Off",                     // 9
      "Vdd 3.3V On",                 // 10
      "Vdd 5V On",                   // 11
      "Cycle Vdd",                   // 12
      "Measure Vdd",                 // 13
      "Measure RS08 Trim - deleted", // 14
      "Test WAITS",                  // 15 //!< - Tests the software counting delays used for BDM communication. (locks up BDM!)
      "Test ALT Speed",              // 16
      "Test BDM Tx Routine",         // 17
      "SWD test",                    // 18
   };

   char const *cmdName = NULL;
   if (cmd < sizeof(debugCommands)/sizeof(debugCommands[0])) {
      cmdName = debugCommands[cmd];
   }
   if (cmdName == NULL) {
      cmdName = "UNKNOWN";
   }
   return cmdName;
}
#endif // LOG

