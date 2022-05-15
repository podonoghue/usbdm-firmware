/*! \file
    \brief Main command procedure for executing BDM commands received over the USB.
*/    
#ifndef _CMDPROCESSING_H_
#define _CMDPROCESSING_H_

#include "BDM.h"

extern void commandLoop(void);
extern uint8_t   compatibleCommandExec(void);
extern uint8_t   optionalReconnect(uint8_t when);

extern uint8_t  commandBuffer[]; // Buffer for USB command in, result out

#ifdef __HC08__
#pragma DATA_SEG __SHORT_SEG Z_PAGE
#endif // __HC08__
extern uint8_t          returnSize;       // Size of command return result
extern BDM_Option_t     bdm_option;       // Options for cable operation
extern CableStatus_t    cable_status;     // Status of the BDM interface
#ifdef __HC08__
#pragma DATA_SEG DEFAULT
#endif

#endif
