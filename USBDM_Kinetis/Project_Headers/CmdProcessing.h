/*! \file
    \brief Main command procedure for executing BDM commands received over the USB.
*/    
#ifndef _CMDPROCESSING_H_
#define _CMDPROCESSING_H_

#include "BDM.h"

extern void commandLoop(void);
extern U8   compatibleCommandExec(void);
extern U8   optionalReconnect(U8 when);

extern U8  commandBuffer[]; // Buffer for USB command in, result out
extern BDM_Option_t     bdm_option;       // Options for cable operation

#endif
