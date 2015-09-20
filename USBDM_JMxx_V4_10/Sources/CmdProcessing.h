/*! \file
    \brief Main command procedure for executing BDM commands received over the USB.
*/    
#ifndef _CMDPROCESSING_H_
#define _CMDPROCESSING_H_

extern void commandLoop(void);
extern U8   compatibleCommandExec(void);
extern U8   optionalReconnect(U8 when);

extern U8  commandBuffer[]; // Buffer for USB command in, result out

#ifdef __HC08__
#pragma DATA_SEG __SHORT_SEG Z_PAGE
#endif // __HC08__
extern U8               returnSize;       // Size of command return result
extern BDM_Option_t     bdm_option;       // Options for cable operation
extern CableStatus_t    cable_status;     // Status of the BDM interface
#pragma DATA_SEG DEFAULT

#endif;
