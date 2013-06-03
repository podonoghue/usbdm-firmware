/*
 * icp.h
 *
 *  Created on: 01/03/2012
 *      Author: podonoghue
 */

#ifndef ICP_H_
#define ICP_H_

void rebootToICP(void);
void rebootToUser(void);
#define MAGIC_REBOOT_KEY_VALUE (0xFF00EE55UL)

typedef struct {
   uint32_t  validKey;          // Must be MAGIC_REBOOT_KEY_VALUE to indicate valid structure
   uint32_t  *userFlashStart;   // Start of user flash memory
   uint32_t  *userFlashEnd;     // End of user flash memory 
   void     (*startUp)(void);   // User startup routine
   uint32_t  *initialSP;        // User initial SP
} ICPData;

extern ICPData icpData;

#endif /* ICP_H_ */
