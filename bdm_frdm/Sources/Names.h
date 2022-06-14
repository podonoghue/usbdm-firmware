/*!
 * \brief Header file for CmdTable.c
*/
#ifndef _NAMES_H_
#define _NAMES_H_

#if defined(DEBUG_BUILD)

char const *getCommandName(unsigned char command);

#else // defined(LOG) || defined(NEED_ALL_NAMES)

// Dummy the routines if logging is not required
static inline char const *getCommandName(unsigned char command)                         { (void) command; return ""; }

#endif

#endif // _NAMES_H_
