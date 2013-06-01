#ifndef _ICP_H_
#define _ICP_H_

/*!  Used to check for ICP during reset
 *
 *  This function must be provided in user code.
 *  It is called from the ICP boot code to
 *  determine if the user code wants ICP on this
 *  reset.
 *
 *  See main.c.
 *
 *  @return \n
 *           0 - normal boot \n
 *           1 - ICP boot
*/
extern U8 userDetectICP(void);

#define ICP_FORCE_LOCATION (0xFFDE)
/*! Force ICP execution on next reset
**
**  This function is provided in the ICP flash area and
**  accessed through a vector in a fixed location.
**  This function may be called from user code to 
**  reboot into ICP mode
*/  
#define forceICPReset (*(void(** far)(void)) ICP_FORCE_LOCATION)

#define ICP_VERSION_SW_LOCATION (0xF800)
#define ICP_VERSION_HW_LOCATION (ICP_VERSION_SW_LOCATION+1)
/*!  Version number of ICP boot code
 *   2 hex digits major.minor
 */
extern const U8 ICP_Version_SW;
/*!  Hardware Version number - see Configure.h
 *   2 hex digits
 *   JM60 has +0x80
 */
extern const U8 ICP_Version_HW;

//! Type for vector table entry
typedef struct {
   char JMP;
   void (* const address)(void);
} UserVector;


#define USER_VECTORTABLE_LOCATION  (0xF7CC)
/* User vector table
**
**  The vector table is relocated to the top of User Flash
**  This is a jump table!
**
**  See UserVectorTable.c
**
*/
extern const UserVector userVectorTable[17] @USER_VECTORTABLE_LOCATION;

#endif