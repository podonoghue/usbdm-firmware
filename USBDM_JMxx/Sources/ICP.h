#ifndef _ICP_H_
#define _ICP_H_

//! ICP version (2 hex digits, major.minor)
#define ICP_VERSION_SW (2<<4|5) // 2.5

/*! Flash Protection boundary
 *
 *  Flash is protected from this address upwards
 *  This region of Flash is used for ICP code


 */ 
#define FLASH_PROTECT_START (0xFC00)

#if (FLASH_PROTECT_START&0x1FF)
#error "Illegal Flash protection boundary - must be multiple of 512"
#endif

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

//!  Structure to contain ICP needed information
//!
typedef struct {
   U8 serialNumber[39];
   U8 terminator;
   U8 *flashStart;
   U8 hardwareVersion;
   U8 softwareVersion;
   U8 (*userDetectICP)(void);
} ICP_dataType;

/*! Various ICP data
**
*/
extern const ICP_dataType ICP_data;
//======================================
//! Located at top of Bootloader memory
//! 0xFFFA | Force ICP vector
//! 0xFFFB | 
//! 0xFFFC | VERSION_SW
//! 0xFFFD | VERSION_HW
//! 0xFFFE | Reset vector
//! 0xFFFF |

#define ICP_FORCE_LOCATION (0xFFFA)
/*! Force ICP execution on next reset
**
**  This function is provided in the ICP flash area and
**  accessed through this vector in a fixed location.
**  This function may be called from user code to 
**  reboot into ICP mode
*/  
#define forceICPReset (*(void(** far)(void)) ICP_FORCE_LOCATION)
#define ICP_VERSION_SW_LOCATION     (0xFFFC)
#define ICP_VERSION_HW_LOCATION     (ICP_VERSION_SW_LOCATION+1)

/*!  Version number of ICP boot code
 *   2 hex digits major.minor
 */
extern const U8 ICP_Version_SW;
/*!  Hardware Version number - see Configure.h
 *   2 hex digits
 *   JM60 has +0x80
 */
extern const U8 ICP_Version_HW;

//! Type for user vector table entry
typedef void (* const vector)(void);

/* User vector table
**
**  The vector table is relocated to the top of User Flash
**
**  See UserVectorTable.c
**
*/
extern const vector userVectorTable[30];

#endif
