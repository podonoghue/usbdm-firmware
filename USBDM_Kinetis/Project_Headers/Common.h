/*! \file
    \brief Some common shared defintions.
*/
#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include "Configure.h"

//! 24-bit value
typedef struct {
	char data[3]; //!< 3 bytes representing the value
} U24;  //!< unsigned 24-bit value

//! Used for manipulating Little/Big-Endian longwords (32 bits)
//!
typedef union {
    uint32_t longword;          //!< Treat as native 32-bit value
    uint8_t  bytes[4];          //!< Treat as array of bytes
} uint32_le;

//! Used for manipulating Little/Big-Endian words (16 bits)
//!
typedef union {
   uint8_t  bytes[2];                     //!< Treat as array of bytes
   uint16_t word;                         //!< Treat as native 16-bit value
   struct {uint8_t lo; uint8_t hi;} le;   //!< Little-endian order
   struct {uint8_t hi; uint8_t lo;} be;   //!< Big-endian order
} uint16_le;

#ifdef __BYTE_ORDER__
   #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
      // Macros only for use with literals!
      //! Convert Native constant to 32-bit Little-Endian
      #define CONST_NATIVE_TO_LE32(x) ((((x)<<24UL)&0xFF000000UL)+(((x)<<8UL)&0xFF0000UL)+(((x)>>8UL)&0xFF00UL)+(((x)>>24UL)&0xFFUL))
      //! Convert Native constant to 32-bit Big-Endian
      #define CONST_NATIVE_TO_BE32(x) (x)
      //! Convert Native constant to 16-bit Little-Endian
      #define CONST_NATIVE_TO_LE16(x) (((((x)&0xFF))<<8)+(((x)>>8)&0xFF))
      //! Convert Native constant to 12-bit Big-Endian
      #define CONST_NATIVE_TO_BE16(x) (x)
      #define LE_TO_NATIVE16(x) ((((x)<<16)&0xFF00)|(((x)>>16)&0xFF))
   #elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      // Macros only for use with literals!
      //! Convert Native constant to 32-bit Little-Endian
      #define CONST_NATIVE_TO_LE32(x) (x)
      //! Convert Native constant to 32-bit Big-Endian
      #define CONST_NATIVE_TO_BE32(x) ((((x)<<24)&0xFF000000)+(((x)<<8)&0xFF0000)+(((x)>>8)&0xFF00)+(((x)>>24)&0xFF))
      //! Convert Native constant to 16-bit Little-Endian
      #define CONST_NATIVE_TO_LE16(x) (x)
      //! Convert Native constant to 12-bit Big-Endian
      #define CONST_NATIVE_TO_BE16(x) (((((x)&0xFF))<<8)+(((x)>>8)&0xFF))
      #define LE_TO_NATIVE16(x) (x)
   #endif
#else
   #error "Please define __BYTE_ORDER__"
#endif

#ifdef __GNUC__
#define __forceinline __attribute__((always_inline))
#endif

#if defined(DEBUG_BUILD)
#define PUTS(x) puts(x)
#define PRINTF(...) printf (__VA_ARGS__)
#else
#define PUTS(x)
#define PRINTF(...)
#endif

#endif //_COMMON_H_
