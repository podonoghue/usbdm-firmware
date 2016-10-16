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
} U32u;

//! Used for manipulating Little/Big-Endian words (16 bits)
//!
typedef union {
   uint8_t  bytes[2];                     //!< Treat as array of bytes
   uint16_t word;                         //!< Treat as native 16-bit value
   struct {uint8_t lo; uint8_t hi;} le;   //!< Little-endian order
   struct {uint8_t hi; uint8_t lo;} be;   //!< Big-endian order
} U16u;


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
