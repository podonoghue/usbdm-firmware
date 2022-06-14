/**
 * @file     utilities.h (derived from 100.ARM_DeviceOptions/Project_Headers/utilities-mk.h)
 * @brief    Utility Routines
 * @version  V4.12.1.160
 * @date     13 May 2013
 */
#ifndef PROJECT_HEADERS_UTILTIES_H_
#define PROJECT_HEADERS_UTILTIES_H_

#include <stdint.h>

/**
 * @brief Concatenate two tokens
 *
 * @param x
 * @param y
 */
#define CONCAT2_(x,y) x ## y
/**
 * @brief Concatenate three tokens
 *
 * @param x
 * @param y
 * @param z
 */
#define CONCAT3_(x,y,z) x ## y ## z
/**
 * @brief Concatenate four tokens
 *
 * @param w
 * @param x
 * @param y
 * @param z
 */
#define CONCAT4_(w,x,y,z) w ## x ## y ## z

/**
 * @brief Create PCR register from port name and bit number
 *
 * @param port Port name e.g. A
 * @param num  Bit number e.g. 3
 */
#define PCR(port,num)          CONCAT2_(PORT,port)->PCR[num]

/**
 * @brief Create GPIO register from port name
 *
 * @param port Port name e.g. A => GPIOA
 */
#define GPIO(port)             CONCAT2_(GPIO,port)

/**
 * @brief Create PDOR register from port name
 *
 * @param port Port name e.g. A => GPIOA->PDOR
 */
#define PDOR(port)             CONCAT2_(GPIO,port)->PDOR
/**
 * @brief Create PSOR register from port name
 *
 * @param port Port name e.g. A => GPIOA->PSOR
 */
#define PSOR(port)             CONCAT2_(GPIO,port)->PSOR
/**
 * @brief Create PCOR register from port name
 *
 * @param port Port name e.g. A => GPIOA->PCOR
 */
#define PCOR(port)             CONCAT2_(GPIO,port)->PCOR
/**
 * @brief Create PTOR register from port name
 *
 * @param port Port name e.g. A => GPIOA-PTOR
 */
#define PTOR(port)             CONCAT2_(GPIO,port)->PTOR
/**
 * @brief Create PDIR register from port name
 *
 * @param port Port name e.g. A => GPIOA->PDIR
 */
#define PDIR(port)             CONCAT2_(GPIO,port)->PDIR
/**
 * @brief Create PDDR register from port name
 *
 * @param port Port name e.g. A => GPIOA->PDDR
 */
#define PDDR(port)             CONCAT2_(GPIO,port)->PDDR

/**
 * @brief Create Clock register mask from port name
 *
 * @param port Port name e.g. A => SIM_SCGC5_PORTA_MASK
 */
#define PORT_CLOCK_MASK(port)  CONCAT4_(SIM_SCGC5,_PORT,port,_MASK)

#ifndef __BYTE_ORDER__
#error "__BYTE_ORDER__ value not defined"
#endif

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define leToNative32(x) (uint32_t)__REV(x)
#define leToNative16(x) (uint16_t)__REV16(x)
#define nativeToLe32(x) (uint32_t)__REV(x)
#define nativeToLe16(x) (uint16_t)__REV16(x)
#define beToNative32(x) (uint32_t)(x)
#define beToNative16(x) (uint16_t)(x)
#define nativeToBe32(x) (uint32_t)(x)
#define nativeToBe16(x) (uint16_t)(x)
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define leToNative32(x) (uint32_t)(x)
#define leToNative16(x) (uint16_t)(x)
#define nativeToLe32(x) (uint32_t)(x)
#define nativeToLe16(x) (uint16_t)(x)
#define beToNative32(x) (uint32_t)__REV(x)
#define beToNative16(x) (uint16_t)__REV16(x)
#define nativeToBe32(x) (uint32_t)__REV(x)
#define nativeToBe16(x) (uint16_t)__REV16(x)
#else
#error "Unexpected __BYTE_ORDER__ value"
#endif

#if defined(__cplusplus) && NEED_ENDIAN_CONVERSIONS
/**
 * Class to encapsulate 16-bit little-endian values
 */
class uint16_le {
private:
   uint16_t value;

public:
   /**
    * @return Value as 16-bit unsigned in native format
    */
   operator uint16_t() const volatile {
      return leToNative16(value);
   }
   /**
    * @return Lower byte of value as 8-bit unsigned value
    */
   uint8_t lo() const volatile {
      return leToNative16(value)&0xFF;
   }
   /**
    * @return Upper byte of value as 8-bit unsigned value
    */
   uint8_t hi() const volatile {
      return (leToNative16(value)>>8)&0xFF;
   }
};

/**
 * Class to encapsulate 32-bit little-endian values
 */
class uint32_le {
private:
   uint32_t value;

public:
   /**
    * @return Value as 32-bit unsigned in native format
    */
   operator uint32_t() const volatile {
      return leToNative32(value);
   }
   /**
    * @return Lower 16-bits of value as unsigned value
    */
   uint16_t lo() const volatile {
      return leToNative16(value)&0xFFFF;
   }
   /**
    * @return Upper 16-bits of value as unsigned value
    */
   uint16_t hi() const volatile {
      return (leToNative16(value)>>16)&0xFFFF;
   }
   /**
    * @return Lowest byte of value as unsigned value
    */
   uint16_t b0() const volatile {
      return leToNative16(value)&0xFF;
   }
   /**
    * @return Lower-middle byte of value as unsigned value
    */
   uint16_t b1() const volatile {
      return (leToNative16(value)>>8)&0xFF;
   }
   /**
    * @return Upper-middle byte of value as unsigned value
    */
   uint16_t b2() const volatile {
      return (leToNative16(value)>>16)&0xFF;
   }
   /**
    * @return Uppermost byte of value as unsigned value
    */
   uint16_t b3() const volatile {
      return (leToNative16(value)>>24)&0xFF;
   }
};

/**
 * Pack 4 bytes into a 32-bit value in LITTLE-ENDIAN order
 *
 * @param  data Data value in LITTLE-ENDIAN order
 *
 * @return Value
 */
static inline
constexpr uint32_t pack32LE(const uint8_t data[4]) {
   return data[0]+(data[1]<<8)+(data[2]<<16)+(data[3]<<24);
}

/**
 * Pack 4 bytes into a 32-bit value in BIG-ENDIAN order
 *
 * @param  data Data value in BIG_ENDIAN order
 *
 * @return Value
 */
static inline
constexpr uint32_t pack32BE(const uint8_t data[4]) {
   return (data[0]<<24)+(data[1]<<16)+(data[2]<<8)+data[3];
}

/**
 * Pack 2 bytes into a 16-bit value in LITTLE-ENDIAN order
 *
 * @param  data Data value in LITTLE-ENDIAN order
 *
 * @return Value
 */
static inline
constexpr uint32_t pack16LE(const uint8_t data[2]) {
   return data[0]+(data[1]<<8);
}

/**
 * Pack 2 bytes into a 16-bit value in BIG-ENDIAN order
 *
 * @param  data Data value in BIG_ENDIAN order
 *
 * @return Value
 */
static inline
constexpr uint32_t pack16BE(const uint8_t data[2]) {
   return (data[0]<<8)+data[1];
}

/**
 * Unpack a 32-bit value into 4 bytes in LE order
 *
 * @param  data    Value to unpack
 * @param  ar   Buffer for data value in LITTLE-ENDIAN order
 *
 * @return Value
 */
static inline
void unpack32LE(uint32_t data, uint8_t ar[4]) {
   ar[3] = data>>24;
   ar[2] = data>>16;
   ar[1] = data>>8;
   ar[0] = data;
}

/**
 * Unpack a 32-bit value into 4 bytes in BE order
 *
 * @param  data    Value to unpack
 * @param  ar      Buffer for data value in BIG-ENDIAN order
 *
 * @return Value
 */
static inline
void unpack32BE(uint32_t data, uint8_t ar[4]) {
   ar[0] = data>>24;
   ar[1] = data>>16;
   ar[2] = data>>8;
   ar[3] = data;
}

/**
 * Unpack a 32-bit value into 4 bytes in LE order
 *
 * @param  data    Value to unpack
 * @param  ar      Buffer for data value in LITTLE-ENDIAN order
 *
 * @return Value
 */
static inline
void unpack16LE(uint32_t data, uint8_t ar[2]) {
   ar[1] = data>>8;
   ar[0] = data;
}

/**
 * Unpack a 32-bit value into 4 bytes in BE order
 *
 * @param  data    Value to unpack
 * @param  ar      Buffer for data value in BIG-ENDIAN order
 *
 * @return Value
 */
static inline
void unpack16BE(uint32_t data, uint8_t ar[2]) {
   ar[0] = data>>8;
   ar[1] = data;
}

#endif /* __cplusplus */

#endif /* PROJECT_HEADERS_UTILTIES_H_ */
