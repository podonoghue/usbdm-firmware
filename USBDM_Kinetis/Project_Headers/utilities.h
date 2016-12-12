/**
 * @file     utilities.h
 * @brief    Convenience macros for port access
 * @version  V4.11.1.70
 * @date     13 May 2013
 */
#ifndef UTILTIES_H_
#define UTILTIES_H_

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __BYTE_ORDER__
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define leToNative32(x) __REV(x)
#define leToNative16(x) __REV16(x)
#define nativeToLe32(x) __REV(x)
#define nativeToLe16(x) __REV16(x)
#define beToNative32(x) (x)
#define beToNative16(x) (x)
#define nativeToBe32(x) (x)
#define nativeToBe16(x) (x)
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define leToNative32(x) (x)
#define leToNative16(x) (x)
#define nativeToLe32(x) (x)
#define nativeToLe16(x) (x)
#define beToNative32(x) __REV(x)
#define beToNative16(x) __REV16(x)
#define nativeToBe32(x) __REV(x)
#define nativeToBe16(x) __REV16(x)
#else
#error "Unexpected __BYTE_ORDER__ value"
#endif
#endif // __BYTE_ORDER__

#if defined(DEBUG_BUILD)
#define PUTS(x)     puts(x)
#define PRINTF(...) printf (__VA_ARGS__)
#else
#define PUTS(x)
#define PRINTF(...)
#endif

#ifdef __cplusplus
   }
#endif

#ifdef __cplusplus

class uint16_le {
   uint16_t value;

public:
   /**
    * @return Value as 16-bit unsigned in native format
    */
   operator uint16_t() const {
      return leToNative16(value);
   }
   /**
    * @return Lower byte of value as 8-bit unsigned value
    */
   uint8_t lo() {
      return leToNative16(value)&0xFF;
   }
   /**
    * @return Upper byte of value as 8-bit unsigned value
    */
   uint8_t hi() {
      return (leToNative16(value)>>8)&0xFF;
   }
};

class uint32_le {
   uint32_t value;

public:
   /**
    * @return Value as 32-bit unsigned in native format
    */
   operator uint32_t() const {
      return leToNative32(value);
   }
   /**
    * @return Lower 16-bits of value as unsigned value
    */
   uint16_t lo() {
      return leToNative16(value)&0xFFFF;
   }
   /**
    * @return Upper 16-bits of value as unsigned value
    */
   uint16_t hi() {
      return (leToNative16(value)>>16)&0xFFFF;
   }
   /**
    * @return Lowest byte of value as unsigned value
    */
   uint16_t b0() {
      return leToNative16(value)&0xFF;
   }
   /**
    * @return Higher-middle byte of value as unsigned value
    */
   uint16_t b1() {
      return (leToNative16(value)>>8)&0xFF;
   }
   /**
    * @return Lower-middle byte of value as unsigned value
    */
   uint16_t b2() {
      return (leToNative16(value)>>16)&0xFF;
   }
   /**
    * @return Highest byte of value as unsigned value
    */
   uint16_t b3() {
      return (leToNative16(value)>>24)&0xFF;
   }
};

#endif /* __cplusplus */

#endif /* UTILTIES_H_ */
