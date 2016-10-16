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

#ifdef __cplusplus

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

#endif

#ifdef __cplusplus
   }
#endif

#endif /* UTILTIES_H_ */
