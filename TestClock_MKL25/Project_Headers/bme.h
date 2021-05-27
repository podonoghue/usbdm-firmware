/**
 * @file     bme.h (derived from bme-cpp.h)
 * @brief    Macros to access bit manipulation engine
 * @version  V4.12.1.50
 * @date     5 Dec 2015
 */
#ifndef INCLUDE_BME_CPP_H
#define INCLUDE_BME_CPP_H

#include <stdint.h>

namespace USBDM {

/*!
 * @addtogroup BME_group Bit Manipulation Engine
 * @brief Macros to support BME
 * @{
 *
 * Based on Freescale Application Note AN4838
 * http://cache.freescale.com/files/microcontrollers/doc/app_note/AN4838.pdf
 *
 * Assumes the location has been allocated to:
 *    - AIPS Peripherals (0x4000_0000-0x4007_FFFF) with BME alias (0x4400_0000-0x5FFF_FFFF)
 *    - SRAM_U           (0x2000_0000-0x2000_02FF) with BME alias (0x2400_0000-0x3FFF_FFFF) (MKE04 & MKE06 only)
 *
 * <b>Note</b> These operations vary in size with the type of the 1st parameter.
 *
 * <b>Examples</b>
 *
 * Setting a clock enable bit within SIM_SCGC
 * @code{.c}
 *    bmeTestAndSet(SIM->SCGC6, SIM_SCGC6_ADC0_SHIFT); // Set SIM_SCGC6.ADC0 bit
 *    -- OR --
 *    bmeOr(SIM->SCGC6, SIM_SCGC6_ADC0_MASK);          // Set SIM_SCGC6.ADC0 bit
 * @endcode
 *
 * Clearing a clock enable bit within SIM_SCGC
 * @code{.c}
 *    bmeTestAndClear(SIM->SCGC6, SIM_SCGC6_ADC0_SHIFT); // Clear SIM_SCGC6.ADC0 bit
 *    -- OR --
 *    bmeAnd(SIM->SCGC6, ~SIM_SCGC6_ADC0_MASK);          // Clear SIM_SCGC6.ADC0 bit
 * @endcode
 *
 * Setting multiple bits
 * @code{.c}
 *    bmeOr(GPIOC->PDOR, (3<<4)); // Set bits 4 and 5
 * @endcode
 *
 * Clearing multiple bits
 * @code{.c}
 *    bmeAnd(GPIOC->PDOR,~(7<<4)); // Clear bits 4, 5 and 7
 * @endcode
 *
 * Toggling multiple bits
 * @code{.c}
 *    bmeXor(GPIOC->PDOR,~(0xF<<4)); // Toggle bits 4 to 7
 * @endcode
 *
 * Spin-lock - Wait until successful at setting the bit i.e. the bit was 0 and now is 1
 * @code{.c}
 *    while(!bmeTestAndSet(lock, 3)) {
 *    }
 * @endcode
 */

// BME operation code

constexpr uint8_t BME_OPCODE_AND        = 1; //!< Opcode for ANDing a value
constexpr uint8_t BME_OPCODE_OR         = 2; //!< Opcode for ORing a value
constexpr uint8_t BME_OPCODE_XOR        = 3; //!< Opcode for XORing a value
constexpr uint8_t BME_OPCODE_BITFIELD   = 4; //!< Opcode for extracting a bit-field

constexpr uint8_t BME_OPCODE_BIT_CLEAR  = 2; //!< Opcode for Clearing a bit
constexpr uint8_t BME_OPCODE_BIT_SET    = 3; //!< Opcode for Setting a bit

/**
 *  Create decorated address for bme operation
 *
 *  @param addr   Base address
 *  @param opcode Opcode to use
 *
 * <b>Examples - </b>
 *
 */
static constexpr __attribute__((always_inline)) inline uint32_t bmeOp(uint32_t addr, uint8_t opcode) {
   return static_cast<uint32_t>((opcode<<26)|addr);
}

/**
 *  Create decorated address for bme operation
 *
 *  @param addr      Base address
 *  @param opcode    Opcode to use
 *  @param bitOffset Offset of start of field
 *
 * <b>Examples - </b>
 *
 */
static constexpr __attribute__((always_inline)) inline uint32_t bmeOp(const uint32_t addr, const uint8_t opcode, const uint8_t bitOffset) {
   return static_cast<uint32_t>((opcode<<26)|(bitOffset<<21)|addr);
}

/**
 *  Create decorated address for bme operation
 *
 *  @param addr      Base address
 *  @param opcode    Opcode to use
 *  @param bitOffset Offset of start of field
 *  @param width     Width of value to insert/extract
 *
 * <b>Examples - </b>
 *
 */
static constexpr __attribute__((always_inline)) inline uint32_t bmeOp(const uint32_t addr, const uint8_t opcode, const uint8_t bitOffset, const uint8_t width) {
   // Need to re-map GPIO from 0x400FF000 => 0x4000F000
   return ((addr & (0xFFFFF000)) == 0x400FF000)?
       (static_cast<uint32_t>((opcode<<26)|(bitOffset<<23)|(width<<19)|(addr&0xFFF0FFFF))):
       (static_cast<uint32_t>((opcode<<26)|(bitOffset<<23)|(width<<19)|addr));
}

/**
 * Mask value is ANDed with a memory location
 *
 * @param ref  Memory location
 * @param mask Fixed mask value to be used
 *
 * <b>Examples - </b>
 *
 * Clearing a clock enable bit within SIM_SCGC
 * @code{.c}
 *    bmeAnd(SIM->SCGC6, ~SIM_SCGC6_ADC0_MASK);          // Clear SIM_SCGC6.ADC0 bit
 * @endcode
 *
 * Clearing multiple bits
 * @code{.c}
 *    bmeAnd(GPIOC->PDOR,~(7<<4)); // Clear bits 4, 5 and 7
 * @endcode
 */
template<typename T> static constexpr __attribute__((always_inline)) inline void bmeAnd(T &ref, const uint32_t mask) {
   *(reinterpret_cast<T*>(bmeOp(reinterpret_cast<uint32_t>(&ref), BME_OPCODE_AND))) = mask;
}

/**
 * Mask value is ORed with a memory location
 *
 * @param ref  Memory location
 * @param mask Fixed mask value to be used
 *
 * <b>Examples - </b>
 *
 * Setting a clock enable bit within SIM_SCGC
 * @code{.c}
 *    bmeOr(SIM->SCGC6, SIM_SCGC6_ADC0_MASK);          // Set SIM_SCGC6.ADC0 bit
 * @endcode
 *
 * Setting multiple bits
 * @code{.c}
 *    bmeOr(GPIOC->PDOR, (3<<4)); // Set bits 4 and 5
 * @endcode
 */
template<typename T> static constexpr __attribute__((always_inline)) inline void bmeOr(T &ref, const uint32_t mask) {
   *(reinterpret_cast<T*>(bmeOp(reinterpret_cast<uint32_t>(&ref), BME_OPCODE_OR))) = mask;
}

/**
 * Mask value is XORed with a memory location
 *
 * @param ref  Memory location
 * @param mask Fixed mask value to be used
 *
 * <b>Example - </b>
 *
 * Toggling multiple bits
 * @code{.c}
 *    bmeXor(GPIOC->PDOR,~(0xF<<4)); // Toggle bits 4 to 7
 * @endcode
 */
template<typename T> static constexpr __attribute__((always_inline)) inline void bmeXor(T &ref, const uint32_t mask) {
   *(reinterpret_cast<T*>(bmeOp(reinterpret_cast<uint32_t>(&ref), BME_OPCODE_XOR))) = mask;
}

/**
 * Insert bit field within a memory location
 *
 * @param ref     Memory location
 * @param bitNum  Starting bit number of field
 * @param width   Width of bit field
 * @param value   Value to insert
 *
 * <b>Example - </b>
 *
 * Modify timer field without affecting other bits in register (note: read-modify-write)
 * @code{.c}
 *   bmeInsert(TPM0->CONTROLS[3].CnSC, TPM_CnSC_ELS_SHIFT, 2, 1); // Set TPM0->CONTROLS[3].CnSC.ELS to 1
 * @endcode
 */
template<typename T> static constexpr __attribute__((always_inline)) inline void bmeInsert(T &ref, const uint8_t bitNum, const uint8_t width, const uint32_t value) {
   *(reinterpret_cast<T*>(bmeOp(reinterpret_cast<uint32_t>(&ref), BME_OPCODE_BITFIELD, bitNum, width-1))) = value<<bitNum;
}

/**
 * Extract bit field from a memory location
 *
 * @param ref     Memory location
 * @param bitNum  Starting bit number of field
 * @param width   Width of bit field
 *
 * <b>Example - </b>
 *
 * Switch statement testing a 3-bit input control (bits 5,6,7)
 * @code{.c}
 * switch(bmeExtract(GPIOA->PDIR, 5, 3)) {
 * case 0: ...;
 *    //...
 * case 7: ...;
 * }
 * @endcode
 */
template<typename T> static constexpr __attribute__((always_inline)) inline uint32_t bmeExtract(T &ref, const uint8_t bitNum, const uint8_t width) {
   return *(reinterpret_cast<T*>(bmeOp(reinterpret_cast<uint32_t>(&ref), BME_OPCODE_BITFIELD, bitNum, width-1)));
}

/**
 * Bit test and clear a bit within a memory location
 * i.e. Clear 1 bit and return original bit value
 *
 * @param ref    Address of memory location
 * @param bitNum Number of bit to test and clear
 *
 * <b>Examples - </b>
 *
 * @code{.c}
 *    bmeTestAndClear(SIM->SCGC6, SIM_SCGC6_ADC0_SHIFT); // Clear SIM_SCGC6.ADC0 bit
 * @endcode
 *
 * Spin-lock - Wait until successful at clearing the bit i.e. the bit was 1 and now is 0
 * @code{.c}
 *    while(!bmeTestAndClear(lock, 3)) {
 *    }
 * @endcode
 */
template<typename T> static constexpr __attribute__((always_inline)) inline uint32_t bmeTestAndClear(T &ref, const uint8_t bitNum) {
   return *(reinterpret_cast<T*>(bmeOp(reinterpret_cast<uint32_t>(&ref), BME_OPCODE_BIT_CLEAR, bitNum)));
}

/**
 * Bit test and set a bit within a memory location
 * i.e. Set 1 bit and return original bit value
 *
 * @param ref    Address of memory location
 * @param bitNum Number of bit to test and set
 *
 * <b>Examples - </b>
 *
 * Setting a clock enable bit within SIM_SCGC
 * @code{.c}
 *    bmeTestAndSet(SIM->SCGC6, SIM_SCGC6_ADC0_SHIFT); // Set SIM_SCGC6.ADC0 bit
 * @endcode
 *
 * Spin-lock - Wait until successful at setting the bit i.e. the bit was 0 and now is 1
 * @code{.c}
 *    while(bmeTestAndClear(lock, 3)) {
 *    }
 * @endcode
 */
template<typename T> static constexpr __attribute__((always_inline)) inline uint32_t bmeTestAndSet(T &ref, const uint8_t bitNum) {
   return *(reinterpret_cast<T*>(bmeOp(reinterpret_cast<uint32_t>(&ref), BME_OPCODE_BIT_SET, bitNum)));
}

/*!
 * @}
 */

} // End namespace USBDM

#endif // INCLUDE_BME_CPP_H
