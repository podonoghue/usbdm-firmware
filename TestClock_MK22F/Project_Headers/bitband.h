/**
 * @file     bitband.h (derived from bitband-cpp-mk.h)
 * @brief    Template functions to access bit-band region
 * @version  V4.12.1.50
 * @date     5 Dec 2015
 */
#ifndef INCLUDE_CPP_BITBAND_H_
#define INCLUDE_CPP_BITBAND_H_

namespace USBDM {

/*!
 * @addtogroup BITBAND_group Bit-band access
 * @brief Cortex-M4 bit-band support
 * @{
 *
 * Based on Freescale Application Note AN4838
 * http://cache.freescale.com/files/microcontrollers/doc/app_note/AN4838.pdf
 *
 * Assumes the location has been allocated to:
 *    - SRAM_U           (0x2000_0000-0x200F_FFFF) with bit-band alias (0x2200_0000-0x23FF_FFFF)
 *    - AIPS/GPIO        (0x4000_0000-0x400F_FFFF) with bit-band alias (0x4200_0000-0x43FF_FFFF)
 *
 * Most Global or Local Static variables end up in SRAM_L so are not usable\n
 * Most Parameters and Local variables end up in SRAM_U
 *
 * Examples:
 *
 * Write a bit to a register
 * @code{.c}
 * bitbandWrite(GPIOA->PDOR, 3, true); // Set bit # 3 of port
 * @endcode
 *
 * Set bit of register
 * @code{.c}
 * bitbandSet(SIM->SCGC5, SIM_SCGC5_PORTA_SHIFT); // Enable clock to PORTA
 * @endcode
 *
 * Assuming a local variable 'local'
 * @code{.c}
 * bitbandClear(local, 3); // Clear bit 3 of local
 * @endcode
 *
 * Note: The efficiency of this operation depends greatly on whether the address is a constant or can be pre-calculated.
 *       The code will usually be quite inefficient for local variables as the required address will be
 *       calculated each time i.e. the address calculation will overwhelm any advantage of the bitband operation
 *
 *       It is probably a good idea to use a dedicated segment in SRAM_U for bit-band variables and access globally.
 *       See linker files.
 *
 *       Fixed locations accessed by casts e.g. GPIOC etc will be very efficient since the calculation is a constant expression.
 *
 *       The generated code will be very inefficient if optimization is not enabled.
 */

/**
 * @brief Write a value to a bit.
 * Only the referenced bit is modified.
 *
 * <b>Example</b> - Write value to bit of register
 * @code{.c}
 * bitbandWrite(GPIOA->PDOR, 3, true); // Set bit # 3 of port
 * @endcode
 *
 * @param[out] ref     Object to manipulate e.g. a register
 * @param[in]  bitNum  Bit number
 * @param[in]  value   Value to modify 0 or 1. Only the LSB is used
 */
template <typename T>
static __attribute__((always_inline)) inline void bitbandWrite(T &ref, const int bitNum, uint32_t value) {
   uint32_t addr = (uint32_t)(&ref);
   const uint32_t mappedAddress = (addr&0xF0000000) + 0x02000000 + ((addr&0xFFFFF)*8*4) + (bitNum*4);
   *(volatile uint32_t *)(mappedAddress) = value;
}

/**
 * @brief Read a bit.
 * Only the referenced bit is returned.
 *
 * <b>Example</b> - Read value of bit withon a register
 * @code{.c}
 * if (bitbandRead(GPIOA->PDIR, 3) {
 *    // Only done if bit # 3 of port is high
 * }
 * @endcode
 *
 * @param[out] ref     Object to examine e.g. a register
 * @param[in]  bitNum  Bit number
 *
 * @return Bit read as boolean value
 */
template <typename T>
static __attribute__((always_inline)) inline uint32_t bitbandRead(T &ref, const int bitNum) {
   uint32_t addr = (uint32_t)(&ref);
   const uint32_t mappedAddress = (addr&0xF0000000) + 0x02000000 + ((addr&0xFFFFF)*8*4) + (bitNum*4);
   return *(volatile uint32_t *)(mappedAddress);
}

/**
 * @brief Set a bit.
 * Only the referenced bit is modified.
 *
 * <b>Example</b> - Set bit of register
 * @code{.c}
 * bitbandSet(SIM->SCGC5, SIM_SCGC5_PORTA_SHIFT); // Enable clock to PORTA
 * @endcode
 *
 * @param[out] ref     Object to manipulate e.g. a register
 * @param[in]  bitNum  Bit number
 */
template <typename T>
static __attribute__((always_inline)) inline void bitbandSet(T &ref, const int bitNum) {
   bitbandWrite(ref, bitNum, true);
}

/**
 * @brief Clear a bit.
 * Only the referenced bit is modified.
 *
 * <b>Example</b> - Clear bit of register
 * @code{.c}
 * bitbandClear(SIM->SCGC5, SIM_SCGC5_PORTA_SHIFT); // Disable clock to PORTA
 * @endcode
 *
 * @param[out] ref     Object to manipulate e.g. a register
 * @param[in]  bitNum  Bit number
 */
template <typename T>
static __attribute__((always_inline)) inline void bitbandClear(T &ref, const int bitNum) {
   bitbandWrite(ref, bitNum, false);
}

/*!
 * @}
 */

} // End namespace USBDM

#endif /* INCLUDE_CPP_BITBAND_H_ */
