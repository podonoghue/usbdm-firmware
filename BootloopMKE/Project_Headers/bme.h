/**
 * @file     bme.h (derived from bme-c.h)
 * @brief    Macros to access bit manipulation engine
 * @version  V4.12.1.50
 * @date     5 Dec 2015
 */
#ifndef __BME_H
#define __BME_H

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @addtogroup BME_group Bit Manipulation Engine
 * @brief Macros to support BME
 * @{
 *
 *
 * Based on Freescale Application Note AN4838
 * http://cache.freescale.com/files/microcontrollers/doc/app_note/AN4838.pdf
 *
 * Assumes the location has been allocated to:
 *    - AIPS Peripherals (0x4000_0000-0x4007_FFFF) with BME alias (0x4400_0000-0x5FFF_FFFF)
 *    - SRAM_U           (0x2000_0000-0x2000_02FF) with BME alias (0x2400_0000-0x3FFF_FFFF) (MKE04 & MKE06 only)
 *
 * Examples:
 *
 * Setting a clock enable bit within SIM_SCGC
 * @code{.c}
 *    BME_BIT_TEST_AND_SETw(&SIM->SCGC6, SIM_SCGC6_ADC0_SHIFT); // Set SIM_SCGC6.ADC0 bit
 * @endcode
 *
 * Clearing a clock enable bit within SIM_SCGC
 * @code{.c}
 *    BME_BIT_TEST_AND_CLEARw(&SIM->SCGC6, SIM_SCGC6_ADC0_SHIFT); // Clear SIM_SCGC6.ADC0 bit
 * @endcode
 *
 * Settting multiple bits
 * @code{.c}
 *    BME_ORw(&SIM_SCGC,SIM_SCGC_ACMP0_MASK|SIM_SCGC_ACMP1_MASK);
 * @endcode
 *
 * Clearing multiple bits
 * @code{.c}
 *    BME_ANDw(&SIM_SCGC,~(SIM_SCGC_ACMP0_MASK|SIM_SCGC_ACMP1_MASK));
 * @endcode
 *
 * Spin-lock - Wait until successful at setting the bit i.e. the bit was 0 and now is 1
 * @code{.c}
 *    while(BME_BIT_TEST_AND_SET(&lock, 3) != 0) {
 *    }
 * @endcode
 */

// BME operation code
#define BME_OPCODE_AND           1 //!< Opcode for ANDing a value
#define BME_OPCODE_OR            2 //!< Opcode for ORing a value
#define BME_OPCODE_XOR           3 //!< Opcode for XORing a value
#define BME_OPCODE_BITFIELD      4 //!< Opcode for extracting a bitfield

#define BME_OPCODE_BIT_CLEAR     2 //!< Opcode for Clearing a bit
#define BME_OPCODE_BIT_SET       3 //!< Opcode for Setting a bit

/**
 * Mask value is ANDed with a 32 bit memory location
 *
 * @param addr Address of memory location
 * @param mask Fixed mask value to be used
 *
 */
#define BME_ANDw(addr, mask)  ((void)((*(volatile uint32_t *)(((uint32_t)(addr)) | (BME_OPCODE_AND<<26))) = ((uint32_t)(mask))))

/**
 * Mask value is ORed with a 32 bit memory location
 *
 * @param addr Address of memory location
 * @param mask Fixed mask value to be used
 *
 */
#define BME_ORw(addr, mask)   ((void)((*(volatile uint32_t *)(((uint32_t)(addr)) | (BME_OPCODE_OR<<26))) = ((uint32_t)(mask))))

/**
 * Mask value is XORed with a 32 bit memory location
 *
 * @param addr Address of memory location
 * @param mask Fixed mask value to be used
 *
 */
#define BME_XORw(addr, mask)  ((void)((*(volatile uint32_t *)(((uint32_t)(addr)) | (BME_OPCODE_XOR<<26))) = ((uint32_t)(mask))))


/**
 * Bit test and clear within 32 bit memory location
 * i.e. Clear 1 bit and return original bit value
 *
 * @param addr   Address of memory location
 * @param bitNum Numbert of bit to test
 *
 */
#define BME_BIT_TEST_AND_CLEARw(addr, bitNum)        (*(volatile uint32_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_CLEAR <<26)  \
                              | (((bitNum & 0x1F))<<21)))

/**
 * Bit test and set within 32 bit memory location
 * i.e. Set 1 bit and return original bit value
 *
 * @param addr   Address of memory location
 * @param bitNum Numbert of bit to test
 *
 */
#define BME_BIT_TEST_AND_SETw(addr, bitNum)        (*(volatile uint32_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_SET <<26)  \
                              | (((bitNum & 0x1F))<<21)))

/**
 * Insert bit field within a 32 bit memory location
 *
 * @param addr   Address of memory location
 * @param bit    Starting bit number of field
 * @param width  Width of bit field
 * @param value  Value to insert
 */
#define BME_BITFIELD_INSERTw(addr, bit, width, value)        ((void)((*(volatile uint32_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19)) = ((value)<<(bit))))

/**
 * Extract a bit field from a 32 bit memory location
 *
 * @param addr   Address of memory location
 * @param bit    Starting bit number of field
 * @param width  Width of bit field
 */
#define BME_BITFIELD_EXTRACTw(addr, bit, width)        (*(volatile uint32_t *)(((uint32_t)(addr))    \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19))

/**
 * Mask value is ANDed with a 16 bit memory location
 *
 * @param addr Address of memory location
 * @param mask Fixed mask value to be used
 *
 */
#define BME_ANDh(addr, mask)  ((void)((*(volatile uint16_t *)(((uint32_t)(addr)) | (BME_OPCODE_AND<<26))) = ((uint16_t)(mask))))

/**
 * Mask value is ORed with a 16 bit memory location
 *
 * @param addr Address of memory location
 * @param mask Fixed mask value to be used
 *
 */
#define BME_ORh(addr, mask)   ((void)((*(volatile uint16_t *)(((uint32_t)(addr)) | (BME_OPCODE_OR<<26))) = ((uint16_t)(mask))))

/**
 * Mask value is XORed with a 16 bit memory location
 *
 * @param addr Address of memory location
 * @param mask Fixed mask value to be used
 *
 */
#define BME_XORh(addr, mask)  ((void)((*(volatile uint16_t *)(((uint32_t)(addr)) | (BME_OPCODE_XOR<<26))) = ((uint16_t)(mask))))


/**
 * Bit test and clear within 16 bit memory location
 * i.e. Clear 1 bit and return original bit value
 *
 * @param addr   Address of memory location
 * @param bitNum Number of bit to test
 *
 */
#define BME_BIT_TEST_AND_CLEARh(addr, bitNum)        (*(volatile uint16_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_CLEAR <<26)  \
                              | (((bitNum & 0x1F))<<21)))

/**
 * Bit test and set within 16 bit memory location
 * i.e. Set 1 bit and return original bit value
 *
 * @param addr   Address of memory location
 * @param bitNum Number of bit to test
 *
 */
#define BME_BIT_TEST_AND_SETh(addr, bitNum)        (*(volatile uint16_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_SET <<26)  \
                              | (((bitNum & 0x1F))<<21)))

/**
 * Insert bit field within a 16 bit memory location
 *
 * @param addr   Address of memory location
 * @param bit    Starting bit number of field
 * @param width  Width of bit field
 * @param value  Value to insert
 */
#define BME_BITFIELD_INSERTh(addr, bit, width, value)        ((void)((*(volatile uint16_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19)) = ((value)<<(bit))))


/**
 * Extract a bit field from a 16 bit memory location
 *
 * @param addr   Address of memory location
 * @param bit    Starting bit number of field
 * @param width  Width of bit field
 */
#define BME_BITFIELD_EXTRACTh(addr, bit, width)        (*(volatile uint16_t *)(((uint32_t)(addr))    \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19))

/**
 * Mask value is ANDed with a 8 bit memory location
 *
 * @param addr Address of memory location
 * @param mask Fixed mask value to be used
 *
 */
#define BME_ANDb(addr, mask)  ((void)((*(volatile uint8_t *)(((uint32_t)(addr)) | (BME_OPCODE_AND<<26))) = ((uint8_t)(mask))))

/**
 * Mask value is ORed with a 8 bit memory location
 *
 * @param addr Address of memory location
 * @param mask Fixed mask value to be used
 *
 */
#define BME_ORb(addr, mask)   ((void)((*(volatile uint8_t *)(((uint32_t)(addr)) | (BME_OPCODE_OR<<26))) = ((uint8_t)(mask))))

/**
 * Mask value is XORed with a 8 bit memory location
 *
 * @param addr Address of memory location
 * @param mask Fixed mask value to be used
 *
 */
#define BME_XORb(addr, mask)  ((void)((*(volatile uint8_t *)(((uint32_t)(addr)) | (BME_OPCODE_XOR<<26))) = ((uint8_t)(mask))))


/**
 * Bit test and clear within 8 bit memory location
 * i.e. Clear 1 bit and return original bit value
 *
 * @param addr   Address of memory location
 * @param bitNum Numbert of bit to test
 *
 */
#define BME_BIT_TEST_AND_CLEARb(addr, bitNum)        (*(volatile uint8_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_CLEAR <<26)  \
                              | (((bitNum & 0x1F))<<21)))

/**
 * Bit test and set within 8 bit memory location
 * i.e. Set 1 bit and return original bit value
 *
 * @param addr   Address of memory location
 * @param bitNum Numbert of bit to test
 *
 */
#define BME_BIT_TEST_AND_SETb(addr, bitNum)        (*(volatile uint8_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_SET <<26)  \
                              | (((bitNum & 0x1F))<<21)))

/**
 * Insert bit field within a 8 bit memory location
 *
 * @param addr   Address of memory location
 * @param bit    Starting bit number of field
 * @param width  Width of bit field
 * @param value  Value to insert
 */
#define BME_BITFIELD_INSERTb(addr, bit, width, value)        ((void)((*(volatile uint8_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19)) = ((value)<<(bit))))


/**
 * Extract a bit field from a 8 bit memory location
 *
 * @param addr   Address of memory location
 * @param bit    Starting bit number of field
 * @param width  Width of bit field
 */
#define BME_BITFIELD_EXTRACTb(addr, bit, width)        (*(volatile uint8_t *)(((uint32_t)(addr))    \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19))


/*!
 * @}
 */

#ifdef __cplusplus
   }
#endif

#endif // __BME_H
