/*
 * Based on Freescale Application Note AN4838
 *
 * Assumes the location has been allocated to:
 *    - AIPS Peripherals (0x4000_0000-0x4007_FFFF) with BME alias (0x4400_0000-0x5FFF_FFFF)
 *    - SRAM_U           (0x2000_0000-0x2000_02FF) with BME alias (0x2400_0000-0x3FFF_FFFF) (MKE04 & MKE06 only)
 *
 * Examples:
 *
 *    // Set a bit
 *    BME_BIT_SET(&SIM_SCGC, SIM_SCGC_ACMP0_SHIFT);
 *
 *    // Clear a bit
 *    BME_BIT_CLEAR(&SIM_SCGC, SIM_SCGC_ACMP0_SHIFT);
 *
 *    // Set multiple bits
 *    BME_OR(&SIM_SCGC,SIM_SCGC_ACMP0_MASK|SIM_SCGC_ACMP1_MASK);
 *
 *    // Clear multiple bits
 *    BME_AND(&SIM_SCGC,~(SIM_SCGC_ACMP0_MASK|SIM_SCGC_ACMP1_MASK));
 *
 *    // Spin-lock - Wait until successful at setting the bit i.e. the bit was 0 and now is 1
 *    while(BME_BIT_TEST_AND_SET(&lock, 3) != 0) {
 *    }
 */

#ifndef __BME_H
#define __BME_H

#ifdef __cplusplus
extern "C" {
#endif

// BME operation code
#define BME_OPCODE_AND           1
#define BME_OPCODE_OR            2
#define BME_OPCODE_XOR	         3
#define BME_OPCODE_BITFIELD	   4

#define BME_OPCODE_BIT_CLEAR     2
#define BME_OPCODE_BIT_SET       3

/*
 * ==== 32 bit word ================================================
 */
// Assigned value is ANDed
#define BME_ANDw(addr, mask)  ((void)((*(volatile uint32_t *)(((uint32_t)(addr)) | (BME_OPCODE_AND<<26))) = ((uint32_t)(mask))))

// Assigned value is ORed
#define BME_ORw(addr, mask)   ((void)((*(volatile uint32_t *)(((uint32_t)(addr)) | (BME_OPCODE_OR<<26))) = ((uint32_t)(mask))))

// Assigned value is XORed
#define BME_XORw(addr, mask)  ((void)((*(volatile uint32_t *)(((uint32_t)(addr)) | (BME_OPCODE_XOR<<26))) = ((uint32_t)(mask))))


// Clear 1 bit and return original value (test-and-clear)
#define BME_BIT_TEST_AND_CLEARw(addr, bitNum)        (*(volatile uint32_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_CLEAR <<26)  \
                              | (((bitNum & 0x1F))<<21)))

// Set 1 bit and return original value (test-and-set)
#define BME_BIT_TEST_AND_SETw(addr, bitNum)        (*(volatile uint32_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_SET <<26)  \
                              | (((bitNum & 0x1F))<<21)))

// Assigned value is inserted as bit field
#define BME_BITFIELD_INSERTw(addr, bit, width, value)        ((void)((*(volatile uint32_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19)) = ((value)<<(bit))))

// Extract value as bit field
#define BME_BITFIELD_EXTRACTw(addr, bit, width)        (*(volatile uint32_t *)(((uint32_t)(addr))    \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19))

/*
 * ==== 16 bit halfword ================================================
 */
// Assigned value is ANDed
#define BME_ANDh(addr, mask)  ((void)((*(volatile uint16_t *)(((uint32_t)(addr)) | (BME_OPCODE_AND<<26))) = ((uint16_t)(mask))))

// Assigned value is ORed
#define BME_ORh(addr, mask)   ((void)((*(volatile uint16_t *)(((uint32_t)(addr)) | (BME_OPCODE_OR<<26))) = ((uint16_t)(mask))))

// Assigned value is XORed
#define BME_XORh(addr, mask)  ((void)((*(volatile uint16_t *)(((uint32_t)(addr)) | (BME_OPCODE_XOR<<26))) = ((uint16_t)(mask))))


// Clear 1 bit and return original value (test-and-clear)
#define BME_BIT_TEST_AND_CLEARh(addr, bitNum)        (*(volatile uint16_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_CLEAR <<26)  \
                              | (((bitNum & 0x1F))<<21)))

// Set 1 bit and return original value (test-and-set)
#define BME_BIT_TEST_AND_SETh(addr, bitNum)        (*(volatile uint16_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_SET <<26)  \
                              | (((bitNum & 0x1F))<<21)))

// Assigned value is inserted as bit field
#define BME_BITFIELD_INSERTh(addr, bit, width, value)        ((void)((*(volatile uint16_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19)) = ((value)<<(bit))))


// Extract value as bit field
#define BME_BITFIELD_EXTRACTh(addr, bit, width)        (*(volatile uint16_t *)(((uint32_t)(addr))    \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19))

/*
 * ==== 8 bit byte ================================================
 */
// Extract value as bit field
#define BME_BITFIELD_EXTRACTb(addr, bit, width)        (*(volatile uint8_t *)(((uint32_t)(addr))    \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19))

// Assigned value is ANDed
#define BME_ANDb(addr, mask)  ((void)((*(volatile uint8_t *)(((uint32_t)(addr)) | (BME_OPCODE_AND<<26))) = ((uint8_t)(mask))))

// Assigned value is ORed
#define BME_ORb(addr, mask)   ((void)((*(volatile uint8_t *)(((uint32_t)(addr)) | (BME_OPCODE_OR<<26))) = ((uint8_t)(mask))))

// Assigned value is XORed
#define BME_XORb(addr, mask)  ((void)((*(volatile uint8_t *)(((uint32_t)(addr)) | (BME_OPCODE_XOR<<26))) = ((uint8_t)(mask))))


// Clear 1 bit and return original value (test-and-clear)
#define BME_BIT_TEST_AND_CLEARb(addr, bitNum)        (*(volatile uint8_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_CLEAR <<26)  \
                              | (((bitNum & 0x1F))<<21)))

// Set 1 bit and return original value (test-and-set)
#define BME_BIT_TEST_AND_SETb(addr, bitNum)        (*(volatile uint8_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BIT_SET <<26)  \
                              | (((bitNum & 0x1F))<<21)))

// Assigned value is inserted as bit field
#define BME_BITFIELD_INSERTb(addr, bit, width, value)        ((void)((*(volatile uint8_t *)(((uint32_t)(addr))   \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19)) = ((value)<<(bit))))


// Extract value as bit field
#define BME_BITFIELD_EXTRACTb(addr, bit, width)        (*(volatile uint8_t *)(((uint32_t)(addr))    \
                              | (BME_OPCODE_BITFIELD <<26)  \
                              | ((((bit) & 0x1F))<<23) | (((width)-1) & 0xF)<<19))

#ifdef __cplusplus
   }
#endif

#endif // __BME_H
