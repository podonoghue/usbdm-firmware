#ifndef _CMDPROCESSINGCFVX_H_
#define _CMDPROCESSINGCFVX_H_

#include <stdint.h>

uint8_t f_CMD_CFVx_RESET(void);
//uint8_t f_CMD_CFVx_CONTROL_INTERFACE(void);
uint8_t f_CMD_SPI_SET_SPEED(void);
uint8_t f_CMD_SPI_GET_SPEED(void);

#if (TARGET_CAPABILITY&CAP_CFVx)
uint8_t f_CMD_CFVx_HALT(void);
uint8_t f_CMD_CFVx_GO(void);
uint8_t f_CMD_CFVx_STEP(void);
uint8_t f_CMD_CFVx_READ_CREG(void);
uint8_t f_CMD_CFVx_WRITE_CREG(void);
uint8_t f_CMD_CFVx_READ_DREG(void);
uint8_t f_CMD_CFVx_READ_STATUS_REG(void);
uint8_t f_CMD_CFVx_WRITE_DREG(void);
uint8_t f_CMD_CFVx_READ_ALL_CORE_REGS(void);
uint8_t f_CMD_CFVx_READ_REG(void);
uint8_t f_CMD_CFVx_WRITE_REG(void);
uint8_t f_CMD_CFVx_READ_MEM(void);
uint8_t f_CMD_CFVx_WRITE_MEM(void);
uint8_t f_CMD_CFVx_RESYNC(void);
#endif
#if (TARGET_CAPABILITY&(CAP_DSC|CAP_JTAG|CAP_ARM_JTAG))
uint8_t f_CMD_JTAG_GOTORESET(void);
uint8_t f_CMD_JTAG_GOTOSHIFT(void);
uint8_t f_CMD_JTAG_WRITE(void);
uint8_t f_CMD_JTAG_READ(void);
uint8_t f_CMD_JTAG_READ_WRITE(void);
uint8_t f_CMD_JTAG_EXECUTE_SEQUENCE(void);
uint8_t f_CMD_JTAG_RESET(void);
#endif // (CAPABILITY&CAP_CFVx)

#endif // _CMDPROCESSINGCFVX_H_
