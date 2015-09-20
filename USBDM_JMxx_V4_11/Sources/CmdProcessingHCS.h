#ifndef _CMDPROCESSINGHCS_H_
#define _CMDPROCESSINGHCS_H_

uint8_t f_CMD_CUSTOM_COMMAND(void);
uint8_t f_CMD_CONNECT(void);
uint8_t f_CMD_SET_SPEED(void);
uint8_t f_CMD_GET_SPEED(void);
//uint8_t f_CMD_CONTROL_INTERFACE(void);

uint8_t f_CMD_READ_STATUS_REG(void);
uint8_t f_CMD_WRITE_CONTROL_REG(void);

uint8_t f_CMD_RESET(void);
uint8_t f_CMD_STEP (void);
uint8_t f_CMD_GO(void);
uint8_t f_CMD_HALT(void);

uint8_t f_CMD_HCS12_WRITE_REG(void);
uint8_t f_CMD_HCS12_READ_REG(void);
uint8_t f_CMD_HCS08_WRITE_REG(void);
uint8_t f_CMD_HCS08_READ_REG(void);

uint8_t f_CMD_HCS08_READ_MEM(void);
uint8_t f_CMD_HCS08_WRITE_MEM(void);

uint8_t f_CMD_HCS12_READ_MEM(void);
uint8_t f_CMD_HCS12_WRITE_MEM(void);

uint8_t f_CMD_WRITE_BD(void);
uint8_t f_CMD_READ_BD(void);

uint8_t f_CMD_WRITE_BKPT(void);
uint8_t f_CMD_READ_BKPT(void);

uint8_t f_CMD_SET_VPP(void);

#endif // _CMDPROCESSINGHCS_H_
