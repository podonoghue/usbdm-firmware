#ifndef _CMDPROCESSINGHCS_H_
#define _CMDPROCESSINGHCS_H_

U8 f_CMD_CONNECT(void);
U8 f_CMD_SET_SPEED(void);
U8 f_CMD_GET_SPEED(void);
U8 f_CMD_CONTROL_INTERFACE(void);

U8 f_CMD_READ_STATUS_REG(void);
U8 f_CMD_WRITE_CONTROL_REG(void);

U8 f_CMD_RESET(void);
U8 f_CMD_STEP (void);
U8 f_CMD_GO(void);
U8 f_CMD_HALT(void);

U8 f_CMD_HCS12_WRITE_REG(void);
U8 f_CMD_HCS12_READ_REG(void);
U8 f_CMD_HCS08_WRITE_REG(void);
U8 f_CMD_HCS08_READ_REG(void);

U8 f_CMD_HCS08_READ_MEM(void);
U8 f_CMD_HCS08_WRITE_MEM(void);

U8 f_CMD_HCS12_READ_MEM(void);
U8 f_CMD_HCS12_WRITE_MEM(void);

U8 f_CMD_WRITE_BD(void);
U8 f_CMD_READ_BD(void);

U8 f_CMD_WRITE_BKPT(void);
U8 f_CMD_READ_BKPT(void);

U8 f_CMD_SET_VPP(void);

#endif // _CMDPROCESSINGHCS_H_
