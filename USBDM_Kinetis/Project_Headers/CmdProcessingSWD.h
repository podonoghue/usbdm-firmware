/*
 * CmdProcessingSWD.h
 *
 *  Created on: 11/08/2012
 *      Author: podonoghuE
 */

#ifndef CMDPROCESSINGSWD_H_
#define CMDPROCESSINGSWD_H_

uint8_t f_CMD_SWD_CONNECT(void);
uint8_t f_CMD_SWD_TARGET_STEP(void);
uint8_t f_CMD_SWD_TARGET_GO(void);
uint8_t f_CMD_SWD_TARGET_HALT(void);
uint8_t f_CMD_SWD_WRITE_MEM(void);
uint8_t f_CMD_SWD_READ_MEM(void);
uint8_t f_CMD_SWD_READ_ALL_CORE_REGS(void);
uint8_t f_CMD_SWD_WRITE_REG(void);
uint8_t f_CMD_SWD_READ_REG(void);
uint8_t f_CMD_SWD_WRITE_DREG(void);
uint8_t f_CMD_SWD_READ_DREG(void);
uint8_t f_CMD_SWD_WRITE_CREG(void);
uint8_t f_CMD_SWD_READ_CREG(void);

#endif /* CMDPROCESSINGSWD_H_ */
