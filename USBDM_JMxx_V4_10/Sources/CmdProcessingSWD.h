/*
 * CmdProcessingSWD.h
 *
 *  Created on: 11/08/2012
 *      Author: podonoghuE
 */

#ifndef CMDPROCESSINGSWD_H_
#define CMDPROCESSINGSWD_H_

U8 f_CMD_SWD_CONNECT(void);
U8 f_CMD_SWD_TARGET_STEP(void); 
U8 f_CMD_SWD_TARGET_GO(void); 
U8 f_CMD_SWD_TARGET_HALT(void); 
U8 f_CMD_SWD_WRITE_MEM(void);
U8 f_CMD_SWD_READ_MEM(void);
U8 f_CMD_SWD_READ_ALL_CORE_REGS(void);
U8 f_CMD_SWD_WRITE_REG(void);
U8 f_CMD_SWD_READ_REG(void);
U8 f_CMD_SWD_WRITE_DREG(void);
U8 f_CMD_SWD_READ_DREG(void);
U8 f_CMD_SWD_WRITE_CREG(void);
U8 f_CMD_SWD_READ_CREG(void);

#endif /* CMDPROCESSINGSWD_H_ */
