/*
 * CmdProcessingARM.h
 *
 *  Created on: 20/08/2012
 *      Author: PODonoghue
 */

#ifndef CMDPROCESSINGARM_H_
#define CMDPROCESSINGARM_H_

U8 f_CMD_ARM_CONNECT(void);
U8 f_CMD_ARM_TARGET_STEP(void); 
U8 f_CMD_ARM_TARGET_GO(void); 
U8 f_CMD_ARM_TARGET_HALT(void); 
U8 f_CMD_ARM_WRITE_MEM(void);
U8 f_CMD_ARM_READ_MEM(void);
U8 f_CMD_ARM_WRITE_REG(void);
U8 f_CMD_ARM_READ_REG(void);
U8 f_CMD_ARM_WRITE_DREG(void);
U8 f_CMD_ARM_READ_DREG(void);
U8 f_CMD_ARM_WRITE_CREG(void);
U8 f_CMD_ARM_READ_CREG(void);

#endif /* CMDPROCESSINGARM_H_ */
