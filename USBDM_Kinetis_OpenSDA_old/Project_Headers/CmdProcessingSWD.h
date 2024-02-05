/*
 * CmdProcessingSWD.h
 *
 *  Created on: 11/08/2012
 *      Author: podonoghuE
 */

#ifndef CMDPROCESSINGSWD_H_
#define CMDPROCESSINGSWD_H_

USBDM_ErrorCode f_CMD_SWD_CONNECT(void);
USBDM_ErrorCode f_CMD_SWD_TARGET_STEP(void);
USBDM_ErrorCode f_CMD_SWD_TARGET_GO(void);
USBDM_ErrorCode f_CMD_SWD_TARGET_HALT(void);
USBDM_ErrorCode f_CMD_SWD_WRITE_MEM(void);
USBDM_ErrorCode f_CMD_SWD_READ_MEM(void);
USBDM_ErrorCode f_CMD_SWD_READ_ALL_CORE_REGS(void);
USBDM_ErrorCode f_CMD_SWD_WRITE_REG(void);
USBDM_ErrorCode f_CMD_SWD_READ_REG(void);
USBDM_ErrorCode f_CMD_SWD_WRITE_DREG(void);
USBDM_ErrorCode f_CMD_SWD_READ_DREG(void);
USBDM_ErrorCode f_CMD_SWD_WRITE_CREG(void);
USBDM_ErrorCode f_CMD_SWD_READ_CREG(void);

#endif /* CMDPROCESSINGSWD_H_ */
