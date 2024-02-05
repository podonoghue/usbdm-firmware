/**
 * @file     CmdProcessingSWD.h
 * @brief    SWD Command Processing
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */
#ifndef CMDPROCESSINGSWD_H_
#define CMDPROCESSINGSWD_H_

#include "commands.h"
#include "cmdProcessing.h"

namespace Swd {

USBDM_ErrorCode f_CMD_CONNECT(void);
USBDM_ErrorCode f_CMD_SET_SPEED(void);
USBDM_ErrorCode f_CMD_GET_SPEED(void);

USBDM_ErrorCode f_CMD_TARGET_STEP(void);
USBDM_ErrorCode f_CMD_TARGET_GO(void);
USBDM_ErrorCode f_CMD_TARGET_HALT(void);

USBDM_ErrorCode f_CMD_WRITE_MEM(void);
USBDM_ErrorCode f_CMD_READ_MEM(void);

USBDM_ErrorCode f_CMD_READ_ALL_CORE_REGS(void);
USBDM_ErrorCode f_CMD_WRITE_REG(void);
USBDM_ErrorCode f_CMD_READ_REG(void);
USBDM_ErrorCode f_CMD_WRITE_DREG(void);
USBDM_ErrorCode f_CMD_READ_DREG(void);
USBDM_ErrorCode f_CMD_WRITE_CREG(void);
USBDM_ErrorCode f_CMD_READ_CREG(void);

}; // End namespace Swd

#endif /* CMDPROCESSINGSWD_H_ */
