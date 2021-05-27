/**
 * @file     bdmCommon.h
 * @brief    Low power timer interface
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */
#ifndef _BDMCOMMON_H_
#define _BDMCOMMON_H_

#include <stdint.h>

#include "commands.h"

USBDM_ErrorCode setTarget(TargetType_t target);
USBDM_ErrorCode checkTargetVdd(void);
void            suspend(void);
USBDM_ErrorCode cycleTargetVddOn(TargetMode_t mode);
USBDM_ErrorCode cycleTargetVdd(TargetMode_t mode);
uint16_t        targetVddMeasure(void);
USBDM_ErrorCode clearStatus(void);
USBDM_ErrorCode enableTargetVdd();
USBDM_ErrorCode setTargetVdd(TargetVddSelect_t targetVdd);

#endif // _BDMCOMMON_H_
