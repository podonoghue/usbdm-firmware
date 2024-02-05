/*
 * targetVddInterface.cpp
 *
 *  Created on: 23Dec.,2016
 *      Author: podonoghue
 */
#include "configure.h"
#include "targetVddInterface.h"
#if HW_CAPABILITY&CAP_VDDCONTROL

void (*TargetVddInterface::fCallback)(VddState) = TargetVddInterface::nullCallback;

VddState TargetVddInterface::vddState = VddState_None;

#endif // HW_CAPABILITY&CAP_VDDCONTROL
