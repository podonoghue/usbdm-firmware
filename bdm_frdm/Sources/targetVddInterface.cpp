/*
 * targetVddInterface.cpp
 *
 *  Created on: 23Dec.,2016
 *      Author: podonoghue
 */

#include "targetVddInterface.h"

void (*TargetVddInterface::fCallback)(VddState) = TargetVddInterface::nullCallback;

VddState TargetVddInterface::vddState = VddState_None;
