/*
 * dummy.c
 *
 *  Created on: Nov 8, 2012
 *      Author: podonoghue
 */
#include <stdint.h>
#include "Commands.h"
#include "SPI.h"
#include "SWD.h"

uint8_t doParityX(uint32_t data) {
   data = (data>>16)^data;
   data = (data>>8)^data;
   data = (data>>4)^data;
   data = (data>>2)^data;
   data = (data>>1)^data;
   return data;
}

uint8_t doParityY(uint8_t data[]) {
   return data[0]^data[1]^data[2]^data[3];
}
