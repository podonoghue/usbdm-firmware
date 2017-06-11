/*
 ============================================================================
 * main.c
 *
 *  Created on: 04/12/2012
 *      Author: podonoghue
 ============================================================================
 */
#include <stdio.h>
#include "system.h"
#include "derivative.h"
#include "utilities.h"

int main(void) {
   // Real programs never die!
   for(;;) {
	   __asm__("wfi");
   }
   return 0;
}
