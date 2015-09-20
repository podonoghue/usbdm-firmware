/*
 * SCI_Debug.h
 *
 *  Created on: 06/08/2011
 *      Author: podonoghue
 */

#ifndef SCI_DEBUG_H_
#define SCI_DEBUG_H_

void debugTx(char ch);
int debugRx(void);
void debugPuts(const char * s);
void debugSCIInit(void);

#endif /* SCI_DEBUG_H_ */
