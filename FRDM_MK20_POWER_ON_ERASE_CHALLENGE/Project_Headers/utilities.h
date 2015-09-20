/*
 * utilities-mk.h
 *
 *  Created on: May 13, 2013
 *      Author: PODonoghue
 */
#ifndef UTILTIES_H_
#define UTILTIES_H_

#ifdef __cplusplus
extern "C" {
#endif

// Used to create port register names in a configurable fashion
//-------------------------------------------------------------
#define CONCAT2_(x,y) x ## y
#define CONCAT3_(x,y,z) x ## y ## z
#define CONCAT4_(w,x,y,z) w ## x ## y ## z

#define PCR(port,num)          CONCAT2_(PORT,port)->PCR[num]

#define GPIO(port)             CONCAT2_(GPIO,port)

#define PDOR(port)             CONCAT2_(GPIO,port)->PDOR
#define PSOR(port)             CONCAT2_(GPIO,port)->PSOR
#define PCOR(port)             CONCAT2_(GPIO,port)->PCOR
#define PTOR(port)             CONCAT2_(GPIO,port)->PTOR
#define PDIR(port)             CONCAT2_(GPIO,port)->PDIR
#define PDDR(port)             CONCAT2_(GPIO,port)->PDDR

#define PORT_CLOCK_MASK(port)  CONCAT4_(SIM_SCGC5,_PORT,port,_MASK)

#ifdef __cplusplus
   }
#endif

#endif /* UTILTIES_H_ */
