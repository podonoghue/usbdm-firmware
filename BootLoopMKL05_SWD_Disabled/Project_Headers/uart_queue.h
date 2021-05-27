/**
 * @file uart_queue.h (180.ARM_Peripherals/Project_Headers/uart_queue.h)
 *
 *  Created on: 12Nov.,2016
 *      Author: podonoghue
 */

#ifndef INCLUDE_USBDM_UART_QUEUE_H_
#define INCLUDE_USBDM_UART_QUEUE_H_

#include "system.h"

namespace USBDM {

/**
 * Simple queue implementation
 *
 * @tparam T          Type of queue items
 * @tparam QUEUE_SIZE Size of queue
 */
template<class T, int QUEUE_SIZE>
class UartQueue {
   volatile T        fBuff[QUEUE_SIZE];
   volatile T        *fHead, *fTail;
   volatile int      fNumberOfElements;

public:
   /*
    * Create empty Queue
    */
   constexpr UartQueue() : fHead(fBuff), fTail(fBuff), fNumberOfElements(0) {
   }

   /**
    * Clear queue i.e. make empty
    */
   void clear() {
      USBDM::CriticalSection cs;
      fHead             = fBuff;
      fTail             = fBuff;
      fNumberOfElements = 0;
   }
   /*
    * Check if empty
    *
    * @return true => empty
    */
   bool isEmpty() {
      return fNumberOfElements == 0;
   }
   /*
    * Check if full
    *
    * @return true => full
    */
   bool isFull() {
      return fNumberOfElements == QUEUE_SIZE;
   }
   /*
    * Add element to queue
    *
    * @param[in]  element Element to add
    */
   void enQueue(T element) {
      bool success = enQueueDiscardOnFull(element);
      (void)success;
      usbdm_assert(success, "Queue full");
   }
   /*
    * Add element to queue. Discards on full.
    *
    * @param[in]  element Element to add
    *
    * @return true  => Element enqueued
    * @return false => Queue full, element not added
    */
   bool enQueueDiscardOnFull(T element) {
      USBDM::CriticalSection cs;
      bool hasSpace = !isFull();
      if (hasSpace) {
         *fTail++ = element;
         fNumberOfElements++;
         if (fTail>=(fBuff+QUEUE_SIZE)) {
            fTail = fBuff;
         }
      }
      return hasSpace;
   }
   /*
    * Remove & return element from queue
    *
    * @param[in]  element Element to add
    */
   T deQueue() {
      USBDM::CriticalSection cs;
      usbdm_assert(!isEmpty(), "Queue empty");
      T t = *fHead++;
      fNumberOfElements--;
      if (fHead>=(fBuff+QUEUE_SIZE)) {
         fHead = fBuff;
      }
      return t;
   }
};

} // End namespace USBDM

#endif /* INCLUDE_USBDM_UART_QUEUE_H_ */
