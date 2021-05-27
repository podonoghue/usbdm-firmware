/*
 * vector.h
 *
 *  Created on: 8 Jun 2017
 *      Author: podonoghue
 */

#ifndef SOURCES_VECTOR_H_
#define SOURCES_VECTOR_H_

namespace USBDM {

class Vector {
public:
   float x, y, z;

   constexpr Vector(float x, float y, float z) : x(x), y(y), z(z) {
   }
   virtual ~Vector() {
   }
};

} /* namespace USBDM */

#endif /* SOURCES_VECTOR_H_ */
