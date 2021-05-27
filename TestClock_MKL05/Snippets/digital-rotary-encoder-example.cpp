/**
 ==============================================================================
 * @file    digital-rotary-encoder-example.cpp (180.ARM_Peripherals/Snippets/)
 * @brief   Basic C++ demo of GPIOs with interrupts to implement an interface
 *          for a 2-phase rotary encoder.
 *
 *  Created on: 26/7/2018
 *      Author: podonoghue
 ==============================================================================
 */
/**
 * This example uses GPIO interrupts.\n
 * It would be more efficient to use the FTM Quadrature decoder if available.
 *
 * It is necessary to enable these in Configure.usbdmProject\n
 * under the "Peripheral Parameters"->GPIOx tab.\n
 * Select irqHandlers option (Class Method - Software ...)
 */
#include "hardware.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

/** Encoder pins as a GPIO field - this assumes consecutive pins on same port */
using Encoder = GpioCField<1, 0, ActiveLow>;

/** Variable used by callback to track encoder position */
static volatile int position;

#if 1
/**
 * Pin IRQ call-back.
 * Used to monitor shaft movements.\n
 * This uses a larger state machine that is more resistant to contact bounce.
 *
 * @note Assumes 4 transitions/detent
 *
 * @param eventMask Mask indicating active channel
 */
void pinIrqCallback(uint32_t eventMask) {
   // Count actions encoded in state
   constexpr uint8_t Inc = 0b010000;
   constexpr uint8_t Dec = 0b100000;

   enum State : uint8_t {
      State_Idle,
      State_CW01,
      State_CW11,
      State_CW10,
      State_CCW10,
      State_CCW11,
      State_CCW01,
      State_IdleInc  = State_Idle|Inc,  // Idle + increment count
      State_IdleDec  = State_Idle|Dec,  // Idle + decrement count
   };

   /**
    * Next state matrix: nextStateTable[current state][current input] -> next state
    * This matrix assumes 4 transitions/detent as shown
    * Expected encoder sequence CW = 00->01->11->10->00,  CCW = 00->10->11->01->00
    */
   static const State nextStateTable[][4] = {
      /* current : encoder -> 00               01           10           11         */
      /* State_Idle  */     { State_Idle,      State_CW01,  State_CCW10, State_Idle  },

      /* State_CW01  */     { State_Idle,      State_CW01,  State_Idle,  State_CW11  },
      /* State_CW11  */     { State_Idle,      State_CW01,  State_CW10,  State_CW11  },
      /* State_CW10  */     { State_IdleDec,   State_Idle,  State_CW10,  State_CW11  },

      /* State_CCW10 */     { State_Idle,      State_Idle,  State_CCW10, State_CCW11 },
      /* State_CCW11 */     { State_Idle,      State_CCW01, State_CCW10, State_CCW11 },
      /* State_CCW01 */     { State_IdleInc,   State_CCW01, State_Idle,  State_CCW11 },
   };

   static State currentState = State_Idle;

   // Check channel
   if (eventMask && Encoder::MASK) {
      // Use state of GPIOs to determine next state
      uint8_t value = Encoder::read();
      State nextState = nextStateTable[currentState][value];
      if (nextState&Inc) {
         position++;
      }
      else if (nextState&Dec) {
         position--;
      }
      currentState = (State)(nextState & 0x0F);
   }
}

#else

/**
 * Pin IRQ call-back.
 * Used to monitor shaft movements.\n
 * This is a simple state machine that seemed to work fine with the encoders tested.
 *
 * @note Assumes 4 transitions/detent
 *
 * @param eventMask Mask indicating active channel
 */
void pinIrqCallback(uint32_t eventMask) {
   // State change matrix: motion[lastState][currentState] -> movement value
   // Expected sequence 00<->10<->01<->11<->00
   static const int8_t motion[4][4] = {
         /* last : current -> 00  01  10  11 */
         /*  00 */          {  0,  0,  0,  0 },
         /*  01 */          { +1,  0,  0,  0 },
         /*  10 */          { -1,  0,  0,  0 },
         /*  11 */          {  0,  0,  0,  0 },
   };

   static uint32_t lastState = 0;

   // Check channel
   if (eventMask && Encoder::MASK) {
      // Use change of state of GPIOs to determine rotation (if any)
      uint32_t currentState = Encoder::read();
      position += motion[lastState][currentState];
      lastState = currentState;
   }
}
#endif

/**
 * Enable shaft encoder interface.
 *
 * Enables encoder interrupts and does any other initialisation required.
 */
void startEncoder() {

   // Start position at zero
   position = 0;

   // Configure encoder pins as inputs with dual-edge interrupts
   Encoder::setCallback(pinIrqCallback);
   Encoder::setInput(PinPull_Up, PinAction_IrqEither, PinFilter_Passive);
   Encoder::enableNvicInterrupts(NvicPriority_Normal);
}

/**
 *  Disables shaft encoder.
 */
void stopEncoder() {
   Encoder::setInput(PinPull_None, PinAction_None);
   Encoder::disableNvicInterrupts();
}

/**
 * Returns the encoder position.
 */
int getPosition() {
   return position;
}

int main() {
   startEncoder();

   int lastMeasurement    = 0;
   int currentMeasurement = 0;

   for(;;) {
      // Only print when changing
      lastMeasurement    = currentMeasurement;
      currentMeasurement = getPosition();
      if (currentMeasurement != lastMeasurement) {
         console.write("Position = ").writeln(currentMeasurement);
      }
   }
}
