#ifndef _BDMCOMMON_H_
#define _BDMCOMMON_H_

#include <stdint.h>

//================================================================================
//  Timer Usage:
//
//   TPMx-CHa - BDM_IN pin, SYNC measuring, ACKN detection (IC rising & falling edges)
//   TPMx-CHb - ACKN & SYNC Timeouts (Output compare)
//
//================================================================================

// Timer constants, 24MHz ticks
#define TIMER_FREQ               (OSC_FREQ/2)
#define TIMER_MICROSECOND(x)     (((x)*(TIMER_FREQ/1000))/1000UL)  // Timer ticks in 1 us

// General Time intervals
#define VDD_RISE_TIMEus    2000U // us - minimum time to allow for controlled target Vdd rise
#define BKGD_WAITus        2000U // us - time to hold BKGD pin low after reset pin rise for special modes (allowance made for slow Reset rise)
#define RESET_SETTLEms        3U // ms - time to wait for signals to settle in us, this should certainly be longer than the soft reset time
#define RESET_RECOVERYms     10U // ms - how long to wait after reset before new commands are allowed

/*! \brief A Macro to wait for given time (makes use of timer).

    @param  t  Time to wait in \e microseconds.

    @note Limited to 2 ms
*/
#define WAIT_US(t) fastTimerWait(TIMER_MICROSECOND(t))
/*! \brief A Macro to wait for given time (makes use of timer).

    @param  t  Time to wait in \e milliseconds (<65000 ms).
*/
#define WAIT_MS(t) millisecondTimerWait((t))
/*! \brief A Macro to wait for given time or until a condition is met

    @param  t  Maximum time to wait in \e microseconds (<???).
    @param  c  Condition to exit wait early
*/
#define WAIT_WITH_TIMEOUT_US(t,c) {                         \
   PIT_LDVAL0 = TIMER_MICROSECOND(t);                       \
   PIT_TFLG0  = PIT_TFLG_TIF_MASK;                          \
   PIT_TCTRL0 = PIT_TCTRL_TEN_MASK;                         \
   while (!(c) && ((PIT_TFLG0&PIT_TFLG_TIF_MASK)==0)) {     \
   }                                                        \
   PIT_TCTRL0 = 0;                                          \
}
/*! \brief A Macro to wait for given time or until a condition is met

    @param  t  Maximum time to wait in \e milliseconds.
    @param  c  Condition to exit wait early
*/
#define WAIT_WITH_TIMEOUT_MS(t,c) {                         \
   int tt = t;                                              \
   PIT_LDVAL0 = TIMER_MICROSECOND(1000);                    \
   PIT_TCTRL0 = PIT_TCTRL_TEN_MASK;                         \
   while (!(c) && (tt-->0)) {                               \
      PIT_TFLG0  = PIT_TFLG_TIF_MASK;                       \
      while ((PIT_TFLG0&PIT_TFLG_TIF_MASK)==0) {            \
      }                                                     \
   }                                                        \
   PIT_TCTRL0 = 0;                                          \
}
/*! \brief A Macro to wait for given time or until a condition is met

    @param  t  Maximum time to wait in \e seconds.
    @param  c  Condition to exit wait early (checked every ~10 ms and affects timing)
*/
#define WAIT_WITH_TIMEOUT_S(t,c) {       \
    int tt = 100*(t);                    \
      do {                               \
         millisecondTimerWait(10);       \
      } while (!(c) & (tt-->0));         \
   }
/*! \brief Wait for given time in milliseconds (makes use of timer).

    @param  delay Time to wait in \e milliseconds.
*/
void millisecondTimerWait(uint16_t delay);
//! Wait for given time in fast timer ticks
//!
//!  @param delay Delay time in fast timer ticks
//!
//!  @note Limited to 2 ms
//!
void fastTimerWait(uint32_t delay);
//! Initialises the timers, input captures and interrupts
//!
uint8_t   initTimers(void);

void      bdm_init(void);
void      bdm_off(void);
uint8_t   bdm_setTarget(uint8_t target);
uint8_t   bdm_checkTargetVdd(void);
void      bdm_suspend(void);
uint8_t   bdm_cycleTargetVddOn(uint8_t mode);
uint8_t   bdm_cycleTargetVdd(uint8_t mode);
uint16_t  bdm_targetVddMeasure(void);
uint8_t   bdm_setTargetVdd( void );  // Low-level - bdm_cycleTargetVddOn() preferred
void      bdm_interfaceOff( void );

uint8_t   bdm_clearStatus(void);

//// Interrupt monitoring routines
//interrupt void timerHandler(void);
//interrupt void kbiHandler(void);
//interrupt void acmpHandler(void);

#endif // _BDMCOMMON_H_
