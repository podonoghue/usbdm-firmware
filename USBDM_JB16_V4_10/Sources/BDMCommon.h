#ifndef _BDMCOMMON_H_
#define _BDMCOMMON_H_

/*==============================================================
  Timer Usage:
  IC_BDM_TIMING_CHANNEL  (either T1CH01 or T2CH01)
      Time delays using Timer overflow (TOF)
      Input capture using Channel 0 (CH0F) - SYNC & ACN timing

  IC_VDD_SENSING_CHANNEL (either T1CH01 or T2CH01)
      Input capture using Channel 0 (CH0F) - Target Vdd changes
//==============================================================*/

// T1 (SYNC & ACKN) timer constants, 6MHz ticks
#define ACKN_TIMER_PRESCALE_MASK (0)
#define ACKN_TIMER_PRESCALE      (1<<ACKN_TIMER_PRESCALE_MASK)
#define ACKN_TIMER_FREQ          (BUS_FREQ/ACKN_TIMER_PRESCALE)
#define ACKN_MICROSECOND(x)      ((U16)(((x)*(ACKN_TIMER_FREQ/1000))/1000UL))  // SYNC timer ticks in 1 us
#define ACKN_T1SC_VALUE          (T1SC_TRST_MASK|ACKN_TIMER_PRESCALE_MASK) // Set prescale & resets timer

// T1 Fast timer constants ~333ns resolution, usable to 20ms 
#define FAST_TIMER_PRESCALE_MASK (1)
#define FAST_TIMER_PRESCALE      (1<<FAST_TIMER_PRESCALE_MASK)
#define FAST_TIMER_FREQ          (BUS_FREQ/FAST_TIMER_PRESCALE)
#define MICROSECOND(x)           ((U16)(((x)*(FAST_TIMER_FREQ/1000))/1000UL))  // Fast timer ticks in 1 us
#define FAST_T1_VALUE            (T1SC_TRST_MASK|FAST_TIMER_PRESCALE_MASK)

// T1 Slow timer constants ~10us resolution, usable to 650ms
#define SLOW_TIMER_PRESCALE_MASK (FAST_TIMER_PRESCALE_MASK+5)
#define SLOW_TIMER_PRESCALE      (1<<SLOW_TIMER_PRESCALE_MASK)
#define SLOW_TIMER_FREQ          (BUS_FREQ/SLOW_TIMER_PRESCALE)
#define MILLISECOND(x)           ((U16)(((x)*SLOW_TIMER_FREQ)/1000UL))  // Slow timer ticks in 1 ms
#define SLOW_T1_VALUE            (T1SC_TRST_MASK|SLOW_TIMER_PRESCALE_MASK)

// TIM-Chz Timer expired
#define WAIT_TIMER_EXPIRED       (IC_BDM_TIMING_TSC_TOF)

// General Time intervals
#define VDD_RISE_TIMEus    2000U // us - minimum time to allow for controlled target Vdd rise
#define BKGD_WAITus        2000U // us - time to hold BKGD pin low after reset pin rise for special modes (allowance made for slow Reset rise)
#define RESET_SETTLEms        3U // ms - time to wait for signals to settle in us, this should certainly be longer than the soft reset time
#define RESET_RECOVERYms     10U // ms - how long to wait after reset before new commands are allowed

#if (SLOW_TIMER_PRESCALE_MASK >= 8)
#error "SLOW_TIMER_PRESCALE_MASK value out of range"
#endif

/*! \brief A Macro to wait for given time (makes use of timer).

    @param  t  Time to wait in \e microseconds.
*/
#define WAIT_US(t) fastTimerWait(MICROSECOND(t))
/*! \brief A Macro to wait for given time (makes use of timer).

    @param  t  Time to wait in \e milliseconds.
*/
#define WAIT_MS(t) slowTimerWait(MILLISECOND(t))
/*! \brief A Macro to wait for given time (makes use of timer).

    @param  t  Time to wait in \e seconds.
*/
#define WAIT_S(t)  {int tt = 4*(t); \
                    while(tt-->0)   \
                       slowTimerWait(MILLISECOND(250));}

/*! \brief A Macro to wait for given time or until a condition is met

    @param  t  Maximum time to wait in \e microseconds.
    @param  c  Condition to exit wait early
*/
#define WAIT_WITH_TIMEOUT_US(t,c) {             \
   IC_BDM_TIMING_TMOD = MICROSECOND(t);         \
   /* reset the timer and start counting  */    \
   IC_BDM_TIMING_TSC = FAST_T1_VALUE;           \
   IC_BDM_TIMING_TSC_TOF = 0;                   \
   while (!(c) && !WAIT_TIMER_EXPIRED) {        \
   }                                            \
}

/*! \brief A Macro to wait for given time or until a condition is met

    @param  t  Maximum time to wait in \e milliseconds.
    @param  c  Condition to exit wait early
*/
#define WAIT_WITH_TIMEOUT_MS(t,c) {             \
   IC_BDM_TIMING_TMOD = MILLISECOND(t);         \
   /* reset the timer and start counting  */    \
   IC_BDM_TIMING_TSC = SLOW_T1_VALUE;           \
   IC_BDM_TIMING_TSC_TOF = 0;                   \
   while (!(c) && !WAIT_TIMER_EXPIRED) {        \
   }                                            \
}

/*! \brief A Macro to wait for given time or until a condition is met

    @param  t  Maximum time to wait in \e seconds.
    @param  c  Condition to exit wait early (checked every ~10 ms and affects timing)
*/
#define WAIT_WITH_TIMEOUT_S(t,c) {       \
    int tt = 100*(t);                    \
      do {                               \
         slowTimerWait(MILLISECOND(10)); \
      } while (!(c) & (tt-->0));         \
   }

void fastTimerWait(U16 delay);
void slowTimerWait(U16 delay);
U8   initTimers(void);

void bdm_init(void);
void bdm_off(void);
U8   bdm_setTarget(U8 target);
U8   bdm_checkTargetVdd(void);
void bdm_suspend(void);
U8   bdm_cycleTargetVddOn(U8 mode);
U8   bdm_cycleTargetVdd(U8 mode);
U16  bdm_targetVddMeasure(void);
U8   bdm_setTargetVdd( void );  // Low-level - bdm_cycleTargetVddOn() preferred
void bdm_interfaceOff( void );

// Interrupt monitoring routines
interrupt void bdm_resetSense(void);
interrupt void bdm_targetVddSense(void);

#endif // _BDMCOMMON_H_
