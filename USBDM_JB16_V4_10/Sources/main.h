#ifndef _MAIN_H_
#define _MAIN_H_

/* the following times are in 100us multiples */
#define SUSPEND_TIME          30    /* time after which the device is put into low power mode in case of no USB activity */
#define RESUME_RECOVERY      370    /* time to wait after resume */

#pragma DATA_SEG __SHORT_SEG Z_PAGE
extern volatile U8 char suspend_timer;
#pragma DATA_SEG DEFAULT

#endif