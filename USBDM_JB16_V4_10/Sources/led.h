#ifndef _LED_H_
#define _LED_H_

#define LED_BLINK_PERIOD  150   /* duration of LED period */
#define LED_OFF_TIME      75    /* how long should the LED be off during the blink period */

typedef enum {
  LED_ON,
  LED_OFF,
  LED_BLINK
} led_state_e;

#pragma DATA_SEG __SHORT_SEG Z_PAGE
extern U8 led_state;
#pragma DATA_SEG DEFAULT

#endif
