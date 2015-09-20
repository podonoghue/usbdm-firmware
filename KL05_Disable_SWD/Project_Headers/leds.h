/**
 * @file    leds.h
 * @brief   Basic LED control for demo boards
 * @date    13 June 2015
 */
#ifndef LEDS_H_
#define LEDS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup Demo_board_LED_control_group Demo board LED control
 * @brief Basic functions for on-board LEDs
 * @{
 */

/**
 * Turn on green LED
 */
void greenLedOn(void);
/**
 * Turn off green LED
 */
void greenLedOff(void);
/**
 * Toggle green LED
 */
void greenLedToggle(void);
/**
 * Turn on red LED
 */
void redLedOn(void);
/**
 * Turn off red LED
 */
void redLedOff(void);
/**
 * Toggle red LED
 */
void redLedToggle(void);
/**
 * Turn on blue LED
 */
void blueLedOn(void);
/**
 * Turn off blue LED
 */
void blueLedOff(void);
/**
 * Toggle blue LED
 */
void blueLedToggle(void);
/**
 * Enable blue LED
 */
void blueLedEnable(void);
/**
 * Disable blue LED
 */
void blueLedDisable(void);
/**
 * Turn off orange LED
 */
void orangeLedOn(void);
/**
 * Turn off orange LED
 */
void orangeLedOff(void);
/**
 * Toggle orange LED
 */
void orangeLedToggle(void);
/**
 * Initialise all LED
 */
void led_initialise(void);

/**
 * @}
 */ /* end of group Demo_board_LED_control_group */

#ifdef __cplusplus
}
#endif

#endif /* LEDS_H_ */
