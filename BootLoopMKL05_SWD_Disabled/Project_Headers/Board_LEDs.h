/**
 * @file    Board_LEDs.h
 * @brief   LED Pin definitions for demo boards
 * @date    13 June 2015
 */
#ifndef BOARD_LEDS_H_
#define BOARD_LEDS_H_

/**
 * @addtogroup Demo_board_LED_control_group Demo board LED control
 * @{
 */

#if defined(MCU_MKM33Z5)
// PROTO_MKM33Z5
//==========================================================
#define LED_RED_PORT               A   //!< LED pin register
#define LED_RED_NUM                0   //!< LED Pin number

#define LED_GREEN_PORT             A   //!< LED pin register
#define LED_GREEN_NUM              1   //!< LED Pin number

#endif

#if defined(MCU_MKL04Z4)
// PROTO_MKL04Z4
//==========================================================
#endif

#if defined(MCU_MK22F12)
// PROTO_MK22F12
//==========================================================
#define LED_RED_PORT               C   //!< LED pin register
#define LED_RED_NUM                3   //!< LED Pin number

#define LED_GREEN_PORT             D   //!< LED pin register
#define LED_GREEN_NUM              4   //!< LED Pin number

#define LED_BLUE_PORT              A   //!< LED pin register
#define LED_BLUE_NUM               2   //!< LED Pin number

#endif

#if defined(MCU_MKV31F51212)
// FRDM_KV31F
//==========================================================
#define LED_RED_PORT               D   //!< LED pin register
#define LED_RED_NUM                1   //!< LED Pin number

#define LED_GREEN_PORT             D   //!< LED pin register
#define LED_GREEN_NUM              7   //!< LED Pin number

#define LED_BLUE_PORT              E   //!< LED pin register
#define LED_BLUE_NUM               25  //!< LED Pin number

#endif

#if defined(MCU_MK20D5)
// FRDM_K20D50M
//==========================================================
#define LED_RED_PORT               C   //!< LED pin register
#define LED_RED_NUM                3   //!< LED Pin number

#define LED_GREEN_PORT             D   //!< LED pin register
#define LED_GREEN_NUM              4   //!< LED Pin number

#define LED_BLUE_PORT              A   //!< LED pin register
#define LED_BLUE_NUM               2   //!< LED Pin number

#endif

#if defined(MCU_MK22F51212)
// FRDM_K22F
//==========================================================
#define LED_RED_PORT               A   //!< LED pin register
#define LED_RED_NUM                1   //!< LED Pin number

#define LED_GREEN_PORT             A   //!< LED pin register
#define LED_GREEN_NUM              2   //!< LED Pin number

#define LED_BLUE_PORT              D   //!< LED pin register
#define LED_BLUE_NUM               5   //!< LED Pin number

#endif

#if defined(MCU_MK64F12)
// FRDM_K64F
//==========================================================
#define LED_RED_PORT               B   //!< LED pin register
#define LED_RED_NUM                22  //!< LED Pin number

#define LED_GREEN_PORT             E   //!< LED pin register
#define LED_GREEN_NUM              26  //!< LED Pin number

#define LED_BLUE_PORT              B   //!< LED pin register
#define LED_BLUE_NUM               21  //!< LED Pin number

#endif

#if defined(MCU_MKE02Z2)
// FRDM_KE02Z
//==========================================================
#define LED_RED_PORT               B   //!< LED pin register
#define LED_RED_NUM                25  //!< LED Pin number

#define LED_GREEN_PORT             B   //!< LED pin register
#define LED_GREEN_NUM              26  //!< LED Pin number

#define LED_BLUE_PORT              B   //!< LED pin register
#define LED_BLUE_NUM               7   //!< LED Pin number

#endif

#if defined(MCU_MKE02Z4)
// FRDM_KE02Z40M
//==========================================================
#define LED_RED_PORT               B   //!< LED pin register
#define LED_RED_NUM                25  //!< LED Pin number

#define LED_GREEN_PORT             B   //!< LED pin register
#define LED_GREEN_NUM              26  //!< LED Pin number

#define LED_BLUE_PORT              B   //!< LED pin register
#define LED_BLUE_NUM               7   //!< LED Pin number

#endif

#if defined(MCU_MKE04Z8M4)
// FRDM_KE04Z
//==========================================================
#define LED_RED_PORT               A   //!< LED pin register
#define LED_RED_NUM                21  //!< LED Pin number

#define LED_GREEN_PORT             A   //!< LED pin register
#define LED_GREEN_NUM              20  //!< LED Pin number

#define LED_BLUE_PORT              A   //!< LED pin register
#define LED_BLUE_NUM               11  //!< LED Pin number

#endif

#if defined(MCU_MKE06Z4)
// FRDM_KE06Z
//==========================================================
#define LED_RED_PORT               B   //!< LED pin register
#define LED_RED_NUM                21  //!< LED Pin number

#define LED_GREEN_PORT             B   //!< LED pin register
#define LED_GREEN_NUM              22  //!< LED Pin number

#define LED_BLUE_PORT              B   //!< LED pin register
#define LED_BLUE_NUM               23  //!< LED Pin number

#endif

#if defined(MCU_MKL02Z4)
// FRDM_KL02Z
//==========================================================
#define LED_RED_PORT               B   //!< LED pin register
#define LED_RED_NUM                7   //!< LED Pin number

#define LED_GREEN_PORT             B   //!< LED pin register
#define LED_GREEN_NUM              6   //!< LED Pin number

#define LED_BLUE_PORT              B   //!< LED pin register
#define LED_BLUE_NUM               10  //!< LED Pin number

#endif

#if defined(MCU_MKL03Z4)
// FRDM_KL03Z
//==========================================================
#define LED_RED_PORT               B   //!< LED pin register
#define LED_RED_NUM                10  //!< LED Pin number

#define LED_GREEN_PORT             B   //!< LED pin register
#define LED_GREEN_NUM              11  //!< LED Pin number

#define LED_BLUE_PORT              B   //!< LED pin register
#define LED_BLUE_NUM               13  //!< LED Pin number

#endif

#if defined(MCU_MKL05Z4)
// FRDM_KL05Z
//==========================================================
#define LED_RED_PORT               B   //!< LED pin register
#define LED_RED_NUM                8   //!< LED Pin number

#define LED_GREEN_PORT             B   //!< LED pin register
#define LED_GREEN_NUM              9   //!< LED Pin number

#define LED_BLUE_PORT              B   //!< LED pin register
#define LED_BLUE_NUM               10  //!< LED Pin number

#endif

#if defined(MCU_MKL25Z4)
// FRDM_KL25Z
//==========================================================
#define LED_RED_PORT               B   //!< LED pin register
#define LED_RED_NUM                18  //!< LED Pin number

#define LED_GREEN_PORT             B   //!< LED pin register
#define LED_GREEN_NUM              19  //!< LED Pin number

#define LED_BLUE_PORT              D   //!< LED pin register
#define LED_BLUE_NUM               1   //!< LED Pin number

#endif

#if defined(MCU_MKL26Z4)
// FRDM_KL26Z
//==========================================================
#define LED_RED_PORT               E   //!< LED pin register
#define LED_RED_NUM                31  //!< LED Pin number

#define LED_GREEN_PORT             E   //!< LED pin register
#define LED_GREEN_NUM              29  //!< LED Pin number

#define LED_BLUE_PORT              D   //!< LED pin register
#define LED_BLUE_NUM               13  //!< LED Pin number

#endif

#if defined(MCU_MKL27Z4)
// MCU_MKL27Z4
//==========================================================
#define LED_RED_PORT               B   //!< LED pin register
#define LED_RED_NUM                18  //!< LED Pin number

#define LED_GREEN_PORT             B   //!< LED pin register
#define LED_GREEN_NUM              19  //!< LED Pin number

#define LED_BLUE_PORT              A   //!< LED pin register
#define LED_BLUE_NUM               13  //!< LED Pin number

#endif

#if defined(MCU_MKL43Z4)
// FRDM_KL43Z
//==========================================================
#define LED_RED_PORT               E   //!< LED pin register
#define LED_RED_NUM                31  //!< LED Pin number

#define LED_GREEN_PORT             D   //!< LED pin register
#define LED_GREEN_NUM              5   //!< LED Pin number

#endif

#if defined(MCU_MKL46Z4)
// FRDM_KL46Z
//==========================================================
#define LED_RED_PORT               D   //!< LED pin register
#define LED_RED_NUM                5   //!< LED Pin number

#define LED_GREEN_PORT             E   //!< LED pin register
#define LED_GREEN_NUM              29  //!< LED Pin number

#endif

#if defined(MCU_MK40D10) || defined(MCU_MK40DZ10)
// TWR_K40
//==========================================================
#define LED_RED_PORT               C   //!< LED pin register
#define LED_RED_NUM                7   //!< LED Pin number

#define LED_ORANGE_PORT            C   //!< LED pin register
#define LED_ORANGE_NUM             8   //!< LED Pin number

#define LED_GREEN_PORT             C   //!< LED pin register
#define LED_GREEN_NUM              9   //!< LED Pin number

#define LED_BLUE_PORT              B   //!< LED pin register
#define LED_BLUE_NUM               11  //!< LED Pin number

#endif

#if defined(MCU_MK60DZ10) || defined(MCU_MK60D10)
// TWR_K60
//==========================================================
#define LED_RED_PORT               A   //!< LED pin register
#define LED_RED_NUM                11  //!< LED Pin number

#define LED_ORANGE_PORT            A   //!< LED pin register
#define LED_ORANGE_NUM             28  //!< LED Pin number

#define LED_GREEN_PORT             A   //!< LED pin register
#define LED_GREEN_NUM              29  //!< LED Pin number

#define LED_BLUE_PORT              A   //!< LED pin register
#define LED_BLUE_NUM               10  //!< LED Pin number

#endif

#if defined(MCU_mcf51jf128)
// TWR_JF128
//==========================================================
#define LED_ORANGE_PORT            C   //!< LED pin register
#define LED_ORANGE_NUM             8   //!< LED Pin number

#define LED_GREEN_PORT             C   //!< LED pin register
#define LED_GREEN_NUM              9   //!< LED Pin number

#endif

#if defined(MCU_mcf51ju128)
// TWR_JU128
//==========================================================
#define LED_ORANGE_PORT            A   //!< LED pin register
#define LED_ORANGE_NUM             0   //!< LED Pin number

#define LED_GREEN_PORT             A   //!< LED pin register
#define LED_GREEN_NUM              1   //!< LED Pin number

#endif

/**
 * @}
 */ /* end of group Demo_board_LED_control_group */

#endif /* BOARD_LEDS_H_ */
