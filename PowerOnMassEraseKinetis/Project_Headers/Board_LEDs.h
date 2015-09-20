#ifndef BOARD_LEDS_H_
#define BOARD_LEDS_H_

#if defined(MCU_MKM33Z5)
// PROTO_MKM33Z5
//==========================================================
#define LED_RED_PORT               A   // Port register
#define LED_RED_NUM                0   // Pin number

#define LED_GREEN_PORT             A   
#define LED_GREEN_NUM              1   

#endif

#if defined(MCU_MKL04Z4)
// PROTO_MKL04Z4
//==========================================================
#endif

#if defined(MCU_MK22F12)
// PROTO_MK22F12
//==========================================================
#define LED_RED_PORT               C   // Port register
#define LED_RED_NUM                3   // Pin number

#define LED_GREEN_PORT             D   
#define LED_GREEN_NUM              4   

#define LED_BLUE_PORT              A   
#define LED_BLUE_NUM               2   

#endif

#if defined(MCU_MK20D5)
// FRDM_K20D50M
//==========================================================
#define LED_RED_PORT               C   // Port register
#define LED_RED_NUM                3   // Pin number

#define LED_GREEN_PORT             D   
#define LED_GREEN_NUM              4   

#define LED_BLUE_PORT              A   
#define LED_BLUE_NUM               2   

#endif

#if defined(MCU_MK22F51212)
// FRDM_K22F
//==========================================================
#define LED_RED_PORT               A   // Port register
#define LED_RED_NUM                1   // Pin number

#define LED_GREEN_PORT             A   
#define LED_GREEN_NUM              2   

#define LED_BLUE_PORT              D   
#define LED_BLUE_NUM               5   

#endif

#if defined(MCU_MK64F12)
// FRDM_K64F
//==========================================================
#define LED_RED_PORT               B   // Port register
#define LED_RED_NUM                22  // Pin number

#define LED_GREEN_PORT             E   
#define LED_GREEN_NUM              26  

#define LED_BLUE_PORT              B   
#define LED_BLUE_NUM               21  

#endif

#if defined(MCU_MKE02Z2)
// FRDM_KE02Z
//==========================================================
#define LED_RED_PORT               B   // Port register
#define LED_RED_NUM                25  // Pin number

#define LED_GREEN_PORT             B   
#define LED_GREEN_NUM              26  

#define LED_BLUE_PORT              B   
#define LED_BLUE_NUM               7   

#endif

#if defined(MCU_MKE02Z4)
// FRDM_KE02Z40M
//==========================================================
#define LED_RED_PORT               B   // Port register
#define LED_RED_NUM                25  // Pin number

#define LED_GREEN_PORT             B   
#define LED_GREEN_NUM              26  

#define LED_BLUE_PORT              B   
#define LED_BLUE_NUM               7   

#endif

#if defined(MCU_MKE04Z8M4)
// FRDM_KE04Z
//==========================================================
#define LED_RED_PORT               A   // Port register
#define LED_RED_NUM                21  // Pin number

#define LED_GREEN_PORT             A   
#define LED_GREEN_NUM              20  

#define LED_BLUE_PORT              A   
#define LED_BLUE_NUM               11  

#endif

#if defined(MCU_MKE06Z4)
// FRDM_KE06Z
//==========================================================
#define LED_RED_PORT               B   // Port register
#define LED_RED_NUM                21  // Pin number

#define LED_GREEN_PORT             B   
#define LED_GREEN_NUM              22  

#define LED_BLUE_PORT              B   
#define LED_BLUE_NUM               23  

#endif

#if defined(MCU_MKL02Z4)
// FRDM_KL02Z
//==========================================================
#define LED_RED_PORT               B   // Port register
#define LED_RED_NUM                7   // Pin number

#define LED_GREEN_PORT             B   
#define LED_GREEN_NUM              6   

#define LED_BLUE_PORT              B   
#define LED_BLUE_NUM               10  

#endif

#if defined(MCU_MKL03Z4)
// FRDM_KL03Z
//==========================================================
#define LED_RED_PORT               B   // Port register
#define LED_RED_NUM                10  // Pin number

#define LED_GREEN_PORT             B   
#define LED_GREEN_NUM              11  

#define LED_BLUE_PORT              B   
#define LED_BLUE_NUM               13  

#endif

#if defined(MCU_MKL05Z4)
// FRDM_KL05Z
//==========================================================
#define LED_RED_PORT               B   // Port register
#define LED_RED_NUM                8   // Pin number

#define LED_GREEN_PORT             B   
#define LED_GREEN_NUM              9   

#define LED_BLUE_PORT              B   
#define LED_BLUE_NUM               10  

#endif

#if defined(MCU_MKL25Z4)
// FRDM_KL25Z
//==========================================================
#define LED_RED_PORT               B   // Port register
#define LED_RED_NUM                18  // Pin number

#define LED_GREEN_PORT             B   
#define LED_GREEN_NUM              19

#define LED_BLUE_PORT              D   
#define LED_BLUE_NUM               1   

#endif

#if defined(MCU_MKL26Z4)
// FRDM_KL26Z
//==========================================================
#define LED_RED_PORT               E   // Port register
#define LED_RED_NUM                31  // Pin number

#define LED_GREEN_PORT             E   
#define LED_GREEN_NUM              29  

#define LED_BLUE_PORT              D   
#define LED_BLUE_NUM               13  

#endif


#if defined(MCU_MKL43Z4)
// FRDM_KL43Z
//==========================================================
#define LED_RED_PORT               E   // Port register
#define LED_RED_NUM                31  // Pin number

#define LED_GREEN_PORT             D
#define LED_GREEN_NUM              5

#endif

#if defined(MCU_MKL46Z4)
// FRDM_KL46Z
//==========================================================
#define LED_RED_PORT               D   // Port register
#define LED_RED_NUM                5   // Pin number

#define LED_GREEN_PORT             E   
#define LED_GREEN_NUM              29  

#endif

#if defined(MCU_MK40D10) || defined(MCU_MK40DZ10)
// TWR_K40
//==========================================================
#define LED_RED_PORT               C   // Port register
#define LED_RED_NUM                7   // Pin number

#define LED_ORANGE_PORT            C   
#define LED_ORANGE_NUM             8   

#define LED_GREEN_PORT             C   
#define LED_GREEN_NUM              9   

#define LED_BLUE_PORT              B   
#define LED_BLUE_NUM               11  

#endif

#if defined(MCU_MK60DZ10) || defined(MCU_MK60D10)
// TWR_K60
//==========================================================
#define LED_RED_PORT               A   // Port register
#define LED_RED_NUM                11  // Pin number

#define LED_ORANGE_PORT            A   
#define LED_ORANGE_NUM             28  

#define LED_GREEN_PORT             A   
#define LED_GREEN_NUM              29  

#define LED_BLUE_PORT              A   
#define LED_BLUE_NUM               10  

#endif

#if defined(MCU_mcf51jf128)
// TWR_JF128
//==========================================================
#define LED_ORANGE_PORT            C   // Port register
#define LED_ORANGE_NUM             8   // Pin number

#define LED_GREEN_PORT             C   
#define LED_GREEN_NUM              9   

#endif

#if defined(MCU_mcf51ju128)
// TWR_JU128
//==========================================================
#define LED_ORANGE_PORT            A   // Port register
#define LED_ORANGE_NUM             0   // Pin number

#define LED_GREEN_PORT             A   
#define LED_GREEN_NUM              1   

#endif

#endif /* BOARD_LEDS_H_ */
