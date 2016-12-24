//
// Created by Admin on 2016-12-24.
//

#ifndef STM32_RTTY_CONFIG_H
#define STM32_RTTY_CONFIG_H
//**************config**************
#define CALLSIGN "NO1LIC-1" // put your callsign here
//*************frequency********************
#define freq  434.150f //Mhz middle frequency
//************band select****************** si4032
#define fb    1
#define fbsel  1
//fb		fbsel 0			1
// 0		208,0000		415,9992
// 1		216,6675		433,3325
// 2		225,3342		450,6658
//********* power definition**************************
#define Smoc  0 // PWR 0...7 0- MIN ... 7 - MAX
//***************************************************

//********** frame delay in msec**********
#define tx_delay  100    // 2500 ~2,5  w polu flaga wpisywany jest tx_delay/1000 modulo 16 czyl;i dla 16000 bedzie 0 póki co.


#endif //STM32_RTTY_CONFIG_H
