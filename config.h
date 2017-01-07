//
// Created by SQ5RWU on 2016-12-24.
//

#ifndef STM32_RTTY_CONFIG_H
#define STM32_RTTY_CONFIG_H
//**************config**************
#define CALLSIGN "NO1LIC-1" // put your callsign here
//*************frequency********************
#define RTTY_FREQUENCY  434.150f //Mhz middle frequency
//************rtty speed****************** si4032
#define RTTY_SPEED  300
//************rtty bits****************** si4032
#define RTTY_7BIT   1
//************rtty stop bits****************** si4032
#define RTTY_USE_2_STOP_BITS   0
//********* power definition**************************
#define Smoc  7 // PWR 0...7 0- MIN ... 7 - MAX
//***************************************************

//********** frame delay in msec**********
#define tx_delay  5000


#endif //STM32_RTTY_CONFIG_H
