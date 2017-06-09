//
// Created by SQ5RWU on 2016-12-24.
//

#ifndef STM32_RTTY_CONFIG_H
#define STM32_RTTY_CONFIG_H

#ifdef USE_EXTERNAL_CONFIG
#include "config_external.h"
#else

//**************config**********************
#define CALLSIGN "DF8OE" // put your RTTY callsign here
#define APRS_CALLSIGN "DF8OE" // put your APRS callsign here
#define APRS_SSID 'B' // put your APRS SSID here
// 0 --> Your primary station usually fixed and message capable
// 1 --> generic additional station, digi, mobile, wx, etc.
// 2 --> generic additional station, digi, mobile, wx, etc.
// 3 --> generic additional station, digi, mobile, wx, etc.
// 4 --> generic additional station, digi, mobile, wx, etc.
// 5 --> Other network sources (Dstar, Iphones, Blackberry's etc)
// 6 --> Special activity, Satellite ops, camping or 6 meters, etc.
// 7 --> walkie talkies, HT's or other human portable
// 8 --> boats, sailboats, RV's or second main mobile
// 9 --> Primary Mobile (usually message capable)
// A --> internet, Igates, echolink, winlink, AVRS, APRN, etc.
// B --> balloons, aircraft, spacecraft, etc.
// C --> APRStt, DTMF, RFID, devices, one-way trackers*, etc.
// D --> Weather stations
// E --> Truckers or generally full time drivers
// F --> generic additional station, digi, mobile, wx, etc.

#define APRS_COMMENT " Hello from the sky!"
#define RTTY_TO_APRS_RATIO 5 //transmit APRS packet with each x RTTY packet

//*************frequency********************
#define RTTY_FREQUENCY  434.500f //Mhz middle frequency
#define APRS_FREQUENCY  432.500f //Mhz middle frequency
//************rtty speed*********************** si4032
#define RTTY_SPEED  75 // RTTY baudrate
// SHIFT is always 450Hz
//************rtty bits************************ si4032
#define RTTY_7BIT   1 // if 0 --> 5 bits
//************rtty stop bits******************* si4032
#define RTTY_USE_2_STOP_BITS   0
//********* power definition**************************
#define TX_POWER  0 // PWR 0...7 0- MIN ... 7 - MAX
// 0 --> -8dBm
// 1 --> -5dBm
// 2 --> -2dBm
// 3 --> 1dBm
// 4 --> 4dBm
// 5 --> 7dBm
// 6 --> 10dBm
// 7 --> 13dBm
//****************************************************
// WARNING: do not use this in flying tracker!
#define ALLOW_DISABLE_BY_BUTTON 1
//********** frame delay in msec**********************
#define tx_delay  5000
#endif

#endif //STM32_RTTY_CONFIG_H
