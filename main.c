// STM32F100 and SI4032 RTTY transmitter
// released under GPL v.2 by anonymous developer
// enjoy and have a nice day
// ver 1.5a
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_rcc.h>
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
#include "f_rtty.h"
#include "fun.h"
#include "init.h"
#include "config.h"





#include "radio.h"

///////////////////////////// test mode /////////////
unsigned char test = 0; // 0 - normal, 1 - short frame only cunter, height, flag
char callsign[15] = {CALLSIGN};

#define gps_RMC_dlugosc        5
#define gps_RMC_dlugosc_len      10
#define gps_RMC_dlugosc_kier    6
#define gps_RMC_dlugosc_kier_len  1
#define gps_RMC_szerokosc      3
#define gps_RMC_szerokosc_len    9
#define gps_RMC_szerokosc_kier    4
#define gps_RMC_szerokosc_kier_len  1
#define gps_RMC_status        2
#define gps_RMC_status_len      1
#define gps_RMC_data        9
#define gps_RMC_data_len      6
#define gps_RMC_czas        1
#define gps_RMC_czas_len      6
#define gps_RMC_predkosc      7
#define gps_RMC_predkosc_len    5
#define gps_RMC_kierunek      8
#define gps_RMC_kierunek_len    5
#define gps_GGA_wysokosc      9
#define gps_GGA_wysokosc_len    5 //*
#define gps_GGA_use_sat        7
#define gps_GGA_use_sat_len      2 //*
#define gps_VTG_predkosc      7
#define gps_VTG_predkosc_len    5
#define GREEN  GPIO_Pin_7
#define RED  GPIO_Pin_8
#define GPS_START  '$'
#define GPS_STOP  0x0a      // LF
#define OFF  GPIO_Pin_12
unsigned int send_cun;        //frame counter
char czas[7] = {"000000"};
char status[2] = {'N'};
char dlugosc[14] = {"0.00000"};
char dlugosc_kier = 0;//'W';
char szerokosc[13] = {"0.00000"};
char szerokosc_kier = 0;// 'S';s
char wysokosc[6] = {"0"};
char predkosc[6] = {"0"};
char kierunek[6] = {"0"};
int8_t temperatura;
char FixOk = 0;
int napiecie;
unsigned int czest;
float fdlugosc = 0;
float fszerokosc = 0;
float deg = 0;
float fbuf = 0;
char use_sat[3] = {'0'};
char flaga = ((((tx_delay / 1000) & 0x0f) << 3) | Smoc);
uint16_t CRC_rtty = 0x12ab;  //checksum
char buf[512];
char buf_rtty[200];
char menu[] = "$$$$$$STM32 RTTY tracker by Blasiu, enjoy and see you on the HUB... \n\r";
char init_trx[] = "\n\rPowering up TX\n\r";
char powitanie[] = "greetings from earth";
unsigned char pun = 0;
unsigned int cun = 10;
char temp;
unsigned char dev = 0;
unsigned char tx_on = 0;
unsigned int tx_on_delay;
unsigned char tx_enable = 0;
rttyStates send_rtty_status = rttyZero;
unsigned char cun_rtty = 0;
char *rtty_buf;
unsigned char GPS_temp;
char GPS_buf[200];
char *wsk_GPS_buf;
char GPS_rec = 0;
char new_GPS_msg = 0;
unsigned char crc_GPS = 0;
char crc_GPS_start = 0;
char crc_GPS_rec = 0;
char crc_GPS_cun = 0;
char confGPSNAV[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
                     0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
char GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};//, 0x4D, 0x3B};
char GPS_ZDA_OFF[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char GPS_GLL_OFF[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char GPS_GSA_OFF[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char GPS_GSV_OFF[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int Button = 0;
unsigned char cun_off = 0;
unsigned char bOFF = 0;
unsigned char bCheckKay = 0;
unsigned char GPSConf = 0;

void USART1_IRQHandler(void) {
  if ((USART1->SR & USART_FLAG_RXNE) != (u16) RESET) {

    GPS_temp = USART_ReceiveData(USART1);
    USART_SendData(USART3, GPS_temp);
    if (GPS_temp == '*') {
      crc_GPS_start = 0;
      crc_GPS_cun = 0;
      crc_GPS_rec = 0;
    }
    if (crc_GPS_start) {
      crc_GPS ^= GPS_temp;
    } else {
      if (GPS_rec) {
        if (crc_GPS_cun < 3 && crc_GPS_cun > 0) {
          crc_GPS_rec *= 16;
          crc_GPS_rec += HexCharToInt(GPS_temp);
        }
        crc_GPS_cun++;
      }

    }
    if (GPS_temp == GPS_START) {
      wsk_GPS_buf = GPS_buf;
      GPS_rec = 1;
      crc_GPS = 0;
      crc_GPS_start = 1;
    }
    if (GPS_rec) {
      *(wsk_GPS_buf++) = GPS_temp;
    }

    if (GPS_temp == GPS_STOP) {
      GPS_rec = 0;
      *(wsk_GPS_buf) = 0x00;

      if (crc_GPS_rec == crc_GPS) {
        new_GPS_msg = 1;
        GPS_rec = 0;
      }
    }
  }
}

void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
  if (tx_on /*&& ++cun_rtty == 17*/) {
    cun_rtty = 0;
    send_rtty_status = send_rtty(rtty_buf);
    if (send_rtty_status == rttyEnd) {
      GPIO_SetBits(GPIOB, RED);
      if (*(++rtty_buf) == 0) {
        tx_on = 0;
        tx_on_delay = tx_delay;//2500;
        tx_enable = 0;
        radio_disable_tx();
      }
    } else if (send_rtty_status == rttyOne) {
      temp = radio_rw_register(0x73, 0x02, 1);
      GPIO_SetBits(GPIOB, RED);
    } else if (send_rtty_status == rttyZero) {
      temp = radio_rw_register(0x73, 0x00, 1);
      GPIO_ResetBits(GPIOB, RED);
    }
  }
  if (!tx_on && --tx_on_delay == 0) {
    tx_enable = 1;
    tx_on_delay--;
  }
  if (--cun == 0) {
    cun_off++;
    if (pun) {
      GPIO_ResetBits(GPIOB, GREEN);
      pun = 0;
    } else {
      if (flaga & 0x80) {
        GPIO_SetBits(GPIOB, GREEN);
      }
      pun = 1;
    }
    cun = 200;
  }
  bCheckKay = 1;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
int main(void) {
#ifdef DEBUG
  debug();
#endif
  RCC_Conf();
  NVIC_Conf();
  init_port();
  init_timer(RTTY_SPEED);

  wsk_GPS_buf = GPS_buf;


  GPIO_SetBits(GPIOB, RED);
  USART_SendData(USART3, 0xc);
  print(menu);
  temp = radio_rw_register(0x02, 0xff, 0);
  //send_hex(temp);

  temp = radio_rw_register(0x03, 0xff, 0);
  temp = radio_rw_register(0x04, 0xff, 0);
  radio_soft_reset();
  print(init_trx);
  // programowanie czestotliwosci nadawania
  radio_set_tx_frequency();

  // Programowanie mocy nadajnika
  temp = radio_rw_register(0x6D, 00 | (Smoc & 0x0007), 1);

  temp = radio_rw_register(0x71, 0x00, 1);
  temp = radio_rw_register(0x87, 0x08, 0);
  temp = radio_rw_register(0x02, 0xff, 0);
  temp = radio_rw_register(0x75, 0xff, 0);
  temp = radio_rw_register(0x76, 0xff, 0);
  temp = radio_rw_register(0x77, 0xff, 0);
  temp = radio_rw_register(0x12, 0x20, 1);
  temp = radio_rw_register(0x13, 0x00, 1);
  temp = radio_rw_register(0x12, 0x00, 1);
  temp = radio_rw_register(0x0f, 0x80, 1);
  rtty_buf = buf_rtty;
  tx_on = 0;
  tx_enable = 1;
  //tx_enable =0;
  Button = ADCVal[1];

  while (1) {
    if (status[0] == 'A') {
      flaga |= 0x80;
    } else {
      flaga &= ~0x80;
    }

    if (tx_on == 0 && tx_enable) {
      start_bits = RTTY_PRE_START_BITS;
      temp = radio_rw_register(0x11, 0xff, 0); //odczyt ADC
      temperatura = (int8_t) (-64 + (temp * 5 / 10) - 16);
      temp = radio_rw_register(0x0f, 0x80, 1);

      napiecie = srednia(ADCVal[0] * 600 / 4096);

      fdlugosc = atoff(dlugosc);
      deg = (int) (fdlugosc / 100);
      fbuf = fdlugosc - (deg * 100);
      fdlugosc = deg + fbuf / 60;
      if (dlugosc_kier == 'W') {
        fdlugosc *= -1;
      }
      fszerokosc = atoff(szerokosc);
      deg = (int) (fszerokosc / 100);
      fbuf = fszerokosc - (deg * 100);
      fszerokosc = deg + fbuf / 60;
      if (szerokosc_kier == 'S') {
        fszerokosc *= -1;
      }
      if (test) {
        sprintf(buf_rtty, "$$$$$$$%d,%d,%02x", send_cun, atoi(wysokosc), flaga);
      } else {
        sprintf(buf_rtty, "$$$$$$$%s,%d,%s,%.5f,%.5f,%d,%d,%s,%d,%d,%d,%02x", callsign, send_cun, czas, fszerokosc,
                fdlugosc, atoi(wysokosc), atoi(predkosc), kierunek, temperatura, napiecie, atoi(use_sat), flaga);
      }
      CRC_rtty = 0xffff;                                              //napiecie      flaga
      CRC_rtty = gps_CRC16_checksum(buf_rtty + 7);
      sprintf(buf_rtty, "%s*%04X\n", buf_rtty, CRC_rtty & 0xffff);
      rtty_buf = buf_rtty;
      radio_enable_tx();
      tx_on = 1;

      send_cun++;
    }

    if (new_GPS_msg) {
      //print(GPS_buf);
      new_GPS_msg = 0;
      if (strncmp(GPS_buf, "$GPRMC", 6) == 0) {
        if (czytaj_GPS(gps_RMC_czas, gps_RMC_czas_len, GPS_buf, czas) == 0) {
          strcpy(czas, "000000");
        }

        if (czytaj_GPS(gps_RMC_kierunek, gps_RMC_kierunek_len, GPS_buf, kierunek) == 0) {
          strcpy(kierunek, "0");
        }
        if (czytaj_GPS(gps_RMC_szerokosc, gps_RMC_szerokosc_len, GPS_buf, szerokosc) == 0) {
          strcpy(szerokosc, "0.00000");
        }

        if (czytaj_GPS(gps_RMC_szerokosc_kier, gps_RMC_szerokosc_kier_len, GPS_buf, &szerokosc_kier) == 0) {
          szerokosc_kier = 0;
        }

        if (czytaj_GPS(gps_RMC_dlugosc, gps_RMC_dlugosc_len, GPS_buf, dlugosc) == 0) {
          strcpy(dlugosc, "0.00000");
        }

        if (czytaj_GPS(gps_RMC_dlugosc_kier, gps_RMC_dlugosc_kier_len, GPS_buf, &dlugosc_kier) == 0) {
          dlugosc_kier = 0;
        }


        if (czytaj_GPS(gps_RMC_status, gps_RMC_status_len, GPS_buf, status) == 0) {
          strcpy(status, "-");
        }


      }
      if (strncmp(GPS_buf, "$GPGGA", 6) == 0) {
        if (czytaj_GPS(gps_GGA_wysokosc, gps_GGA_wysokosc_len, GPS_buf, wysokosc) == 0) {
          strcpy(wysokosc, "0");
        }
        if (czytaj_GPS(gps_GGA_use_sat, gps_GGA_use_sat_len, GPS_buf, use_sat) == 0) {
          strcpy(use_sat, "0");
        }

      }

      if (strncmp(GPS_buf, "$GPVTG", 6) == 0) {
        if (czytaj_GPS(gps_VTG_predkosc, gps_VTG_predkosc_len, GPS_buf, predkosc) == 0) {
          strcpy(predkosc, "0");
        }
      }
      if (strncmp(GPS_buf, "$GPZDA", 6) == 0) {
        switch (GPSConf) {
          case 0:
            sendtogps(confGPSNAV, sizeof(confGPSNAV) / sizeof(uint8_t));
            break;
          case 1:
            sendtogps(GPS_GSV_OFF, sizeof(GPS_GSV_OFF) / sizeof(uint8_t));
            break;
          case 2:
            sendtogps(GPS_GSA_OFF, sizeof(GPS_GSA_OFF) / sizeof(uint8_t));
            break;
          case 3:
            sendtogps(GPS_GLL_OFF, sizeof(GPS_GLL_OFF) / sizeof(uint8_t));
            break;
          case 4:
            sendtogps(GPS_ZDA_OFF, sizeof(GPS_ZDA_OFF) / sizeof(uint8_t));
            break;
          default:break;
        }
        GPSConf++;
      }
    }

  }
}

#pragma clang diagnostic pop

#ifdef  DEBUG
void assert_failed(uint8_t* file, uint32_t line)
{
    while (1);
}
#endif
