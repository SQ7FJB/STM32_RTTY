//
// Created by Admin on 2016-12-24.
//

#ifndef STM32_RTTY_RADIO_H
#define STM32_RTTY_RADIO_H

#include "config.h"
#include <stdint.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_gpio.h>

static const uint16_t radioNSELpin = GPIO_Pin_13;
static const uint8_t WR = 0x80;

//************ do not touch bellow this line;) *********************
#define gen_div    3  //Stała nie zmieniac
#define gen  ((26.0/gen_div) *(fbsel+1)) //26 ->26MHZ kwarc napedzajacy nadajnik
#define fc    (((freq/gen) - fb - 24) * 64000)


uint8_t _spi_sendrecv(const uint16_t data_word);
uint8_t radio_rw_register(const uint8_t register_addr, uint8_t value, uint8_t write);
void radio_set_tx_frequency();

#endif //STM32_RTTY_RADIO_H
