#include <stm32f10x.h>
#include <core_cm3.h>
#include <stm32f10x_rcc.h>
#include <misc.h>
#include "delay.h"

void delay_init() {
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}

void _delay_us(uint32_t us, uint8_t precise) {
  SysTick->CTRL  &= ~(SysTick_CTRL_ENABLE_Msk);
  SysTick->VAL = us * 6;
  SysTick->LOAD = us * 6;
  SysTick->CTRL  |= (SysTick_CTRL_ENABLE_Msk);
  while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {}
}

inline void _delay_ms(uint32_t ms) {
  while(ms-- > 0){
    _delay_us(1000, 0);
  }
}


