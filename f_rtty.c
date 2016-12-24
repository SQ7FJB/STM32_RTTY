#include <stdint.h>
#include "f_rtty.h"

uint8_t start_bits;
rttyStates send_rtty(char *znak) {
  static uint8_t nr_bit = 0;
  nr_bit++;
  if (start_bits){
    start_bits--;
    return rttyOne;
  }

  if (nr_bit == 1) {
    return rttyZero;
  }
  if (nr_bit > 1 && nr_bit < 10) {
    if ((*(znak) >> (nr_bit - 2)) & 0x01) {
      return rttyOne;
    } else {
      return rttyZero;
    }
  }

  if (nr_bit == 10) {
    return rttyOne;
  }
  if (nr_bit == 11) {
    return rttyOne;
  }

  nr_bit = 0;
  return rttyEnd;
}
