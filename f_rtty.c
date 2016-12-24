#include "f_rtty.h"

volatile unsigned char nr_bit = 0;

rttyStates send_rtty(char *znak) {
  nr_bit++;
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
