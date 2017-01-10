//
// Created by Admin on 2017-01-09.
//

#include "aprs.h"
#include "QAPRSBase.h"
#include "stdio.h"


QAPRSBase qaprs;

void aprs_init(){
  qaprs.init(0, 0, (char *) "SQ5RWU", '0', (char *) "APZQAP", '0', (char *) "WIDE1-1");
}

void aprs_timer_handler() {
  qaprs.timerInterruptHandler();
}

uint8_t aprs_is_active() {
  return qaprs.enabled;
}
void aprs_test(uint16_t x) {
  char packet_buffer[128];
  sprintf(packet_buffer, ":T %d", x);
  qaprs.sendData(packet_buffer);
}

void aprs_change_tone_time(uint16_t x) {
  qaprs._toneSendTime = x;
}

void t(){
//  // nadanie paketu typu komentarz
//  packet_buffer = ":TEST TEST TEST de SQ5RWU";
//  // zmiana adresu źródłowego i ssida
//  QAPRS.setFromAddress("SQ5R", '1');
//  QAPRS.sendData(packet_buffer);
//  // nadanie pakietu z pozycja i symbolem wahadlowca
//  packet_buffer = "!5215.68N/02057.48ES#";
//  // zmiana adresu źródłowego, ssida i ścieżki
//  QAPRS.setFromAddress("SQ5RWU", '2');
//  QAPRS.setRelays("WIDE2-2");
//  QAPRS.sendData(packet_buffer);
//  // nadanie danych pogodowych bez pozycji
//  packet_buffer = "_07071805c025s009g008t030r000p000P000h00b10218";
//  // zmiana ścieżki
//  QAPRS.setRelays("WIDE1-1");
//  QAPRS.sendData(packet_buffer);
//  delay(5000);
}