int HexCharToInt(char ch);
void print( char* s);
void sendtogps(uint8_t* s, unsigned char cun);
void send_hex(unsigned char data);

unsigned char czytaj_GPS(unsigned char pos,unsigned char len,  char *source, char * destination);
uint16_t gps_CRC16_checksum (char *string);
int srednia (int dana);
