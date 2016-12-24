__IO uint16_t ADCVal[2];
void NVIC_Conf();
void RCC_Conf();
void init_port();
void init_timer(const int rtty_speed);
