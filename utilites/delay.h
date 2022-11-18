void delay_ms(unsigned int value){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC     = 36000-1;             //настроить делитель для формирования миллисекунд
	TIM6->CR1     = TIM_CR1_OPM;        //режим одного импульса

	TIM6->ARR = value;  //загрузить значение задержки
	TIM6->CNT = 0;

	TIM6->EGR |= TIM_EGR_UG;
	TIM6->SR &= ~(TIM_SR_UIF);

	TIM6->CR1 = TIM_CR1_CEN; //запустить таймер

	while((TIM6->SR & TIM_SR_UIF)==0){} //дождаться конца задержки
	TIM6->SR &= ~TIM_SR_UIF;	      //сбросить флаг

	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC     = 36000-1;             //настроить делитель для формирования миллисекунд
	TIM6->CR1     = TIM_CR1_OPM;        //режим одного импульса

	TIM6->ARR = value;  //загрузить значение задержки
	TIM6->CNT = 0;

	TIM6->EGR |= TIM_EGR_UG;
	TIM6->SR &= ~(TIM_SR_UIF);

	TIM6->CR1 = TIM_CR1_CEN; //запустить таймер

	while((TIM6->SR & TIM_SR_UIF)==0){} //дождаться конца задержки
	TIM6->SR &= ~TIM_SR_UIF;	      //сбросить флаг
};

void delay_us(unsigned int value){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC     = 72-1;             //настроить делитель для формирования миллисекунд
	TIM6->CR1     = TIM_CR1_OPM;        //режим одного импульса

	TIM6->ARR = value;  //загрузить значение задержки
	TIM6->CNT = 0;

	TIM6->EGR |= TIM_EGR_UG;
	TIM6->SR &= ~(TIM_SR_UIF);

	TIM6->CR1 = TIM_CR1_CEN; //запустить таймер

	while((TIM6->SR & TIM_SR_UIF)==0){} //дождаться конца задержки
	TIM6->SR &= ~TIM_SR_UIF;	      //сбросить флаг

};

