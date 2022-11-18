void delay_ms(unsigned int value){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC     = 36000-1;             //��������� �������� ��� ������������ �����������
	TIM6->CR1     = TIM_CR1_OPM;        //����� ������ ��������

	TIM6->ARR = value;  //��������� �������� ��������
	TIM6->CNT = 0;

	TIM6->EGR |= TIM_EGR_UG;
	TIM6->SR &= ~(TIM_SR_UIF);

	TIM6->CR1 = TIM_CR1_CEN; //��������� ������

	while((TIM6->SR & TIM_SR_UIF)==0){} //��������� ����� ��������
	TIM6->SR &= ~TIM_SR_UIF;	      //�������� ����

	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC     = 36000-1;             //��������� �������� ��� ������������ �����������
	TIM6->CR1     = TIM_CR1_OPM;        //����� ������ ��������

	TIM6->ARR = value;  //��������� �������� ��������
	TIM6->CNT = 0;

	TIM6->EGR |= TIM_EGR_UG;
	TIM6->SR &= ~(TIM_SR_UIF);

	TIM6->CR1 = TIM_CR1_CEN; //��������� ������

	while((TIM6->SR & TIM_SR_UIF)==0){} //��������� ����� ��������
	TIM6->SR &= ~TIM_SR_UIF;	      //�������� ����
};

void delay_us(unsigned int value){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC     = 72-1;             //��������� �������� ��� ������������ �����������
	TIM6->CR1     = TIM_CR1_OPM;        //����� ������ ��������

	TIM6->ARR = value;  //��������� �������� ��������
	TIM6->CNT = 0;

	TIM6->EGR |= TIM_EGR_UG;
	TIM6->SR &= ~(TIM_SR_UIF);

	TIM6->CR1 = TIM_CR1_CEN; //��������� ������

	while((TIM6->SR & TIM_SR_UIF)==0){} //��������� ����� ��������
	TIM6->SR &= ~TIM_SR_UIF;	      //�������� ����

};

