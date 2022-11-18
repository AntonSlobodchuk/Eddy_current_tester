/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f37x.h"
#include "delay.h"
#include "itoa_ltoa.h"
#include "math.h"

//#define __FPU_PRESENT 1
//#define __FPU_USED 1






#define SDATA_AD9833_on     GPIOB->ODR |= GPIO_ODR_0
#define SDATA_AD9833_off     GPIOB->ODR &=~ GPIO_ODR_0
#define SCLK_AD9833_on     GPIOA->ODR |= GPIO_ODR_6
#define SCLK_AD9833_off     GPIOA->ODR &=~ GPIO_ODR_6

#define FSYNC_1_AD9833_on     GPIOA->ODR |= GPIO_ODR_5
#define FSYNC_1_AD9833_off     GPIOA->ODR &=~ GPIO_ODR_5

#define FSYNC_2_AD9833_on     GPIOA->ODR |= GPIO_ODR_0
#define FSYNC_2_AD9833_off     GPIOA->ODR &=~ GPIO_ODR_0

#define buzer_on     GPIOB->ODR |= GPIO_ODR_2
#define buzer_off     GPIOB->ODR &=~ GPIO_ODR_2


#define APBCLK   36000000UL
#define BAUDRATE 115200UL //115200


unsigned char s;


float data_old, data_new, k=0.02125897, y, t;




char buf[10];

int temp_sdadc_vphs;
int temp_sdadc_vmag;

unsigned char counter_sdadc_conversion_vphs;
unsigned char counter_sdadc_conversion_vmag;

unsigned long int swr_sdadc_vphs, swr_sdadc_data_vphs;
unsigned long int swr_sdadc_vmag, swr_sdadc_data_vmag;

unsigned int vphs_swr_sdadc;
unsigned int vmag_swr_sdadc;

unsigned char sdadc_swr_vphs_finish=0;
unsigned char sdadc_swr_vmag_finish=0;

unsigned long int XmV_vphs;
unsigned long int XmV_vmag;

unsigned long int ad9833_freq;

int ad9833_freq_LSB, ad9833_freq_MSB;

unsigned int phase;

unsigned char usart_rx_data;

unsigned char data_TX;

int i;




void WriteAD9833(unsigned int data){ // Функция установки нового значения частоты

	unsigned long int ad9833_temp=0x8000;
	unsigned char ad9833_i;





	SCLK_AD9833_on;

	delay_us(1);



	delay_us(1);

	for(ad9833_i=0;ad9833_i<16;ad9833_i++){ //
		SCLK_AD9833_on;

		delay_us(1);

		if(data & ad9833_temp){SDATA_AD9833_on;}else{SDATA_AD9833_off;};

		delay_us(1);
		SCLK_AD9833_off;
		delay_us(1);
		ad9833_temp=ad9833_temp>>1;
		delay_us(1);
	};


	delay_us(1);
};

void uart_print(char *usart_string){
	u8 tmp=0;
	while (usart_string[tmp]) // Пока не конец строки...
		{
		 while (!(USART2->ISR & USART_ISR_TXE)) {} // Ждать освобождения буфера.
		 USART2->TDR=usart_string[tmp]; tmp++; // Отправить байт.
		}
};

void uart_tx_data(char data){

		 while (!(USART2->ISR & USART_ISR_TXE)) {} // Ждать освобождения буфера.
		 USART2->TDR=data; // Отправить байт.

};

void SDADC3_IRQHandler(void)
{
	//if((SDADC3->ISR & SDADC_ISR_ROVRF)!=0)
		//{
			NVIC_DisableIRQ (SDADC3_IRQn);

			SDADC3->ISR |= SDADC_ISR_CLRROVRF; // ?????

			temp_sdadc_vphs=SDADC3->RDATAR;


			data_old=y;

			data_new=temp_sdadc_vphs;

			//if(s==0){s=1; GPIOB->ODR |= GPIO_ODR_2;}else{s=0; GPIOB->ODR &=~ GPIO_ODR_2;};



			GPIOB->ODR |= GPIO_ODR_2;



			y=k*data_new+(1-k)*data_old;



			t=powf(2,y);



			GPIOB->ODR &=~ GPIO_ODR_2;







 SDADC3->CR2 |= SDADC_CR2_RSWSTART;

	 NVIC_EnableIRQ (SDADC3_IRQn);



	};

void SDADC2_IRQHandler(void)
{
	//if((SDADC3->ISR & SDADC_ISR_ROVRF)!=0)
		//{
			NVIC_DisableIRQ (SDADC2_IRQn);

			SDADC2->ISR |= SDADC_ISR_CLRROVRF; // ?????


			counter_sdadc_conversion_vmag++;

			//GPIOB->ODR |= GPIO_ODR_2;

			//GPIOB->ODR &=~ GPIO_ODR_2;


			if(counter_sdadc_conversion_vmag<11){
				temp_sdadc_vmag=SDADC2->RDATAR;
				if((temp_sdadc_vmag>=0)&(temp_sdadc_vmag<=32767)){temp_sdadc_vmag=temp_sdadc_vmag+32768;}else{temp_sdadc_vmag=temp_sdadc_vmag-32768;};
				swr_sdadc_vmag=swr_sdadc_vmag+temp_sdadc_vmag;
			}else{

				counter_sdadc_conversion_vmag=0;


				swr_sdadc_vmag=swr_sdadc_vmag/10;

				swr_sdadc_data_vmag=swr_sdadc_vmag;

				swr_sdadc_vmag=0;

				sdadc_swr_vmag_finish=1;


			//};



		}







	 NVIC_EnableIRQ (SDADC2_IRQn);

	};

void USART2_IRQHandler(void)
{
	if((USART2->ISR & USART_ISR_RXNE)!=0)
		{
			usart_rx_data = USART2->RDR;
		}
	if (USART2->ISR & USART_ISR_ORE){USART2->ICR |= USART_ICR_ORECF;}; //очистка флага переполнения приемопередатчика

		NVIC_DisableIRQ (USART2_IRQn);

		if(usart_rx_data==0x31){

			data_TX=1;


		};

		NVIC_EnableIRQ (USART2_IRQn);

	};

			

int main(void)
{


	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	GPIOB->MODER |= GPIO_MODER_MODER2_0; //  PB2 на выход // БУЗЕР
	GPIOB->OTYPER &=~ GPIO_OTYPER_OT_2;   //Output push-pull
	GPIOB->OSPEEDR	|= GPIO_OSPEEDER_OSPEEDR2; //High speed


	///////////////////////////AD9833///////////////////////
	////////////////////////////////////////////////////////
	GPIOB->MODER |= GPIO_MODER_MODER0_0; //  PB0 на выход // SDATA_AD9833
	GPIOB->OTYPER &=~ GPIO_OTYPER_OT_0;   //Output push-pull
	GPIOB->OSPEEDR	|= GPIO_OSPEEDER_OSPEEDR0; //High speed

	GPIOA->MODER |= GPIO_MODER_MODER6_0; //  PA6 на выход // SCLK_AD9833
	GPIOA->OTYPER &=~ GPIO_OTYPER_OT_6;   //Output push-pull
	GPIOA->OSPEEDR	|= GPIO_OSPEEDER_OSPEEDR6; //High speed

	GPIOA->MODER |= GPIO_MODER_MODER5_0; //  PA5 на выход // FSYNC_1_AD9833
	GPIOA->OTYPER &=~ GPIO_OTYPER_OT_5;   //Output push-pull
	GPIOA->OSPEEDR	|= GPIO_OSPEEDER_OSPEEDR5; //High speed

	GPIOA->MODER |= GPIO_MODER_MODER0_0; //  PA4 на выход // FSYNC_2_AD9833
	GPIOA->OTYPER &=~ GPIO_OTYPER_OT_0;   //Output push-pull
	GPIOA->OSPEEDR	|= GPIO_OSPEEDER_OSPEEDR0; //High speed
	////////////////////////////////////////////////////////




///////////////////////////////////////////////
	GPIOA->MODER   |= GPIO_MODER_MODER1_1;         // PA1 - Alternate function mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;      // PA1 - High speed

	GPIOA->AFR[0] |= 0x00000010; //PA1 находятся в AFR[0] TIM2_CH2

	//PA1 - TIM2_CH2 (AD9833_MCLK) - 18MHz

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                     // Enable clock
	TIM2->PSC = 1;  // Prescaler = 1
	TIM2->ARR = 1;                                      // Auto-reload = PWM frequency in half microseconds = 10ms period
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;     // OC1M = 110 for PWM Mode 1 output on ch2
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;                         // Output 1 preload enable
	TIM2->CR1 |= TIM_CR1_ARPE;                              // Auto-reload preload enable
	TIM2->CCER |= TIM_CCER_CC2E;                            // Enable output for ch2
	TIM2->CCR2 = 1;                                      // CCR2
	TIM2->EGR |= TIM_EGR_UG;                                // Force update
	TIM2->SR &= ~TIM_SR_UIF;                                // Clear the update flag
	TIM2->CR1 |= TIM_CR1_CEN;
	///////////////////////////////////////////////

	/////////////////////////////////////////////// 36kHz
	GPIOA->MODER   |= GPIO_MODER_MODER4_1;         // PA4 - Alternate function mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;      // PA4 - High speed

	GPIOA->AFR[0] |= 0x00020000; //PA4 находятся в AFR[0] TIM3_CH2

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                     // Enable clock
	TIM3->PSC = 1;  // Prescaler = 1
	TIM3->ARR = 1000;                                      // Auto-reload = PWM frequency in half microseconds = 10ms period
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;     // OC1M = 110 for PWM Mode 1 output on ch2
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;                         // Output 1 preload enable
	TIM3->CR1 |= TIM_CR1_ARPE;                              // Auto-reload preload enable
	TIM3->CCER |= TIM_CCER_CC2E;                            // Enable output for ch2
	TIM3->CCR2 = 575;                                      // CCR2
	TIM3->EGR |= TIM_EGR_UG;                                // Force update
	TIM3->SR &= ~TIM_SR_UIF;                                // Clear the update flag
	TIM3->CR1 |= TIM_CR1_CEN;
	///////////////////////////////////////////////


	///////////////USART2////////////////////
	GPIOA->MODER   |= GPIO_MODER_MODER2_1;         // PA2 - Alternate function mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;      // PA2 - High speed

	GPIOA->AFR[0] |= 0x00000700; //PA2 находятся в AFR[1] USART3_TX

	GPIOA->MODER   |= GPIO_MODER_MODER3_1;         // PA3 - Alternate function mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;      // PA3 - High speed

	GPIOA->AFR[0] |= 0x00007000; //PA3 находятся в AFR[0] USART3_RX
	/////////////////////////////////////////

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //Включаем тактирование UART2





	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;

	GPIOB->MODER   |= GPIO_MODER_MODER14;         // PB14 - 11: Analog mode
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR14;      // PB14 - 11: High speed

	GPIOE->MODER   |= GPIO_MODER_MODER8;         // PE8 - 11: Analog mode
	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;      // PE8 - 11: High speed

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;


	///////////////////////VPHS////////////////////////
	RCC->APB2ENR |= RCC_APB2ENR_SDADC3EN;
	RCC->CFGR |= RCC_CFGR_SDADCPRE_DIV48; // 72MHz/48=1.5MHz

	//Частота дискретизации равна 1,5МГц/360тактов = 4166Гц

	PWR->CR |= PWR_CR_SDADC3EN;

	SDADC3->CR1 &=~ SDADC_CR1_REFV;// 00: External reference where the VREFSD+ pin must be forced externally

	//SDADC1->CR1 |= SDADC_CR1_REFV;

	delay_ms(10);

	SDADC3->CR2 |= SDADC_CR2_ADON; //1: SDADC is enabled.
	while (SDADC3-> ISR & SDADC_ISR_STABIP);

	SDADC3->CR1 |= SDADC_CR1_INIT;
	while ((SDADC3-> ISR & SDADC_ISR_INITRDY) == 0);

	SDADC3->CR2 |= SDADC_CR2_RCH_3; //Channel 8 is selected as regular channel

	delay_ms(10);


	SDADC3->CONF0R |= SDADC_CONF0R_COMMON0_1; //Common mode for configuration 0     10: VDD

	//SDADC2->CONF1R |= SDADC_CONF1R_COMMON1_1;

	//SDADC2->CONF2R |= SDADC_CONF2R_COMMON2_1;

	delay_ms(10);

	SDADC3->CONF0R |= SDADC_CONF0R_SE0; //Single-ended offset mode: The corresponding SE[1:0] bits must be set to “01”
	//SDADC2->CONF1R |= SDADC_CONF1R_SE1;
	//SDADC2->CONF2R |= SDADC_CONF2R_SE2;

	delay_ms(10);

	SDADC3->CONFCHR2 &=~ SDADC_CONFCHR2_CONFCH8; //00: Channel i uses the configuration specified in SDADC_CONF0R

	delay_ms(10);

/*
	SDADC1->CONF0R |= SDADC_CONF0R_SE0_0; //01: Conversions are executed in single-ended offset mode
	SDADC1->CONF1R |= SDADC_CONF0R_SE0_0;
	SDADC1->CONF2R |= SDADC_CONF0R_SE0_0;
*/

    SDADC3->CR1 &=~ SDADC_CR1_INIT;// Выпадение из режима инициализации

    delay_ms(10);

    SDADC3->CR2 |= SDADC_CR2_STARTCALIB;// Калибровка
    while (SDADC3-> ISR & SDADC_ISR_CALIBIP);

    delay_ms(10);

    SDADC3->CR1 |= SDADC_CR1_REOCIE;

    NVIC_EnableIRQ (SDADC3_IRQn);

    SDADC3->CR2 |= SDADC_CR2_RCONT;

	SDADC3->CR2 |= SDADC_CR2_RSWSTART; //Software start of a conversion on the regular channel

	//delay_ms(10);
	///////////////////////////////////////////////////
/*
	///////////////////////VPHS////////////////////////
	RCC->APB2ENR |= RCC_APB2ENR_SDADC2EN;
	RCC->CFGR |= RCC_CFGR_SDADCPRE_DIV48; // 72MHz/48=1.5MHz

	PWR->CR |= PWR_CR_SDADC2EN;

	SDADC2->CR1 &=~ SDADC_CR1_REFV;// 00: External reference where the VREFSD+ pin must be forced externally

	//SDADC2->CR1 |= SDADC_CR1_REFV;

	delay_ms(10);

	SDADC2->CR2 |= SDADC_CR2_ADON; //1: SDADC is enabled.
	while (SDADC2-> ISR & SDADC_ISR_STABIP);

	SDADC2->CR1 |= SDADC_CR1_INIT;
	while ((SDADC2-> ISR & SDADC_ISR_INITRDY) == 0);

	SDADC2->CR2 |= SDADC_CR2_RCH_3; //Channel 8 is selected as regular channel

	delay_ms(10);


	SDADC2->CONF0R |= SDADC_CONF0R_COMMON0_1; //Common mode for configuration 0     10: VDD

	//SDADC2->CONF1R |= SDADC_CONF1R_COMMON1_1;

	//SDADC2->CONF2R |= SDADC_CONF2R_COMMON2_1;

	delay_ms(10);

	SDADC2->CONF0R |= SDADC_CONF0R_SE0; //Single-ended offset mode: The corresponding SE[1:0] bits must be set to “01”
	//SDADC2->CONF1R |= SDADC_CONF1R_SE1;
	//SDADC2->CONF2R |= SDADC_CONF2R_SE2;

	delay_ms(10);

	SDADC2->CONFCHR2 &=~ SDADC_CONFCHR2_CONFCH8; //00: Channel i uses the configuration specified in SDADC_CONF0R

	delay_ms(10);


	//SDADC2->CONF0R |= SDADC_CONF0R_SE0_0; //01: Conversions are executed in single-ended offset mode
	//SDADC2->CONF1R |= SDADC_CONF0R_SE0_0;
	//SDADC2->CONF2R |= SDADC_CONF0R_SE0_0;


    SDADC2->CR1 &=~ SDADC_CR1_INIT;// Выпадение из режима инициализации

    delay_ms(10);

    SDADC2->CR2 |= SDADC_CR2_STARTCALIB;// Калибровка
    while (SDADC2-> ISR & SDADC_ISR_CALIBIP);

    delay_ms(10);

    SDADC2->CR1 |= SDADC_CR1_REOCIE;

    NVIC_EnableIRQ (SDADC2_IRQn);

	SDADC2->CR2 |= SDADC_CR2_RSWSTART; //Software start of a conversion on the regular channel

	delay_ms(10);
	///////////////////////////////////////////////////

*/


	ad9833_freq=220000;

	ad9833_freq=(ad9833_freq*1491)/100;

	//ad9833_freq=0x80000000;

	ad9833_freq_LSB =  (ad9833_freq & 0x3fff) | 0x4000; //0x50c7
	//ad9833_freq=ad9833_freq>>14;
	ad9833_freq_MSB = ((ad9833_freq>>14) & 0x3fff) | 0x4000; //0x4000




	SCLK_AD9833_on;
	FSYNC_1_AD9833_off;
	FSYNC_2_AD9833_off;
  	WriteAD9833(ad9833_freq_LSB); //0101 0000 1100 0111 - Freq0 LSB (4295)
  	FSYNC_1_AD9833_on;
  	FSYNC_2_AD9833_on;

  	SCLK_AD9833_on;
  	FSYNC_1_AD9833_off;
  	FSYNC_2_AD9833_off;
  	WriteAD9833(ad9833_freq_MSB); //0100 0000 0000 0000 - Freq0 MSB (0)
  	FSYNC_1_AD9833_on;
  	FSYNC_2_AD9833_on;

  	SCLK_AD9833_on;
  	FSYNC_1_AD9833_off;
  	WriteAD9833(0xC000+0); //1100 0000 0000 0000 - Phase0 (0) ФАЗА DDS_1
  	FSYNC_1_AD9833_on;


  	//
  	SCLK_AD9833_on;
  	FSYNC_2_AD9833_off;
  	WriteAD9833(0xC000+0); //1100 0000 0000 0000 - Phase0 (90) ФАЗА DDS_2
  	FSYNC_2_AD9833_on;


  	SCLK_AD9833_on;
    FSYNC_1_AD9833_off;
    FSYNC_2_AD9833_off;
  	WriteAD9833(0x2000); //0010 0000 0000 0000 - Синусоидальный сигнал
  	FSYNC_1_AD9833_on;
  	FSYNC_2_AD9833_on;




	SCLK_AD9833_on;
	FSYNC_1_AD9833_off;
	FSYNC_2_AD9833_off;
	WriteAD9833(0x2100); //0010 0001 0000 0000 - Reset + DB28
	FSYNC_1_AD9833_on;
	FSYNC_2_AD9833_on;

  	SCLK_AD9833_on;
  	FSYNC_1_AD9833_off;
  	FSYNC_2_AD9833_off;
  	WriteAD9833(0x2000); //0010 0000 0000 0000 - Exit Reset
  	FSYNC_1_AD9833_on;
  	FSYNC_2_AD9833_on;





  	//delay_us(10);
/*
  	SCLK_AD9833_on;
	FSYNC_1_AD9833_off;
	FSYNC_2_AD9833_off;
	WriteAD9833(0x2180); //0010 0001 0001 0000 - Reset + DB28 + OPBITEN
	FSYNC_1_AD9833_on;
	FSYNC_2_AD9833_on;

  	SCLK_AD9833_on;
	FSYNC_1_AD9833_off;
	FSYNC_2_AD9833_off;
	WriteAD9833(0x2110); //0010 0001 0001 0000 - Reset + DB28 + OPBITEN
	FSYNC_1_AD9833_on;
	FSYNC_2_AD9833_on;
*/

  	swr_sdadc_data_vphs=0;

/*
  while(((swr_sdadc_data_vphs>31700)&(swr_sdadc_data_vphs<32000))==0){
    	SCLK_AD9833_on;
    	FSYNC_2_AD9833_off;
    	WriteAD9833(0xC000+phase); //1100 0000 0000 0000 - Phase0 (90) ФАЗА DDS_2
    	FSYNC_2_AD9833_on;
    	phase++;

    	if(phase==4096){phase=0;};
    	delay_ms(1);

		SDADC3->CR2 |= SDADC_CR2_RSWSTART; //Software start of a conversion on the regular channel
  };




	USART2->CR1 |=(USART_CR1_RE | USART_CR1_TE); // Разрешить выводы RX, TX

	USART2->CR1 |= USART_CR1_UE; //Включаем USART2
	USART2->CR1 &= ~USART_CR1_M; //Данные - 8 бит
	USART2->CR2 &= ~USART_CR2_STOP; //1 стоп-бит
	USART2->BRR =(APBCLK+BAUDRATE/2)/BAUDRATE; //скорость usart
	USART2->CR1 |= USART_CR1_TE; //Включаем передатчик USART2
	USART2->CR1 |= USART_CR1_RE; //Включаем приемник USART2

	USART2->CR1 |= USART_CR1_RXNEIE; //Прерывание по приему байта
	NVIC_EnableIRQ (USART2_IRQn);

	for(i=0;i<1000;i++){
		buzer_on;
		delay_us(200);
		buzer_off;
		delay_us(200);
	};

	*/

  	//SDADC3->CR2 |= SDADC_CR2_RSWSTART;


	while(1){


		//if(data_TX==1){
			//data_TX=0;


			//while(sdadc_swr_vphs_finish==0){SDADC3->CR2 |= SDADC_CR2_RSWSTART;};
			//sdadc_swr_vphs_finish=0;

			//while(sdadc_swr_vmag_finish==0){SDADC2->CR2 |= SDADC_CR2_RSWSTART;};
			//sdadc_swr_vmag_finish=0;

/*
			uart_print("SDADC3_ch8_VPHS: ");
			itoa(swr_sdadc_data_vphs, buf);
			uart_print(buf);
			uart_tx_data(0x0d); // перенос строки

			uart_print("SDADC2_ch8_VMAG: ");
			itoa(swr_sdadc_data_vmag, buf);
			uart_print(buf);
			uart_tx_data(0x0d); // перенос строки


			vphs_swr_sdadc=swr_sdadc_data_vphs;
			vmag_swr_sdadc=swr_sdadc_data_vmag;

			uart_tx_data(vphs_swr_sdadc>>8);
			uart_tx_data(vphs_swr_sdadc);

			uart_tx_data(vmag_swr_sdadc>>8);
			uart_tx_data(vmag_swr_sdadc);
*/


		//};
		//GPIOB->ODR |= GPIO_ODR_2;
		//delay_ms(50);

		//GPIOB->ODR &=~ GPIO_ODR_2;
		//delay_ms(50);





/*

		delay_ms(200);
		SDADC3->CR2 |= SDADC_CR2_RSWSTART; //Software start of a conversion on the regular channel
		SDADC2->CR2 |= SDADC_CR2_RSWSTART; //Software start of a conversion on the regular channel

		if(sdadc_swr_vphs_finish==1){
			sdadc_swr_vphs_finish=0;

			uart_print("SDADC3_ch8_VPHS: ");
			itoa(swr_sdadc_data_vphs, buf);
			uart_print(buf);
			uart_tx_data(0x0d); // перенос строки

			XmV_vphs=(swr_sdadc_data_vphs*1765)/65535;
			uart_print("VPHS_mV: ");
			itoa(XmV_vphs, buf);
			uart_print(buf);
			uart_tx_data(0x0d); // перенос строки
			uart_tx_data(0x0d); // перенос строки


		};

		if(sdadc_swr_vmag_finish==1){
			sdadc_swr_vmag_finish=0;

			uart_print("SDADC2_ch8_VMAG: ");
			itoa(swr_sdadc_data_vmag, buf);
			uart_print(buf);
			uart_tx_data(0x0d); // перенос строки

			XmV_vmag=(swr_sdadc_data_vmag*1765)/65535;
			uart_print("VMAG_mV: ");
			itoa(XmV_vmag, buf);
			uart_print(buf);
			uart_tx_data(0x0d); // перенос строки
			uart_tx_data(0x0d); // перенос строки


		};

*/


		/*

		delay_ms(100);

		temp=SDADC3->RDATAR;
		if((temp>=0)&(temp<=32767)){temp=temp+32768;}else{temp=temp-32768;};


		itoa(temp, buf);
		uart_print(buf);
		uart_tx_data(0x0d); //

		*/
	};
}
