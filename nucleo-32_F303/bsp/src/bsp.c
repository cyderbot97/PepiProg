/*
 * bsp.c
 *
 *  Created on: 5 août 2017
 *      Author: Laurent
 */

#include "stm32f3xx.h"
#include "bsp.h"

void BSP_Console_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
	GPIOA->MODER |=  (0x02 <<4U) | (0x02 <<6U);

	// Set PA2 and PA3 to AF7 (USART2)
	GPIOA->AFR[0] &= ~(0x0000FF00);
	GPIOA->AFR[0] |=  (0x00007700);

	// Enable USART2 clock
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART2->CR1 = 0x00000000;
	USART2->CR2 = 0x00000000;
	USART2->CR3 = 0x00000000;

	// Baud Rate = 115200
	USART2->CR1 &= ~USART_CR1_OVER8;
	USART2->BRR = 0x0115;

	// Enable both Transmitter and Receiver
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Enable USART2
	USART2->CR1 |= USART_CR1_UE;
}



void adc_init(void)
{
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;

	//Enable gpioA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	//configure PA0 as analog
	GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;
	GPIOA->MODER |= (0x03 << GPIO_MODER_MODER0_Pos);

	//Start calibration
	ADC1->CR &= ~ADC_CR_ADVREGEN;
	ADC1->CR |= ADC_CR_ADVREGEN_0;

	for(int i=0; i<4200; i++);

	ADC1->CR &= ~ADC_CR_ADCALDIF; // calibration in Single-ended inputs Mode.
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);

	//ADC configuration
	ADC1->CFGR |= ADC_CFGR_CONT;
	//ADC1->CFGR &= ADC_CFGR_RES; //disable ADC_CFGR_CONT ?

	ADC1->SQR1 |= ADC_SQR1_SQ1_0;
	ADC1->SQR1 &= ~ADC_SQR1_L;

	ADC1->SMPR1 |= ADC_SMPR1_SMP7_1 | ADC_SMPR1_SMP7_0;

	ADC1->CR |= ADC_CR_ADEN; // Enable ADC1
	while(!ADC1->ISR & ADC_ISR_ADRD); // wait for ADRDY
 	ADC1->CR |= ADC_CR_ADSTART;

}

void servo_init(void)
{
	//Enable TIMER clock
	RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//Enable gpioA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	//Enable gpioB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	//
	GPIOA->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk| GPIO_MODER_MODER11_Msk );
	GPIOA->MODER |= (0x02<<GPIO_MODER_MODER8_Pos)| (0x02<<GPIO_MODER_MODER9_Pos)| (0x02<<GPIO_MODER_MODER10_Pos)| (0x02<<GPIO_MODER_MODER11_Pos);

	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER4_Msk| GPIO_MODER_MODER5_Msk );
	GPIOB->MODER |= (0x02<<GPIO_MODER_MODER0_Pos)| (0x02<<GPIO_MODER_MODER1_Pos)| (0x02<<GPIO_MODER_MODER4_Pos)| (0x02<<GPIO_MODER_MODER5_Pos);

	//Set alternate function
	GPIOA->AFR[1] &= ~(0x0000FFFF);
	GPIOA->AFR[1] |=  (0x0000B666);

	GPIOB->AFR[0] &= ~(0x00FF00FF);
	GPIOB->AFR[0] |=  (0x00220022);

	// reset tim configuration
	TIM1->CR1  = 0x0000;
	TIM1->CR2  = 0x0000;
	TIM1->CCER = 0x0000;

	TIM3->CR1  = 0x0000;
	TIM3->CR2  = 0x0000;
	TIM3->CCER = 0x0000;

	//set TIM prescaler
	TIM1->PSC =  (uint16_t) 64-1;
	TIM3->PSC =  (uint16_t) 64-1;

	//Set auto reload
	TIM1->ARR = (uint16_t) 11000;
	TIM3->ARR = (uint16_t) 11000;

	// Enable Auto-Reload Preload register
	TIM1->CR1 |= TIM_CR1_ARPE;
	TIM3->CR1 |= TIM_CR1_ARPE;

	// Setup Input Capture
	TIM1->CCMR1 = 0x0000;
	TIM1->CCMR2 = 0x0000;

	TIM3->CCMR1 = 0x0000;
	TIM3->CCMR2 = 0x0000;

	// Setup PWM mode 1 output ch1,ch2,ch3,ch4
	TIM1->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
	TIM1->CCMR1 |= (0x06 <<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM1->CCMR2 |= (0x06 <<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	TIM1->CCMR2 |= (0x06 <<TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

	TIM3->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
	TIM3->CCMR1 |= (0x06 <<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM3->CCMR2 |= (0x06 <<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	TIM3->CCMR2 |= (0x06 <<TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;


	// Set default PWM values
	TIM1->CCR1 = 1500;
	TIM1->CCR2 = 1500;
	TIM1->CCR3 = 1500;
	TIM1->CCR4 = 1500;

	TIM3->CCR1 = 1500;
	TIM3->CCR2 = 1500;
	TIM3->CCR3 = 1500;
	TIM3->CCR4 = 1500;

	// Enable Outputs
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	// Enable Main output
	TIM1->BDTR |= TIM_BDTR_MOE;
	//TIM3->BDTR |= TIM_BDTR_MOE;

	// Enable TIM2
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
}


