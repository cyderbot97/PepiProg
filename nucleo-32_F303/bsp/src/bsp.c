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

	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN; // enable timer 2 clock
	RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN; // enable timer 1 clock

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	// enable gpioA clock

	//Alternate function for PA0 PA1 PA2 PA3
	GPIOA->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk | GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk| GPIO_MODER_MODER11_Msk);
	GPIOA->MODER |= (0x02<<GPIO_MODER_MODER0_Pos) | (0x02<<GPIO_MODER_MODER1_Pos)| (0x02<<GPIO_MODER_MODER2_Pos)| (0x02<<GPIO_MODER_MODER3_Pos)| (0x02<<GPIO_MODER_MODER8_Pos)| (0x02<<GPIO_MODER_MODER9_Pos)| (0x02<<GPIO_MODER_MODER10_Pos)| (0x02<<GPIO_MODER_MODER11_Pos);

	//Set alternate function AF2 for pin 0/1/2/3 and function AF5 for pin 6/7
	GPIOA->AFR[0] &= ~(0x0000FFFF);
	GPIOA->AFR[0] |=  (0x00001111);

	GPIOA->AFR[1] &= ~(0x0000FFFF);
	GPIOA->AFR[1] |=  (0x0000B666);

	// reset tim2 configuration
	TIM2->CR1  = 0x0000;
	TIM2->CR2  = 0x0000;
	TIM2->CCER = 0x0000;

	TIM1->CR1  = 0x0000;
	TIM1->CR2  = 0x0000;
	TIM1->CCER = 0x0000;

	//set TIM2 prescaler
	TIM2->PSC =  (uint16_t) 64-1;
	TIM1->PSC =  (uint16_t) 64-1;

	//Set auto reload

	TIM2->ARR = (uint16_t) 11000;
	TIM1->ARR = (uint16_t) 11000;

	// Enable Auto-Reload Preload register
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM1->CR1 |= TIM_CR1_ARPE;

	// Setup Input Capture
	TIM2->CCMR1 = 0x0000;
	TIM2->CCMR2 = 0x0000;

	TIM1->CCMR1 = 0x0000;
	TIM1->CCMR2 = 0x0000;

	// Setup PWM mode 1 output ch1,ch2,ch3,ch4
	TIM2->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
	TIM2->CCMR1 |= (0x06 <<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM2->CCMR2 |= (0x06 <<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	TIM2->CCMR2 |= (0x06 <<TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

	TIM1->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
	TIM1->CCMR1 |= (0x06 <<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM1->CCMR2 |= (0x06 <<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	TIM1->CCMR2 |= (0x06 <<TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;


	// Set default PWM values
	TIM2->CCR1 = 1000;
	TIM2->CCR2 = 1250;
	TIM2->CCR3 = 1500;
	TIM2->CCR4 = 2000;

	TIM1->CCR1 = 1000;
	TIM1->CCR2 = 1250;
	TIM1->CCR3 = 1500;
	TIM1->CCR4 = 2000;

	// Enable Outputs
	TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	// Enable Main output
	TIM1->BDTR |= TIM_BDTR_MOE;

	// Enable TIM2
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM1->CR1 |= TIM_CR1_CEN;

}


