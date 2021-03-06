/*
 * bsp.c
 *
 *  Created on: 5 ao�t 2017
 *      Author: Laurent
 */

#include "stm32f3xx.h"
#include "bsp.h"


void BSP_NVIC_Init(){

	NVIC_SetPriority(USART1_IRQn, 1);
	NVIC_EnableIRQ(USART1_IRQn);

	NVIC_SetPriority(TIM6_DAC_IRQn, 2);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

}
void BSP_TIMER_Timebase_Init()
{
	// Enable TIM6 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Reset TIM6 configuration
	TIM6->CR1 = 0x0000;
	TIM6->CR2 = 0x0000;

	// Set TIM6 prescaler
	// Fck = 64MHz -> /64 = 1MHz counting frequency
	TIM6->PSC = (uint16_t) 64 -1;

	// Set TIM6 auto-reload register for 1.5ms
	TIM6->ARR = (uint16_t) 1500 -1;

	// Enable auto-reload preload
	TIM6->CR1 |= TIM_CR1_ARPE;

	// Enable Interrupt upon Update Event
	TIM6->DIER |= TIM_DIER_UIE;

	// Start TIM6 counter
	TIM6->CR1 |= TIM_CR1_CEN;


	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

	// Reset TIM6 configuration
	TIM7->CR1 = 0x0000;
	TIM7->CR2 = 0x0000;

	// Set TIM6 prescaler
	// Fck = 64MHz -> /64 = 1kHz counting frequency
	TIM7->PSC = (uint16_t) 64000 -1;

	// Set TIM6 auto-reload register for 1s
	TIM7->ARR = (uint16_t) 10000;

	// Enable auto-reload preload
	TIM7->CR1 |= TIM_CR1_ARPE;


	// Start TIM6 counter
	TIM7->CR1 |= TIM_CR1_CEN;
}

extern uint8_t rx_dma_buffer[16];

void uart_init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
	GPIOA->MODER |=  (0x02 << GPIO_MODER_MODER9_Pos) | (0x02 << GPIO_MODER_MODER10_Pos);

	// Set PA9 and PA3 to AF10 (USART1)
	GPIOA->AFR[1] &= ~(0x00000FF0);
	GPIOA->AFR[1] |=  (0x00000770);

	// Enable USART1 clock
	RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;

	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART1->CR1 = 0x00000000;
	USART1->CR2 = 0x00000000;
	USART1->CR3 = 0x00000000;

	RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;

	// Baud Rate = 115200
	USART1->CR1 &= ~USART_CR1_OVER8;
	USART1->BRR = 0x0115;

	// Enable both Transmitter and Receiver
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;


	//Start DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	//Reset DMA1 channel 5 configuration
	DMA1_Channel5->CCR &= 0x00000000;

	//Set direction Peripheral to memory
	DMA1_Channel5->CCR &= ~DMA_CCR_DIR;

	//P�ripheral is USART1 RDR
	DMA1_Channel5->CPAR = (uint32_t)&USART1->RDR;

	//peripheral data size is 8-bit (byte)
	DMA1_Channel5->CCR |= (0x00<<DMA_CCR_PSIZE_Pos);

	//Disable auto-increment Peripheral adress
	DMA1_Channel5->CCR &= ~DMA_CCR_PINC;

	// Memory is rx_dma_buffer
	DMA1_Channel5->CMAR = (uint32_t)rx_dma_buffer;

	// Memory data size is 8-bit (byte)
	DMA1_Channel5->CCR |= (0x00 <<DMA_CCR_MSIZE_Pos);

	// Enable auto-increment Memory address
	DMA1_Channel5->CCR |= DMA_CCR_MINC;

	// Set Memory Buffer size
	DMA1_Channel5->CNDTR = 16;

	// DMA mode is circular
	//DMA1_Channel5->CCR |= DMA_CCR_CIRC;
	DMA1_Channel5->CCR &= ~DMA_CCR_CIRC;

	// Enable DMA1 Channel 5
	DMA1_Channel5->CCR |= DMA_CCR_EN;

	// Enable USART2 DMA Request on RX
	USART1->CR3 |= USART_CR3_DMAR;

	USART1->RTOR = 120;

	// Enable RTO and RTO interrupt
	USART1->CR2 |= USART_CR2_RTOEN;
	USART1->CR1 |= USART_CR1_RTOIE;



	// Enable USART2
	USART1->CR1 |= USART_CR1_UE;
}

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


	//Start DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	//Reset DMA1 channel 6 configuration
	DMA1_Channel6->CCR &= 0x00000000;

	//Set direction Peripheral to memory
	DMA1_Channel6->CCR &= ~DMA_CCR_DIR;

	//P�ripheral is USART2 RDR
	DMA1_Channel6->CPAR = (uint32_t)&USART2->RDR;

	//peripheral data size is 8-bit (byte)
	DMA1_Channel6->CCR |= (0x00<<DMA_CCR_PSIZE_Pos);

	//Disable auto-increment Peripheral adress
	DMA1_Channel6->CCR &= ~DMA_CCR_PINC;

	// Memory is rx_dma_buffer
	DMA1_Channel6->CMAR = (uint32_t)rx_dma_buffer;

	// Memory data size is 8-bit (byte)
	DMA1_Channel6->CCR |= (0x00 <<DMA_CCR_MSIZE_Pos);

		// Enable auto-increment Memory address
	DMA1_Channel6->CCR |= DMA_CCR_MINC;

		// Set Memory Buffer size
	DMA1_Channel6->CNDTR = 8;

	// DMA mode is circular
	DMA1_Channel6->CCR |= DMA_CCR_CIRC;

	// Enable DMA1 Channel 5
	DMA1_Channel6->CCR |= DMA_CCR_EN;

	// Enable USART2 DMA Request on RX
	USART2->CR3 |= USART_CR3_DMAR;

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
	GPIOA->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER11_Msk );
	GPIOA->MODER |= (0x02<<GPIO_MODER_MODER8_Pos)| (0x02<<GPIO_MODER_MODER11_Pos);

	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER4_Msk| GPIO_MODER_MODER5_Msk );
	GPIOB->MODER |= (0x02<<GPIO_MODER_MODER0_Pos)| (0x02<<GPIO_MODER_MODER1_Pos)| (0x02<<GPIO_MODER_MODER4_Pos)| (0x02<<GPIO_MODER_MODER5_Pos);

	//Set alternate function
	GPIOA->AFR[1] &= ~(0x0000F00F);
	GPIOA->AFR[1] |=  (0x0000B006);

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

	// Enable TIM3 and TIM1
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
}

void BSP_LED_Init()
{
	// Enable GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// Configure PB3 as output
	GPIOB->MODER &= ~GPIO_MODER_MODER3;
	GPIOB->MODER |= (0x01 <<6U);

	// Configure PB3 as Push-Pull output
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_3;

	// Configure PB3 as High-Speed Output
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR3_Msk;
	GPIOB->OSPEEDR |= (0x03 <<GPIO_OSPEEDER_OSPEEDR3_Pos);

	// Disable PB3 Pull-up/Pull-down
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR3_Msk;

	// Set Initial State OFF
	GPIOB->BSRR = GPIO_BSRR_BR_3;
}

void BSP_LED_Toggle()
{
	GPIOB->ODR ^= GPIO_ODR_3;
}
