#include "stm32f3xx.h"
#include "main.h"
#include "bsp.h"
#include "delay.h"
#include "math.h"

#define PI 3.14

static uint8_t SystemClock_Config(void);

int angle(int);
int map(int, int, int, int, int);

const uint16_t mot1[19] = { 60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78};
const uint16_t mot2[19] = { 60,62,64,67,70,73,76,79,81,84,87,90,93,97,100,104,109,114,120};


int main()
{
	// Configure System Clock (64MHz/72MHz depending on HSI/HSE selection)
	SystemClock_Config();
	int value;
	servo_init();

	// Initialize Debug Console
	BSP_Console_Init();
	my_printf("\r\nConsole Ready!\r\n");
	my_printf("SYSCLK = %d Hz\r\n", SystemCoreClock);

	// Loop forever
	while(1)
	{
		my_printf("\r\n BASCULE \r\n");
		for(float i = 0; i < 18.5; i=i+0.5)
		{
			value = 60+i+52-acos((sin(i*M_PI/180)*50+25)/41)*180/M_PI;

			my_printf("Valeur moteur de poussée = %d\r\n", value);

			TIM3->CCR1 = angle(value);
			TIM3->CCR2 = angle(60+i);

			delay_ms(10);

		}
		delay_ms(2000);
		my_printf("\r\n DESCENTE \r\n");
		for(float i = 18; i >= 0; i=i-0.5)
		{
			value = 60+i+52-acos((sin(i*M_PI/180)*50+25)/41)*180/M_PI;

			my_printf("Valeur moteur de poussée = %d\r\n", value);

			TIM3->CCR1 = angle(value);
			TIM3->CCR2 = angle(60+i);

			delay_ms(10);

		}

		delay_ms(2000);

		/* Valeur pré-enregistré
		for(int i = 0; i < 19; i++)
		{

			TIM3->CCR1 = angle(mot2[i]);
			TIM3->CCR2 = angle(mot1[i]);
			delay_ms(50);

		}
		*/
	}
}
int  angle(int angle_deg){

	return map(angle_deg,0,120,1000,2000);
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 * 	Clock configuration for the Nucleo STM32F303K8 board
 *
 * 	Default solder bridges configuration is for HSI (HSE is not connected)
 * 	You can change this by swapping solder bridges SB4 and SB6 (see Nucleo User manual)
 *
 * 	HSI clock configuration provides :
 * 	- SYSCLK, AHB	-> 64MHz
 * 	- APB2			-> 64MHz
 * 	- APB1			-> 32MHz (periph) 64MHz (timers)
 *
 * 	HSE clock configuration from ST-Link 8MHz MCO provides :
 * 	- SYSCLK, AHB	-> 72MHz
 * 	- APB2			-> 72MHz
 * 	- APB1			-> 36MHz (periph) 72MHz (timers)
 *
 * 	Select configuration by setting one of these symbol in your build configuration
 * 	- USE_HSI
 * 	- USE_HSE
 *
 */


#ifdef USE_HSI

static uint8_t SystemClock_Config()
{
	uint32_t	status;
	uint32_t	timeout = 0;

	// Start 8MHz HSI (it should be already started at power on)
	RCC->CR |= RCC_CR_HSION;

	// Wait until HSI is stable
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_HSIRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (1);	// HSI error


	// Configure Flash latency according to the speed (2WS)
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= 0x02 <<0;

	// Set HSI as PLL input
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;	// 4MHz from HSI

	// Configure the main PLL
	#define PLL_MUL		16					// 4MHz HSI to 64MHz
	RCC->CFGR |= (PLL_MUL-2) <<18;

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// Configure AHB/APB prescalers
	// AHB  Prescaler = /1	-> 64 MHz
	// APB1 Prescaler = /2  -> 32/64 MHz
	// APB2 Prescaler = /1  -> 64 MHz
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2);	// PLL error


	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL is switched on
	timeout = 1000;

	do
	{
		status = RCC->CFGR & RCC_CFGR_SWS;
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (3);	// SW error

	// Update System core clock
	SystemCoreClockUpdate();
	return (0);
}

#endif


#ifdef USE_HSE

static uint8_t SystemClock_Config()
{
	uint32_t	status;
	uint32_t	timeout;

	// Start HSE
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// Wait until HSE is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_HSERDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (1); 	// HSE error


	// Configure Flash latency according to the speed (2WS)
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= 0x02 <<0;

	// Configure the main PLL
	#define PLL_MUL		9	// 8MHz HSE to 72MHz
	RCC->CFGR |= (PLL_MUL-2) <<18;

	// Set HSE as PLL input
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV;

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// Configure AHB/APB prescalers
	// AHB  Prescaler = /1	-> 72 MHz
	// APB1 Prescaler = /2  -> 36/72 MHz
	// APB2 Prescaler = /1  -> 72 MHz
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2); 	// PLL error


	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL is switched on
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY;
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (3); 	// SW error


	// Update System core clock
	SystemCoreClockUpdate();
	return (0);
}

#endif
