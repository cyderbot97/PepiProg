#include "stm32f3xx.h"
#include "main.h"
#include "bsp.h"
#include "delay.h"
#include "math.h"
#include "i2c.h"

#define PI 3.14

void i2c_init(void);
static uint8_t SystemClock_Config(void);

int angle(int);
int map(int, int, int, int, int);
#define 	WHO_AM_I     0x0F

#define     CTRL_REG1    0x20
#define     CTRL_REG2    0x21
#define     CTRL_REG3    0x22
#define     CTRL_REG4    0x23
#define     CTRL_REG5    0x24

#define     STATUS_REG   0x27
#define     OUT_X_L      0x28
#define     OUT_X_H      0x29
#define     OUT_Y_L      0x2A
#define     OUT_Y_H      0x2B
#define     OUT_Z_L      0x2C
#define     OUT_Z_H      0x2D
#define     TEMP_OUT_L   0x2E
#define     TEMP_OUT_H   0x2F
#define     INT_CFG      0x30
#define     INT_SRC      0x31
#define     INT_THS_L    0x32
#define     INT_THS_H    0x33

int main(void)
{
	uint8_t		rx_data,tx_data;
	uint8_t x[2],y[2],z[2];
	// Configure System Clock for 48MHz from 8MHz HSE
	SystemClock_Config();


	// Initialize Debug Console
	BSP_Console_Init();
	my_printf("\r\nConsole Ready!\r\n");
	my_printf("SYSCLK = %d Hz\r\n", SystemCoreClock);

	// Initialize I2C1 peripheral
	BSP_I2C1_Init();

	// Start I2C transaction
	//I2C1->CR2 |= I2C_CR2_START;  // <-- Breakpoint here

	// BSP_I2C1_Read(0x1E, 0x0F, rx_data, 2); // who am i

	tx_data = 0x70;
	BSP_I2C1_Write(0x1E, CTRL_REG1, &tx_data, 2);
	delay_ms(50);

	tx_data = 0x00;
	BSP_I2C1_Write(0x1E, CTRL_REG1, &tx_data, 2);
	delay_ms(50);

	tx_data = 0x00;
	BSP_I2C1_Write(0x1E, CTRL_REG3, &tx_data, 2);
	delay_ms(50);

	tx_data = 0x0C;
	BSP_I2C1_Write(0x1E, CTRL_REG4, &tx_data, 2);
	delay_ms(50);

	while(1)
	{
		BSP_I2C1_Read(0x1E, STATUS_REG, &rx_data, 2);
		//my_printf("Status_reg = 0x%02x\r\n",rx_data);

		if((rx_data & 0x08)==0x08){
			BSP_I2C1_Read(0x1E, OUT_X_L, &x[0], 2);
			BSP_I2C1_Read(0x1E, OUT_X_H, &x[1], 2);
			my_printf("XH = %d\r\n",(uint16_t)(x[1]<<8U | x[0]));

			BSP_I2C1_Read(0x1E, OUT_Y_L, &y[0], 2);
			BSP_I2C1_Read(0x1E, OUT_Y_H, &y[1], 2);
			my_printf("YH = %d\r\n",(uint16_t)(y[1]<<8U | y[0]));

			BSP_I2C1_Read(0x1E, OUT_Z_L, &z[0], 2);
			BSP_I2C1_Read(0x1E, OUT_Z_H, &z[1], 2);
			my_printf("ZH = %d\r\n",(uint16_t)(z[1]<<8U | z[0]));

			x[0] = 0;
			x[1] = 0;

			y[0] = 0;
			y[1] = 0;

			z[0] = 0;
			z[1] = 0;

			delay_ms(100);
		}





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
