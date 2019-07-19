/*
 * bsp.h
 *
 *  Created on: 5 ao�t 2017
 *      Author: Laurent
 */

#ifndef BSP_INC_BSP_H_
#define BSP_INC_BSP_H_

#include "stm32f3xx.h"

/*
 * LED driver functions
 */

void adc_init	(void);
void servo_init (void);


/*
 * Debug Console driver functions
 */

void	BSP_Console_Init	(void);



#endif /* BSP_INC_BSP_H_ */
