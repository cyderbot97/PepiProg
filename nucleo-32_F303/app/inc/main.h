/*
 * main.h
 *
 *  Created on: 24 ao�t 2018
 *      Author: Laurent
 */

#ifndef APP_INC_MAIN_H_
#define APP_INC_MAIN_H_


#include "stm32f3xx.h"
#include "bsp.h"
#include "delay.h"
#include "math.h"
#include "i2c.h"
#include "mpu6050.h"


/* Global functions */

int my_printf	(const char *format, ...);
int my_sprintf	(char *out, const char *format, ...);


#endif /* APP_INC_MAIN_H_ */
