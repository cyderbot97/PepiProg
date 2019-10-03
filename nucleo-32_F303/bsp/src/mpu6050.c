/*
 * mpu6050.c
 *
 *  Created on: 19 sept. 2019
 *      Author: cyril
 */
#include "mpu6050.h"



void BSP_MPU6050_init(void){

	uint8_t tx_data[1];

	tx_data[0] = 0x08;

	BSP_I2C1_Write(0x68, 0x1b, tx_data, 1); // set gyro = 500�/s
	delay_ms(10);

	tx_data[0] = 0x00;

	BSP_I2C1_Write(0x68, 0x1C, tx_data, 1); // set accel = 2g
	delay_ms(10);

	tx_data[0] = 0x05;
	BSP_I2C1_Write(0x68, 0x1A, tx_data, 1); // set low pass filter to 10hz
	delay_ms(10);

	tx_data[0] = 0x00;
	BSP_I2C1_Write(0x68, 0x6b, tx_data, 1);
	delay_ms(10);
}
