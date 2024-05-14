#ifndef _MPU6050_H
#define _MPU6050_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"

#define R 2.5 	//Measurement Covariance

void MPU6050_Init();

void MPU6050_Get_Data(float *E_Angle, float *Angular_V);


#endif
