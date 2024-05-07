#ifndef __SELF_BALANCING_CAR_H__
#define __SELF_BALANCING_CAR_H__


#include "MPU6050.h"
#include "Motor.h"

#include "usart.h"
#include "gpio.h"
#include "stm32f1xx.h"
#include "tim.h"


#define balance_point 4.5



void Car_Init();

void Car_Main();

void Car_Test();

#endif
