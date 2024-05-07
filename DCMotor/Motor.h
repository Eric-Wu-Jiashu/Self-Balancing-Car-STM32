#ifndef __MOTOR_H__
#define __MOTOR_H_

#include "stm32f1xx.h"
#include "tim.h"
#include "gpio.h"

#define mkp 200
#define mki 10
#define DeadZone 200

#define Speed_Filter_Ratio 1


typedef struct{
	int Speed;
	int Prev_Speed;
	int Target;
	int Error_Total;
	int PWM_Out;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Positive;
	uint16_t GPIO_Negative;
	uint32_t* PWM_Regester;

}Motor_PID;

extern Motor_PID Motor_Left, Motor_Right;

void Motor_Init();

void Motor_Drive(Motor_PID* Motor);

void Motor_PID_PWM(Motor_PID* Motor);

void Motor_Set_Speed(Motor_PID* Motor, int Speed_Target);

void Read_Motor_Speed();

#endif
