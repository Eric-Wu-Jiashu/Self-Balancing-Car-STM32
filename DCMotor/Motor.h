#ifndef __MOTOR_H__
#define __MOTOR_H_

#include "stm32f1xx.h"
#include "tim.h"
#include "gpio.h"

#define mkp 200 //Popution Modulier for Speed PID ring
#define mki 10 //Intergreation Mutplier for Speed PID ring
#define DeadZone 200 //PWM dead zone, PWM set to zero between -DeadZong and DeadZong


typedef struct{
	int Speed; //Total encoder count in 10ms, read from TIM counter registor
	int Prev_Speed;//previus Speed
	int Target;//Trget Speed
	int Error_Total;//Entergreation of Error
	int PWM_Out;//PWM pulse
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Positive;
	uint16_t GPIO_Negative;
	uint32_t* PWM_Regester;//CCR registor for current Motor PWM output

}Motor_PID;

extern Motor_PID Motor_Left, Motor_Right;

void Motor_Init();

void Motor_Drive(Motor_PID* Motor);

void Motor_PID_PWM(Motor_PID* Motor);

void Motor_Set_Speed(Motor_PID* Motor, int Speed_Target);

void Read_Motor_Speed();

#endif
