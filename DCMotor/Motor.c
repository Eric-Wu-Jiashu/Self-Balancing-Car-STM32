#include "Motor.h"



Motor_PID Motor_Left, Motor_Right;

void Motor_Init(){


	//Motor initlize

	Motor_Left.GPIOx = GPIOB; // GPIO_Group of GPIO_Posititive and GPIO_Negative pin
	Motor_Left.GPIO_Positive = GPIO_PIN_0; // when GPIO_Positive sets high and GPIO_Negative sets low, encoder reading will be positive. 
	Motor_Left.GPIO_Negative = GPIO_PIN_1;
	Motor_Left.PWM_Regester = &(TIM1->CCR1);
	Motor_Left.Prev_Speed = 0;

	Motor_Right.GPIOx = GPIOB;
	Motor_Right.GPIO_Positive = GPIO_PIN_10;
	Motor_Right.GPIO_Negative = GPIO_PIN_11;
	Motor_Right.PWM_Regester = &(TIM1->CCR2);
	Motor_Right.Prev_Speed = 0;

}


//SET PWM duty cycle
//PWM set to zero and direction remain unchanged when output is between -DeadZong to DeadZong
void Motor_Drive(Motor_PID* Motor){
	if (Motor->PWM_Out < -DeadZone){
		//Set Motor rotation direction
		HAL_GPIO_WritePin(Motor->GPIOx, Motor->GPIO_Positive, 0);
		HAL_GPIO_WritePin(Motor->GPIOx, Motor->GPIO_Negative, 1);
		//Overflow Protection
		if (-7199 > Motor->PWM_Out){Motor->PWM_Out = -7199;}
		//Update PWM duty cycle
		*(Motor->PWM_Regester)= Motor->PWM_Out * (-1);
	}
	else if(Motor->PWM_Out > DeadZone){

		HAL_GPIO_WritePin(Motor->GPIOx, Motor->GPIO_Positive, 1);
		HAL_GPIO_WritePin(Motor->GPIOx, Motor->GPIO_Negative, 0);

		if (7199 < Motor->PWM_Out){Motor->PWM_Out = 7199;}

		*(Motor->PWM_Regester) = Motor->PWM_Out;
	}
	else{
		//Output Within PWM DeadZong
		//Speed set to zero and direction remain unchanged
		//Prevnt vibration
		*(Motor->PWM_Regester) = 0;
	}
}

//Motor PID Controller
//Conver target speed in TIM counter to PWM duty cycle
void Motor_PID_PWM(Motor_PID* Motor){
	int16_t Error = Motor->Target - Motor->Speed;

	Motor->Error_Total += Error;

	Motor->PWM_Out = mkp * Error + mki * Motor->Error_Total;

}

void Motor_Set_Speed(Motor_PID* Motor, int Speed_Target){
	
	//Over flow protection, pass veriable to following functions
	if (70 < Speed_Target){
		Motor->Target = 70;
	}
	else if(-70 > Speed_Target){
		Motor->Target = -70;
	}
	else{
		Motor->Target = Speed_Target;
	}
	Motor_PID_PWM(Motor);
	Motor_Drive(Motor);
}


//Read encoder counter for each motor, set counter to zero when finished
void Read_Motor_Speed(){
	Motor_Left.Speed = (int16_t) __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);

	Motor_Right.Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3,0);
}

//Read Speed every 10 ms
void  PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if (htim == &htim4){
		Read_Motor_Speed;
	}
}









