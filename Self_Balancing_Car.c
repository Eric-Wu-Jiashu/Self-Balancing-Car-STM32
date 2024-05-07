#include "Self_Balancing_Car.h"


#define receive_length 4
#define PID_Filter 0.8


float Euler_Angle, Angular_Velocity, Angle_Error_Total;

int Target1, Prev_Target;

uint8_t UART_Recive[receive_length];

int number, amount;

float skp = 1.3;
float ski = 0.22;
float skd = -5.2;

uint32_t system_tick, prev_system_tick;


void Car_Init(){
	MPU6050_Init();
	Motor_Init();

}

void Car_Test(){

	Car_PID();

	Motor_Set_Speed(&Motor_Left, Target1);
	Motor_Set_Speed(&Motor_Right, Target1);

	HAL_Delay(2);
}

void Car_Main(){
	//MPU6050_Get_Data(&Euler_Angle, &Angular_Velocity);
}

void Car_PID(){
	MPU6050_Get_Data(&Euler_Angle, &Angular_Velocity);

	if (Euler_Angle < -45 || Euler_Angle > 45){
		Target1 = 0;
		Angle_Error_Total = 0;
		return;
	}

	float Angle_Error = balance_point - Euler_Angle;

	Angle_Error_Total = Angle_Error_Total + Angle_Error;

	if ( Angle_Error_Total < -250 ) Angle_Error_Total = -250;

	if (Angle_Error_Total > 250) Angle_Error_Total = 250;

	//if (Angle_Error == 0){Angle_Error_Total = Angle_Error_Total * (0.1);}

	Target1 = Angle_Error * skp + Angle_Error_Total * ski + Angular_Velocity * skd;

	Target1 = PID_Filter * Target1 + (1-PID_Filter) * Prev_Target;
	Prev_Target = Target1;
}

