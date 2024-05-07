#include "MPU6050.h"

#include "math.h"

#define MPU6050 0x68 << 1

#define Who_Am_I 0x75
#define Power_Manage 0x6b

#define ACC_Config 0x1c
#define GYRO_Config 0x1b

#define ACC_Start 0x3b
#define GYRO_Start 0x43

#define GYRO_Factor 65.5
#define ACC_Factor 16384.0

#define ACC_Size 6
#define Gyro_Size 4

#define PI 3.14


uint16_t GyroX_Offset = -51;
uint16_t GyroZ_Offset = -251;


uint8_t Acc_Read[ACC_Size], Gyro_Read[Gyro_Size], Self_Check;

int16_t GyroX, GyroZ;

float AccAngle, GyroAngle[2], gyroX, Kalman_Angle, Delta_Time, Kalman_Gain[2], Angular_Velocity_X;

float P[2][2] = {{1, 0.0},{0.0, 0.2}};
float Q[2][2] = {{0.0, 0.0},{0.0, 1}};

float angle = 0;


uint32_t prev_systick = 0;






void MPU6050_Init(){

	while(Self_Check == 0){
		HAL_Delay(50);
		HAL_I2C_Mem_Read(&hi2c1, MPU6050, Who_Am_I, 1, &Self_Check, 1, HAL_MAX_DELAY);
	}
	HAL_I2C_Mem_Write(&hi2c1, MPU6050, Power_Manage, 1, (uint8_t*)0x00, 1, HAL_MAX_DELAY);
	uint8_t write[3] = {0,0x08,0};
	HAL_I2C_Mem_Write(&hi2c1, MPU6050, 0x1a, 1, write, 3, HAL_MAX_DELAY);
	//HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050, ACC_Start, 1, &acc_read, 2);
	prev_systick = HAL_GetTick();


}


void MPU6050_Get_Data(float *E_Angle, float *Angular_V){
	HAL_I2C_Mem_Read(&hi2c1, MPU6050, ACC_Start, 1, Acc_Read, ACC_Size, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MPU6050, GYRO_Start, 1, Gyro_Read, Gyro_Size, HAL_MAX_DELAY);

	Delta_Time = (HAL_GetTick() - prev_systick) / 1000.0;
	prev_systick = HAL_GetTick();

	float total = 0;
	for (int i = 0; i < (ACC_Size/2); i ++){
		total += (int16_t)(Acc_Read[2*i] << 8 | Acc_Read[2*i+1]) * (int16_t)(Acc_Read[2*i] << 8 | Acc_Read[2*i+1]);
	}
	total = sqrt(total);

	AccAngle = - asin((int16_t)(Acc_Read[4] << 8 | Acc_Read[5])/total) * 180 / PI;

	GyroZ = (Gyro_Read[0] << 8 | Gyro_Read[1]) - GyroZ_Offset;
	GyroX = (Gyro_Read[2] << 8 | Gyro_Read[3]) - GyroX_Offset;



	GyroAngle[0] = Kalman_Angle + (GyroX * Delta_Time) / GYRO_Factor;


	P[0][0] = P[0][0] + (P[0][1] + P[1][0] + P[1][1] * Delta_Time) * Delta_Time + Q[0][0];
	P[0][1] = P[0][1] + P[1][1] * Delta_Time;
	P[1][0] = P[1][0] + P[1][1] * Delta_Time;
	P[1][1] = P[1][1] + Q[1][1];

	Kalman_Gain[0] = P[0][0] / (P[0][0] + R);
	Kalman_Gain[1] = P[1][0] / (P[0][0] + R);

	Angular_Velocity_X = Kalman_Angle;

	Kalman_Angle = GyroAngle[0] + Kalman_Gain[0] * (AccAngle - GyroAngle[0]);

	Angular_Velocity_X = Kalman_Angle - Angular_Velocity_X;

	float P_New[2][2];
	P_New[0][0] = P[0][0]*(1-Kalman_Gain[0]) * (1-Kalman_Gain[0]) + R * Kalman_Gain[0] * Kalman_Gain[0];
	P_New[0][1] = (P[0][1] - Kalman_Gain[1] * P[0][0]) * (1 - Kalman_Gain[0]) + R * Kalman_Gain[0] * Kalman_Gain[1];
	P_New[1][0] = (P[1][0] - P[0][0] * Kalman_Gain[1]) * (1 - Kalman_Gain[0]) + R * Kalman_Gain[0] * Kalman_Gain[1];
	P_New[1][1] = - Kalman_Gain[1] * (P[1][0] - P[0][0] * Kalman_Gain[1]) - P[0][1] * Kalman_Gain[1] + P[1][1] + R * Kalman_Gain[1] * Kalman_Gain[1];

	P[0][0] = P_New[0][0];
	P[0][1] = P_New[0][1];
	P[1][0] = P_New[1][0];
	P[1][1] = P_New[1][1];

	*E_Angle = Kalman_Angle;
	*Angular_V = Angular_Velocity_X;

}

