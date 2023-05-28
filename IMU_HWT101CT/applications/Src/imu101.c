//=====================================================================================================
// imu605.c
//=====================================================================================================
// See: https://github.com/tungchiahui
//
// Date : 28/05/2023		             
//
// Organization : 机电创新学会 Vinci机器人队
//
// Author : Tung Chia-hui
//
// Notes : Initial release
//
//=====================================================================================================

#include "imu101.h"

IMU_t imu;

bool_t imu_flag;

uint8_t rx_buffer[1];

uint8_t imu_buffer[11];

uint8_t unlocking[5] = {0xff, 0xaa, 0x69, 0x88, 0xb5};
uint8_t make_zero[5] = {0xff, 0xaa, 0x76, 0x00, 0x00};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		IMU_Read_Data(rx_buffer);
	}
}

void IMU_Read_Data(uint8_t *rx_buffer)
{
	static int16_t imu_cnt = 0;
	imu_buffer[imu_cnt] = *rx_buffer;
	if(imu_buffer[0] == 0x55)
	{
		imu_cnt++;
		if(imu_flag == 1)
		{
		if(imu_buffer[1] == Gyro_Type)
		{
			imu.Gyro.Y = (int16_t)((int16_t)imu_buffer[5]<<8|imu_buffer[4]);
			imu.Gyro.Z = (int16_t)((int16_t)imu_buffer[7]<<8|imu_buffer[6]);
			
			imu.Gyro.Y = imu.Gyro.Y / 32768.0f * 2000.0f;
			imu.Gyro.Z = imu.Gyro.Z / 32768.0f * 2000.0f;
		}
		else if(imu_buffer[1] == Euler_Type)
		{
			imu.Euler.yaw = (int16_t)((int16_t)imu_buffer[7]<<8|imu_buffer[6]);
			
			imu.Euler.yaw = imu.Euler.yaw / 32768.0f * 180.0f;
		}
		imu_flag = 0;
		for(int i = 0;i < 11;i++)
		{
			imu_buffer[i] = 0x00;
		}
	}
	}
	else
	{
		IMU_ZERO:
		imu_cnt = 0;
		return;
	}
	if(imu_cnt >= 11)
	{
		imu_flag = 1;
		goto IMU_ZERO;
	}
	
}


