#include "imu605_485.h"

IMU_t imu;

bool_t imu_flag;

uint8_t rx_buffer[1];

uint8_t imu_buffer[31];



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		IMU_Read_Data(rx_buffer);
	}
}




void IMU_Unlock_Instruct(void)
{
	uint8_t unlock_buffer[8] = {0x50,0x06,0x00,0x69,0xB5,0x88,0x22,0xA1};
	for(int i = 0;i < 8;i++)
	{
		HAL_UART_Transmit(&huart1,(unlock_buffer+i),1,HAL_MAX_DELAY);
	}
}

void IMU_Save_Instruct(void)
{
	uint8_t save_buffer[8] = {0x50,0x06,0x00,0x00,0x00,0x00,0x84,0x4B};
	for(int i = 0;i < 8;i++)
	{
		HAL_UART_Transmit(&huart1,(save_buffer+i),1,HAL_MAX_DELAY);
	}
}

void IMU_Request_Data(uint16_t ADDR,uint16_t LEN,uint16_t Data_CRC)
{
	uint8_t Modbus_Addr = 0x50;
	uint8_t Modbus_Read = 0x03;
	
	uint8_t ADDRH,ADDRL;
	uint8_t LENH,LENL;
	uint8_t CRCH,CRCL;
	
	uint8_t request_buffer[8];
	
	ADDRH = ADDR >> 8;
	ADDRL = ADDR;
	
	LENH = LEN >> 8;
	LENL = LEN;
	
	CRCH = Data_CRC >> 8;
	CRCL = Data_CRC;
	
	request_buffer[0] = Modbus_Addr;
	request_buffer[1] = Modbus_Read;
	request_buffer[2] = ADDRH;
	request_buffer[3] = ADDRL;
	request_buffer[4] = LENH;
	request_buffer[5] = LENL;
	request_buffer[6] = CRCH;
	request_buffer[7] = CRCL;
	
	for(int i = 0;i < 8;i++)
	{
		HAL_UART_Transmit(&huart1,(request_buffer+i),1,HAL_MAX_DELAY);
	}
}


void IMU_Read_Data(uint8_t *rx_buffer)
{
	static int16_t imu_cnt = 0;
	imu_buffer[imu_cnt] = *rx_buffer;
	if(imu_buffer[0] == 0x50)
	{
		imu_cnt++;
		if(imu_buffer[1]==0x03)
		{
		if(imu_flag == 1)
		{
			imu.Accel.X = (int16_t)((int16_t)imu_buffer[4]<<8|imu_buffer[3]);
			imu.Accel.Y = (int16_t)((int16_t)imu_buffer[6]<<8|imu_buffer[5]);
			imu.Accel.Z = (int16_t)((int16_t)imu_buffer[8]<<8|imu_buffer[7]);
			imu.Other_Data.temp = (int16_t)((int16_t)imu_buffer[28]<<8|imu_buffer[27]);
			
			imu.Accel.X = imu.Accel.X / 32768.0f * 16.0f * 9.8f;
			imu.Accel.Y = imu.Accel.Y / 32768.0f * 16.0f * 9.8f;
			imu.Accel.Z = imu.Accel.Z / 32768.0f * 16.0f * 9.8f;
			imu.Other_Data.temp = imu.Other_Data.temp / 100.0f;

			imu.Accel.Z -= 9.8f;

			imu.Gyro.X = (int16_t)((int16_t)imu_buffer[10]<<8|imu_buffer[9]);
			imu.Gyro.Y = (int16_t)((int16_t)imu_buffer[12]<<8|imu_buffer[11]);
			imu.Gyro.Z = (int16_t)((int16_t)imu_buffer[14]<<8|imu_buffer[13]);
			
			imu.Gyro.X = imu.Gyro.X / 32768.0f * 2000.0f;
			imu.Gyro.Y = imu.Gyro.Y / 32768.0f * 2000.0f;
			imu.Gyro.Z = imu.Gyro.Z / 32768.0f * 2000.0f;
	

			imu.Magnet.X = (int16_t)((int16_t)imu_buffer[16]<<8|imu_buffer[15]);
			imu.Magnet.Y = (int16_t)((int16_t)imu_buffer[18]<<8|imu_buffer[17]);
			imu.Magnet.Z = (int16_t)((int16_t)imu_buffer[20]<<8|imu_buffer[19]);
		

			imu.Euler.roll = (int16_t)((int16_t)imu_buffer[22]<<8|imu_buffer[21]);
			imu.Euler.pitch = (int16_t)((int16_t)imu_buffer[24]<<8|imu_buffer[23]);
			imu.Euler.yaw = (int16_t)((int16_t)imu_buffer[26]<<8|imu_buffer[25]);
			
			imu.Euler.roll = imu.Euler.roll / 32768.0f * 180.0f;
			imu.Euler.pitch = imu.Euler.pitch / 32768.0f * 180.0f;
			imu.Euler.yaw = imu.Euler.yaw / 32768.0f * 180.0f;
		

//			imu.Quaternion.quat[0]=(int16_t)((int16_t)imu_buffer[3]<<8|imu_buffer[2]);
//			imu.Quaternion.quat[1]=(int16_t)((int16_t)imu_buffer[5]<<8|imu_buffer[4]);
//			imu.Quaternion.quat[2]=(int16_t)((int16_t)imu_buffer[7]<<8|imu_buffer[6]);
//			imu.Quaternion.quat[3]=(int16_t)((int16_t)imu_buffer[9]<<8|imu_buffer[8]);
//			
//			for(int i = 0;i < 4;i++)
//			{
//				imu.Quaternion.quat[i] /= 32768.0f;
//			}
		imu_flag = 0;
		for(int i = 0;i < 31;i++)
		{
			imu_buffer[i] = 0x00;
		}
		}
	}
	}
	else
	{
		IMU_ZERO:
		imu_cnt = 0;
		return;
	}
	if(imu_cnt >= 31)
	{
		imu_flag = 1;
		goto IMU_ZERO;
	}
	
}
