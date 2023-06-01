#include "imu605_485.h"

void IMU_Unlock_Instruct(void)
{
	uint8_t unlock_buffer[5] = {0xFF,0xAA,0x69,0x88,0xB5};
	for(int i = 0;i < 5;i++)
	{
		HAL_UART_Transmit(&huart1,(unlock_buffer+i),1,HAL_MAX_DELAY);
	}
}

void IMU_Save_Instruct(void)
{
	uint8_t save_buffer[5] = {0xFF,0xAA,0x00,0x00,0x00};
	for(int i = 0;i < 5;i++)
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
	
}
