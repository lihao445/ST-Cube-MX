#include "imu101ct.h"

uint8_t rx_buffer[1];
uint8_t imu_data_buffer[11];
int16_t imu_euler_count = 0;
int16_t imu_gyro_count = 0;

uint8_t tx_buffer[5];


IMU_t imu;



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
				if(imu_euler_count > 10)
				{
					 imu_euler_count = 0;
					 Clear_buffer(rx_buffer);
				}
				if(imu_gyro_count > 10)
				{
					 imu_gyro_count = 0;
					 Clear_buffer(rx_buffer);
				}
        IMU_Read_Gryo(rx_buffer);
        IMU_Read_Euler(rx_buffer);
    }
}

void Clear_buffer(uint8_t *rx_buffer)
{
	for(int16_t i = 0;i < 11;i++)
	{
		*(rx_buffer + i) = 0;
	}
}


void IMU_Read_Gryo(uint8_t *rx_buffer)
{
    imu_data_buffer[imu_gyro_count] = *rx_buffer;
    if(imu_data_buffer[0] == 0x55)
    {
        imu_data_buffer[imu_gyro_count++] = *rx_buffer;
        if(imu_data_buffer[1] ==0x52)
        {
            for(;imu_gyro_count <= 10;imu_gyro_count++)
            imu_data_buffer[imu_gyro_count] = *rx_buffer;
        }
    }
    imu.Gyro.Y = (int16_t)((int16_t)imu_data_buffer[5]<<8|imu_data_buffer[4]);
    imu.Gyro.Z = (int16_t)((int16_t)imu_data_buffer[7]<<8|imu_data_buffer[6]);

    imu.Gyro.Y = imu.Gyro.Y / 32768.0f * 2000.0f;
    imu.Gyro.Z = imu.Gyro.Z / 32768.0f * 2000.0f;
}


void IMU_Read_Euler(uint8_t *rx_buffer)
{
    imu_data_buffer[imu_euler_count] = *rx_buffer;
    if(imu_data_buffer[0] == 0x55)
    {
        imu_data_buffer[imu_euler_count++] = *rx_buffer;
        if(imu_data_buffer[1] ==0x53)
        {
            for(;imu_euler_count <= 10;imu_euler_count++)
            imu_data_buffer[imu_euler_count] = *rx_buffer;
        }
    }
    imu.Euler.yaw = (int16_t)((int16_t)imu_data_buffer[7]<<8|imu_data_buffer[6]);

    imu.Euler.yaw = imu.Euler.yaw / 32768.0f * 180.0f;
}


void IMU_Write_Register(UART_HandleTypeDef *huart,uint8_t reg_addr,uint16_t tx_data)
{
    tx_buffer[0] = 0xFF;
    tx_buffer[1] = 0xAA;
    tx_buffer[2] = reg_addr;
    tx_buffer[3] = tx_data;
    tx_buffer[4] = tx_data >> 8;
    for(int i = 0;i < 5;i++)
    {
        HAL_UART_Transmit(huart,&(tx_buffer[i]),1,HAL_MAX_DELAY);
    }
}
