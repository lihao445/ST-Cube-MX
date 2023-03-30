#include "hwt101ct.h"

// 陀螺仪串口接收
uint8_t value_IMU;			// 串口输入值
uint8_t getBuffer[22];		// 存放数据的数组
uint8_t Enter[] = "\r\n";
fp32 IMU_DATA;		// 角度值
uint8_t IMU_DATAL;	// 数据位低8位
uint8_t IMU_DATAH;	// 数据位高8位
volatile fp32 IMU_Z = 0.00f;
volatile fp32 IMU_init_data = 0.00f;		// 陀螺仪初始角度
volatile int8_t first_flag = 0;						// 判断第一次录入陀螺仪角度

uint8_t countOfGetBuffer = 0;	// 位数累计
int8_t flag = 0;	// 判断接收开始

int8_t now_uart_flag = 0;		// 当前输入值
int8_t last_uart_flag = 0;	// 上一个输入值


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		last_uart_flag = now_uart_flag;		// 上一次的串口输入值
		now_uart_flag = value_IMU;			// 当前的串口输入值
	if(last_uart_flag == 0x55 && now_uart_flag == 0x52)		// 协议判断输入角度值
	{
		//printf("IMU\r\n");
		flag = 1;		// 开启接收程序
	}
	//printf("%c", value_IMU);
 
	if(flag == 1)
	{
		getBuffer[countOfGetBuffer++] = value_IMU; 
		if(countOfGetBuffer == 18)		// 位长
		{
			flag = 0;

			//while(HAL_UART_Transmit(&huart3, (uint8_t*)getBuffer, countOfGetBuffer, 5000)!= HAL_OK);
			//while(HAL_UART_Transmit(&huart3, (uint8_t*)Enter, 4, 5000)!= HAL_OK);

			IMU_DATAL = getBuffer[16];
			IMU_DATAH = getBuffer[17];
			IMU_DATA = (((short)IMU_DATAH<<8) | IMU_DATAL);		// 高8位低8位

			IMU_Z = ((float)IMU_DATA/32768)*180-180;	// 把0°至360°范围改为-180°至180°

			if(first_flag == 0)		// 单片机复位第一次获取陀螺仪值设置为初始正向
			{
				IMU_init_data = IMU_Z;
				first_flag = 1;
//				printf("Start:IMU_init_data:%.2f\r\n", IMU_init_data);
			}	

			//printf("IMU_init_data:%.2f\r\n", IMU_init_data);
			memset(getBuffer, 0, countOfGetBuffer);		// 清空数组  string.h
			countOfGetBuffer = 0;	// 位数清零
		}
	}
	}
}

