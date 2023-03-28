#include "hwt101ct.h"

struct SAngle 	stcAngle;
float yaw;

uint8_t value_IMU;

uint8_t make_zero[5] = {0xff, 0xaa, 0x76, 0x00, 0x00};
uint8_t unlocking[5] = {0xff, 0xaa, 0x69, 0x88, 0xb5};

void Yaw_Init(void)
{
	for(int t=0;t<5;t++)
	{
		HAL_UART_Transmit(&huart1, &unlocking[t],1,HAL_MAX_DELAY);         
		while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);
	}
	
//陀螺仪归零
		for(int t=0;t<5;t++)
	{
		HAL_UART_Transmit(&huart1, &make_zero[t],1,HAL_MAX_DELAY);         
		while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);
	}
	
}


void CopeSerial2Data(unsigned char ucData)
{
	static uint8_t ucRxBuffer[250];
	static uint8_t ucRxCnt = 0;	
	
	

	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		if(ucRxBuffer[1]==0x53)//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			memcpy(&stcAngle,&ucRxBuffer[2],8);   //string.h
		}
		ucRxCnt=0;//清空缓存区
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		static uint8_t Res;
		static int qwe=0;
		if(__HAL_UART_GET_FLAG(&huart1, UART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
			qwe++;
//		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
			Res = value_IMU;
			CopeSerial2Data(Res);
			yaw = (float)stcAngle.Angle[2]/32768*180.0f;
		}
	}
}

