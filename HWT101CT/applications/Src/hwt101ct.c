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
	
//�����ǹ���
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
	
	

	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		if(ucRxBuffer[1]==0x53)//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			memcpy(&stcAngle,&ucRxBuffer[2],8);   //string.h
		}
		ucRxCnt=0;//��ջ�����
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		static uint8_t Res;
		static int qwe=0;
		if(__HAL_UART_GET_FLAG(&huart1, UART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
			qwe++;
//		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
			Res = value_IMU;
			CopeSerial2Data(Res);
			yaw = (float)stcAngle.Angle[2]/32768*180.0f;
		}
	}
}

