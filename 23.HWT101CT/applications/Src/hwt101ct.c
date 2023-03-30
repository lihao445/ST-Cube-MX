#include "hwt101ct.h"

// �����Ǵ��ڽ���
uint8_t value_IMU;			// ��������ֵ
uint8_t getBuffer[22];		// ������ݵ�����
uint8_t Enter[] = "\r\n";
fp32 IMU_DATA;		// �Ƕ�ֵ
uint8_t IMU_DATAL;	// ����λ��8λ
uint8_t IMU_DATAH;	// ����λ��8λ
volatile fp32 IMU_Z = 0.00f;
volatile fp32 IMU_init_data = 0.00f;		// �����ǳ�ʼ�Ƕ�
volatile int8_t first_flag = 0;						// �жϵ�һ��¼�������ǽǶ�

uint8_t countOfGetBuffer = 0;	// λ���ۼ�
int8_t flag = 0;	// �жϽ��տ�ʼ

int8_t now_uart_flag = 0;		// ��ǰ����ֵ
int8_t last_uart_flag = 0;	// ��һ������ֵ


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		last_uart_flag = now_uart_flag;		// ��һ�εĴ�������ֵ
		now_uart_flag = value_IMU;			// ��ǰ�Ĵ�������ֵ
	if(last_uart_flag == 0x55 && now_uart_flag == 0x52)		// Э���ж�����Ƕ�ֵ
	{
		//printf("IMU\r\n");
		flag = 1;		// �������ճ���
	}
	//printf("%c", value_IMU);
 
	if(flag == 1)
	{
		getBuffer[countOfGetBuffer++] = value_IMU; 
		if(countOfGetBuffer == 18)		// λ��
		{
			flag = 0;

			//while(HAL_UART_Transmit(&huart3, (uint8_t*)getBuffer, countOfGetBuffer, 5000)!= HAL_OK);
			//while(HAL_UART_Transmit(&huart3, (uint8_t*)Enter, 4, 5000)!= HAL_OK);

			IMU_DATAL = getBuffer[16];
			IMU_DATAH = getBuffer[17];
			IMU_DATA = (((short)IMU_DATAH<<8) | IMU_DATAL);		// ��8λ��8λ

			IMU_Z = ((float)IMU_DATA/32768)*180-180;	// ��0����360�㷶Χ��Ϊ-180����180��

			if(first_flag == 0)		// ��Ƭ����λ��һ�λ�ȡ������ֵ����Ϊ��ʼ����
			{
				IMU_init_data = IMU_Z;
				first_flag = 1;
//				printf("Start:IMU_init_data:%.2f\r\n", IMU_init_data);
			}	

			//printf("IMU_init_data:%.2f\r\n", IMU_init_data);
			memset(getBuffer, 0, countOfGetBuffer);		// �������  string.h
			countOfGetBuffer = 0;	// λ������
		}
	}
	}
}

