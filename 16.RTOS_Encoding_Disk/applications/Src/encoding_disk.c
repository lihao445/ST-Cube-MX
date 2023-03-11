#include "encoding_disk.h"

extern osMessageQId Encoding_Disk_QueueHandle;


/**
 * @brief 解析结果变量，如需跨文件调用，需要外部声明
 */
float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;



void USART1_IRQHandler(void)                	//串口6中断服务程序
{
	int8_t Res;
	if(huart1.Instance->SR & UART_FLAG_RXNE)									//接收中断
	{
		__HAL_UART_CLEAR_PEFLAG(&huart1);
		Res = huart1.Instance->DR;
	}
//	 else if(huart6.Instance->SR & UART_FLAG_IDLE)          //空闲中断
//  {
//		__HAL_UART_CLEAR_PEFLAG(&huart6);
//    Res = huart6.Instance->DR;
//	}
	
	xQueueOverwriteFromISR(Encoding_Disk_QueueHandle,&Res,NULL);
//	Data_Analyse(Res);
} 



/**
 * @brief 数据解析任务  如更换MCU平台或更换软件库，只需将串口接收到的值传入该任务即可解析
 * @param  rec 串口接收到的字节数据
 */
void encoding_disk_task(void const * argument)
{
	int8_t rec;
	while(1)
	{
		xQueueReceive(Encoding_Disk_QueueHandle,&rec,portMAX_DELAY);
		static uint8_t ch;
	static union
	{
		uint8_t date[24];
		float ActVal[6];
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;

	ch=rec;
	switch(count)
	{
		case 0:
			if(ch==0x0d)
				count++;
			else
				count=0;
			break;
		case 1:
			if(ch==0x0a)
			{
				i=0;
				count++;
			}
			else if(ch==0x0d);
			else
				count=0;
			break;
		case 2:
			posture.date[i]=ch;
			i++;
			if(i>=24)
			{
				i=0;
				count++;
			}
			break;
		case 3:
			if(ch==0x0a)
				count++;
			else
				count=0;
			break;
		case 4:
			if(ch==0x0d)
			{
				zangle=posture.ActVal[0];
				xangle=posture.ActVal[1];
				yangle=posture.ActVal[2];
				pos_x=posture.ActVal[3];
				pos_y=posture.ActVal[4];
				w_z=posture.ActVal[5];
			}
			count=0;
			break;
		default:
			count=0;
		break;
	}
		
	}
}



