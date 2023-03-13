#include "HWT9052_485.h"

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;


uint8_t cmd[8] = {0X50,0X03,0X00,0X34,0X00,0X0f,0X49,0X81};//读取0X34之后的12个寄存器
static uint8_t TxBuffer[256];
static uint8_t TxCounter=0;
static uint8_t count=0; 




void USART1_IRQHandler(void)
{
		if(__HAL_UART_GET_FLAG(&huart1,UART_IT_TXE) != RESET)
		{
				USART1->DR = (TxBuffer[TxCounter++] & (uint16_t)0x01FF);
			  __HAL_UART_CLEAR_FLAG(&huart1,UART_IT_TXE);
				if(TxCounter == count)
			 {
					__HAL_UART_DISABLE_IT(&huart1,UART_IT_TXE);
			 }
		}
		else if(__HAL_UART_GET_FLAG(&huart1,UART_IT_RXNE) != RESET)
		{
				CopeSerial2Data((uint8_t)USART2->DR);//处理数据
				__HAL_UART_CLEAR_FLAG(&huart1,UART_IT_RXNE);
		}
		__HAL_UART_CLEAR_OREFLAG(&huart1);
}
	
	



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance==USART1)
//	{
//			if(__HAL_UART_GET_FLAG(huart,UART_IT_TXE) != RESET)
//		{
//				USART1->DR = (TxBuffer[TxCounter++] & (uint16_t)0x01FF);
//			__HAL_UART_CLEAR_FLAG(huart,UART_IT_TXE);
//				if(TxCounter == count)
//			{
//					__HAL_UART_DISABLE_IT(huart,UART_IT_TXE);
//			}
//		}
//		else if(__HAL_UART_GET_FLAG(huart,UART_IT_RXNE) != RESET)
//		{
//					CopeSerial2Data((uint8_t)USART2->DR);//处理数据
//					__HAL_UART_CLEAR_FLAG(huart,UART_IT_RXNE);
//		}
//		__HAL_UART_CLEAR_OREFLAG(huart);
//	}
//}


void CharToShort(unsigned char cTemp[],short sTemp[],short sShortNum)
{
	int i;
	for (i = 0;i<3;i++) 
		sTemp[i] = (cTemp[2*i+sShortNum]<<8)|(cTemp[2*i+sShortNum+1]&0xff);
}


void CharToInt(unsigned char cTemp[],int iTemp[],int iShortNum)
{
	int i;
	for (i = 0;i<3;i++) 
		iTemp[i] = (cTemp[4*i+iShortNum]<<8)|(cTemp[4*i+iShortNum+1]&0xff)|(cTemp[4*i+iShortNum+2]<<24)|(cTemp[4*i+iShortNum+3]<<16);
}


//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	

	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x50) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<35) {return;}//数据不满29个，则返回
	else
	{
		ucRxCnt=0;//清空缓存区，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
		CharToShort(ucRxBuffer,stcAcc.a,3);
		CharToShort(ucRxBuffer,stcGyro.w,9);
		CharToShort(ucRxBuffer,stcMag.h,15);
		CharToInt(ucRxBuffer,stcAngle.Angle,21);		
	}
}


void CopeSerial1Data(unsigned char ucData)
{	
	UART2_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
}



void UART2_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_TXE);       //需要改串口句柄
}

void UART2_Put_String(unsigned char *Str,unsigned char length)
{
	int i;
	for( i = 0; i<length;i++)
	{
		UART2_Put_Char(*Str);
		Str++;
	}
}
