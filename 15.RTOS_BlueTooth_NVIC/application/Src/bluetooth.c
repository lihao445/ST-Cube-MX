#include "bluetooth.h"

uint8_t rx_buffer[1];
extern osThreadId LED_GREENHandle;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		switch(rx_buffer[0])
		{
			case '0':vTaskSuspend(LED_GREENHandle);break;
			case '1':vTaskResume(LED_GREENHandle);break;
			default:;
		}
		HAL_Delay(50);
	}
}

