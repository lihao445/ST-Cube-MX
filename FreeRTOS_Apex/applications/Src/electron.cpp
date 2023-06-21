#include "electron.h"

extern "C"
void electromagnet_task(void const * argument)
{
	while(true)
	{
		HAL_GPIO_TogglePin(Electromagnet_GPIO_Port,Electromagnet_Pin);
		osDelay(500);
	}
	
	                                                                                                                                                                                                                                                                                                                                                  
}
