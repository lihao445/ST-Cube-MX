#ifndef __HWT101CT_H_
#define __HWT101CT_H_
#include "include.h"

 typedef struct SAngle
{
	short Angle[3];
	short T;
}SAngle;


void Yaw_Init(void);

void CopeSerial2Data(unsigned char ucData);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
