#ifndef __SERVO_MOTOR_H_
#define __SERVO_MOTOR_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include "./SYSTEM/sys/sys.h"

extern TIM_HandleTypeDef htim1;     /* ¶¨Ê±Æ÷x¾ä±ú */
void TIM_PWM_Init(uint16_t psc,uint16_t arr);


#ifdef __cplusplus
}
#endif


#endif
