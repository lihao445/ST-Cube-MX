#ifndef __TIMER_USER_H_
#define __TIMER_USER_H_
#include "include.h"


#define ABS(x)	((x>0) ? (x) : (-x))  //ͨ���;���ֵ����(x��ʲô���;���ʲô����)


//�����ٶ�
typedef struct
{
    fp32 vx;
    fp32 vy;
    fp32 vw;
} Chassis_Speed_t;


#endif



