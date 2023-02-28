#ifndef __CHASSIS_TASK_H_
#define __CHASSIS_TASK_H_
#include "main.h"

//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2



static void chassis_feedback_update(motor_measure_t *chassis_move_update);
static void chassis_set_contorl(motor_measure_t *chassis_move_control);
static void chassis_control_loop(void);

void RC_speed_chassis_data(void);


void chassis_task(void const * argument);




#endif
