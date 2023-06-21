#include "catch_task.h"

fp32 Catch_Speed_Target[2];

fp32 Catch_Current_Target[2];

extern RC_ctrl_t rc_ctrl;

void catch_task(void const * argument)
{
	while(1)
	{
		Catch_Speed_Target[0] = rc_ctrl.rc.ch[1] * 3.0f/3.0f;
		Catch_Speed_Target[1] = -rc_ctrl.rc.ch[1] * 3.0f/3.0f;
		
		Catch_Current_Target[0] = PID_velocity_realize_1(Catch_Speed_Target[0],5);
		Catch_Current_Target[1] = PID_velocity_realize_1(Catch_Speed_Target[1],6);
		
		CAN1_CMD_2(Catch_Current_Target[0],Catch_Current_Target[1],0,0);
		osDelay(2);
	}
}



