#include "Claw_task.h"

extern RC_ctrl_t rc_ctrl;

fp32 position;
fp32 current;


void claw_task_test(void const * argument)
{
	PID_devices_Init();
	while(1)
	{
			if(rc_ctrl.rc.ch[1] >= 580)
		{
			position = -6.0f / 17.0f * 8191 * 36;
		}
		else if(rc_ctrl.rc.ch[1] <= -580)
		{
			position = 6.0f / 17.0f * 8191 * 36;
		}
		else
		{
			position = 0;
		}
		current = pid_call_1(position,1);
		CAN1_CMD_1(current,0,0,0);
		vTaskDelay(2);             //ÑÓÊ±²»ÄÜÌ«¾Ã
	}
}
