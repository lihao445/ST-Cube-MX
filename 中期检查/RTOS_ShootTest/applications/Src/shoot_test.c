#include "shoot_test.h"

extern RC_ctrl_t rc_ctrl;

void shoot_test_task(void const * argument)
{
	PID_devices_Init();
	while(1)
	{
//			if(rc_ctrl.rc.ch[3] >= 580)
//	{
//		CAN1_CMD_1(PID_velocity_realize_1(1000,1),PID_velocity_realize_1(-1000,2),0,0);
//	}
//		else if(rc_ctrl.rc.ch[3] <= -580)
//	{
//		CAN1_CMD_1(PID_velocity_realize_1(2000,1),PID_velocity_realize_1(-2000,2),0,0);
//	}
//	
//	
//		if(rc_ctrl.rc.ch[2] >= 580)
//	{
//		CAN1_CMD_1(PID_velocity_realize_1(4000,1),PID_velocity_realize_1(-4000,2),0,0);
//	}
//		else if(rc_ctrl.rc.ch[2] <= -580)
//	{
//		CAN1_CMD_1(PID_velocity_realize_1(3000,1),PID_velocity_realize_1(-3000,2),0,0);
//	}
//	
//	
//		if(rc_ctrl.rc.ch[1] >= 580)
//	{
//		CAN1_CMD_1(PID_velocity_realize_1(5000,1),PID_velocity_realize_1(-5000,2),0,0);
//	}
//		else if(rc_ctrl.rc.ch[1] <= -580)
//	{
//		CAN1_CMD_1(PID_velocity_realize_1(6000,1),PID_velocity_realize_1(-6000,2),0,0);
//	}
//	
//		if(rc_ctrl.rc.ch[0] >= 580)
//	{
//		CAN1_CMD_1(PID_velocity_realize_1(8000,1),PID_velocity_realize_1(-8000,2),0,0);
//	}
//		else if(rc_ctrl.rc.ch[0] <= -580)
//	{
//		CAN1_CMD_1(PID_velocity_realize_1(7000,1),PID_velocity_realize_1(-7000,2),0,0);
//	}
//	

//	
//	if(rc_ctrl.rc.ch[2] == 0 && rc_ctrl.rc.ch[3] == 0 && rc_ctrl.rc.ch[4] == 0 && rc_ctrl.rc.ch[1] == 0 && rc_ctrl.rc.ch[0] == 0)
//	{
//		CAN1_CMD_1(PID_velocity_realize_1(0,1),PID_velocity_realize_1(0,2),0,0);
//	}
	

		
		
//		CAN1_CMD_1(PID_velocity_realize_1(rc_ctrl.rc.ch[3]*2,1),PID_velocity_realize_1(-rc_ctrl.rc.ch[3],2),0,0); 
//		CAN1_CMD_1(PID_velocity_realize_1(-3100,1),PID_velocity_realize_1(3100,2),0,0);
		vTaskDelay(2);             //ÑÓÊ±²»ÄÜÌ«¾Ã
		CAN1_CMD_1(PID_velocity_realize_1(469,1),PID_velocity_realize_1(-469,2),PID_velocity_realize_1(469,3),PID_velocity_realize_1(-469,4));
		
	}
}
