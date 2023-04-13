#include "chassis_task.h"

extern RC_ctrl_t rc_ctrl;

extern Chassis_Speed_t absolute_chassis_speed;

extern osSemaphoreId Chassis_BinarySemHandle;

void chassis_task(void const * argument)
{
	// wait a time
	//空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME);

	//PID parameter initialization
	//PID参数初始化
	PID_devices_Init();
	
	while(1)
	{
		xSemaphoreTake(Chassis_BinarySemHandle,portMAX_DELAY);
		
		//calculate the speed of the chassis
		//底盘速度解算
		Chassis_Sports_Calc(absolute_chassis_speed);

		//chassis control pid calculate
		//底盘控制PID计算与数据发送
		Chassis_Loop_Out();

		//chassis task control time
		//底盘任务控制间隔
		osDelay(CHASSIS_CONTROL_TIME_MS);
	}
		

}
