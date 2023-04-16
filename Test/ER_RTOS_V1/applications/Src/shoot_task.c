#include "shoot_task.h"

fp32 shoot_current[2];
fp32 shoot_speed[2];

extern Other_Devices_t absolute_robot_control;

extern osSemaphoreId Shoot_BinarySemHandle;


void shoot_task(void const * argument)
{
    while(1)
    {
        xSemaphoreTake(Shoot_BinarySemHandle,portMAX_DELAY);
        
        for(int i = 0;i < 2;i++)
        {
            shoot_speed[i] = absolute_robot_control.shoot_speed[i];
        }
        for(int i = 0;i < 2;i++)
        {
            shoot_current[i] = PID_velocity_realize_1(shoot_speed[i],i + 5);
        }
        CAN1_CMD_2(shoot_current[0],shoot_current[1],0,0);
        osDelay(2);
    }
}
