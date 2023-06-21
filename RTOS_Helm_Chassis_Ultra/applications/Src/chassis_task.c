/**    《遥控器舵轮控制》
 *           _____                    _____
 *          /\    \                  /\    \
 *         /::\    \                /::\    \
 *        /::::\    \              /::::\    \
 *       /::::::\    \            /::::::\    \
 *      /:::/\:::\    \          /:::/\:::\    \
 *     /:::/__\:::\    \        /:::/  \:::\    \
 *    /::::\   \:::\    \      /:::/    \:::\    \
 *   /::::::\   \:::\    \    /:::/    / \:::\    \
 *  /:::/\:::\   \:::\____\  /:::/    /   \:::\    \
 * /:::/  \:::\   \:::|    |/:::/____/     \:::\____\
 * \::/   |::::\  /:::|____|\:::\    \      \::/    /
 *  \/____|:::::\/:::/    /  \:::\    \      \/____/
 *        |:::::::::/    /    \:::\    \
 *        |::|\::::/    /      \:::\    \
 *        |::| \::/____/        \:::\    \
 *        |::|  ~|               \:::\    \
 *        |::|   |                \:::\    \
 *        \::|   |                 \:::\____\
 *         \:|   |                  \::/    /
 *          \|___|                   \/____/
 */

#include "chassis_task.h"
#include "chassis_api.h"
#include "pid_user.h"

extern Chassis_Speed_t absolute_chassis_speed;

void chassis_task(void const *argument)
{
	// wait a time
	//空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME);

	//底盘初始化
	PID_devices_Init();

	while (1)
	{
		Remote_Control_Chassis_Set_Mode();
		
		Remote_Control_Chassis_Mode(&absolute_chassis_speed);
		
		chassis_feedback_update();
		
		Chassis_Sports_Calc();
		
		Chassis_Loop_Out();
		
		// os delay
		//系统延时
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
}

