#include "chassis_api.h"
#include "helm_wheel.h"

extern RC_ctrl_t rc_ctrl;

eChassisAction actChassis = CHASSIS_NORMAL; //底盘默认遥控行走

Chassis_Speed_t absolute_chassis_speed;

Chassis_measure_t absolute_chassis_measure;

fp32 Angle_Helm_Target[4];
fp32 Speed_Motor_Target[4];

fp32 M3508_Target[4];
fp32 M2006_Target[4];

//extern HWT605_Modbus_t hwt605;
//extern ops9_t ops9;


/**
 * @brief  设定遥控器控制底盘模式
 * @param  void
 * @retval void
 * @attention
 */
void Remote_Control_Chassis_Set_Mode(void)
{
    if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 3) //底盘正常模式
    {
        actChassis = CHASSIS_NORMAL;
    }
    else if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 1) //底盘大陀螺模式   无云台无法实现
    {
        actChassis = CHASSIS_GYROSCOPE;
    }
}


/**
 * @brief  遥控器控制方式
 * @param  chassis_speed 底盘速度
 * @retval void
 * @attention
 */
void Remote_Control_Chassis_Mode(Chassis_Speed_t *chassis_speed)
{
    /***********************************确定底盘的目标速度*****************************************/
    switch (actChassis)
    {
    case CHASSIS_NORMAL: //正常模式
        chassis_speed->vx = (fp32)rc_ctrl.rc.ch[2]; 
        chassis_speed->vy = (fp32)rc_ctrl.rc.ch[3]; 
        chassis_speed->vw = -(fp32)rc_ctrl.rc.ch[4]; 

        chassis_speed->vx *= 6;
        chassis_speed->vy *= 6;
        chassis_speed->vw *= 1;
        break;

    case CHASSIS_GYROSCOPE: //小陀螺模式
        chassis_speed->vx = (fp32)rc_ctrl.rc.ch[2];
        chassis_speed->vy = (fp32)rc_ctrl.rc.ch[3];
        chassis_speed->vw = -330.0f;

        chassis_speed->vx *= 4;
        chassis_speed->vy *= 4;
        chassis_speed->vw *= 1;
        break;
    default:
        break;
    }
}


/**
 * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
void chassis_feedback_update(void)
{
		absolute_chassis_measure.Speed.vx = absolute_chassis_speed.vx;
		absolute_chassis_measure.Speed.vy = absolute_chassis_speed.vy;
		absolute_chassis_measure.Speed.vw = absolute_chassis_speed.vw;
		
//		absolute_chassis_measure.Euler.yaw = -hwt605.Euler.yaw;
//		absolute_chassis_measure.Euler.pitch = hwt605.Euler.pitch;
//		absolute_chassis_measure.Euler.roll = hwt605.Euler.roll;
//		
//		absolute_chassis_measure.Position.Pos_X = ops9.Position.pos_x;
//		absolute_chassis_measure.Position.Pos_Y = ops9.Position.pos_y;
		
//		//获取目标Yaw
//		Get_Chassis_Yaw_Target();
//		
//		//获取修正后的Yaw
//		Get_Chassis_Yaw_Offset();
//		
//		//获取累计Yaw
//		Get_Chassis_Total_Yaw();
}




/**
 * @brief  底盘运动解析式计算
 * @param  speed 底盘速度
 * @retval void
* @attention  此函数是舵轮底盘电机的速度解析式
 */
void Chassis_Sports_Calc(void)
{
	helm_wheel_angle_calc();
	helm_wheel_speed_calc();
}




fp32 speed_yaw[4] = {0};
fp32 position_yaw[4] = {0};
/**
 * @brief  底盘电机输出
 * @param  void
 * @retval void
 * @attention
 */
void Chassis_Loop_Out(void)
{
//		speed_yaw[0] = PID_Yaw_realize(0.0f);
//		speed_yaw[1] = PID_Yaw_realize(0.0f);
//		speed_yaw[2] = PID_Yaw_realize(0.0f);
	
//		speed_yaw[0] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
//		speed_yaw[1] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
//		speed_yaw[2] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
//		speed_yaw[3] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
	
    M3508_Target[0] = PID_velocity_realize_1(Speed_Motor_Target[0] + speed_yaw[0], 1);
    M3508_Target[1] = PID_velocity_realize_1(Speed_Motor_Target[1] + speed_yaw[1], 2);
    M3508_Target[2] = PID_velocity_realize_1(Speed_Motor_Target[2] + speed_yaw[2], 3);
		M3508_Target[3] = PID_velocity_realize_1(Speed_Motor_Target[3] + speed_yaw[3], 4);
	
		M2006_Target[0] = pid_call_2(Angle_Helm_Target[0] + position_yaw[0], 1);
    M2006_Target[1] = pid_call_2(Angle_Helm_Target[1] + position_yaw[1], 2);
    M2006_Target[2] = pid_call_2(Angle_Helm_Target[2] + position_yaw[2], 3);
    M2006_Target[3] = pid_call_2(Angle_Helm_Target[3] + position_yaw[3], 4);
	
    CAN1_CMD_1(M3508_Target[0], M3508_Target[1], M3508_Target[2], M3508_Target[3]);
	  CAN2_CMD_1(M2006_Target[0], M2006_Target[1], M2006_Target[2], M2006_Target[3]);
}
