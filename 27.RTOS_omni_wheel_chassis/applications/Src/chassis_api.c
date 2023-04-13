#include "chassis_api.h"

extern RC_ctrl_t rc_ctrl;

eChassisAction actChassis = CHASSIS_NORMAL; //底盘默认遥控行走

Chassis_Speed_t absolute_chassis_speed;

//fp32 Angle_Helm_Target[4];
fp32 Speed_Motor_Target[4];

fp32 M3508_Target[4];




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
 * @param  void
 * @retval void
 * @attention
 */
void Remote_Control_Chassis_Mode(Chassis_Speed_t *chassis_speed)
{
    /***********************************确定底盘四个电机的目标速度*****************************************/
    switch (actChassis)
    {
    case CHASSIS_NORMAL: //正常模式
        chassis_speed->vx = (fp32)rc_ctrl.rc.ch[2]; 
        chassis_speed->vy = (fp32)rc_ctrl.rc.ch[3]; 
        chassis_speed->vw = (fp32)rc_ctrl.rc.ch[4]; 

        chassis_speed->vx *= 6;
        chassis_speed->vx *= 6;
        chassis_speed->vx *= 5;
        break;

    case CHASSIS_GYROSCOPE: //小陀螺模式
        chassis_speed->vx = (fp32)rc_ctrl.rc.ch[2];
        chassis_speed->vy = (fp32)rc_ctrl.rc.ch[3];
        chassis_speed->vw = 330.0f;

        chassis_speed->vx *= 6;
        chassis_speed->vx *= 6;
        chassis_speed->vx *= 5;
        break;
    default:
        break;
    }
}

/**
 * @brief  底盘运动解析式计算
 * @param  void
 * @retval void
 * @attention  此函数是全向轮底盘解析式
 */
void Chassis_Sports_Calc(Chassis_Speed_t speed)
{
        Speed_Motor_Target[0] =   speed.vy + speed.vx + speed.vw;
		Speed_Motor_Target[1] = - speed.vy + speed.vx + speed.vw;
		Speed_Motor_Target[2] = - speed.vy - speed.vx + speed.vw;
		Speed_Motor_Target[3] =   speed.vy - speed.vx + speed.vw;
}





/**
 * @brief  底盘电机输出
 * @param  void
 * @retval void
 * @attention
 */
void Chassis_Loop_Out(void)
{
    M3508_Target[0] = PID_velocity_realize_1(Speed_Motor_Target[0], 1);
    M3508_Target[1] = PID_velocity_realize_1(Speed_Motor_Target[1], 2);
    M3508_Target[2] = PID_velocity_realize_1(Speed_Motor_Target[2], 3);
    M3508_Target[3] = PID_velocity_realize_1(Speed_Motor_Target[3], 4);

    CAN1_CMD_1(M3508_Target[0], M3508_Target[1], M3508_Target[2], M3508_Target[3]);
}


