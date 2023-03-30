/***      《 舵轮角度计算 》

 *      ┌─┐       ┌─┐ + +
 *   ┌──┘ ┴───────┘ ┴──┐++
 *   │                 │
 *   │       ───       │++ + + +
 *   ███████───███████│+
 *   │                 │+
 *   │      ─┴─        │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │   + +
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘  + + + +
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘  + + + +
 *
 *
 */
#include "helm_wheel.h"

////计算遥控器遥杆角度
//float calc_angle_helm_wheel(float set_ch2, float set_ch3)
//{
//	if (ABS(set_ch2) > 120 || ABS(set_ch3) > 120)
//		return (atan2(-set_ch2, set_ch3)) * 180.00f / 3.14159f; // atan2函数math.h
//	else
//		return 0.00f;
//}

////计算电机应该转动的圈数，防止跳变
//float calc_motor_round_cnt(float angle, float last_angle)
//{
//	static float round_cnt = 0.00f;
//	if (angle - last_angle > 260) //记录轮子转动圈数
//		round_cnt--;
//	else if (angle - last_angle < -260)
//		round_cnt++;
//	return round_cnt;
//}

////优化舵轮转动角度，就近原则
//float calc_min_angle(float set_angle, float last_set_angle)
//{
//	float target_angle;
//	if (fabs(set_angle - last_set_angle) > 90)
//	{
//		if (set_angle >= 90 && set_angle <= 181)
//		{
//			target_angle = set_angle - 180.00f;
//		}
//		if (set_angle <= -90 && set_angle >= -181)
//		{
//			target_angle = set_angle + 180.00f;
//		}
//	}
//	else
//	{
//		target_angle = set_angle;
//	}
//	return target_angle;
//}

eChassisAction actChassis = CHASSIS_NORMAL; //底盘默认遥控行走

extern RC_ctrl_t rc_ctrl;

Chassis_Speed_t absolute_chassis_speed;

extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];

fp32 Angle_Helm_Target[4];
fp32 Speed_Motor_Target[4];

fp32 M2006_Target[4];
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
    else if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 1) //底盘小陀螺模式
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
void Remote_Control_Chassis_Mode(void)
{
    /***********************************确定底盘四个电机的目标速度*****************************************/
    switch (actChassis)
    {
    case CHASSIS_NORMAL: //正常模式
        absolute_chassis_speed.vx = (fp32)rc_ctrl.rc.ch[2];
        absolute_chassis_speed.vy = (fp32)rc_ctrl.rc.ch[3];
        absolute_chassis_speed.vw = (fp32)rc_ctrl.rc.ch[4];
        break;
    case CHASSIS_GYROSCOPE: //小陀螺模式
        absolute_chassis_speed.vx = (fp32)rc_ctrl.rc.ch[2];
        absolute_chassis_speed.vy = (fp32)rc_ctrl.rc.ch[3];
        absolute_chassis_speed.vw = 330;
        break;
    default:
        break;
    }
}


/**
 * @brief  计算底盘驱动电机的目标速度
 * @param  speed 底盘坐标的速度
 * @param  out_speed 3508目标速度
 * @retval void
 * @attention
 */
int8_t drct = 1;               //决定驱动电机正反转
//fp32 WHEEL_PERIMETER = 1;      //车轮周长
//fp32 CHASSIS_DECELE_RATIO = 1; //底盘减速器比例
fp32 Radius = 1;               //半径
void helm_wheel_speed_calc(Chassis_Speed_t *speed, fp32 *out_speed)
{
    // 3508目标速度计算
    fp32 helm_wheel_rpm[4];
    fp32 wheel_rpm_ratio; //轮子速度比率

//    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;
    wheel_rpm_ratio = 1;

    helm_wheel_rpm[0] = sqrt(pow(speed->vy + speed->vw * Radius * 0.707107f, 2) + pow(speed->vx - speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    helm_wheel_rpm[1] = sqrt(pow(speed->vy - speed->vw * Radius * 0.707107f, 2) + pow(speed->vx - speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    helm_wheel_rpm[2] = sqrt(pow(speed->vy - speed->vw * Radius * 0.707107f, 2) + pow(speed->vx + speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    helm_wheel_rpm[3] = sqrt(pow(speed->vy + speed->vw * Radius * 0.707107f, 2) + pow(speed->vx + speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;

			if(speed->vy > 0)    //反向轮转
		{
			for(int i = 0;i < 4;i++)
			{
				helm_wheel_rpm[i] *= -1;
			}
		}
	
    for (int i = 0; i < 4; i++)
    {
        out_speed[i] = drct * helm_wheel_rpm[i] * 5;
    }
}



/**
 * @brief  计算底盘航向电机的目标角度
 * @param  speed 底盘坐标的速度
 * @param  out_angle 2006目标角度
 * @retval void
 * @attention
 */
bool_t flag = 1;
void helm_wheel_angle_calc(Chassis_Speed_t *speed, fp32 *out_angle)
{
    fp32 angle_temp[4];
    fp32 helm_wheel_angle[4]; // 2006编码器目标角度
    fp32 helm_wheel_angle_last[4];
    fp64 atan_angle[4];

    // 2006目标角度计算
    if (!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0)) //防止除数为零
    {
        atan_angle[0] = atan2((speed->vx + speed->vw * Radius * 0.707107f), (speed->vy + speed->vw * Radius * 0.707107f)) * 180.0f / PI;   //旋转第1象限
        atan_angle[1] = atan2((speed->vx + speed->vw * Radius * 0.707107f), (speed->vy - speed->vw * Radius * 0.707107f)) * 180.0f / PI;   //旋转第4象限
        atan_angle[2] = atan2((speed->vx - speed->vw * Radius * 0.707107f), (speed->vy - speed->vw * Radius * 0.707107f)) * 180.0f / PI;   //旋转第3象限
        atan_angle[3] = atan2((speed->vx - speed->vw * Radius * 0.707107f), (speed->vy + speed->vw * Radius * 0.707107f)) * 180.0f / PI;   //旋转第2象限
    }

    //就近原则
    for(int i = 0;i < 4;i++)
    {
			angle_temp[i] = motor_can2[i].angle * 360.0f / 8191  / 36.0f * (20.0f / 67.0f);
      helm_wheel_angle[i] = calc_min_angle(atan_angle[i],angle_temp[i]);
    }
		
		for(int i = 0;i < 4;i++)
		{
			helm_wheel_angle[i] *= -1;
		}
		
		
    if (speed->vx == 0 && speed->vy == 0 && speed->vw == 0) //摇杆回中时
    {
        for (int i = 0; i < 4; i++) // memcpy狗都不用
            out_angle[i] = helm_wheel_angle_last[i];
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            out_angle[i] = (helm_wheel_angle[i] / 360.0f * 8191) * 36 * (67.0f / 20.0f);
            helm_wheel_angle_last[i] = out_angle[i];
        }
    }
}



/**
 * @brief  底盘电机输出
 * @param  void
 * @retval void
 * @attention
 */
void Chassis_Loop_Out(void)
{
	M3508_Target[0] = PID_velocity_realize_1(Speed_Motor_Target[0],1);
	M3508_Target[1] = PID_velocity_realize_1(Speed_Motor_Target[1],2);
	M3508_Target[2] = PID_velocity_realize_1(Speed_Motor_Target[2],3);
	M3508_Target[3] = PID_velocity_realize_1(Speed_Motor_Target[3],4);

	M2006_Target[0] = pid_call_2(Angle_Helm_Target[0],1);
	M2006_Target[1] = pid_call_2(Angle_Helm_Target[1],2);
	M2006_Target[2] = pid_call_2(Angle_Helm_Target[2],3);
	M2006_Target[3] = pid_call_2(Angle_Helm_Target[3],4);

	CAN1_CMD_1(M3508_Target[0], M3508_Target[1], M3508_Target[2], M3508_Target[3]);
	CAN2_CMD_1(M2006_Target[0], M2006_Target[1], M2006_Target[2], M2006_Target[3]);
}

/**
 * @brief  优化舵轮转动角度，就近原则
 * @param  set_angle 即将要转到的角度
 * @param  last_set_angle 当前角度
 * @retval target_angle 就近原则后的角度
 * @attention
 */
float calc_min_angle(float set_angle, float last_set_angle)
{
	float target_angle;
	if (fabs(set_angle - last_set_angle) > 90.0f)        //math.h  计算浮点数绝对值
	{
		if (180.0f >= set_angle && set_angle >= 90.0f)
		{
			target_angle = set_angle - 180.00f;
		}
		if (-90.0f >= set_angle && set_angle >= -180.f)
		{
			target_angle = set_angle + 180.00f;
		}
	}
	else
	{
		target_angle = set_angle;
	}
	return target_angle;
}


