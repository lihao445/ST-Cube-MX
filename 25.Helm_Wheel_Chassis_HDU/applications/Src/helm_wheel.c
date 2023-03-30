/***      �� ���ֽǶȼ��� ��

 *      ������       ������ + +
 *   �������� �ة��������������� �ة�����++
 *   ��                 ��
 *   ��       ������       ��++ + + +
 *   ������������������������������������+
 *   ��                 ��+
 *   ��      ���ة�        ��
 *   ��                 ��
 *   ����������         ����������
 *       ��         ��
 *       ��         ��   + +
 *       ��         ��
 *       ��         ��������������������������������
 *       ��                        ��
 *       ��                        ������
 *       ��                        ������
 *       ��                        ��
 *       ������  ��  �����������������Щ�����  ��������  + + + +
 *         �� ���� ����       �� ���� ����
 *         �������ة�����       �������ة�����  + + + +
 *
 *
 */
#include "helm_wheel.h"

eChassisAction actChassis = CHASSIS_NORMAL; //����Ĭ��ң������

extern RC_ctrl_t rc_ctrl;

Chassis_Speed_t absolute_chassis_speed;

extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];

fp32 Angle_Helm_Target[4];
fp32 Speed_Motor_Target[4];

fp32 M2006_Target[4];
fp32 M3508_Target[4];

/**
 * @brief  �趨ң�������Ƶ���ģʽ
 * @param  void
 * @retval void
 * @attention
 */
void Remote_Control_Chassis_Set_Mode(void)
{
    if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 3) //��������ģʽ
    {
        actChassis = CHASSIS_NORMAL;
    }
    else if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 1) //����С����ģʽ
    {
        actChassis = CHASSIS_GYROSCOPE;
    }
}

/**
 * @brief  ң�������Ʒ�ʽ
 * @param  void
 * @retval void
 * @attention
 */
void Remote_Control_Chassis_Mode(void)
{
    /***********************************ȷ�������ĸ������Ŀ���ٶ�*****************************************/
    switch (actChassis)
    {
    case CHASSIS_NORMAL: //����ģʽ
        absolute_chassis_speed.vx = (fp32)rc_ctrl.rc.ch[2] / 300;
        absolute_chassis_speed.vy = (fp32)rc_ctrl.rc.ch[3] / 300;
        absolute_chassis_speed.vw = (fp32)rc_ctrl.rc.ch[4] / 300;
        break;
    case CHASSIS_GYROSCOPE: //С����ģʽ
        absolute_chassis_speed.vx = (fp32)rc_ctrl.rc.ch[2] / 300;
        absolute_chassis_speed.vy = (fp32)rc_ctrl.rc.ch[3] / 300;
        absolute_chassis_speed.vw = -2;
        break;
    default:
        break;
    }
}

/**
 * @brief  ���̵�����
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
 * @brief  ����������������Ŀ���ٶ�
 * @param  speed ����������ٶ�
 * @param  out_speed 3508Ŀ���ٶ�
 * @retval void
 * @attention
 */
int8_t drct = 1;               //���������������ת
fp32 WHEEL_PERIMETER = 1;      //�����ܳ�
fp32 CHASSIS_DECELE_RATIO = 1; //���̼���������
fp32 Radius = 1;               //�뾶
void helm_wheel_speed_calc(Chassis_Speed_t *speed, fp32 *out_speed)
{
    // 3508Ŀ���ٶȼ���
    fp32 helm_wheel_rpm[5];
    fp32 wheel_rpm_ratio; //�����ٶȱ���

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;

    helm_wheel_rpm[0] = sqrt(pow(speed->vy + speed->vw * Radius * 0.707107f, 2) + pow(speed->vx - speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    helm_wheel_rpm[1] = sqrt(pow(speed->vy - speed->vw * Radius * 0.707107f, 2) + pow(speed->vx - speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    helm_wheel_rpm[2] = sqrt(pow(speed->vy - speed->vw * Radius * 0.707107f, 2) + pow(speed->vx + speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    helm_wheel_rpm[3] = sqrt(pow(speed->vy + speed->vw * Radius * 0.707107f, 2) + pow(speed->vx + speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;

    for (int i = 0; i < 4; i++)
    {
        out_speed[i] = drct * helm_wheel_rpm[i];
    }
}

/**
 * @brief  ������̺�������Ŀ��Ƕ�
 * @param  speed ����������ٶ�
 * @param  out_angle 2006Ŀ��Ƕ�
 * @retval void
 * @attention
 */
bool_t flag = 1;
void helm_wheel_angle_calc(Chassis_Speed_t *speed, fp32 *out_angle)
{
    int16_t angle_temp;
    fp32 helm_wheel_angle[4]; // 2006������Ŀ��Ƕ�
    fp32 helm_wheel_angle_last[4];
    fp64 atan_angle[4];

    // 2006Ŀ��Ƕȼ���
    if (!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0)) //��ֹ����Ϊ��
    {
        atan_angle[0] = atan2((speed->vx - speed->vw * Radius * 0.707107f), (speed->vy + speed->vw * Radius * 0.707107f)) * 180.0f / PI;
        atan_angle[1] = atan2((speed->vx - speed->vw * Radius * 0.707107f), (speed->vy - speed->vw * Radius * 0.707107f)) * 180.0f / PI;
        atan_angle[2] = atan2((speed->vx + speed->vw * Radius * 0.707107f), (speed->vy - speed->vw * Radius * 0.707107f)) * 180.0f / PI;
        atan_angle[3] = atan2((speed->vx + speed->vw * Radius * 0.707107f), (speed->vy + speed->vw * Radius * 0.707107f)) * 180.0f / PI;
    }

    helm_wheel_angle[0] = (fp32)(atan_angle[0] * 22.75);
    helm_wheel_angle[1] = (fp32)(atan_angle[1] * 22.75);
    helm_wheel_angle[2] = (fp32)(atan_angle[2] * 22.75);
    helm_wheel_angle[3] = (fp32)(atan_angle[3] * 22.75);

    AngleLoop_f(&helm_wheel_angle[0], 8192*36); //�����ͽǶȻػ�
    AngleLoop_f(&helm_wheel_angle[1], 8192*36);
    AngleLoop_f(&helm_wheel_angle[2], 8192*36);
    AngleLoop_f(&helm_wheel_angle[3], 8192*36);

    switch (actChassis)
    {
    case CHASSIS_NORMAL:
    {
        angle_temp = motor_can2[0].angle;
        AngleLoop_int((int32_t *)&angle_temp, 8192*36); //���ͽǶȻػ�
        if (fabs(Find_min_Angle(angle_temp, helm_wheel_angle[1])) > 2048*36)
        {
            for (int i = 1; i < 5; i++)
                helm_wheel_angle[i] += 4096*36;
            if (flag) //���������ת ʹ����ִ��ʱֻ��ȡ��һ��
            {
                drct = -drct;
                flag = 0;
            }
        }
        else
            flag = 1;

        if ((speed->vx == 0 || speed->vy == 0) && speed->vw == 0)
        {
            flag = 1;
            drct = 1;
        }

        AngleLoop_f(&helm_wheel_angle[0], 8192*36); //�ٴνǶȻػ�,��Ϊ�ӹ�4096
        AngleLoop_f(&helm_wheel_angle[1], 8192*36);
        AngleLoop_f(&helm_wheel_angle[2], 8192*36);
        AngleLoop_f(&helm_wheel_angle[3], 8192*36);
        break;
    }

    case CHASSIS_GYROSCOPE:
    {
        //��Ҫ�ټ�
        break;
    }
    default:
    {
        break;
    }
    }
    if (speed->vx == 0 && speed->vy == 0 && speed->vw == 0) //ҡ�˻���ʱ
    {
        for (int i = 0; i < 4; i++) // memcpy��������
            out_angle[i] = helm_wheel_angle_last[i];
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            out_angle[i] = helm_wheel_angle[i];
            helm_wheel_angle_last[i] = helm_wheel_angle[i];
        }
    }
}

/**
 * @brief  �ǶȻػ� ����
 * @param  angle �Ƕ�ֵ
 * @param  max ���Ƕ�ֵ
 * @retval void
 * @attention
 */
void AngleLoop_f(fp32 *angle, fp32 max)
{
    while ((*angle < -(max / 2)) || (*angle > (max / 2)))
    {
        if (*angle < -(max / 2))
        {
            *angle += max;
        }
        else if (*angle > (max / 2))
        {
            *angle -= max;
        }
    }
}

/**
 * @brief  �ǶȻػ� ����
 * @param  angle �Ƕ�ֵ
 * @param  max ���Ƕ�ֵ
 * @retval void
 * @attention
 */
void AngleLoop_int(int32_t *angle, int32_t max)
{
    while ((*angle < -(max / 2)) || (*angle > (max / 2)))
    {
        if (*angle < -(max / 2))
        {
            *angle += max;
        }
        else if (*angle > (max / 2))
        {
            *angle -= max;
        }
    }
}


/**
  * @brief  �ҳ����ǵĽ�С��ֵ
  * @param  ��1����2
  * @retval ���ǵĽ�С��ֵ
  * @attention 
  */
fp32 Find_min_Angle(int32_t angle1,fp32 angle2)
{
	  fp32 err;
    err = (fp32)angle1 - angle2;
    if(fabs(err) > 4096*36)
    {
        err = 8192*36 - fabs(err);
    }
    return err;
}
