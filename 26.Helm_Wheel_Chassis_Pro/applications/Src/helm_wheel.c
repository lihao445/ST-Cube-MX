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
    else if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 1) //���̴�����ģʽ   ����̨�޷�ʵ��
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
        absolute_chassis_speed.vx = (fp32)rc_ctrl.rc.ch[2];
        absolute_chassis_speed.vy = (fp32)rc_ctrl.rc.ch[3];
        absolute_chassis_speed.vw = (fp32)rc_ctrl.rc.ch[4];
        break;
    case CHASSIS_GYROSCOPE: //С����ģʽ
        absolute_chassis_speed.vx = (fp32)rc_ctrl.rc.ch[2];
        absolute_chassis_speed.vy = (fp32)rc_ctrl.rc.ch[3];
        absolute_chassis_speed.vw = 330.0f;
        break;
    default:
        break;
    }
}

/**
 * @brief  ����������������Ŀ���ٶ�
 * @param  speed ����������ٶ�
 * @param  out_speed 3508Ŀ���ٶ�
 * @retval void
 * @attention
 */
int8_t dir = -1; //���������������ת
// fp32 WHEEL_PERIMETER = 1;      //�����ܳ�
// fp32 CHASSIS_DECELE_RATIO = 1; //���̼���������
fp32 Radius = 1; //�뾶
void helm_wheel_speed_calc(Chassis_Speed_t *speed, fp32 *out_speed)
{
    // 3508Ŀ���ٶȼ���
    fp32 helm_wheel_rpm[4];
    fp32 wheel_rpm_ratio; //�����ٶȱ���

    //    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;
    wheel_rpm_ratio = 1;

    helm_wheel_rpm[0] = sqrt(pow(speed->vy - speed->vw * Radius * 0.707107f, 2) + pow(speed->vx + speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio; //��η���ƽ����
    helm_wheel_rpm[1] = sqrt(pow(speed->vy - speed->vw * Radius * 0.707107f, 2) + pow(speed->vx - speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    helm_wheel_rpm[2] = sqrt(pow(speed->vy - speed->vw * Radius * 0.707107f, 2) + pow(speed->vx + speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    helm_wheel_rpm[3] = sqrt(pow(speed->vy + speed->vw * Radius * 0.707107f, 2) + pow(speed->vx + speed->vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;

    for (int i = 0; i < 4; i++)
    {
        out_speed[i] = dir * helm_wheel_rpm[i] * 5;
    }
}

/**
 * @brief  ������̺�������Ŀ��Ƕ�
 * @param  speed ����������ٶ�
 * @param  out_angle 2006Ŀ��Ƕ�
 * @retval void
 * @attention
 */
fp32 temp_x[4] = {0, 0, 0, 0}, temp_y[4] = {0, 0, 0, 0};
fp32 helm_wheel_angle[4] = {0, 0, 0, 0}; // 2006������Ŀ��Ƕ�
fp32 helm_wheel_angle_temp[4] = {0, 0, 0, 0};
fp32 helm_wheel_angle_last[4] = {0, 0, 0, 0};
fp32 helm_atan_angle[4] = {0, 0, 0, 0}; //�Ǿ�̬��������ʱ������������޸�Ϊvolatile fp32 atan_angle[4];
fp32 helm_round_cnt = 0;;
void helm_wheel_angle_calc(Chassis_Speed_t *speed, fp32 *out_angle)
{
    // 2006Ŀ��Ƕȼ���
    if (!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0)) //��ֹ����Ϊ��
    {
        temp_x[0] = speed->vx + speed->vw * Radius * 0.707107f;
        temp_y[0] = speed->vy + speed->vw * Radius * 0.707107f;
        temp_x[1] = speed->vx - speed->vw * Radius * 0.707107f;
        temp_y[1] = speed->vy + speed->vw * Radius * 0.707107f;
        temp_x[2] = speed->vx + speed->vw * Radius * 0.707107f;
        temp_y[2] = speed->vy + speed->vw * Radius * 0.707107f;
        temp_x[3] = speed->vx - speed->vw * Radius * 0.707107f;
        temp_y[3] = speed->vy + speed->vw * Radius * 0.707107f;

        for (int i = 0; i < 4; i++)
        {
            helm_atan_angle[i] = atan2_angle_calc(temp_x[i], temp_y[i]);
            helm_atan_angle[i] = rad2deg(helm_atan_angle[i]);
        }
    }
    calc_min_angle_round(helm_atan_angle, helm_wheel_angle_last, helm_wheel_angle_temp, &helm_round_cnt, &dir);

    if (speed->vx == 0 && speed->vy == 0 && speed->vw == 0) //ҡ�˻���ʱ
    {
        for (int i = 0; i < 4; i++) // memcpy��������
                                    //            out_angle[i] = 0; = helm_wheel_angle_last[i];
            out_angle[i] = 0;
    }
    else //������ʱ
    {
        for (int i = 0; i < 4; i++)
        {
            out_angle[i] = (helm_wheel_angle[i] / 360.0f * 8191.0f) * 36.0f * (67.0f / 20.0f);
            //            helm_wheel_angle_last[i] = out_angle[i];
        }
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
    M3508_Target[0] = PID_velocity_realize_1(Speed_Motor_Target[0], 1);
    M3508_Target[1] = PID_velocity_realize_1(Speed_Motor_Target[1], 2);
    M3508_Target[2] = PID_velocity_realize_1(Speed_Motor_Target[2], 3);
    M3508_Target[3] = PID_velocity_realize_1(Speed_Motor_Target[3], 4);

    M2006_Target[0] = pid_call_2(Angle_Helm_Target[0], 1);
    M2006_Target[1] = pid_call_2(Angle_Helm_Target[1], 2);
    M2006_Target[2] = pid_call_2(Angle_Helm_Target[2], 3);
    M2006_Target[3] = pid_call_2(Angle_Helm_Target[3], 4);

    CAN1_CMD_1(M3508_Target[0], M3508_Target[1], M3508_Target[2], M3508_Target[3]);
    CAN2_CMD_1(M2006_Target[0], M2006_Target[1], M2006_Target[2], M2006_Target[3]);
}

/**
 * @brief  �����ң�˵������ĽǶ�ֵ
 * @param  x x������ĵ�ַ
 * @param  y y������ĵ�ַ
 * @retval temp �����ĽǶ�
 * @attention
 */
fp32 atan2_angle_calc(fp32 x, fp32 y)
{
    volatile fp32 temp;
    temp = atan2(-x, y); //���������������
                         //    if(y >= 0)
                         //    {
                         //        temp = atan2(x,y);
                         //    }
                         //    else if(y < 0)
                         //    {
                         //        temp = atan2(x,-y);
                         //    }
    return temp;
}

/**
 * @brief  �Ż�����ת���Ƕȣ��Ż��ͽ�ԭ��
 * @param  set_angle ����Ҫת���ĽǶ�
 * @param  last_angle ��һ�εĽǶ�
 * @retval target_angle �����ȷ�ͽ��Ƕ�ֵ
 * @attention
 */
void calc_min_angle_round(fp32 *set_angle, fp32 *previous_angle, fp32 *angle_temp, fp32 *round_cnt, int8_t *direction_coefficient)
{
    for (int i = 0; i < 4; i++)
    {
        static fp32 round1 = 0.0f, round2 = 0.0f;
        /*****************���־ͽ�ԭ��*******************/
        if (fabs(set_angle[i] - previous_angle[i]) > 90.0f)
        {
            if (90.0f <= set_angle[i] && set_angle[i] <= 181.0f)
            {
                angle_temp[i] = set_angle[i] - 180.0f;
            }
            if (-181.0f <= set_angle[i] && set_angle[i] <= -90.0f)
            {
                angle_temp[i] = set_angle[i] + 180.0f;
            }
        }
        else
        {
            angle_temp[i] = set_angle[i];
        }

        /*****************���ּ�Ȧ*******************/
        if ((set_angle[i] + angle_temp[i]) - previous_angle[i] > 260)
        {
            
						(*round_cnt)--;
        }
        else if ((set_angle[i] + angle_temp[i]) - previous_angle[i] < -260)
        {
            (*round_cnt)++;
        }

        /*****************���ּ�Ȧ����*******************/
        round1 = fabs(previous_angle[i] - angle_temp[i]);
        round2 = fabs(set_angle[i] - previous_angle[i]);

        if (round1 >= round2)
        {
            angle_temp[i] = 0.00f;
            *direction_coefficient = 1.00f;
        }
        if (round1 < round2)
        {
            set_angle[i] = 0.00f;
            *direction_coefficient = -1.00f;
        }
        previous_angle[i] = set_angle[i] + angle_temp[i];
    }
}

/**
 * @brief  �Ƕ�ת��Ϊ����
 * @param  deg �Ƕ�ֵ
 * @retval rad ����ֵ
 * @attention
 */
fp32 deg2rad(fp32 deg)
{
    fp32 rad;
    rad = deg * (PI / 180.0f);
    return rad;
}

/**
 * @brief  ����ת��Ϊ�Ƕ�
 * @param  rad ����ֵ
 * @retval deg �Ƕ�ֵ
 * @attention
 */
fp32 rad2deg(fp32 rad)
{
    fp32 deg;
    deg = rad * (180.0f / PI);
    return deg;
}

/**
 * @brief  �Ż�����ת���Ƕȣ��ͽ�ԭ��
 * @param  set_angle ����Ҫת���ĽǶ�
 * @param  last_angle ��һ�εĽǶ�
 * @retval target_angle �����ȷ�ͽ��Ƕ�ֵ
 * @attention
 */
/* void calc_min_angle(fp32* set_angle,fp32* last_angle,fp32* target_angle)
{
    fp32 temp_angle;
    if(*set_angle > 91.0f)
    {
        temp_angle = *set_angle - 180.0f;
    }
    else if(*set_angle < -89.0f)
    {
        temp_angle = *set_angle + 180.0f;
    }
    else
    {
        temp_angle = *set_angle;
    }

    if(fabs(temp_angle - *last_angle) > 90.0f)
    {
        if(-89.0f < temp_angle && temp_angle < 1.0f)
        {
            *target_angle = temp_angle + 180.0f;
        }
        else if(1.0f < temp_angle < 91.0f)
        {
            *target_angle = temp_angle -180.0f;
        }
    }
    else
    {
        *target_angle = temp_angle;
    }
} */
