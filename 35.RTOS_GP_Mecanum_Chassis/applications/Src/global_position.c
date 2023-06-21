#include "global_position.h"

extern Chassis_measure_t absolute_chassis_measure;
extern HWT605_Modbus_t hwt605;

/**
 * @brief  ��õ���Ŀ��Yaw
 * @param  void
 * @retval void
 * @attention
 */
void Get_Chassis_Yaw_Target(void)
{
//	static fp32 RC_atan2_value = 0;
	static fp32 yaw_delta = 0;
	static fp32 control_ratio = 0.4f;   //�ٶ�ϵ��
	
	//��Ȧ���ƽǶ�(������)
//		RC_atan2_value = rad2deg(atan2(absolute_chassis_measure.Speed.vx,absolute_chassis_measure.Speed.vy));
	//��Ȧ���ƽǶ�
		yaw_delta = absolute_chassis_measure.Speed.vw / 660.0f * control_ratio;
		absolute_chassis_measure.Euler.yaw_target += yaw_delta;
}


/**
 * @brief  ��õ�������Yaw������������Yaw
 * @param  void
 * @retval void
 * @attention
 */
void Get_Chassis_Yaw_Offset(void)
{
		static bool_t imu_offset_flags = 1;
			if(imu_offset_flags == 1)
		{
			absolute_chassis_measure.Euler.yaw_offset = hwt605.Euler.yaw;
			imu_offset_flags = 0;
		}
		absolute_chassis_measure.Euler.yaw_last_angle = absolute_chassis_measure.Euler.yaw_rel;
		absolute_chassis_measure.Euler.yaw_rel = absolute_chassis_measure.Euler.yaw - absolute_chassis_measure.Euler.yaw_offset;
}


/**
 * @brief  ��õ�����Yaw
 * @param  void
 * @retval void
 * @attention  ����ʵ�ֶ�Ȧ
 */
void Get_Chassis_Total_Yaw(void)
{
		if(absolute_chassis_measure.Euler.yaw_rel - absolute_chassis_measure.Euler.yaw_last_angle < -300.0f)
		{
			absolute_chassis_measure.Euler.yaw_round_cnt++;
		}
		else if(absolute_chassis_measure.Euler.yaw_rel - absolute_chassis_measure.Euler.yaw_last_angle > 300.0f)
		{
			absolute_chassis_measure.Euler.yaw_round_cnt--;
		}
		absolute_chassis_measure.Euler.yaw_total = 360.0f * absolute_chassis_measure.Euler.yaw_round_cnt + absolute_chassis_measure.Euler.yaw_rel;
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

