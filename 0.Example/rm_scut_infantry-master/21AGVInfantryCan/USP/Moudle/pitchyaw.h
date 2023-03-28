#ifndef PITCH_YAW_H
#define PITCH_YAW_H

#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "motor.h"
#include "mpu6050_config.h"

#define on 1
#define off 0

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern MPUData_Typedef MPUData;
/*pitchyaw_test below*/
extern Motor_GM6020 GrimalMotor[2];

extern myPID Yaw[2];//�����ã����������Ϊauto_gimbal��
extern myPID Pitch[2];//�����ã����������Ϊauto_gimbal��

extern float calcangle_yaw(float now_imu_data , int8_t status);//�����ã����������Ϊauto_gimbal��
extern float calcangle_pitch(float now_imu_data , int8_t status);//�����ã����������Ϊauto_gimbal��

/*pitchyaw_test above*/

extern Motor_C610 Turnplate[1];//���̵��
extern Motor_C620 FriMotor[4];



class gimbal//������̨�������Խ�����ʹ��
{
	public:
		gimbal();//��ʼ��
		void calculatePitchIMU(float, int8_t, uint8_t);//pitch��360���ƽǶȴ�Ȧ������
		void calculateYawIMU(float, int8_t, uint8_t);//yaw��360���ƽǶȴ�Ȧ������
		float getPitchIMU();//��ȡ360����pitch���Ȧ������
		float getYawIMU();//��ȡ360����yaw���Ȧ������
		float getPitchScale();//��ȡpitch��ң�ر���
		float getYawScale();//��ȡyaw��ң�ر���
		myPID pitchAngle,pitchSpeed,yawAngle,yawSpeed;//SRML��pid
		void targetGet(uint8_t);//Ŀ��ֵ��ȡ
		void pitchContralCalculate(float*, short*);//pitch��pid����
		void yawContralCalculate(float*, short*);//yaw��pid����
		void gimbalControl(Motor_GM6020*, MPUData_Typedef*, uint8_t);//��̨����
		void sendMassageClear();//can������������
		void sendMassageWrite(int16_t, int16_t, int16_t, int16_t);//can�����������
	protected:
		float pitchMPU_total_angle;//360����pitch���Ȧ������
		float yawMPU_total_angle;//360����yaw���Ȧ������
		float pitchSpeedOffset;
		float yawSpeedOffset;
		float pitch_scale;//pitch��ң�����ʱ���
		float yaw_scale;//yaw��ң�����ʱ���
		uint8_t gimbalctrl[8];//can�������ݴ洢
		/*�����˽�г�Ա*/
};

class auto_gimbal:public gimbal//�Զ�������̨������һ��yaw��
{
	public:
		auto_gimbal();
		void calculateYawBelowIMU(float, int8_t, uint8_t);//yaw��360���ƽǶȴ�Ȧ������
		float getBelowYawIMU();//��ȡ360����yaw���Ȧ������
		float getBelowYawScale();//��ȡyaw��ң�ر���
		myPID yawBelowAngle,yawBelowSpeed;
		void targetBelowGet(uint8_t);//Ŀ��ֵ��ȡ
		void yawBelowControlCalculate();//��yaw��pid����
		void autoGimbalControl(Motor_GM6020*, uint8_t);//��̨����
	protected:
		float yawMPU_below_total_angle;//360������yaw���Ȧ������
		float yaw_below_scale;//��yaw��ң�����ʱ���
};

#endif
