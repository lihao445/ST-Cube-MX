#ifndef AGV_CHASSIS_H
#define AGV_CHASSIS_H

#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "motor.h"
#include "math.h"
#include "autoInfantryConfig.h"



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Motor_GM6020 SteerMotor[4];
extern Motor_C620 Wheel[4];

class Chassis_AGV
{
	public:
		//����
		//��ʼ��
		Chassis_AGV(); 
		//��������Ʋ���
		void setMaxSpeed(int16_t);                    												//��������ٶ�
		void setMaxPower(int16_t);                    												//���������
		void speedChange();                           												//�ٶȽ���
		void speedCalculate();																								//�ٶȴ���
	  void AGVControl();                            												//���̿���
		//���ܺ�������
	  float abs(float);                             												//ȡ����ֵ
		void writeMovingTarget(uint8_t*);																		  //��ȡĿ���ٶ�
		void writeResetMessage(uint8_t*);																			//��ȡ������־
		void sendMassageClear(uint8_t*);																			//can������������
		void sendMassageWrite(uint8_t*, int16_t, int16_t, int16_t, int16_t);	//can�����������
		void movingContralCalculate(float, int8_t);
		void writeRefereemsg(referee_Classdef *, uint8_t *);
//		void checkLink();
		//����
		//���ͨ�Ż�ȡ�ı���
		uint8_t write_state1[8];//can����7λ(receive[6])�õ��ı�־λ
		uint8_t write_state2[8];//can����8λ(receive[7])�õ��ı�־λ
		uint8_t sendRefereeMsg[8];
		uint8_t state1,state2,state_all;//can�������ܴ�С
		float receiveMovingTarget[3];
		float movingTarget[3];
		uint8_t linkState;
		//�ֿ���
		myPID movingMotor[4];//����pid
		float movingSpeedTarget[4];
		float powerSpeedScale;
		uint8_t ctrl_3508_message[8];		//can���������ݴ洢
		uint8_t ctrl_can_message[8];		//can���Ͷ����ݴ洢
		float pi;
		float w_scale;
		//���ʿ���
		float motorPowerMax;
		float sourcePowerMax; 
		float maxSpeed[3];
		int motorOutput[4];
		float power_scale;
		uint8_t chassis_cap_states;
		uint8_t resetFlag;
//		/*�����˽�г�Ա*/
};

extern Chassis_AGV auto_infantry;

#endif

