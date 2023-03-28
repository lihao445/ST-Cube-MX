#ifndef CHASSIS_AGV_H
#define CHASSIS_AGV_H

#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "motor.h"
#include "math.h"
#include "referee.h"
#define angleCtrl 0
#define speedCtrl 1


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*pitchyaw_test below*/
extern Motor_GM6020 SteerMotor[4];
extern Motor_C620 Wheel[4];


/*pitchyaw_test above*/
extern referee_Classdef Referee;



class Chassis_AGV
{
	public:
		Chassis_AGV();//��ʼ��
		void calculateRoundCnt(Motor_GM6020*, int8_t, int8_t, uint8_t);//���¶���Ȧ��
		void calculateTargetRoundCnt(int8_t motorNum , int8_t status , uint8_t opposite);
		float getSteeringAngle(int8_t);
		void writeMovingTarget(uint8_t can_rx_data[]);
		void writeResetMessage(uint8_t*);																			//��ȡ������־
		void writeMovingLimitScale(uint8_t can_rx_data[]);
		float getMovingSpeedTarget(int8_t);
		float getSteeringAngleTarget(int8_t);
		void AGVTargetTransmit();
		myPID movingMotor[4];
//		void streeingContralCalculate(float*, float*);
		void movingContralCalculate(float*);
	  
		void sendMassageClear(uint8_t*);//can������������
		void sendMassageWrite(uint8_t*, int16_t, int16_t, int16_t, int16_t);//can�����������
		
		void streeingContralCalculate(Motor_GM6020* stmotor, int8_t motor_num);
		
		
		void speedChange();
		void motor_6020_control();
		void AGVControl();
		float abs(float);
		//���ͨ�Ż�ȡ�ı���
		uint8_t write_state1[8];//can����7λ(receive[6])�õ��ı�־λ
		uint8_t write_state2[8];//can����8λ(receive[7])�õ��ı�־λ
		uint8_t state1,state2,state_all;//can�������ܴ�С
		float receiveMovingTarget[3];
		float movingTarget[3];
		
		//�����
		myPID steeringMotor[4][2];//����pid
		float steeringAngleTarget[4];
		float lastSteeringAnglearget[4];
		float steeringAngleOffset[4];
		float steeringAngle[4];
		int32_t motor_count[4];
		int32_t motor_target_count[4];
		int32_t turnFlag[4];
		uint8_t ctrl_6020_message[8];		//can���Ͷ����ݴ洢
		uint8_t ctrl_duoji[1];
		float pi;
		uint8_t resetFlag;
		/*�����˽�г�Ա*/
};

extern Chassis_AGV auto_infantry;

#endif

