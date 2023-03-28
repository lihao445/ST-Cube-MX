#include "AGV_chassis.h"



Chassis_AGV auto_infantry;


//���id�����и���
Motor_GM6020 SteerMotor[4] = {Motor_GM6020(1),Motor_GM6020(2),Motor_GM6020(3),Motor_GM6020(4)};
Motor_C620 Wheel[4] = {Motor_C620(1),Motor_C620(2),Motor_C620(3),Motor_C620(4)};


float Radius = 0.212;

void Chassis_AGV::AGVControl()
{
	
	speedCalculate();	//��̨�·���ǰ�����ҡ���ת�ٶ�
	speedChange();		//�������ٶ�ת��Ϊ�ĸ����ֵ����ٶ�
	
	//������pid���
	movingContralCalculate(Wheel[0].getSpeed(),0);	
	movingContralCalculate(Wheel[1].getSpeed(),1);
	movingContralCalculate(Wheel[2].getSpeed(),2);
	movingContralCalculate(Wheel[3].getSpeed(),3);
	
	//�洢�����ֵ
	motorOutput[0] = movingMotor[0].Out;
	motorOutput[1] = movingMotor[1].Out;
	motorOutput[2] = movingMotor[2].Out;
	motorOutput[3] = movingMotor[3].Out;
	
	
	//���õ�ǰ���ʼ����ݹ������õ�ǰ�ٶ�
	setMaxPower(Referee.GameRobotState.classis_power_limit);
	setMaxSpeed(motorPowerMax);
	
	//���ż��ٽӿڹ�������ʹ��
	if((write_state2[7] == 1)&&(SourceManage.capObj.Voltage >= 15))
	{
		motorPowerMax = 200;
	}
	else
	{
		
	}
	
	//�жϰ��ͨ���Ƿ�����
	linkState++;
	if(linkState>50)
	{
		setMaxSpeed(0);
	}
	
	//���ù��ʿ��Ʋ���
	PowerCtrl.Set_PE_Target(sourcePowerMax, motorPowerMax, 40);
	PowerCtrl.Control(
		SourceManage.power.pow_In,
		SourceManage.power.pow_motor,
		Referee.PowerHeatData.chassis_power_buffer,
		motorOutput);
	
	power_scale = PowerCtrl.Get_limScale();	//���ʿع�֮�������
	sendMassageWrite(
		ctrl_3508_message,
		(int16_t)(movingMotor[0].Out * power_scale),
		(int16_t)(movingMotor[1].Out * power_scale),
		(int16_t)(movingMotor[2].Out * power_scale),
		(int16_t)(movingMotor[3].Out * power_scale));

	CANx_SendData(&hcan2,0x200,ctrl_3508_message,8);	//������������
	
	sendMassageWrite(
		ctrl_can_message,
		(int16_t)(movingTarget[0]/10.0),
		(int16_t)(movingTarget[1]/10.0),
		(int16_t)(movingTarget[2]/10.0),
		(int16_t)(0));

CANx_SendData(&hcan2,0x112,ctrl_can_message,8);	//���������ٶȸ�����ư�
	
	writeRefereemsg(&Referee,sendRefereeMsg);	//����̨������Ҫ�Ĳ���ϵͳ����
}


Chassis_AGV::Chassis_AGV()//�޸ĳ�ʼ������
{
	
	
	movingTarget[0] = 0;
	movingTarget[1] = 0;
	movingTarget[2] = 0;
	
	power_scale = 1.0f;
	powerSpeedScale = 1.0f;
	pi = 3.14159265;
	w_scale = 4.7;
	resetFlag = 0;
	linkState = 0;
	
	movingSpeedTarget[0] = 0;
	movingSpeedTarget[1] = 0;
	movingSpeedTarget[2] = 0;
	movingSpeedTarget[3] = 0;
	
	write_state1[0] = false;
	write_state1[1] = false;
	write_state1[2] = false;
	write_state1[3] = false;
	write_state1[4] = false;
	write_state1[5] = false;
	write_state1[6] = false;
	write_state1[7] = false;
	
	write_state2[0] = false;
	write_state2[1] = false;
	write_state2[2] = false;
	write_state2[3] = false;
	write_state2[4] = false;
	write_state2[5] = false;
	write_state2[6] = false;
	write_state2[7] = false;
}





void Chassis_AGV::writeMovingTarget(uint8_t can_rx_data[])	//���ͨ�ŵõ��ٶ����ݼ������ְ������ݲ�������
{
	//Ŀ��ֵд��
	
	receiveMovingTarget[0] = ((int16_t)((can_rx_data[0]<<8)+can_rx_data[1]));//ǰ���ٶ�
	receiveMovingTarget[1] = ((int16_t)((can_rx_data[2]<<8)+can_rx_data[3]));//����ƽ���ٶ�
	receiveMovingTarget[2] = ((int16_t)((can_rx_data[4]<<8)+can_rx_data[5]));//��ת�ٶ�
	
	write_state1[0] = 1&(can_rx_data[6]>>7);
	write_state1[1] = 1&(can_rx_data[6]>>6);
	write_state1[2] = 1&(can_rx_data[6]>>5);
	write_state1[3] = 1&(can_rx_data[6]>>4);
	write_state1[4] = 1&(can_rx_data[6]>>3);
	write_state1[5] = 1&(can_rx_data[6]>>2);
	write_state1[6] = 1&(can_rx_data[6]>>1);
	write_state1[7] = 1&(can_rx_data[6]>>0);
	
	state1 = can_rx_data[6];

	write_state2[0] = 1&(can_rx_data[7]>>7);
	write_state2[1] = 1&(can_rx_data[7]>>6);
	write_state2[2] = 1&(can_rx_data[7]>>5);
	write_state2[3] = 1&(can_rx_data[7]>>4);
	write_state2[4] = 1&(can_rx_data[7]>>3);
	write_state2[5] = 1&(can_rx_data[7]>>2);
	write_state2[6] = 1&(can_rx_data[7]>>1);
	write_state2[7] = 1&(can_rx_data[7]>>0);
	
	state2 = can_rx_data[7];
	
	state_all = (can_rx_data[6] << 8) + can_rx_data[7];
	
	linkState = 0;

}





void Chassis_AGV::writeResetMessage(uint8_t can_rx_data[])
{
	resetFlag = 1;
	receiveMovingTarget[0] = 0;//ǰ���ٶ�
	receiveMovingTarget[1] = 0;//����ƽ���ٶ�
	receiveMovingTarget[2] = 0;//��ת�ٶ�
}





//
void Chassis_AGV::speedChange()	//�������ٽ���
{
	float theta = atan(1.0/1.0);

	movingSpeedTarget[0]	
	= sqrt(	pow(movingTarget[1] - movingTarget[2]*Radius*sin(theta),2)
				+	pow(movingTarget[0] - movingTarget[2]*Radius*cos(theta),2));
	
	movingSpeedTarget[1] 
	= sqrt(	pow(movingTarget[1] + movingTarget[2]*Radius*sin(theta),2)
				+	pow(movingTarget[0] - movingTarget[2]*Radius*cos(theta),2));      																																																																														
	
	movingSpeedTarget[2] 
	= sqrt(	pow(movingTarget[1] + movingTarget[2]*Radius*sin(theta),2)
				+	pow(movingTarget[0] + movingTarget[2]*Radius*cos(theta),2));
																																																																														
	movingSpeedTarget[3] 
	= sqrt(	pow(movingTarget[1] - movingTarget[2]*Radius*sin(theta),2)
				+	pow(movingTarget[0] + movingTarget[2]*Radius*cos(theta),2));
		
}




//
void Chassis_AGV::speedCalculate()	
{
	static uint8_t compare_flag = 1;
	movingTarget[0] = receiveMovingTarget[0] * maxSpeed[0] / 10000.0;
	movingTarget[1] = receiveMovingTarget[1] * maxSpeed[1] / 10000.0;
	movingTarget[2] = receiveMovingTarget[2] * maxSpeed[2] / 10000.0;
	if((sqrt(movingTarget[0]*movingTarget[0]+movingTarget[1]*movingTarget[1]))<1000)
	{
		if(abs(movingTarget[2]) >= 2000)
		{
			compare_flag = 1;
		}
		else if(abs(movingTarget[2])<2000)
		{
			if(abs(movingTarget[2])<1000)
			{
				movingTarget[2] = 0;
				compare_flag = 0;
			}
			else if(compare_flag == 0)
			{
				movingTarget[2] = 0;
			}
			else
			{}
		}
		else
		{}
	}
//	movingTarget[0] = 0;
//	movingTarget[1] = 0;
//	movingTarget[2] = 0;
}





//������������
void Chassis_AGV::sendMassageClear(uint8_t* sendMessage)
{
	*sendMessage = 0;
	*(sendMessage + 1) = 0;
	*(sendMessage + 2) = 0;
	*(sendMessage + 3) = 0;
	*(sendMessage + 4) = 0;
	*(sendMessage + 5) = 0;
	*(sendMessage + 6) = 0;
	*(sendMessage + 7) = 0;
}



//�����������
void Chassis_AGV::sendMassageWrite(uint8_t* sendMessage, int16_t current_control1, int16_t current_control2, int16_t current_control3, int16_t current_control4)
{
	*sendMessage = (int8_t)(current_control1 >> 8);
	*(sendMessage + 1) = current_control1;
	*(sendMessage + 2) = (int8_t)(current_control2 >> 8);;
	*(sendMessage + 3) = current_control2;
	*(sendMessage + 4) = (int8_t)(current_control3 >> 8);;
	*(sendMessage + 5) = current_control3;
	*(sendMessage + 6) = (int8_t)(current_control4 >> 8);;
	*(sendMessage + 7) = current_control4;
}









//��pid����
void Chassis_AGV::movingContralCalculate(float speed_now, int8_t motor_num)
{
	movingMotor[motor_num].Target = movingSpeedTarget[motor_num];
	movingMotor[motor_num].Current = speed_now;
	movingMotor[motor_num].Adjust();
}





//���ݹ�����������ٶ�
void Chassis_AGV :: setMaxSpeed(int16_t powerMax)
{
	if(powerMax<20)
	{
		maxSpeed[0] = 1000;
		maxSpeed[1] = 1000;
		maxSpeed[2] = 1000;
	}
	else if(powerMax > 200)
	{
		maxSpeed[0] = 9500;
		maxSpeed[1] = 9500;
		maxSpeed[2] = 9500 * w_scale;
	}
	else
	{
		maxSpeed[0] = powerMax * 30 + 2500;
		maxSpeed[1] = powerMax * 30 + 2500;
		maxSpeed[2] = powerMax * 30 + 2500;
		maxSpeed[0] *= powerSpeedScale;
		maxSpeed[1] *= powerSpeedScale;
		maxSpeed[2] *= powerSpeedScale * w_scale;
	}
	
}




//��ǰ����������ʲ���
void Chassis_AGV :: setMaxPower(int16_t powerMax)
{
	static uint8_t temp1,temp2;
	
	if(SourceManage.capObj.Voltage <= 15)	//�ͻرȽ�
	{
		temp1 = 0;
	}
	else if(SourceManage.capObj.Voltage >= 19)
	{
		temp1 = 1;
	}
	if(SourceManage.capObj.Voltage <= 19)	//�ͻرȽ�
	{
		temp2 = 0;
	}
	else if(SourceManage.capObj.Voltage >= 21)
	{
		temp2 = 1;
	}
	
	if((temp1 == 0)&&(temp2 == 0))
	{
		motorPowerMax = powerMax - 20;
		sourcePowerMax = powerMax;
		chassis_cap_states = 0x01;
	}
	else if((temp1 == 1)&&(temp2 == 0))
	{
		sourcePowerMax = powerMax ;	//�Ӳ���ϵͳ��ȡ��ǰ����
		motorPowerMax = sourcePowerMax;
		if(write_state2[1] == 1)
		{
			motorPowerMax = motorPowerMax + 40;
			if(write_state2[6] == 0)
			{
				motorPowerMax -= 20;
			}
		}
		else
		{
			motorPowerMax = sourcePowerMax*1.0f;
		}
		chassis_cap_states = 0x02;
	}
	else if((temp1 == 1)&&(temp2 == 1))
	{
		sourcePowerMax = powerMax ;	//�Ӳ���ϵͳ��ȡ��ǰ����
		motorPowerMax = sourcePowerMax + 20;
		if(write_state2[1] == 1)
		{
			motorPowerMax = motorPowerMax + 20;
		}
		else
		{}
		if(write_state2[6] == 0)
		{
			motorPowerMax -= 20;
		}
		chassis_cap_states = 0x03;
	}
	if(SourceManage.capObj.Voltage >= 23)
	{
		chassis_cap_states = 0x04;
	}
}





//
float Chassis_AGV::abs(float a)
{
	return a > 0 ? a : -a;
}



//����̨���Ͳ���ϵͳ����
void Chassis_AGV ::writeRefereemsg(referee_Classdef * ref, uint8_t * tran)	
{
	if((ref->GameRobotState.classis_power_limit > 200)||(ref->GameRobotState.classis_power_limit < 30))
	{
		tran[0] = 60;
	}
	else
	{
		tran[0] = ref->GameRobotState.classis_power_limit;
	}
	tran[1] = ref->GameRobotState.shooter_id1_17mm_speed_limit;
	tran[2] = ref->GameRobotState.shooter_id1_17mm_cooling_rate>>8;
	tran[3] = ref->GameRobotState.shooter_id1_17mm_cooling_rate;
	tran[4] = ref->GameRobotState.shooter_id1_17mm_cooling_limit>>8;
	tran[5] = ref->GameRobotState.shooter_id1_17mm_cooling_limit;
	tran[6] = ref->PowerHeatData.shooter_id1_17mm_cooling_heat>>8;
	tran[7] = ref->PowerHeatData.shooter_id1_17mm_cooling_heat;
	CANx_SendData(&hcan1,0x221,tran,8);
	float bu_speed = ref->ShootData.bullet_speed;
	tran = (uint8_t *)(&bu_speed);
	tran[4] = ref->GameRobotState.robot_id;
	tran[5] = 0;
	tran[5] = chassis_cap_states;
	CANx_SendData(&hcan1,0x220,tran,8);
}
