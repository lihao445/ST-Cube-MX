//ǰ��"pitchyaw.h"�˽����
#include "pitchyaw.h"


Motor_C610 Turnplate[1] = {Motor_C610(1)};//���̵��
Motor_C620 FriMotor[4] = {Motor_C620(2),Motor_C620(3),Motor_C620(4),Motor_C620(7)};

Motor_GM6020 GrimalMotor[2] = {Motor_GM6020(1),Motor_GM6020(2)};//0Ϊyaw��1Ϊpitch



/*������*/



myPID Yaw[2];
myPID Pitch[2];



float calcangle_yaw(float now_imu_data , int8_t status)
{
	static int16_t round_cnt;
	static float last_imu_yaw;
	if(now_imu_data - last_imu_yaw > 180) round_cnt --;
	else if(now_imu_data - last_imu_yaw < -180) round_cnt ++;
	last_imu_yaw = now_imu_data;
	if(status == 0)round_cnt = 0;//Ȧ������
	return (now_imu_data + round_cnt*360);
}



float calcangle_pitch(float now_imu_data , int8_t status)
{
	static int16_t round_cnt;
	static float last_imu_yaw;
	if(now_imu_data - last_imu_yaw > 180) round_cnt --;
	else if(now_imu_data - last_imu_yaw < -180) round_cnt ++;
	last_imu_yaw = now_imu_data;
	if(status == 0)round_cnt = 0;//Ȧ������
	return (now_imu_data + round_cnt*360);
}



/*���Բ��ֽ���*/



gimbal::gimbal()//�޸ĳ�ʼ������
{
	pitchMPU_total_angle = 0;
	yawMPU_total_angle = 0;
	pitchSpeedOffset = 46;
	yawSpeedOffset = 22;
	pitch_scale = 140;
	yaw_scale = 5;
	sendMassageClear();
	pitchAngle.Target = 0;
	pitchAngle.SetPIDParam(10,0, 0, 0, 30000);
	pitchSpeed.SetPIDParam(5,15, 0, 10000, 30000);
//	pitchAngle.SetPIDParam(0,0, 0, 0, 30000);
//	pitchSpeed.SetPIDParam(0,0, 0, 10000, 30000);
	pitchSpeed.I_SeparThresh = 1200;
	yawAngle.Target = 0;
	yawAngle.SetPIDParam(-150,0, 0, 0, 10000);//-100��-180
	yawSpeed.SetPIDParam(40,1400, 0, 10000, 30000);
//	yawAngle.SetPIDParam(0,0, 0, 0, 10000);//-100��-180
//	yawSpeed.SetPIDParam(0,0, 0, 10000, 30000);
}



//pitch��
//��ڲ����ֱ�Ϊ��ǰ�����ǻ�ȡ�ĽǶȡ�״̬���ò�����״̬���ò������ڽ�Ȧ����0
void gimbal::calculatePitchIMU(float now_imu_data , int8_t status , uint8_t opposite)
{
	static int16_t round_cnt = 0;
	static float last_imu_yaw = 3300;
	if(now_imu_data - last_imu_yaw > 4096) round_cnt --;
	else if(now_imu_data - last_imu_yaw < -4096) round_cnt ++;
	last_imu_yaw = now_imu_data;
	if(status == 0)round_cnt = 0;//Ȧ������
	if(opposite == 0)
	{
		pitchMPU_total_angle = now_imu_data + round_cnt*8192;
	}
	else if(opposite == 1)
	{
		pitchMPU_total_angle = -now_imu_data - round_cnt*8192;
	}
}



//yaw��
//��ڲ����ֱ�Ϊ��ǰ�����ǻ�ȡ�ĽǶȡ�״̬���ò�����״̬���ò������ڽ�Ȧ����0
void gimbal::calculateYawIMU(float now_imu_data , int8_t status , uint8_t opposite)
{
	static int16_t round_cnt = 0;
	static float last_imu_yaw = 0;
	if(now_imu_data - last_imu_yaw > 180) round_cnt --;
	else if(now_imu_data - last_imu_yaw < -180) round_cnt ++;
	last_imu_yaw = now_imu_data;
	if(status == 0)round_cnt = 0;//Ȧ������
	if(opposite == 0)
	{
		yawMPU_total_angle = now_imu_data + round_cnt*360;
	}
	else if(opposite == 1)
	{
		yawMPU_total_angle = -now_imu_data - round_cnt*360;
	}
}



//��ȡpitch�ܽǶ�
float gimbal::getPitchIMU()
{
	return pitchMPU_total_angle;
}



//��ȡyaw�ܽǶ�
float gimbal::getYawIMU()
{
	return yawMPU_total_angle;
}



//��ȡpitchң�ر���
float gimbal::getPitchScale()
{
	return pitch_scale;
}



//��ȡyawң�ر���
float gimbal::getYawScale()
{
	return yaw_scale;
}



//������������
void gimbal::sendMassageClear()
{
	gimbalctrl[0] = 0;
	gimbalctrl[1] = 0;
	gimbalctrl[2] = 0;
	gimbalctrl[3] = 0;
	gimbalctrl[4] = 0;
	gimbalctrl[5] = 0;
	gimbalctrl[6] = 0;
	gimbalctrl[7] = 0;
}



//�����������
void gimbal::sendMassageWrite(int16_t current_control1, int16_t current_control2, int16_t current_control3, int16_t current_control4)
{
	gimbalctrl[0] = (int8_t)(current_control1 >> 8);
	gimbalctrl[1] = current_control1;
	gimbalctrl[2] = (int8_t)(current_control2 >> 8);
	gimbalctrl[3] = current_control2;
	gimbalctrl[4] = (int8_t)(current_control3 >> 8);
	gimbalctrl[5] = current_control3;
	gimbalctrl[6] = (int8_t)(current_control4 >> 8);
	gimbalctrl[7] = current_control4;
}



//pitch����
void gimbal::pitchContralCalculate(float* encoder, short* speed)//������
{
	pitchAngle.Current = *encoder;
	pitchAngle.Adjust();
	pitchSpeed.Target = pitchAngle.Out;
	pitchSpeed.Current = *speed + pitchSpeedOffset;
	pitchSpeed.Adjust();
}



//yaw����
void gimbal::yawContralCalculate(float* encoder, short* speed)//������
{
	yawAngle.Current = *encoder;
	yawAngle.Adjust();
	yawSpeed.Target = yawAngle.Out;
	yawSpeed.Current = -*speed - yawSpeedOffset; 
	yawSpeed.Adjust();
}



//��̨����
void gimbal::gimbalControl(Motor_GM6020* pitchMotor, MPUData_Typedef* MPUDateIn, uint8_t controlStatus)
{
	int16_t pitchout;
	int16_t yawout;
	if(controlStatus == on)
	{
		float pitchMotorencoder = pitchMotor->getencoder();
		calculatePitchIMU(pitchMotorencoder,1,0);
		calculateYawIMU(MPUData.yaw,1,1);
		pitchContralCalculate(&pitchMPU_total_angle,&MPUData.gy);//pitch��pid����
		pitchout = (int16_t)pitchSpeed.Out;//pitch���
		yawContralCalculate(&yawMPU_total_angle,&MPUData.gz);//yaw��pid����
		yawout = (int16_t)yawSpeed.Out;//yaw���
	}
	else if(controlStatus == off)
	{
		float pitchMotorencoder = pitchMotor->getencoder();
		calculatePitchIMU(pitchMotorencoder,0,0);
		calculateYawIMU(MPUData.yaw,0,1);
		pitchAngle.Target = pitchMPU_total_angle;
		yawAngle.Target = yawMPU_total_angle;
		pitchout = 0;
		yawout = 0;
	}
	sendMassageWrite(0,pitchout,0,0);//can�����������
	CANx_SendData(&hcan1,0x1ff,gimbalctrl,8);
	sendMassageWrite(yawout,0,0,0);//can�����������
	CANx_SendData(&hcan2,0x1ff,gimbalctrl,8);
}



//�Զ�������̨��ʼ��
auto_gimbal::auto_gimbal()
{
	
}



//