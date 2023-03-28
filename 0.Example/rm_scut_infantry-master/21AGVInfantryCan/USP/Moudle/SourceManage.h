/**
******************************************************************************
* Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
* @file    SourceManage.h
* @author  ������ 15013073869
* @brief   Header file of SourceManage.
******************************************************************************
* @attention
* 
* if you had modified this file, please make sure your code does not have many 
* bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding 
* through your new brief.
*
* <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
* All rights reserved.</center></h2>
******************************************************************************
*/

#ifndef __SOURCEMANAGE_H
#define __SOURCEMANAGE_H

#ifdef __cplusplus

/*Include------------------------------------------------------------------------*/
//#include "includes.h"
#include "drv_can.h"

extern DAC_HandleTypeDef hdac;

#define LITTLE_CAP	1		/* ��������Ϊ2000J����10000J����Ӧ�ĳ���ѹ���ǲ�ͬ�� */
	
#if LITTLE_CAP
#define CAPVOL_LOW 			13.0f	//���ݵ�ѹ��
#define CAPVOL_HIGH 		23.2f	//���ݵ�ѹ��
#define BOOSTCHARGE_dV 	4.0f	//������ѹ���ѹ��
#define LOWCURRENT_VOL 	22.5f	//����С�������ĵ�ѹ
#define LOWCURRENT	 		2.0f	//С������ֵ
#else
#define CAPVOL_LOW 			12.0f	//���ݵ�ѹ��
#define CAPVOL_HIGH 		19.0f	//���ݵ�ѹ��
#define BOOSTCHARGE_dV 	5.0f	//������ѹ���ѹ��
#define LOWCURRENT_VOL 	18.0f	//����С�������ĵ�ѹ
#define LOWCURRENT	 		2.0f	//С������ֵ
#endif
	
	
#define SWITCH_ON GPIO_PIN_SET //���Ͽ���
#define SWITCH_OFF GPIO_PIN_RESET //�Ͽ�����
	 
#define G1_SWITCH_PIN GPIO_PIN_0
#define G2_SWITCH_PIN GPIO_PIN_1
#define G3_SWITCH_PIN GPIO_PIN_3
#define G4_SWITCH_PIN GPIO_PIN_2
#define G5_SWITCH_PIN GPIO_PIN_4
#define G_SWITCH_PORT GPIOC
	 
#define CHARGE_CTRL_Pin GPIO_PIN_0
#define CHARGE_CTRL_GPIO_Port GPIOB	 

/* ADC_DMA��ȡ�������λ�� */
#define VOL_IN	3
#define CUR_IN	4
#define VOL_CAP	0
#define VOL_OUT	5
#define CUR_OUT	2	/* 20�꿪ʼȫ��ʹ�ô����̣�����ʹ��С���̣����������������� */

#define Convert_To_RealVol(ADC_Val) ADC_Val*(3.3f/4096.0f)*11		/* 11Ϊ����� */
#define Convert_To_RealCur_IN(ADC_Val) ADC_Val*(3.3f/4096.0f)*(1/2e-3)/100.0		/* I = U/R 150.5Ϊ��ַŴ������� */
#define Convert_To_RealCur_Out(ADC_Val) ADC_Val*(3.3f/4096.0f)*(1/2e-3)/50.0
#define Calc_RealPower(voltage, current) voltage*current

typedef enum  
{
	Block_LOW = 0,
	Block_MID,
	Block_HIGH,
}_3_Block_e;//״̬��⽨

typedef struct 
{
	uint8_t statusCnt;
	_3_Block_e statusFlag;
}statusCheck_t;//״̬��⽨

typedef enum  
{
	LOW = 0,
	MID2,
	FULL,
}capStatus_e;//����״̬���ŵ�/���

typedef enum 
{
	CLOSE = 0,
	OPEN
}boostBoardStatus_e;

typedef struct 
{
	float vol_In;
	float vol_Cap;
	float vol_Out;
}voltage_t;

typedef struct 
{
	float cur_In;
	float cur_Out;
}current_t;

typedef struct 
{
	float pow_In;
	float pow_motor;
	float pow_Charge;
}power_t;

/* ���ݶ����Ϊ�ŵ�(��ѹ)�ͳ����������
	Ӧ�ò�ͨ����ȡ��Щ����ʹ����Ӧ���� */
typedef struct 
{
	float Voltage;
	float chargePower_Now;
	float chargePower_Set;
	float charge_DAC_Value;
	capStatus_e volStatus;	
	capStatus_e chargeStatus;
	statusCheck_t volStatusCheck;
	statusCheck_t chargeStatusCheck;	
}capObj_t;

typedef enum 
{
	BAT,
	CAP
}DischargeMode;

typedef struct 
{
	uint16_t ID;
	uint8_t msg_send[8];
}canMsg_t;

class SourceManage_ClassDef
{
private:
	voltage_t voltage; 
	current_t current;
	DischargeMode Mode;

	void switchCtrl(uint8_t _switchNumber,GPIO_PinState switchStatus);

	void ADC_To_Real(uint32_t *_ADCReadBuff);
	void Calc_Power();
	
	/* ���ݵ�ѹ��� */
	void Check_CapStatus();	
	void Block_Check(float min,float max,float real, statusCheck_t* p_statusCheck, int32_t cnt);	
	/* ���ݳ�� 	*/
	void Set_ChargeCurrent();
	void StopCharge();
	void StartCharge();

public:
	boostBoardStatus_e BAT_BOOST; 	/* �����ѹ�� */
	boostBoardStatus_e ESC_BOOST; 	/* �����ѹ�� */
	capObj_t capObj;								/* ���ݶ��� */
	canMsg_t ESC_Boost_Board_Msg;
	canMsg_t BAT_Boost_Board_Msg;

	void SourceManage_Init();
	/* ADC���ݸ��� */
	void Update(uint32_t *_ADCReadBuff);
	void BATToCAP();	
	void ALLCLOSE();
	void CAPToBAT();
	void Set_Mode(DischargeMode _mode);
	/* ���ݹ��� --���ݵ�ѹ��⡢���״̬��� */
	void Cap_Manage(void);
	void Cap_Boot_Manage();/* ������������ */
	/* �趨��繦�� */
	void Set_ChargePower(float inputPower);
	
	/* ��ѹ����� */
	void BAT_Boost_Manage();
	void ESC_Boost_Manage();

	/* ���̵�Դ���������� */
	void Manage(void);

	power_t power;

	
};
#endif




#endif
