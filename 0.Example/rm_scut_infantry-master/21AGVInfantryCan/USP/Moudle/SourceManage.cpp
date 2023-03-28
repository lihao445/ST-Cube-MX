/**
******************************************************************************
* Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
* @file    SourceManage.cpp
* @author  ������ 15013073869
* @brief   ���̵�Դ����
* @date    2019-11-12
* @version 2.0
* @par Change Log��
* <table>
* <tr><th>Date        	<th>Version  <th>Author    		<th>Description
* <tr><td>2019-11-8   	<td> 1.0     <td>������        <td>
* </table>
*
==============================================================================
										##### How to use this driver #####
==============================================================================
	@note
		-# SourceManage_Init
		-# Update(ADC�ɼ�����)  ����ɼ�˳��仯  Ҫ����.h������Ӧ������λ�ú궨��
		-# Set_ChargePower(�趨��繦��)
		-# Cap_Manage  BAT_Boost_Manage  ESC_Boost_Manage
		
		example��
			void SourceManage_Task(void const * argument)
			{
				TickType_t xLastWakeTime;
				const TickType_t xFrequency = 2;
				xLastWakeTime = xTaskGetTickCount();
				
				static uint32_t ADCReadBuff[8];
				HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCReadBuff, 7);
				HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

				ChassisSource.SourceManage_Init();	

				for(;;)
				{
					vTaskDelayUntil(&xLastWakeTime,xFrequency);
					��������
					SourceManage.Update(ADCReadBuff);
					SourceManage.Set_ChargePower(PowerCtrl.Get_ChargePower());
					
					����  ��ѹ�����
					SourceManage.Manage();	
				}
			}
			
	@warning

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

/* Includes ------------------------------------------------------------------*/


#include "SourceManage.h"
#include "string.h"
/**
 * @brief ���µ�ѹ�����Ȳ���
 * @note 
 * @param ADC�ɼ����� ˳�����.h�ļ���
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::Update(uint32_t *_ADCReadBuff)
{
	ADC_To_Real(_ADCReadBuff);
	Calc_Power();
	capObj.Voltage = voltage.vol_Cap;
	capObj.chargePower_Now = power.pow_Charge;
}

/**
 * @brief ��ADCֵת���ɵ�ѹ������ʵֵ
 * @note 
 * @param ADC�ɼ�����
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::ADC_To_Real(uint32_t *_ADCReadBuff)
{	
	/* �����ѹ��ֵ */
	voltage.vol_In = Convert_To_RealVol(_ADCReadBuff[VOL_IN]);
	voltage.vol_Cap = Convert_To_RealVol(_ADCReadBuff[VOL_CAP]);
	voltage.vol_Out = Convert_To_RealVol(_ADCReadBuff[VOL_OUT]);
	
	/* ���������ֵ */	
	current.cur_In = Convert_To_RealCur_IN(_ADCReadBuff[CUR_IN]);
	current.cur_Out = Convert_To_RealCur_Out(_ADCReadBuff[CUR_OUT]);//������
	if(current.cur_Out < 0)	{current.cur_Out = 0;}
	
}

/**
 * @brief ���������빦�ʡ�������ʡ����ݹ���
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::Calc_Power(void)
{

	power.pow_In = Calc_RealPower(voltage.vol_In, current.cur_In);
	power.pow_motor = Calc_RealPower(voltage.vol_Out, current.cur_Out);
		
	power.pow_Charge = power.pow_In - power.pow_motor;
	if(power.pow_Charge < 0)		{power.pow_Charge = 0;}
}


uint8_t switchNumber[6]={0,G1_SWITCH_PIN,G2_SWITCH_PIN,G3_SWITCH_PIN,G4_SWITCH_PIN,G5_SWITCH_PIN};
/**
  * @brief ���ƿ��ص�ͨ��  
  * @param  _switchNumber [ѡ��Ŀ�������1-5];
						switchStatus[����״̬��SWITCH_ON �򿪿��� SWITCH_OFF �Ͽ�����]
  * @return none
  * @author Joanna
  */
/**
 * @brief ���ƿ��ص�ͨ��  
 * @note 
 * @param _switchNumber [ѡ��Ŀ�������1-5];
						switchStatus[����״̬��SWITCH_ON �򿪿��� SWITCH_OFF �Ͽ�����]
 * @retval None
 * @author Joanna
 */
void SourceManage_ClassDef::switchCtrl(uint8_t _switchNumber,GPIO_PinState switchStatus)
{
 	HAL_GPIO_WritePin(G_SWITCH_PORT,switchNumber[_switchNumber], switchStatus);
}
void SourceManage_ClassDef::StopCharge()
{
	HAL_GPIO_WritePin(CHARGE_CTRL_GPIO_Port,CHARGE_CTRL_Pin,GPIO_PIN_RESET);
}
void SourceManage_ClassDef::StartCharge()
{
	HAL_GPIO_WritePin(CHARGE_CTRL_GPIO_Port,CHARGE_CTRL_Pin,GPIO_PIN_SET);
}

/**
 * @brief �ɵ�ع���ת�����ݹ���  
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::BATToCAP()
{
	switchCtrl(1,SWITCH_OFF); 
	HAL_Delay(1);
	switchCtrl(4,SWITCH_ON); 
	HAL_Delay(1);
	switchCtrl(2,SWITCH_OFF); 
	HAL_Delay(1);
	switchCtrl(3,SWITCH_ON); 
	HAL_Delay(1);
	switchCtrl(5,SWITCH_ON); 
}

/**
 * @brief �ɵ��ݹ���ת����Դ����  
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::CAPToBAT()
{
	switchCtrl(3,SWITCH_OFF); 
	HAL_Delay(1);
	switchCtrl(2,SWITCH_ON); 
	HAL_Delay(1);
	switchCtrl(4,SWITCH_OFF); 
	HAL_Delay(1);
	switchCtrl(1,SWITCH_ON); 
	HAL_Delay(1);
	switchCtrl(5,SWITCH_OFF); 
			
}

/**
 * @brief 24V���ȫ���ر�
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::ALLCLOSE()
{
	switchCtrl(3,SWITCH_OFF); 
	switchCtrl(2,SWITCH_OFF); 
	switchCtrl(4,SWITCH_OFF); 
	switchCtrl(1,SWITCH_OFF); 
	switchCtrl(5,SWITCH_OFF); 		
}

/** 
 * @brief �������� 
 * @note 	1.��һ����min max�ͺ� 
					2.cnt���ܳ���255
					3.min = maxʱ��ֻ�᷵��LOW��HIGH
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::Block_Check(float min,float max,float real, statusCheck_t* p_statusCheck, int32_t cnt)
{
	if(real < min)
	{
		p_statusCheck->statusCnt++;
		if(p_statusCheck->statusCnt >= cnt)
		{
			p_statusCheck->statusCnt = 0;
			p_statusCheck->statusFlag = Block_LOW;
		}
	}
	else if(real >= max)
	{
		p_statusCheck->statusCnt++;
		if(p_statusCheck->statusCnt >= cnt)
		{
			p_statusCheck->statusCnt = 0;
			p_statusCheck->statusFlag = Block_HIGH;
		}
	}
	else
	{
		p_statusCheck->statusCnt = 0;		
		p_statusCheck->statusFlag = Block_MID;	
	}
}

/** 
 * @brief ���ݹ���
 * @note ������������ ����״̬��� ���㲢�趨���ݳ�����
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::Cap_Manage(void)
{
	Check_CapStatus();
	Set_ChargeCurrent();
}

/**
 * @brief ������״̬ �������״̬�ͷŵ�״̬
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::Check_CapStatus(void)
{
	/* ���ݵ������ */		
	Block_Check(CAPVOL_LOW, CAPVOL_HIGH, capObj.Voltage, &capObj.volStatusCheck, 50);
	if(capObj.volStatusCheck.statusFlag == Block_LOW)
	{
		capObj.volStatus = LOW;		/* ���ݵ�ѹ���� */		
	}
	else if(capObj.volStatusCheck.statusFlag == Block_HIGH)
	{
		capObj.volStatus = FULL;	/* ���ݳ��� */		
	}
	else
	{
		capObj.volStatus = MID2;		
	}
	
	/* ���ݳ���� */		
	Block_Check(voltage.vol_In-BOOSTCHARGE_dV, voltage.vol_In-BOOSTCHARGE_dV, capObj.Voltage, &capObj.chargeStatusCheck, 50);
	if(capObj.chargeStatusCheck.statusFlag == Block_LOW)
	{
		capObj.chargeStatus = LOW;
	}
	else
	{
		capObj.chargeStatus = MID2;	/* ��Ҫ������ѹ��� */
	}	
}

/**
 * @brief �趨���ݳ�繦��
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::Set_ChargePower(float inputPower)
{
	capObj.chargePower_Set = inputPower;
}

/**
 * @brief �趨���ݳ�����
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::Set_ChargeCurrent(void)
{
	float output = 0;
	float chargeCurrent = capObj.chargePower_Set/capObj.Voltage;
	
	/* �жϵ����Ƿ���� */
	if(capObj.volStatus == FULL)
	{
		chargeCurrent = 0;
	}
	else
	{
		/* �ж��Ƿ���ҪС������� */
		if(capObj.Voltage >= LOWCURRENT_VOL)
			if(chargeCurrent >= LOWCURRENT)
				chargeCurrent = LOWCURRENT;
			
		if(chargeCurrent >= 10.0f)
			chargeCurrent = 10.0f;		
	}
	
	output = (uint32_t)(chargeCurrent / 5 / (3.3f/4096.0f));
	capObj.charge_DAC_Value = output;
}

/** 
 * @brief �����ѹ�����
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::BAT_Boost_Manage()
{
	if(capObj.chargeStatus == MID2)/* ��Ҫ������ѹ��� */
		BAT_BOOST = OPEN;
	else
		BAT_BOOST = CLOSE;
	
//	uint8_t msg_send[8] = {0};
//	msg_send[0] = 0x55;
//	if(BAT_BOOST == OPEN)//���������ѹ��
//		msg_send[1] = 0x33;
//	else
//		msg_send[1] = 0x44;
	
//  BAT_Boost_Board_Msg.ID = 0x200;
//	memcpy(BAT_Boost_Board_Msg.msg_send, msg_send, 8);/* ������200  �ȴ�����ѹ����� */
	
}

/** 
 * @brief �����ѹ�����
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::ESC_Boost_Manage()
{
	uint8_t msg_send[8] = {0};	
	
	ESC_BOOST=OPEN;

	msg_send[0]=4;//24V��ѹ���
	
	ESC_Boost_Board_Msg.ID = 0x303;
	memcpy(ESC_Boost_Board_Msg.msg_send, msg_send, 8);
}

/**
 * @brief ��Դ�����ʼ��
 * @note 
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::SourceManage_Init()
{	 
	/* �����建���� */
	ALLCLOSE();
	HAL_Delay(300);
  StartCharge();
	HAL_Delay(300);
	
	Set_ChargePower(0);
		
	ESC_BOOST=CLOSE;
	BAT_BOOST=CLOSE;
}

/**
 * @brief ������������
 * @note ��ʼ��ʱֻ�е����ݴ���15Vʱ�Ŵ򿪵������
 * @param None
 * @retval None
 * @author ������
 */
void SourceManage_ClassDef::Cap_Boot_Manage(void)
{
	static uint8_t bootFlag = 0;
	/* �ȴ�������ٿ�CAP���� */
	if(bootFlag == 0)
	{
		if(capObj.Voltage>=15)
		{
			BATToCAP();	
			bootFlag=1;		
		}
		else
			ALLCLOSE();
	}
}

void  SourceManage_ClassDef::Set_Mode(DischargeMode _mode)
{
	Mode = _mode;
}


void SourceManage_ClassDef::Manage(void)
{
	/* ����  ��ѹ����� */
	if(Mode == CAP)
		Cap_Boot_Manage();	/* ֻ��������ɵ��ݲŻ���� */
	else
		CAPToBAT();

	Cap_Manage();
	BAT_Boost_Manage();
	ESC_Boost_Manage();
}
