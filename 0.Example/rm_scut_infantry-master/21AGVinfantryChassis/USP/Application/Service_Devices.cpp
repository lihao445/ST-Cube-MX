/**
  ******************************************************************************
  * @file   Service_Devices.cpp
  * @brief  Devices service running file.
  ******************************************************************************
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
  ===============================================================================
                                    Task List
  ===============================================================================
  * <table>
  * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
  * <tr><td>              <td>                  <td>                <td>    
  * </table>
  *
 */
/* Includes ------------------------------------------------------------------*/
#include "Service_Devices.h"
#include "Service_Communication.h"
#include "autoInfantryConfig.h"
/* Private define ------------------------------------------------------------*/
TaskHandle_t DeviceActuators_Handle;
TaskHandle_t DeviceDR16_Handle;
TaskHandle_t Device_Referee_UI_Handle;
TaskHandle_t DeviceSensors_Handle;
TaskHandle_t RecvReferee_Handle;
TaskHandle_t DeviceAGVcontrol_Handle;
extern referee_Classdef Referee;
extern C_SourceManage_Classdef SourceManage;
/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Device_Actuators(void *arg);
void Device_Sensors(void *arg);
void Device_Referee_UI(void *arg);
void Device_DR16(void *arg);
void Recv_Referee(void *arg);
void Device_AGVcontrol(void *arg);
/* Exported devices ----------------------------------------------------------*/
/* Motor & ESC & Other actuators*/
//Motor_AK80_9  Test_Motor(1, 0, 0);
/* Remote control */

/* IMU & NUC & Other sensors */

/* Other boards */

/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialization of device management service
* @param  None.
* @return None.
*/
void Service_Devices_Init(void)
{
//  xTaskCreate(Device_Actuators, "Dev.Actuator" , Tiny_Stack_Size,    NULL, PrioritySuperHigh,   &DeviceActuators_Handle);
  xTaskCreate(Device_Referee_UI,      "Dev.Referee_UI"     , Normal_Stack_Size,    NULL, PriorityHigh,        &Device_Referee_UI_Handle);
  xTaskCreate(Device_Sensors,   "Dev.Sensors"  , Small_Stack_Size,    NULL, PriorityRealtime,        &DeviceSensors_Handle);
	xTaskCreate(Recv_Referee,    "Rx.Referee"   , Large_Stack_Size,    NULL, PriorityRealtime,  &RecvReferee_Handle);
	xTaskCreate(Device_AGVcontrol,   "Dev.AGVcontrol"  , Small_Stack_Size,    NULL, PriorityHigh,        &DeviceAGVcontrol_Handle);
}




/**
* @brief    ����ϵͳ���ӻ�UI��������
* @note     ���Ƴ����ߡ����ݵ�ѹ���ٷֱȣ��Լ�����ָʾ��־��С���ݡ��������ݡ�����
* @return   None.
*/
void Device_Referee_UI(void *arg)
{
    static TickType_t _xPreviousWakeTime;

    //��ʼ����ʱ�Ļ��ƴ������������������ʽϸߣ���ֵ���ʵ�����
    static uint8_t enable_cnt = 20;             
 
    //��׹UI��ߵ�ˮƽ�̶��߳��ȡ����롢��ɫ����ֱ���ܳ�����Ϊ��ˮƽ�̶��߾���֮��
		uint16_t line_distance[6] = {10,30,30,35/*�ڱ�*/,30,50};
		uint16_t line_length[6] = {120,80,70,60,20,20};
		colorType_e ruler_color[7] = {WHITE, WHITE, WHITE, WHITE, YELLOW, YELLOW, WHITE};     //���һ��Ϊ��ֱ����ɫ

		//�״�վ���Լ�����
		uint16_t cpos_x[4] = {100,160,220,260};						                        //ȫ��UI�Ƽ�����
		uint16_t cpos_y[4] = {730,730,730,730};
		uint16_t frame_x[2] = {100,100};							
		uint16_t frame_y[2] = {800,670};

		uint16_t spos_x[3] = {100,200,80};							                        //ר��UI�Ƽ�����
		uint16_t spos_y[3] = {610,610,560};	

    //ͼ���ȶ���Ҫһ��ʱ�����ʱ
    vTaskDelay(500);
    Referee.clean_all();

    vTaskDelay(2000);                        
 
    for(;;)
    { 
				if(auto_infantry.write_state2[4] == 1)
				{
					enable_cnt =20;
					auto_infantry.write_state2[4] = false;
				}
        //�տ�ʼʱ��λ���ͼ�Σ�ȷ�����ڶ�̬UI�տ���ʱ˳������ͼ��
        if(enable_cnt)
        {
            //�����ߡ���׹��߻���
            Referee.Draw_Robot_Limit(180, 80, 961, 3, YELLOW, ADD_PICTURE);
            Referee.Hero_UI_ruler(5, 961, 538, line_distance, line_length, ruler_color, ADD_PICTURE);	

            //���Ƶ���ʣ������
            Referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 17, enable_cnt, 420,800);			

            //�״�վ���Լ�����
						Referee.Radar_Strategy_Frame(frame_x, frame_y);

            enable_cnt--;
        }
        else
        {
            Referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 17, enable_cnt, 420,800);	//���Ƶ���ʣ������
						Referee.Draw_Boost(auto_infantry.write_state2[1], 1600, 740, 10, PINK);					//���Ƴ�������״̬
						Referee.Draw_Spin(auto_infantry.write_state2[2], 1400, 740, 10, BLUE);					//����С���ݿ���״̬
						Referee.Draw_Bullet(auto_infantry.write_state2[3], 1800, 740, 8, GREEN);					//���Ƶ��ֿ���״̬
						Referee.Draw_Auto_Lock(1-auto_infantry.write_state2[5], 1400, 680, 8, WHITE);					//�������鿪��״̬
//						Referee.Draw_No_Bullet(Chassis.bulletNum, 861, 738, ORANGE);						//���ƿյ���ʾ
			
						//Radar
						Referee.Radar_CStrategy_Update(0, 0, Referee.robot_rec_data[RADAR].data[0], cpos_x, cpos_y);		//�״�վͨ�ò��Լ�
						Referee.Radar_SStrategy_Update(Referee.robot_rec_data[RADAR].data[1], spos_x, spos_y);				//�״�վר�ò��Լ�
        }
    }
}






void Device_Sensors(void *arg)
{
  /* Cache for Task */
  
  /* Pre-Load for task */
	
  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
	
	
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCReadBuff, 7);
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//	SourceManage.SourceManage_Init();
	
  for(;;)
  {
		vTaskDelayUntil(&xLastWakeTime_t,10);
    SourceManage.Update(ADCReadBuff);		//���µ�ѹ������������
		SourceManage.Set_ChargePower(PowerCtrl.Get_capChargePower());		//���µ��ݵ����������
		SourceManage.Manage();		//���й����߼�����
		
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, SourceManage.capObj.charge_DAC_Value);		//���õ��ݳ�����
    
  /* Pass control to the next task ------------------------------------------*/
    
  }
}





void Recv_Referee(void *arg)
{
  /* Pre-Load for task */
	static USART_COB* referee_pack;
  static TickType_t xLastWakeTime_t = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t,1);
//		Sent_Contorl(&huart1);
		if(xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *) &referee_pack, 0) == pdTRUE)
		{
			Referee.unPackDataFromRF((uint8_t*)referee_pack->address, referee_pack->len);		//���²���ϵͳ����
		}
		/* Pass control to the next task */
    
  }
}



void Device_AGVcontrol(void *arg)
{
  /* Cache for Task */

  /* Pre-Load for task */

  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
	vTaskDelay(2000);
  for(;;)
  {
		/* Pass control to the next task */
		vTaskDelayUntil(&xLastWakeTime_t,1);
		
    auto_infantry.AGVControl();		//���̿��ƺ���
    
    
  }
}
/* User Code End Here ---------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
