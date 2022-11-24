#include "can_receive.h"

float Target_1,Target_2,Target_3,Target_4,Target_5,Target_6,Target_7,Target_8;
float Speed_Motor_Target_1,Speed_Motor_Target_2,Speed_Motor_Target_3,Speed_Motor_Target_4,Speed_Motor_Target_5,Speed_Motor_Target_6,Speed_Motor_Target_7;
float	Position_Motor_Target_8,Position_Motor_Target_1,Position_Motor_Target_2,Position_Motor_Target_3,Position_Motor_Target_4;

//#define get_motor_measure(ptr, data) \
//{ \
// (ptr)->last_angle = (ptr)->angle; \
// (ptr)->angle = (uint16_t)((data)[0] << 8 | (data)[1]); \
// (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]); \
// (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
// (ptr)->temperate = (data)[6]; \
//}-



motor_measure_t motor_can1[8];
motor_measure_t motor_can2[8];

static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];


void get_motor_measure(motor_measure_t *ptr,uint8_t data[])                                                     
    {   
        (ptr)->last_angle = (ptr)->angle;                                                          
        (ptr)->angle = data[0] << 8 | data[1];           
        (ptr)->speed_rpm = data[2] << 8 | data[3];     
        (ptr)->given_current = data[4] << 8 | data[5]; 
        (ptr)->temperate = data[6];                                              
//				((ptr)->angle) = (int32_t)(((ptr)->ecd) - ((ptr)->last_ecd));

					if(ptr->angle - ptr->last_angle > 4096)
						ptr->round_cnt --;
					else if (ptr->angle - ptr->last_angle < -4096)
						ptr->round_cnt ++;
					ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
    }
		
void get_moto_offset(motor_measure_t *ptr, uint8_t data[])
{
	ptr->angle = data[0]<<8 |data[1] ;
	ptr->offset_angle = ptr->angle;
}
#define ABS(x)	( (x>0) ? (x) : (-x) )

void get_total_angle(motor_measure_t *p)
	{
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//���ܵ����
		res1 = p->angle + 8192 - p->last_angle;	//��ת��delta=+
		res2 = p->angle - p->last_angle;				//��ת	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//��ת	delta -
		res2 = p->angle - p->last_angle;				//��ת	delta +
	}
	//��������ת���϶���ת�ĽǶ�С���Ǹ������
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}





void CAN1_Filter_Init(void)
{

    CAN_FilterTypeDef CAN_Filter_st;
    CAN_Filter_st.FilterActivation = ENABLE;
    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter_st.FilterIdHigh = 0x0000;
    CAN_Filter_st.FilterIdLow = 0x0000;
    CAN_Filter_st.FilterMaskIdHigh = 0x0000;
    CAN_Filter_st.FilterMaskIdLow = 0x0000;
    CAN_Filter_st.FilterBank = 0;
    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	  HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_st);        //�˲�����ʼ��
   
}



void CAN2_Filter_Init(void)
{

    CAN_FilterTypeDef CAN_Filter_st;
    CAN_Filter_st.FilterActivation = ENABLE;
    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter_st.FilterIdHigh = 0x0000;
    CAN_Filter_st.FilterIdLow = 0x0000;
    CAN_Filter_st.FilterMaskIdHigh = 0x0000;
    CAN_Filter_st.FilterMaskIdLow = 0x0000;
    CAN_Filter_st.FilterBank = 0;
    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	  HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter_st);        //�˲�����ʼ��
   
}





void CAN1_cmd_chassis_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)    //can1_motor1-4ΪChassis_motor
{
 uint32_t send_mail_box;
	
 chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
 chassis_tx_message.IDE = CAN_ID_STD;
 chassis_tx_message.RTR = CAN_RTR_DATA;
 chassis_tx_message.DLC = 0x08;
 chassis_can_send_data[0] = motor1 >> 8;
 chassis_can_send_data[1] = motor1;
 chassis_can_send_data[2] = motor2 >> 8;
 chassis_can_send_data[3] = motor2;
 chassis_can_send_data[4] = motor3 >> 8;
 chassis_can_send_data[5] = motor3;
 chassis_can_send_data[6] = motor4 >> 8;
 chassis_can_send_data[7] = motor4;
 HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}




void CAN1_cmd_chassis_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8) //can1_motor5Ϊcollect_motor    can1_motor6Ϊshoot_motor1,  can1_motor7Ϊshoot_motor2
{
 uint32_t send_mail_box;
	
 chassis_tx_message.StdId = 0x1FF;
 chassis_tx_message.IDE = CAN_ID_STD;
 chassis_tx_message.RTR = CAN_RTR_DATA;
 chassis_tx_message.DLC = 0x08;
 chassis_can_send_data[0] = motor5 >> 8;
 chassis_can_send_data[1] = motor5;
 chassis_can_send_data[2] = motor6 >> 8;
 chassis_can_send_data[3] = motor6;
 chassis_can_send_data[4] = motor7 >> 8;
 chassis_can_send_data[5] = motor7;
 chassis_can_send_data[6] = motor8 >> 8;
 chassis_can_send_data[7] = motor8;
 HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}




void CAN2_cmd_chassis_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)       //can2_motor1Ϊcam_motor
{
 uint32_t send_mail_box;
	
 chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
 chassis_tx_message.IDE = CAN_ID_STD;
 chassis_tx_message.RTR = CAN_RTR_DATA;
 chassis_tx_message.DLC = 0x08;
 chassis_can_send_data[0] = motor1 >> 8;
 chassis_can_send_data[1] = motor1;
 chassis_can_send_data[2] = motor2 >> 8;
 chassis_can_send_data[3] = motor2;
 chassis_can_send_data[4] = motor3 >> 8;
 chassis_can_send_data[5] = motor3;
 chassis_can_send_data[6] = motor4 >> 8;
 chassis_can_send_data[7] = motor4;
 HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


void CAN2_cmd_chassis_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
 uint32_t send_mail_box;
	
 chassis_tx_message.StdId = 0x1FF;
 chassis_tx_message.IDE = CAN_ID_STD;
 chassis_tx_message.RTR = CAN_RTR_DATA;
 chassis_tx_message.DLC = 0x08;
 chassis_can_send_data[0] = motor5 >> 8;
 chassis_can_send_data[1] = motor5;
 chassis_can_send_data[2] = motor6 >> 8;
 chassis_can_send_data[3] = motor6;
 chassis_can_send_data[4] = motor7 >> 8;
 chassis_can_send_data[5] = motor7;
 chassis_can_send_data[6] = motor8 >> 8;
 chassis_can_send_data[7] = motor8;
 HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN1)
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
			case CAN_3508_M8_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				motor_can1[i].msg_cnt++ <= 50	?	get_moto_offset(&motor_can1[i], rx_data) : get_motor_measure(&motor_can1[i], rx_data);
				get_motor_measure(&motor_can1[i], rx_data);
				break;
			}
			default:
			{
			break;
			}	
		}
		
	}
		
	if(hcan->Instance==CAN2)
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
			case CAN_3508_M8_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				motor_can2[i].msg_cnt++ <= 50	?	get_moto_offset(&motor_can2[i], rx_data) : get_motor_measure(&motor_can2[i], rx_data);
				get_motor_measure(&motor_can2[i], rx_data);
				break;
			}
			default:
			{
			break;
			}	
		}		
 }
	
}


void CAN_Start(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_Start(hcan);

	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
	}

}
