#ifndef __IMU101CT_H_
#define __IMU101CT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "include.h"

#define SAVE_ADDR 0X00 //����/����/�ָ�����
#define RRATE_ADDR 0x03 //�������
#define BAUD_ADDR  0x04 //���ڲ�����
#define READADDR_ADDR 0x27 //��ȡ�Ĵ���
#define GY_ADDR 0x38 //���ٶ�Y
#define GZ_ADDR 0X39 //���ٶ�Z
#define YAW_ADDR 0x3F //Yaw


typedef struct
{
	__packed struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Accel;

	__packed struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Gyro;
	
	__packed struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Magnet;
	
	__packed struct
	{
		fp32 yaw;    //ƫ����  ǰ����ƫ��
		fp32 pitch;  //������	 ǰ������°ڶ�
		fp32 roll;   //������  ���ҵ����°ڶ�
	}Euler;//ŷ����
	
	fp32 Temp;
	
}IMU_t;

void Clear_buffer(uint8_t *rx_buffer);

void IMU_Read_Gryo(uint8_t *rx_buffer);

void IMU_Read_Euler(uint8_t *rx_buffer);

void IMU_Write_Register(UART_HandleTypeDef *huart,uint8_t reg_addr,uint16_t tx_data);

#ifdef __cplusplus
}
#endif

#endif
