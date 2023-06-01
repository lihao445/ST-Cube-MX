#ifndef __IMU605_485_H_
#define __IMU605_485_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "usart.h"
#include "struct_typedef.h"

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
	
	__packed struct
	{
		fp32 quat[4];
	}Quaternion;
	
	__packed struct
	{
		fp32 temp;
	}Other_Data;
	
}IMU_t;




void IMU_Unlock_Instruct(void);
void IMU_Save_Instruct(void);
void IMU_Request_Data(uint16_t ADDR,uint16_t LEN,uint16_t Data_CRC);
void IMU_Read_Data(uint8_t *rx_buffer);


#ifdef __cplusplus
}
#endif

#endif

