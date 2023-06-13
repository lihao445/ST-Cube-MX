#ifndef __IMU605_485_H_
#define __IMU605_485_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "usart.h"
#include "struct_typedef.h"
#include "cmsis_os.h"

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
		fp32 yaw;    //偏航�?  前进的偏�?
		fp32 pitch;  //俯仰�?	 前后的上下摆�?
		fp32 roll;   //翻滚�?  左右的上下摆�?
	}Euler;//欧拉�?
	
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

