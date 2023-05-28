#ifndef __IMU101CT_H_
#define __IMU101CT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "include.h"

#define SAVE_ADDR 0X00 //保存/重启/恢复出厂
#define RRATE_ADDR 0x03 //输出速率
#define BAUD_ADDR  0x04 //串口波特率
#define READADDR_ADDR 0x27 //读取寄存器
#define GY_ADDR 0x38 //角速度Y
#define GZ_ADDR 0X39 //角速度Z
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
		fp32 yaw;    //偏航角  前进的偏移
		fp32 pitch;  //俯仰角	 前后的上下摆动
		fp32 roll;   //翻滚角  左右的上下摆动
	}Euler;//欧拉角
	
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
