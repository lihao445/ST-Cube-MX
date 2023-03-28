#ifndef __PCvision_H__
#define __PCvision_H__


#include <stdint.h> 

#ifdef __cplusplus

/**************************************************************************
* @brief	�Ӿ���������Э��һ�µ����ݰ�
* @note		Null
**************************************************************************/
#pragma pack(1)
typedef struct 
{
	uint8_t flag;
	uint8_t shootMode;
	float yawData;
	float pitchData;
	uint8_t End;
}PackFromVisionDef;
#pragma pack()

/**************************************************************************
* @brief	ͨ�������ڴ潫�Ӿ��������������Զ�ͬ����PackFromVision
* @note		ʹ��UsartData�����Ӿ������ݣ�PackFromVision�ж�Ӧ�Ĳ��������޸�
**************************************************************************/
typedef union 
{
	uint8_t UsartData[11];
	PackFromVisionDef PackFromVision;
}PackFromVisionUnionDef;

/**************************************************************************
* @brief	���ظ��Ӿ���Э��һ�µ����ݰ�
* @note		Null
**************************************************************************/
#pragma pack(1)
typedef struct
{
	uint8_t head;
	
	float	pitchAngle;
	float	yawAngle;
	uint8_t color;
	uint8_t bulletSpeed;
	uint8_t mode;
	uint8_t end;
	
}PackToVision_Def;
#pragma pack()

/**************************************************************************
* @brief	ͨ�������ڴ潫�����������Զ�ͬ����PackToVision
* @note		ͨ���޸�PackToVision�еĲ������ٽ�UsartData���鷢�ͳ�����
**************************************************************************/
typedef union 
{
	uint8_t UsartData[12];
	PackToVision_Def PackToVision;
}PackToVisionUnionDef;

/**************************************************************************
* @brief	ͨ�������ڴ漰�����޸�float���͵Ĵ�С��
* @note		
**************************************************************************/
typedef union 
{
        float				f;
        char				c[4];
}Float_Conv;

enum PCvisionStatusDef
{
	Connected = 1,
	Unconnected = 0,
};

extern PackFromVisionUnionDef PackFromVisionUnion;
extern PackToVisionUnionDef PackToVisionUnion;
extern PCvisionStatusDef PCvisionStatus;
extern uint8_t PCCheckLinkFlag;

void PC_CheckLink(void);
uint32_t GetViaionData(uint8_t* Recv_Data, uint16_t ReceiveLen);
void Vision_Init();
void SendGimbleStatus(PackToVisionUnionDef* _PackToVisionUnion);
float BLEndianFloat(float fValue);
extern "C"{
#endif
	
	
#ifdef  __cplusplus
}
#endif

#endif

