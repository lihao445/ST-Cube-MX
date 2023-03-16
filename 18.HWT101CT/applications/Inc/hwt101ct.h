#ifndef __HWT101CT_H_
#define __HWT101CT_H_
#include "include.h"
#include "REG.h"

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80


void AutoScanSensor(void);
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);


#endif
