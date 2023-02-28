#ifndef __CAN_RECEIVE_H_
#define __CAN_RECEIVE_H_
#include "can.h"
#include "pid_user.h"

#define CHASSIS_CAN hcan1

typedef enum 
{ CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID= 0x204,
} can_msg_id_e;

typedef struct
{
 uint16_t angle;
 int16_t speed_rpm;
 int16_t given_current;
 uint8_t temperate;
 int16_t last_angle;
		int32_t total_angle;
		int32_t	round_cnt;
		uint16_t offset_angle;
		uint32_t			msg_cnt;
} motor_measure_t;

void get_motor_measure(motor_measure_t *ptr,uint8_t data[]);
void get_moto_offset(motor_measure_t *ptr, uint8_t data[]);
void get_total_angle(motor_measure_t *p);




void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN_Start(CAN_HandleTypeDef *hcan);
void CAN1_Filter_Init(void);
void CAN2_Filter_Init(void);


#endif

