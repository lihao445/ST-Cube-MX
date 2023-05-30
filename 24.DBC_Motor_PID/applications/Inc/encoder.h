#ifndef __ENCODER_H_
#define __ENCODER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "include.h"

#define ABS(x)	( (x>0) ? (x) : (-x) ) 

#define PULSE_PER_REVOLUTION  13.0f	/* ���� */
#define FRE_DOU_RATIO         4.0f  	/* ��Ƶϵ�������ݱ�����ģʽѡ */
#define REDUCTION_RATIO       30.0f  /* ���ٱ�30:1 */



typedef struct
{
	int32_t encoder_now;
	int32_t encoder_pre;
	int32_t encoder_delta;
	int32_t encoder_delta_sum;
}Encoder_TypeDef;


/* ��������ṹ�� */
typedef struct 
{
  bool_t state;          /*���״̬*/
  fp32 current;          /*�������*/
  fp32 volatage;         /*�����ѹ*/
  fp32 power;            /*�������*/
  fp32 rotor_speed;      /*���[ת��]ʵ���ٶ�*/
	fp32 motor_speed;		   /*���[ת��]ʵ���ٶ�*/
	fp32 rotor_position;	 /*���[ת��]ʵ��ת������*/
	fp32 motor_position;	 /*���[ת��]ʵ��ת������*/
	fp32 rotor_round_cnt;  /*���[ת��]ʵ��ת��Ȧ��*/
	fp32 motor_round_cnt;  /*���[ת��]ʵ��ת��Ȧ��*/
  int32_t motor_pulse;     /*���ñȽ�ֵ��С */
}Motor_TypeDef;




int32_t get_encoder_value(void);
void motor_message_filtering(Encoder_TypeDef *encoder,Motor_TypeDef *motor,uint8_t ms);

#ifdef __cplusplus
}
#endif

#endif
