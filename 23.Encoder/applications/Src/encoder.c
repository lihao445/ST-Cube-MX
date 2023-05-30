#include "encoder.h"

volatile int32_t motor_encoder_count;   //�������

Encoder_TypeDef encoder_data;

Motor_TypeDef motor_data;


/**
 * @brief       ��ʱ�������жϻص�����
 * @param        htim:��ʱ�����ָ��
 * @note        �˺����ᱻ��ʱ���жϺ�����ͬ���õ�
 * @retval      ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3) == 1)    /* �ж�CR1��DIRλ */
		{
			motor_encoder_count--;                           /* DIRλΪ1��Ҳ���ǵݼ����� */
		}
		else
		{
			motor_encoder_count++;                          /* DIRλΪ0��Ҳ���ǵ������� */
		}
	}
	if(htim->Instance == TIM7)
	{
		encoder_data.encoder_now = get_encoder_value();                             /* ��ȡ������ֵ�����ڼ����ٶ� */
		
    motor_message_filtering(&encoder_data,&motor_data,50);                       /* ��λƽ��ֵ�˳��������������ݣ�50ms����һ���ٶ�*/
	}
}

/**
 * @brief       ��ȡ��������ֵ
 * @param       ��
 * @retval      ������ֵ
 */
int32_t get_encoder_value(void)
{
	int32_t encoder_value;
	encoder_value = __HAL_TIM_GET_COUNTER(&htim3) + motor_encoder_count * 65536;       /* ��ǰ����ֵ+֮ǰ�ۼƱ�������ֵ=�ܵı�����ֵ */
	
	return encoder_value;
}


/**
 * @brief       ����ٶȼ���
 * @param       encode_now����ǰ�������ܵļ���ֵ
 *              ms�������ٶȵļ�����ж�1ms����һ�Σ�����ms = 5��5ms����һ���ٶ�
 * @retval      ��
 */
void motor_message_filtering(Encoder_TypeDef *encoder,Motor_TypeDef *motor,uint8_t ms)
{
		//��ͨ�˲�ϵ��
		fp32 q = 0.9;
	
	   //ð������
    uint8_t i = 0, j = 0;
		static uint8_t bubble_sort_number = 0;
		//���棬�䵱ð�����򷨵Ľ��������ð�����򷨴������ٶ��ܺͻ���
    fp32 temp = 0.0f;
	
    static uint8_t time_count = 0;
    static fp32 speed_arr[10] = {0.0f};                     /* �洢�ٶȽ����˲����� */

    if (time_count == ms)                                     /* ����һ���ٶ� */
    {
        /* ������ת�� 
           ��һ�� ������ms�����ڼ����仯��
           �ڶ��� ������1min�ڼ����仯����g_encode.speed * ((1000 / ms) * 60 ��
           ������ �����Ա�������תһȦ�ļ�����������Ƶ���� * �������ֱ��ʣ�
           ���Ĳ� �����Լ��ٱȼ��ɵó����ת��
        */
			
        encoder->encoder_delta = (encoder->encoder_now - encoder->encoder_pre);    /* �������������ֵ�ı仯�� */
			if(ABS(encoder->encoder_delta) >= 60000.0f)
			{
				encoder->encoder_delta = 0.0f;
			}
        encoder->encoder_delta_sum += encoder->encoder_delta;
        speed_arr[bubble_sort_number++] = (fp32)(encoder->encoder_delta * ((1000 / ms) * 60.0) / (FRE_DOU_RATIO) / (PULSE_PER_REVOLUTION));    /* ������ת�� */
        
        encoder->encoder_pre = encoder->encoder_now;          /* ���浱ǰ��������ֵ */

        /* �ۼ�10���ٶ�ֵ�����������˲�*/
        if (bubble_sort_number == 10)
        {
            for (i = 10; i >= 1; i--)                       /* ð������*/
            {
                for (j = 0; j < (i - 1); j++) 
                {
                    if (speed_arr[j] > speed_arr[j + 1])    /* ��ֵ�Ƚ� */
                    { 
                        temp = speed_arr[j];                /* ��ֵ��λ */
                        speed_arr[j] = speed_arr[j + 1];
                        speed_arr[j + 1] = temp;
                    }
                }
            }
						
            temp = 0.0;
            
            for (i = 2; i < 8; i++)                         /* ȥ�����߸ߵ����� */
            {
                temp += speed_arr[i];                       /* ���м���ֵ�ۼ� */
            }
            
            temp = (fp32)(temp / 6);                       /*���ٶ�ƽ��ֵ*/
            
            /* һ�׵�ͨ�˲�
             * ��ʽΪ��Y(n)= qX(n) + (1-q)Y(n-1)
             * ����X(n)Ϊ���β���ֵ��Y(n-1)Ϊ�ϴ��˲����ֵ��Y(n)Ϊ�����˲����ֵ��qΪ�˲�ϵ��
             * qֵԽС����һ������Ա������Ӱ��Խ����������Խƽ�ȣ����Ƕ����ٶȱ仯����ӦҲ��Խ��
             */
            motor->rotor_speed = (fp32)((fp32)(q * temp) + (motor->rotor_speed * (fp32)(1.0f-q)) );  //ͨ����ͨ�˲�����ת���ٶ�
						motor->motor_speed = motor->rotor_speed / (REDUCTION_RATIO);     //������ٺ���ת����ٶ�
						
						motor->rotor_position =  encoder->encoder_delta_sum;//���λ��
						motor->motor_position =  motor->rotor_position / (REDUCTION_RATIO);  //������ٺ���ת���λ��
						
						motor->rotor_round_cnt = motor->rotor_position / (FRE_DOU_RATIO) / (PULSE_PER_REVOLUTION);
						motor->motor_round_cnt = motor->rotor_round_cnt / (REDUCTION_RATIO);  //������ٺ���ת���Ȧ��
						
						

						
            bubble_sort_number = 0;
        }
        time_count = 0;
    }
    time_count ++;
}



