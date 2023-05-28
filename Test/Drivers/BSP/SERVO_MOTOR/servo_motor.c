#include "servo_motor.h"

TIM_HandleTypeDef htim1;     /* ��ʱ��x��� */
TIM_OC_InitTypeDef htim1oc  = {0};         /* ��ʱ��PWM������� */

void TIM_PWM_Init(uint16_t psc,uint16_t arr)
{
	GPIO_InitTypeDef gpio_init_struct;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();
	
	
	  htim1.Instance = TIM1;                  /* ��ʱ��x */
    htim1.Init.Prescaler = psc;                       /* ��ʱ����Ƶ */
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;      /* ��������ģʽ */
    htim1.Init.Period = arr;                          /* �Զ���װ��ֵ */
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; /*ʹ��TIMx_ARR���л��� */
    htim1.Init.RepetitionCounter = 0;                 /* �ظ���������ʼֵ */
    while(HAL_TIM_PWM_Init(&htim1) != HAL_OK)			/* ��ʼ��PWM */
		{
		
		}
		
		gpio_init_struct.Pin = GPIO_PIN_8;               								 /* ͨ��y��GPIO�� */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                           /* ����������� */
    gpio_init_struct.Pull = GPIO_PULLUP;                               /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                     /* ���� */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
		
		
		htim1oc.OCMode = TIM_OCMODE_PWM1;                         /* ģʽѡ��PWM 1*/
    htim1oc.Pulse = 1500;                                  /* ���ñȽ�ֵ,��ֵ����ȷ��ռ�ձ� */
                                            /* ����Ĭ�����ñȽ�ֵΪת0�ȵıȽ�ֵ,��ռ�ձ�Ϊ7.5% */
    htim1oc.OCPolarity = TIM_OCPOLARITY_HIGH;                 /* ����Ƚϼ���Ϊ�� */
    HAL_TIM_PWM_ConfigChannel(&htim1, &htim1oc, TIM_CHANNEL_1); /* ����TIMxͨ��y */
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    /* ������ӦPWMͨ�� */
		
}
