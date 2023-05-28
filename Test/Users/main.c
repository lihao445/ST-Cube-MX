/**
 ******************************************************************************
 * @file     main.c
 * @author   ����ԭ���Ŷ�(ALIENTEK)
 * @version  V1.0
 * @date     2020-08-20
 * @brief    �½�����ʵ��-HAL��汾 ʵ��
 * @license  Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ******************************************************************************
 * @attention
 * 
 * ʵ��ƽ̨:����ԭ�� STM32F103 ������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 ******************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "led.h"
#include "servo_motor.h"

int main(void)
{
    HAL_Init();                      /* ��ʼ��HAL�� */
		SystemClock_Config();            /* ����ʱ��, 16Mhz */
    delay_init(16);                  /* ��ʱ��ʼ�� */
		usart1_init(115200);							//��ʼ�����ڵ�������
		usart2_init(115200);							//��ʼ������ģ�鴮��
		TIM_PWM_Init(16-1,20000-1);				//��ʼ�����
    led_init();                      /* LED��ʼ�� */
	
    while(1)
    { 
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);    /* PC13��1 */ 
        delay_ms(500);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);  /* PC13��0 */
        delay_ms(500); 
    }
}


