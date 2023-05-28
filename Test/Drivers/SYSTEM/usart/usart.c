/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-20
 * @brief       ���ڳ�ʼ������(һ���Ǵ���1)��֧��printf
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F103������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211103
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "servo_motor.h"

/* ���ʹ��os,����������ͷ�ļ�����. */
#if SYS_SUPPORT_OS
#include "includes.h" /* os ʹ�� */
#endif

/******************************************************************************************/
/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");  /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE �� stdio.h���涨��. */
FILE __stdout;

/* MDK����Ҫ�ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);     /* �ȴ���һ���ַ�������� */

    USART1->DR = (uint8_t)ch;             /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
    return ch;
}
#endif
/******************************************************************************************/

uint8_t rx_buffer[1];  /* HAL��ʹ�õĴ��ڽ��ջ��� */

UART_HandleTypeDef huart1;  /* UART��� */
UART_HandleTypeDef huart2;

/**
 * @brief       ����X��ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @note        ע��: ����������ȷ��ʱ��Դ, ���򴮿ڲ����ʾͻ������쳣.
 *              �����USART��ʱ��Դ��sys_stm32_clock_init()�������Ѿ����ù���.
 * @retval      ��
 */
void usart1_init(uint32_t baudrate)
{
    /*UART ��ʼ������*/
    huart1.Instance = USART1;                                       /* USART_UX */
    huart1.Init.BaudRate = baudrate;                                  /* ������ */
    huart1.Init.WordLength = UART_WORDLENGTH_8B;                      /* �ֳ�Ϊ8λ���ݸ�ʽ */
    huart1.Init.StopBits = UART_STOPBITS_1;                           /* һ��ֹͣλ */
    huart1.Init.Parity = UART_PARITY_NONE;                            /* ����żУ��λ */
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* ��Ӳ������ */
    huart1.Init.Mode = UART_MODE_TX_RX;                               /* �շ�ģʽ */
    HAL_UART_Init(&huart1);                                           /* HAL_UART_Init()��ʹ��UART1 */
}

/**
 * @brief       ����X��ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @note        ע��: ����������ȷ��ʱ��Դ, ���򴮿ڲ����ʾͻ������쳣.
 *              �����USART��ʱ��Դ��sys_stm32_clock_init()�������Ѿ����ù���.
 * @retval      ��
 */
void usart2_init(uint32_t baudrate)
{
    /*UART ��ʼ������*/
    huart2.Instance = USART2;                                       /* USART_UX */
    huart2.Init.BaudRate = baudrate;                                  /* ������ */
    huart2.Init.WordLength = UART_WORDLENGTH_8B;                      /* �ֳ�Ϊ8λ���ݸ�ʽ */
    huart2.Init.StopBits = UART_STOPBITS_1;                           /* һ��ֹͣλ */
    huart2.Init.Parity = UART_PARITY_NONE;                            /* ����żУ��λ */
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* ��Ӳ������ */
    huart2.Init.Mode = UART_MODE_TX_RX;                               /* �շ�ģʽ */
    HAL_UART_Init(&huart2);                                           /* HAL_UART_Init()��ʹ��UART1 */
	    /* �ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ��������������� */
    HAL_UART_Receive_IT(&huart2, rx_buffer, 1); 
}




/**
 * @brief       UART�ײ��ʼ������
 * @param       huart: UART�������ָ��
 * @note        �˺����ᱻHAL_UART_Init()����
 *              ���ʱ��ʹ�ܣ��������ã��ж�����
 * @retval      ��
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (huart->Instance == USART1)                            /* ����Ǵ���1�����д���1 MSP��ʼ�� */
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();                             /* ʹ�ܴ��ڽ�ʱ�� */
        __HAL_RCC_USART1_CLK_ENABLE();                              /* ʹ�ܴ���ʱ�� */

        gpio_init_struct.Pin = GPIO_PIN_9;               /* ���ڷ������ź� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* ����������� */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;			/* IO�ٶ�����Ϊ���� */
			
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
                
        gpio_init_struct.Pin = GPIO_PIN_10;               /* ����RX�� ģʽ���� */
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;    
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);   /* ����RX�� �������ó�����ģʽ */
        
//        HAL_NVIC_EnableIRQ(USART1_IRQn);                      /* ʹ��USART1�ж�ͨ�� */
//        HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);              /* ��2��������ȼ�:��ռ���ȼ�3�������ȼ�3 */
    }
		    if (huart->Instance == USART2)                            /* ����Ǵ���1�����д���1 MSP��ʼ�� */
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();                             /* ʹ�ܴ��ڽ�ʱ�� */
        __HAL_RCC_USART2_CLK_ENABLE();                              /* ʹ�ܴ���ʱ�� */

        gpio_init_struct.Pin = GPIO_PIN_2;               /* ���ڷ������ź� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* ����������� */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;			/* IO�ٶ�����Ϊ���� */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
                
        gpio_init_struct.Pin = GPIO_PIN_3;               /* ����RX�� ģʽ���� */
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;
				gpio_init_struct.Pull = GPIO_NOPULL;
				gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;			/* IO�ٶ�����Ϊ���� */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);   /* ����RX�� �������ó�����ģʽ */
        
        HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);              /* ��2��������ȼ�:��ռ���ȼ�3�������ȼ�3 */
			  HAL_NVIC_EnableIRQ(USART2_IRQn);                      /* ʹ��USART2�ж�ͨ�� */
    }
}



void USART2_IRQHandler(void)
{
		HAL_UART_IRQHandler(&huart2);
	  while (HAL_UART_Receive_IT(&huart2, rx_buffer, 1) != HAL_OK)     /* ���¿����жϲ��������� */
    {
        /* �������Ῠ�������� */
    }
}


/**
 * @brief       �������ݽ��ջص�����
                ���ݴ������������
 * @param       huart:���ھ��
 * @retval      ��
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		switch(*rx_buffer)
		{
			case '0':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,500);
			printf("\r\n��ʱ��������ת��Ϊ-90��");
			break;
			case '1':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000);
			printf("\r\n��ʱ��������ת��Ϊ-45��");
			break;
			case '2':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1500);
			printf("\r\n��ʱ��������ת��Ϊ0��");
			break;
			case '3':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2000);
			printf("\r\n��ʱ��������ת��Ϊ45��");
			break;
			case '4':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
			printf("\r\n��ʱ��������ת��Ϊ90��");
			break;
			default:;
		}
	}
}
