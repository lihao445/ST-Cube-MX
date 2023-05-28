/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-20
 * @brief       串口初始化代码(一般是串口1)，支持printf
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211103
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "servo_motor.h"

/* 如果使用os,则包括下面的头文件即可. */
#if SYS_SUPPORT_OS
#include "includes.h" /* os 使用 */
#endif

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");  /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* MDK下需要重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);     /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;             /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif
/******************************************************************************************/

uint8_t rx_buffer[1];  /* HAL库使用的串口接收缓冲 */

UART_HandleTypeDef huart1;  /* UART句柄 */
UART_HandleTypeDef huart2;

/**
 * @brief       串口X初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @note        注意: 必须设置正确的时钟源, 否则串口波特率就会设置异常.
 *              这里的USART的时钟源在sys_stm32_clock_init()函数中已经设置过了.
 * @retval      无
 */
void usart1_init(uint32_t baudrate)
{
    /*UART 初始化设置*/
    huart1.Instance = USART1;                                       /* USART_UX */
    huart1.Init.BaudRate = baudrate;                                  /* 波特率 */
    huart1.Init.WordLength = UART_WORDLENGTH_8B;                      /* 字长为8位数据格式 */
    huart1.Init.StopBits = UART_STOPBITS_1;                           /* 一个停止位 */
    huart1.Init.Parity = UART_PARITY_NONE;                            /* 无奇偶校验位 */
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* 无硬件流控 */
    huart1.Init.Mode = UART_MODE_TX_RX;                               /* 收发模式 */
    HAL_UART_Init(&huart1);                                           /* HAL_UART_Init()会使能UART1 */
}

/**
 * @brief       串口X初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @note        注意: 必须设置正确的时钟源, 否则串口波特率就会设置异常.
 *              这里的USART的时钟源在sys_stm32_clock_init()函数中已经设置过了.
 * @retval      无
 */
void usart2_init(uint32_t baudrate)
{
    /*UART 初始化设置*/
    huart2.Instance = USART2;                                       /* USART_UX */
    huart2.Init.BaudRate = baudrate;                                  /* 波特率 */
    huart2.Init.WordLength = UART_WORDLENGTH_8B;                      /* 字长为8位数据格式 */
    huart2.Init.StopBits = UART_STOPBITS_1;                           /* 一个停止位 */
    huart2.Init.Parity = UART_PARITY_NONE;                            /* 无奇偶校验位 */
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* 无硬件流控 */
    huart2.Init.Mode = UART_MODE_TX_RX;                               /* 收发模式 */
    HAL_UART_Init(&huart2);                                           /* HAL_UART_Init()会使能UART1 */
	    /* 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量 */
    HAL_UART_Receive_IT(&huart2, rx_buffer, 1); 
}




/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置
 * @retval      无
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (huart->Instance == USART1)                            /* 如果是串口1，进行串口1 MSP初始化 */
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();                             /* 使能串口脚时钟 */
        __HAL_RCC_USART1_CLK_ENABLE();                              /* 使能串口时钟 */

        gpio_init_struct.Pin = GPIO_PIN_9;               /* 串口发送引脚号 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;			/* IO速度设置为高速 */
			
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
                
        gpio_init_struct.Pin = GPIO_PIN_10;               /* 串口RX脚 模式设置 */
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;    
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);   /* 串口RX脚 必须设置成输入模式 */
        
//        HAL_NVIC_EnableIRQ(USART1_IRQn);                      /* 使能USART1中断通道 */
//        HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);              /* 组2，最低优先级:抢占优先级3，子优先级3 */
    }
		    if (huart->Instance == USART2)                            /* 如果是串口1，进行串口1 MSP初始化 */
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();                             /* 使能串口脚时钟 */
        __HAL_RCC_USART2_CLK_ENABLE();                              /* 使能串口时钟 */

        gpio_init_struct.Pin = GPIO_PIN_2;               /* 串口发送引脚号 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;			/* IO速度设置为高速 */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
                
        gpio_init_struct.Pin = GPIO_PIN_3;               /* 串口RX脚 模式设置 */
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;
				gpio_init_struct.Pull = GPIO_NOPULL;
				gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;			/* IO速度设置为高速 */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);   /* 串口RX脚 必须设置成输入模式 */
        
        HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);              /* 组2，最低优先级:抢占优先级3，子优先级3 */
			  HAL_NVIC_EnableIRQ(USART2_IRQn);                      /* 使能USART2中断通道 */
    }
}



void USART2_IRQHandler(void)
{
		HAL_UART_IRQHandler(&huart2);
	  while (HAL_UART_Receive_IT(&huart2, rx_buffer, 1) != HAL_OK)     /* 重新开启中断并接收数据 */
    {
        /* 如果出错会卡死在这里 */
    }
}


/**
 * @brief       串口数据接收回调函数
                数据处理在这里进行
 * @param       huart:串口句柄
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		switch(*rx_buffer)
		{
			case '0':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,500);
			printf("\r\n此时舵机输出轴转角为-90°");
			break;
			case '1':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000);
			printf("\r\n此时舵机输出轴转角为-45°");
			break;
			case '2':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1500);
			printf("\r\n此时舵机输出轴转角为0°");
			break;
			case '3':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2000);
			printf("\r\n此时舵机输出轴转角为45°");
			break;
			case '4':
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
			printf("\r\n此时舵机输出轴转角为90°");
			break;
			default:;
		}
	}
}
