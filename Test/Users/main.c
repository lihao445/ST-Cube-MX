/**
 ******************************************************************************
 * @file     main.c
 * @author   正点原子团队(ALIENTEK)
 * @version  V1.0
 * @date     2020-08-20
 * @brief    新建工程实验-HAL库版本 实验
 * @license  Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ******************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 STM32F103 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 ******************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "led.h"
#include "servo_motor.h"

int main(void)
{
    HAL_Init();                      /* 初始化HAL库 */
		SystemClock_Config();            /* 设置时钟, 16Mhz */
    delay_init(16);                  /* 延时初始化 */
		usart1_init(115200);							//初始化串口调试助手
		usart2_init(115200);							//初始化蓝牙模块串口
		TIM_PWM_Init(16-1,20000-1);				//初始化舵机
    led_init();                      /* LED初始化 */
	
    while(1)
    { 
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);    /* PC13置1 */ 
        delay_ms(500);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);  /* PC13置0 */
        delay_ms(500); 
    }
}


