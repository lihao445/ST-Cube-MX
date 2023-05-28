#include "servo_motor.h"

TIM_HandleTypeDef htim1;     /* 定时器x句柄 */
TIM_OC_InitTypeDef htim1oc  = {0};         /* 定时器PWM输出配置 */

void TIM_PWM_Init(uint16_t psc,uint16_t arr)
{
	GPIO_InitTypeDef gpio_init_struct;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();
	
	
	  htim1.Instance = TIM1;                  /* 定时器x */
    htim1.Init.Prescaler = psc;                       /* 定时器分频 */
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;      /* 递增计数模式 */
    htim1.Init.Period = arr;                          /* 自动重装载值 */
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; /*使能TIMx_ARR进行缓冲 */
    htim1.Init.RepetitionCounter = 0;                 /* 重复计数器初始值 */
    while(HAL_TIM_PWM_Init(&htim1) != HAL_OK)			/* 初始化PWM */
		{
		
		}
		
		gpio_init_struct.Pin = GPIO_PIN_8;               								 /* 通道y的GPIO口 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                           /* 复用推完输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                               /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                     /* 高速 */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
		
		
		htim1oc.OCMode = TIM_OCMODE_PWM1;                         /* 模式选择PWM 1*/
    htim1oc.Pulse = 1500;                                  /* 设置比较值,此值用来确定占空比 */
                                            /* 这里默认设置比较值为转0度的比较值,即占空比为7.5% */
    htim1oc.OCPolarity = TIM_OCPOLARITY_HIGH;                 /* 输出比较极性为高 */
    HAL_TIM_PWM_ConfigChannel(&htim1, &htim1oc, TIM_CHANNEL_1); /* 配置TIMx通道y */
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    /* 开启对应PWM通道 */
		
}
