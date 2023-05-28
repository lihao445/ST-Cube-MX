#include "led.h"

/**
 * @brief       初始化LED相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;
    __HAL_RCC_GPIOC_CLK_ENABLE();                          /* IO口PB时钟使能 */

    gpio_initstruct.Pin = GPIO_PIN_13;                      /* LED引脚 */
    gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_initstruct.Pull = GPIO_NOPULL;                    /* 不上拉不下了 */
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_LOW;         	 /* 低速 */
    HAL_GPIO_Init(GPIOC, &gpio_initstruct);                /* 初始化LED0引脚 */

}
