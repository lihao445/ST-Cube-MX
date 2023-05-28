#include "led.h"

/**
 * @brief       ��ʼ��LED���IO��, ��ʹ��ʱ��
 * @param       ��
 * @retval      ��
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;
    __HAL_RCC_GPIOC_CLK_ENABLE();                          /* IO��PBʱ��ʹ�� */

    gpio_initstruct.Pin = GPIO_PIN_13;                      /* LED���� */
    gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;            /* ������� */
    gpio_initstruct.Pull = GPIO_NOPULL;                    /* ������������ */
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_LOW;         	 /* ���� */
    HAL_GPIO_Init(GPIOC, &gpio_initstruct);                /* ��ʼ��LED0���� */

}
