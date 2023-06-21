#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "struct_typedef.h"

void delay_init(uint16_t sysclk);       /* ��ʼ���ӳٺ��� */
void delay_ms(uint16_t nms);            /* ��ʱnms */
void delay_us(uint32_t nus);            /* ��ʱnus */

void HAL_Delay(uint32_t Delay);     /* HAL�����ʱ������HAL���ڲ��õ� */


#ifdef __cplusplus
}
#endif


#endif

