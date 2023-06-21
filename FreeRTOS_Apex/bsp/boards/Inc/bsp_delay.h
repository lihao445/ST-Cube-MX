#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "struct_typedef.h"

void delay_init(uint16_t sysclk);       /* 初始化延迟函数 */
void delay_ms(uint16_t nms);            /* 延时nms */
void delay_us(uint32_t nus);            /* 延时nus */

void HAL_Delay(uint32_t Delay);     /* HAL库的延时函数，HAL库内部用到 */


#ifdef __cplusplus
}
#endif


#endif

