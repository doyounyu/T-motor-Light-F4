#ifndef __EC_RCC_H
#define __EC_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//#include "stm32f411xe.h"

#define HSI (0)
#define HSE (1)

void RCC_HSI_init(void);
void RCC_HSE_init(void);
void RCC_PLL_init(unsigned int clkType);
void RCC_GPIOA_enable(void);
void RCC_GPIOB_enable(void);
void RCC_GPIOC_enable(void);
void exam_RCC_PLL_init(int CLKSOURCE, int divM, int multN, int divP);
// void RCC_GPIO_enable(GPIO_TypeDef * GPIOx);

extern int EC_SYSCL;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
