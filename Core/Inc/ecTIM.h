/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
* 
******************************************************************************
*/

#ifndef __EC_TIM_H 
#define __EC_TIM_H

#include "stm32f411xe.h"
#include "stm32f4xx.h"


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */






typedef struct{
	GPIO_TypeDef *port;
	int pin;   
	TIM_TypeDef *timer;
	int ch;  		//int Timer Channel
	int ICnum;  //int IC number
} IC_t;
//Input Capture


void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin);
void ICAP_setup(IC_t *ICx, int IC_number, int edge_type);
void ICAP_counter_us(IC_t *ICx, int usec);

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);

void ICAP_pinmap(IC_t *timer_pin);

















/* Timer Configuration */
void TIM_init(TIM_TypeDef *TIMx, uint32_t msec);  
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);  
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);



void TIM_INT_init(TIM_TypeDef* TIMx, uint32_t msec); 
void TIM_INT_init_usec(TIM_TypeDef* TIMx, uint32_t usec);
	
void TIM_INT_enable(TIM_TypeDef* TIMx);
void TIM_INT_disable(TIM_TypeDef* TIMx);

uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
