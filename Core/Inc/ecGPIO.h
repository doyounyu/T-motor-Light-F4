/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : SSS LAB
Created          : 05-03-2021
Modified         : 09-20-2022
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/


#include "stm32f411xe.h"
#include "ecRCC.h"
#include <math.h>

/*==============================================================*/
/*                            MACRO                             */
/*==============================================================*/


#define EC_SET_BIT(x, pos, length) (x |= ((unsigned int) (pow(2,  length) - 1) << pos*length))
#define EC_CLEAR_BIT(x, pos, length) (x &= (~((0xFF>>)<< pos*length)))
#define EC_CHECK_BIT(x, pos) (x & (1UL << pos) )



#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  (0x00)
#define OUTPUT (0x01)
#define AF     (0x02)
#define ANALOG (0x03)

//Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
#define LOW_SPEED      (0x00)
#define MEDIUM_SPEED   (0x01)
#define FAST_SPEED     (0x02)
#define HIGH_SPEED     (0x03)


// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
#define PUSH_PULL      (0x00)
#define OPEN_DRAIN   (0x01)


// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
#define NO_PUPD        (0x00)
#define PULLUP         (0x01)
#define PULLDOWN       (0x02)


#define HIGH 1
#define LOW  0

#define LED_PIN 	5
#define BUTTON_PIN 13

#define FALL 0
#define RISE 1
#define BOTH 2

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
void LED_toggle(void);
void GPIO_init(GPIO_TypeDef *Port, uint32_t pin, uint32_t mode);
void GPIO_write(GPIO_TypeDef *Port, uint32_t pin, uint32_t Output);

uint32_t  GPIO_read(GPIO_TypeDef *Port, uint32_t pin);

void GPIO_mode(GPIO_TypeDef* Port, uint32_t pin, uint32_t mode);

void GPIO_ospeed(GPIO_TypeDef* Port, uint32_t pin, uint32_t speed);
void GPIO_otype(GPIO_TypeDef* Port, uint32_t pin, uint32_t type);

void GPIO_pupd(GPIO_TypeDef* Port, uint32_t pin, uint32_t pupd);

void GPIO_OUT(GPIO_TypeDef* Port, uint32_t pin, uint32_t speed, uint32_t type, uint32_t pupd);
void GPIO_IN(GPIO_TypeDef* Port, uint32_t pin, uint32_t pupd);

void sevensegment_decoder(uint32_t cnt);
void sevensegment_decoder_init(void);
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
