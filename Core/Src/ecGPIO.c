/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : SSS LAB
Created          : 05-03-2021
Modified         : 09-20-2022
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"



void LED_toggle(void)
{
	GPIOA -> ODR ^= 1 << LED_PIN;
}


void GPIO_init(GPIO_TypeDef *Port, uint32_t pin, uint32_t mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();

	if (Port == GPIOB)
		RCC_GPIOB_enable();

	if (Port == GPIOC)
		RCC_GPIOC_enable();



	GPIO_mode(Port, pin, mode);
	
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, uint32_t pin, uint32_t mode){
	
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, uint32_t pin, uint32_t speed){

	Port->OSPEEDR &= ~(3UL << (2 * pin)); //reset
	Port->OSPEEDR |= speed << (2 * pin);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, uint32_t pin, uint32_t type){

	Port->OTYPER &= ~(type << pin);
					 
}





// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, uint32_t pin, uint32_t pupd){

	Port->PUPDR &= ~(3UL << (2 * pin)); //reset
	Port->PUPDR |= (pupd << (2 * pin));

}

uint32_t GPIO_read(GPIO_TypeDef *Port, uint32_t pin){

	uint32_t btVal = 0;
	//Read bit value of Button
	btVal = ((Port->IDR) & (1UL << pin))>>pin;

	return btVal;    
}


void GPIO_OUT(GPIO_TypeDef* Port, uint32_t pin, uint32_t speed, uint32_t type, uint32_t pupd)
{
	GPIO_init(Port,  pin,  OUTPUT);
	GPIO_ospeed(Port, pin, speed);
	GPIO_otype(Port, pin, type);
	GPIO_pupd(Port, pin, pupd); 

}

void GPIO_IN(GPIO_TypeDef* Port, uint32_t pin, uint32_t pupd)
{
	GPIO_init(Port,  pin,  INPUT);
	GPIO_pupd(Port,  pin,  pupd);
}

//void GPIO_AF(GPIO_TypeDef* Port, uint32_t pin, uint32_t pupd)
//{
//	GPIO_init(Port,  pin,  AF);
//	GPIO_pupd(Port,  pin,  pupd);
//}

void GPIO_write(GPIO_TypeDef *Port, uint32_t pin, uint32_t Output)
{	
	Port->ODR &= ~(1UL << (pin));  //reset
	Port->ODR |= (Output << pin);
}



void sevensegment_decoder_init(void){

		GPIO_init(GPIOB, 9, OUTPUT);
		GPIO_otype(GPIOB, 9, 0);
		GPIO_pupd(GPIOB, 9, 0);
		GPIO_ospeed(GPIOB, 9, 1);
	
		GPIO_init(GPIOA, 6, OUTPUT);
		GPIO_otype(GPIOA, 6, 0);
		GPIO_pupd(GPIOA, 6, 0);
		GPIO_ospeed(GPIOA, 6, 1);

		GPIO_init(GPIOA, 7, OUTPUT);
		GPIO_otype(GPIOA, 7, 0);
		GPIO_pupd(GPIOA, 7, 0);
		GPIO_ospeed(GPIOA, 7, 1);

		GPIO_init(GPIOB, 6, OUTPUT);
		GPIO_otype(GPIOB, 6, 0);
		GPIO_pupd(GPIOB, 6, 0);
		GPIO_ospeed(GPIOB, 6, 1);

		GPIO_init(GPIOC, 7, OUTPUT);
		GPIO_otype(GPIOC, 7, 0);
		GPIO_pupd(GPIOC, 7, 0);
		GPIO_ospeed(GPIOC, 7, 1);

		GPIO_init(GPIOA, 9, OUTPUT);
		GPIO_otype(GPIOA, 9, 0);
		GPIO_pupd(GPIOA, 9, 0);
		GPIO_ospeed(GPIOA, 9, 1);

		GPIO_init(GPIOA, 8, OUTPUT);
		GPIO_otype(GPIOA, 8, 0);
		GPIO_pupd(GPIOA, 8, 0);
		GPIO_ospeed(GPIOA, 8, 1);

		GPIO_init(GPIOB, 10, OUTPUT);
		GPIO_otype(GPIOB, 10, 0);
		GPIO_pupd(GPIOB, 10, 0);
		GPIO_ospeed(GPIOB, 10, 1);


}

// 
void sevensegment_decoder(uint32_t cnt)
{
	
	uint32_t num[10][8]=
		{
	 // {e,d,c,DP,g,f,a,b}, PA5~PB10
			{0,0,0,1,1,0,0,0},          //zero
			{1,1,0,1,1,1,1,0},          //one
			{0,0,1,1,0,1,0,0},          //two
			{1,0,0,1,0,1,0,0},          //three
			{1,1,0,1,0,0,1,0},          //four
			{1,0,0,1,0,0,0,1},          //five
			{0,0,0,1,0,0,0,1},          //six
			{1,1,0,1,1,1,0,0},          //seven
			{0,0,0,1,0,0,0,0},          //eight
			{1,0,0,1,0,0,0,0},          //nine

		};
			
		GPIO_write(GPIOB, 9, num[cnt][0]);   //e
		GPIO_write(GPIOA, 6, num[cnt][1]);   //d
		GPIO_write(GPIOA, 7, num[cnt][2]);    //c
		GPIO_write(GPIOB, 6, num[cnt][3]);   //DP
		
		GPIO_write(GPIOC, 7, num[cnt][4]);    //g
		GPIO_write(GPIOA, 9, num[cnt][5]);     //f
		GPIO_write(GPIOA, 8, num[cnt][6]);    //a
		GPIO_write(GPIOB, 10, num[cnt][7]);    //b
}