/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
* 
******************************************************************************
*/


#include "ecTIM.h"
#include "ecGPIO.h"

/* Timer Configuration */

void TIM_init(TIM_TypeDef* TIMx, uint32_t msec){ 
	
// 1. Enable Timer CLOCK
	if     (TIMx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(TIMx ==TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(TIMx ==TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(TIMx ==TIM4)  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(TIMx ==TIM5)  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(TIMx ==TIM9)  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	else if(TIMx ==TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

	
	
// 2. Set CNT period
	TIM_period_ms(TIMx, msec); 
	
	
// 3. CNT Direction
	TIMx->CR1 |=    TIM_CR1_DIR;					// Upcounter	
	// timerx->CR1 |= TIM_CR1_DIR;				// Downcounter	
	
	
// 4. Enable Timer Counter
	TIMx->CR1 |= TIM_CR1_CEN;		
}



void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec)
	{   
	// Period usec = 1 to 1000
	// 1us(1MHz, ARR=1) to 65msec (ARR=0xFFFF)
	uint32_t prescaler;
	uint32_t ARRval_32;
	uint16_t ARRval_16;
	
	// Check System CLK: PLL or HSI
	if     ((RCC->CFGR & (3<<0)) == 2)      { prescaler = 96; }
	else if((RCC->CFGR & (3<<0)) == 0)      { prescaler = 16; }
	
			if (TIMx== TIM2 || TIMx == TIM5) 
			{
				ARRval_32 = usec ;
				TIMx->PSC = prescaler-1;					
				TIMx->ARR = ARRval_32-1;	
			}
			
			else
			{
				ARRval_16 = (uint16_t)( usec) ;
				TIMx->PSC = prescaler-1;					
				TIMx->ARR = ARRval_16-1;	
			
			}			
	
	
}


void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec){ 

	uint16_t prescaler;
	uint32_t ARRval_32;
	uint16_t ARRval_16;
	

	// Check System CLK: PLL or HSI
	if((RCC->CFGR & (3<<0)) == 2)      { prescaler = 9600; }  // for msec 84MHz/1000
	else if((RCC->CFGR & (3<<0)) == 0) { prescaler = 1600; }
	

	
	
  //	uint32_t prescaler = 8400;
	
			if (TIMx== TIM2 || TIMx == TIM5) 
			{
				ARRval_32 = 10 * msec ;
				TIMx->PSC = prescaler-1;					
				TIMx->ARR = ARRval_32-1;	
			}
			
			else
			{
				ARRval_16 = (uint16_t)(10 * msec) ;
				TIMx->PSC = prescaler-1;					
				TIMx->ARR = ARRval_16-1;	
			
			}
	
	
		

}
// Update Event Interrupt
void TIM_INT_init(TIM_TypeDef* TIMx, uint32_t msec){
// 1. Initialize Timer	
	TIM_init(TIMx,msec);
	
// 2. Enable Update Interrupt
	TIM_INT_enable(TIMx);
	
// 3. NVIC Setting
	uint32_t IRQn_reg =0;
	if     (TIMx == TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(TIMx == TIM2)       IRQn_reg = TIM2_IRQn;
	else if(TIMx == TIM3)       IRQn_reg = TIM3_IRQn;
	else if(TIMx ==TIM4)   			IRQn_reg = TIM4_IRQn;
	else if(TIMx ==TIM5)   			IRQn_reg = TIM5_IRQn;
	else if(TIMx ==TIM9)   			IRQn_reg = TIM1_BRK_TIM9_IRQn;
	else if(TIMx ==TIM11)  			IRQn_reg = TIM1_TRG_COM_TIM11_IRQn;
	
				
	NVIC_SetPriority(IRQn_reg,2);
	NVIC_EnableIRQ(IRQn_reg);
	
}



void TIM_INT_enable(TIM_TypeDef* TIMx){
	TIMx->DIER |=   TIM_DIER_UIE;			// Enable Timer Update Interrupt		
}

void TIM_INT_disable(TIM_TypeDef* TIMx){
	TIMx->DIER &= ~(TIM_DIER_UIE);				// Disable Timer Update Interrupt		
}

uint32_t is_UIF(TIM_TypeDef *TIMx){
	return TIMx->SR & TIM_DIER_UIE;
}

void clear_UIF(TIM_TypeDef *TIMx){
	TIMx->SR &= ~TIM_DIER_UIE;
}


void TIM_INT_init_usec(TIM_TypeDef* TIMx, uint32_t usec){

// 1. Enable Timer CLOCK
	if     (TIMx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(TIMx ==TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(TIMx ==TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(TIMx ==TIM4)  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(TIMx ==TIM5)  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(TIMx ==TIM9)  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	else if(TIMx ==TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

	
// 2. Set CNT period
	TIM_period_us(TIMx, usec); 
	
	
// 3. CNT Direction
	TIMx->CR1 |=    TIM_CR1_DIR;					// Upcounter	
	// timerx->CR1 |= TIM_CR1_DIR;				// Downcounter	
	
	
// 4. Enable Timer Counter
	TIMx->CR1 |= TIM_CR1_CEN;		
	
// 2. Enable Update Interrupt
	TIM_INT_enable(TIMx);
	
// 3. NVIC Setting
	uint32_t IRQn_reg =0;
	if     (TIMx == TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(TIMx == TIM2)       IRQn_reg = TIM2_IRQn;
	else if(TIMx == TIM3)       IRQn_reg = TIM3_IRQn;
	else if(TIMx ==TIM4)   			IRQn_reg = TIM4_IRQn;
	else if(TIMx ==TIM5)   			IRQn_reg = TIM5_IRQn;
	else if(TIMx ==TIM9)   			IRQn_reg = TIM1_BRK_TIM9_IRQn;
	else if(TIMx ==TIM11)  			IRQn_reg = TIM1_TRG_COM_TIM11_IRQn;
	
				
	NVIC_SetPriority(IRQn_reg,2);
	NVIC_EnableIRQ(IRQn_reg);
	
}


/* -------- Timer Input Capture -------- */

void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin){
// 0. Match Input Capture Port and Pin for TIMx
	ICx->port = port;
	ICx->pin  = pin;
	ICAP_pinmap(ICx);	  										// Port, Pin --(mapping)--> TIMx, Channel
	
	TIM_TypeDef *TIMx = ICx->timer;
	int TIn = ICx->ch; 		
	int ICn = TIn;
	ICx->ICnum = ICn;													// (default) TIx=ICx

// GPIO configuration ---------------------------------------------------------------------	
// 1. Initialize GPIO port and pin as AF
	GPIO_init(port, pin, AF);   							// GPIO init as AF=2
	GPIO_ospeed(port, pin, HIGH);  						// speed VHIGH=3	

// 2. Configure GPIO AFR by Pin num.
	if     (TIMx == TIM1 || TIMx == TIM2)					    	       port->AFR[pin >> 3] |= 0x01U << (4*(pin % 8));
	else if(TIMx == TIM3 || TIMx == TIM4  || TIMx == TIM5)     port->AFR[pin >> 3] |= 0x02U << (4*(pin % 8));  
  else if(TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11)    port->AFR[pin >> 3] |= 0x03U << (4*(pin % 8));
	
// TIMER configuration ---------------------------------------------------------------------			
// 1. Initialize Timer 
	TIM_init(TIMx, 1);
// 2. Initialize Timer Interrpt 
	TIM_INT_init(TIMx, 1);        					// TIMx Interrupt initialize 
// 3. Modify ARR Maxium for 1MHz
	TIMx->PSC = 84-1;						  					// Timer counter clock: 1MHz(1us)  for PLL
	TIMx->ARR = 0xFFFF;											// Set auto reload register to maximum (count up to 65535)
// 4. Disable Counter during configuration
	TIMx->CR1 &= ~TIM_CR1_CEN;  						// Disable Counter during configuration


	
// Input Capture configuration ---------------------------------------------------------------------			
// 1. Select Timer channel(TIx) for Input Capture channel(ICx)
	// Default Setting

	TIMx->CCMR1 |= 	TIM_CCMR1_CC1S_0;              	//01<<0   CC1S    TI1=IC1
	TIMx->CCMR1 |= 	TIM_CCMR1_CC2S_0;  				     	//01<<8   CC2s    TI2=IC2
	TIMx->CCMR2 |= 	TIM_CCMR2_CC3S_0;      		  		//01<<0   CC3s    TI3=IC3
	TIMx->CCMR2 |= 	TIM_CCMR2_CC4S_0;				      	//01<<8   CC4s    TI4=IC4


// 2. Filter Duration (use default)

// 3. IC Prescaler (use default)

// 4. Activation Edge: CCyNP/CCyP	
	TIMx->CCER &= ~(15UL<<((ICn-1)*4));     // clear

// 5.	Enable CCy Capture, Capture/Compare interrupt
  TIMx->CCER &= ~(5UL<<(((ICn-1)*4)+1));					// CCn(ICn) Capture Enable	

// 6.	Enable Interrupt of CC(CCyIE), Update (UIE)
	TIMx->DIER |= 1UL<<ICn;					// Capture/Compare Interrupt Enable	for ICn
	TIMx->DIER |= TIM_DIER_UIE;							// Update Interrupt enable	

// 7.	Enable Counter 
	TIMx->CR1	 |= TIM_CR1_CEN;							// Counter enable	
}


// Configure Selecting TIx-ICy and Edge Type
void ICAP_setup(IC_t *ICx, int ICn, int edge_type){
	TIM_TypeDef *TIMx = ICx->timer;	// TIMx
	int 				CHn 	= ICx->ch;		// Timer Channel CHn
	ICx->ICnum = ICn;

// Disable  CC. Disable CCInterrupt for ICn. 
  TIMx->CCER &= ~(1UL<<(4*(ICn-1)));   		    // Capture Disable
	TIMx->DIER &= ~(1UL<<ICn);						    // CCn Interrupt Disable	
	
// Configure  IC number(user selected) with given IC pin(TIMx_CHn)
	switch(ICn){
			case 1:
					TIMx->CCMR1 &= ~TIM_CCMR1_CC1S;											//reset   CC1S
					if   (ICn==CHn) TIMx->CCMR1 |= TIM_CCMR1_CC1S_0;    //01<<0   CC1S    Tx_Ch1=IC1
					else TIMx->CCMR1 |= TIM_CCMR1_CC1S_1;               //10<<0   CC1S    Tx_Ch2=IC1
					break;
			case 2:
					TIMx->CCMR1 &= ~TIM_CCMR1_CC2S;								  		//reset   CC2S
					if   (ICn==CHn) TIMx->CCMR1 |= 	TIM_CCMR1_CC2S_0;   //01<<0   CC2S    Tx_Ch2=IC2
					else TIMx->CCMR1 |= TIM_CCMR1_CC2S_1;       				//10<<0   CC2S    Tx_Ch1=IC2
					break;
			case 3:
					TIMx->CCMR2 &= ~TIM_CCMR2_CC3S;											//reset   CC3S
					if   (ICn==CHn) TIMx->CCMR2 |= TIM_CCMR2_CC3S_0;	  //01<<0   CC3S    Tx_Ch3=IC3
					else TIMx->CCMR2 |= TIM_CCMR2_CC3S_1; 	     				//10<<0   CC3S    Tx_Ch4=IC3
					break;
			case 4:
					TIMx->CCMR2 &= ~TIM_CCMR2_CC4S;									  	//reset   CC4S
					if   (ICn==CHn) TIMx->CCMR2 |= TIM_CCMR2_CC4S_0;	  //01<<0   CC4S    Tx_Ch4=IC4
					else TIMx->CCMR2 |= TIM_CCMR2_CC4S_1;     					//10<<0   CC4S    Tx_Ch3=IC4
					break;
			default: break;
		}


// Configure Activation Edge direction
	TIMx->CCER &= ~(5<<(((ICn-1)*4)+1));	  									// Clear CCnNP/CCnP bits for ICn
	switch(edge_type){
		case RISE: TIMx->CCER &= ~(5<<(((ICn-1)*4)+1));	 break; //rising:  00
		case FALL: TIMx->CCER |= (1<<(((ICn-1)*4)+1));	 break; //falling: 01
		case BOTH: TIMx->CCER |= (5<<(((ICn-1)*4)+1));	 break; //both:    11
	}
	
// Enable CC. Enable CC Interrupt. 
	TIMx->CCER |= 1 << (4*(ICn - 1)); 										// Capture Enable
	TIMx->DIER |= 1 << ICn; 															// CCn Interrupt enabled	
}

// Time span for one counter step
void ICAP_counter_us(IC_t *ICx, int usec){	
	TIM_TypeDef *TIMx = ICx->timer;	
	TIMx->PSC = 84*usec-1;						  // Timer counter clock: 1us * usec
	TIMx->ARR = 0xFFFF;									// Set auto reload register to maximum (count up to 65535)
}

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum){
	switch(ccNum){
    case 1 : return ((TIMx->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF);
    case 2 : return ((TIMx->SR & TIM_SR_CC2IF) == TIM_SR_CC2IF);
    case 3 : return ((TIMx->SR & TIM_SR_CC3IF) == TIM_SR_CC3IF);
    case 4 : return ((TIMx->SR & TIM_SR_CC4IF) == TIM_SR_CC4IF);
  }
}

void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum){
	switch(ccNum){
    case 1 : TIMx->SR &= ~TIM_SR_CC1IF; break;
    case 2 : TIMx->SR &= ~TIM_SR_CC2IF;	break;
    case 3 : TIMx->SR &= ~TIM_SR_CC3IF; break;
    case 4 : TIMx->SR &= ~TIM_SR_CC4IF;	break;
  }
}

//DO NOT MODIFY THIS
void ICAP_pinmap(IC_t *timer_pin){
   GPIO_TypeDef *port = timer_pin->port;
   int pin = timer_pin->pin;
   
   if(port == GPIOA) {
      switch(pin){
         case 0 : timer_pin->timer = TIM2; timer_pin->ch = 1; break;
         case 1 : timer_pin->timer = TIM2; timer_pin->ch = 2; break;
         case 5 : timer_pin->timer = TIM2; timer_pin->ch = 1; break;
         case 6 : timer_pin->timer = TIM3; timer_pin->ch = 1; break;
         //case 7: timer_pin->timer = TIM1; timer_pin->ch = 1N; break;
         case 8 : timer_pin->timer = TIM1; timer_pin->ch = 1; break;
         case 9 : timer_pin->timer = TIM1; timer_pin->ch = 2; break;
         case 10: timer_pin->timer = TIM1; timer_pin->ch = 3; break;
         case 15: timer_pin->timer = TIM2; timer_pin->ch = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: timer_pin->timer = TIM1; timer_pin->ch = 2N; break;
         //case 1: timer_pin->timer = TIM1; timer_pin->ch = 3N; break;
         case 3 : timer_pin->timer = TIM2; timer_pin->ch = 2; break;
         case 4 : timer_pin->timer = TIM3; timer_pin->ch = 1; break;
         case 5 : timer_pin->timer = TIM3; timer_pin->ch = 2; break;
         case 6 : timer_pin->timer = TIM4; timer_pin->ch = 1; break;
         case 7 : timer_pin->timer = TIM4; timer_pin->ch = 2; break;
         case 8 : timer_pin->timer = TIM4; timer_pin->ch = 3; break;
         case 9 : timer_pin->timer = TIM4; timer_pin->ch = 3; break;
         case 10: timer_pin->timer = TIM2; timer_pin->ch = 3; break;
         
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : timer_pin->timer = TIM3; timer_pin->ch = 1; break;
         case 7 : timer_pin->timer = TIM3; timer_pin->ch = 2; break;
         case 8 : timer_pin->timer = TIM3; timer_pin->ch = 3; break;
         case 9 : timer_pin->timer = TIM3; timer_pin->ch = 4; break;
         
         default: break;
      }
   }
}


