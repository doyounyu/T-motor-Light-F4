/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sumd.h"
#include "ICM20602.h"
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecPWM.h"
#include "ecGPIO.h"
#include "ecTIM.h"
#include <stdio.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// To use UART printf() Function
int _write(int file, char *p, int len)
{

    for(int i = 0; i < len; ++i)
    {
    	LL_USART_TransmitData8(USART1, *(p+i));  //send data to usart1
    	LL_mDelay(1);
    }

    return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Number of Channel in Receiver.
 * Check the 3rd sumd_rx_buf array value(sumd_rx_buf[2]) to see how many channel that your SUMD Receiver Uses*/
#define CH_NUM (uint8_t)(16)
#define SUMD_BUFFER_LENGTH (uint8_t)((CH_NUM + 1)*2 + 2)
#define GAYJOYGO (double)( 0.25)
#define DEG2RAD (double)(0.017453)
#define RAD2DEG (double)(57.295779)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*SUMD related variables---------------------------------------------------------*/
extern uint8_t sumd_rx_buf[37];		// sumd buffer
extern uint8_t sumd_rx_cplt_flag; // sumd read complete flag

extern uint8_t uart2_rx_flag;
extern uint8_t uart2_rx_data;


/*UART command parsing related variables-----------------------------------------*/

extern uint8_t uart1_rx_flag;
extern uint8_t uart1_rx_data;

/*Gyro offset related varables---------------------------------------------------*/
double gyro_x_offset = 0.00;
double gyro_y_offset = 0.00;
double gyro_z_offset = 0.00;

/*Timer for Control loop related variables---------------------------------------*/

extern uint8_t tim2_1ms_flag;

/*PID gain related variables-----------------------------------------------------*/

double p_gain_roll = 0.8F; double p_gain_pitch = 0.8F;double p_gain_yaw = 0.0F;
double i_gain_roll = 0.052F; double i_gain_pitch = 0.052F;double i_gain_yaw = 0.0F;
double d_gain_roll = 0.1F; double d_gain_pitch = 0.1F;double d_gain_yaw = 0.0F;

double pitch_rate_Kp=6.0; double roll_rate_Kp=6.0;double yaw_rate_Kp=15.0;
double pitch_rate_Ki=5.0; double roll_rate_Ki=5.0;double yaw_rate_Ki=0.0;
double pitch_rate_Kd=1.5; double roll_rate_Kd=1.5;double yaw_rate_Kd=2.0;


double pitch_ref =0.0;
double pitch_p=0.0;
double pitch_i=0.0;
double pitch_d=0.0;
double pitch_error=0.0;
double pitch_error_sum=0.0;
double pitch_error_derivative=0.0;
double pitch_pid=0.0;

double pitch_rate_ref =0.0;
double pitch_rate_p=0.0;
double pitch_rate_i=0.0;
double pitch_rate_d=0.0;
double pitch_rate_error=0.0;
double pitch_rate_error_sum=0.0;
double pitch_rate_error_derivative=0.0;
double pitch_rate_pid=0.0;

double roll_ref =0.0;
double roll_p=0.0;
double roll_i=0.0;
double roll_d=0.0;
double roll_error=0.0;
double roll_error_sum=0.0;
double roll_error_derivative=0.0;
double roll_pid=0.0;

double roll_rate_ref =0.0;
double roll_rate_p=0.0;
double roll_rate_i=0.0;
double roll_rate_d=0.0;
double roll_rate_error=0.0;
double roll_rate_error_sum=0.0;
double roll_rate_error_derivative=0.0;
double roll_rate_pid=0.0;

double yaw_ref =0.0;
double yaw_p=0.0;
double yaw_i=0.0;
double yaw_d=0.0;
double yaw_error=0.0;
double yaw_error_sum=0.0;
double yaw_error_derivative=0.0;
double yaw_pid=0.0;

double yaw_rate_ref =0.0;
double yaw_rate_p=0.0;
double yaw_rate_i=0.0;
double yaw_rate_d=0.0;
double yaw_rate_error=0.0;
double yaw_rate_error_sum=0.0;
double yaw_rate_error_derivative=0.0;
double yaw_rate_pid=0.0;

double ICM20602gyro_x_prev=0.0;
double ICM20602gyro_y_prev=0.0;
double ICM20602gyro_z_prev=0.0;

/*Final motor write value variables----------------------------------------------*/
double Motor1_pulsewidth = 0.0;	double Motor2_pulsewidth = 0.0;
double Motor3_pulsewidth = 0.0;	double Motor4_pulsewidth = 0.0;

double stick_offset_us = 187.5; //(250 - 150) / 2 us

/*PWM pin structures and variables*/
typedef struct
{
	GPIO_TypeDef *port;
	uint16_t pin;
} _Pin;


_Pin MOTOR_pin[4] =
{
	{GPIOB, 4}, // TIM3 Ch4
	{GPIOB, 5},	// TIM3 Ch5
	{GPIOB, 6}, // TIM4 Ch6
	{GPIOB, 7}  // TIM4 Ch7
};

PWM_t MOTOR[4];

double oneShot125[16] = {0.0,};

int gyro_offset_cnt = 0;
int arm_flag =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void motor_setup(void);
uint8_t IsGyroCalibrated(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  motor_setup();

  TIM_INT_init(TIM2, 1);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Waiting for Pin Initialization...\n\r");
  LL_mDelay(2000);

  ICM20602_Initialization();


  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_RXNE(USART1);  //USART bluetooth or usb cabe intterupt enable



  printf("USART2 pin for SUMD Interrupt enabled\n\r");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {






	  /*===============================================================*/
	  /*                    1kHz Control loop Part                     */
	  /*===============================================================*/
	  if((tim2_1ms_flag == 1) && (IsGyroCalibrated()==1))
	  {
		  LL_GPIO_TogglePin(LEDpin_GPIO_Port, LEDpin_Pin);
		  tim2_1ms_flag = 0;
		  //PITCH PID CONTROL
		  pitch_ref		   		 = (oneShot125[2] - stick_offset_us)*0.8f;// -50 ~ 50 deg
		  pitch_error 	   		 = pitch_ref - ICM20602.angle_pitch;
		  pitch_error_sum 		+= pitch_error * 0.001;
		  if (sumd_ch.ch1< 8725 || arm_flag == 0) pitch_error_sum = 0.0;
		  (pitch_error_sum  >  50.0) ? (pitch_error_sum=  50.0) :
		  (pitch_error_sum  < -50.0) ? (pitch_error_sum= -50.0) :0 ;
		  pitch_error_derivative = ICM20602.gyro_x;

		  pitch_p 		   		 = p_gain_pitch * pitch_error;
		  pitch_i          		 = i_gain_pitch * pitch_error_sum;
		  pitch_d 			     = d_gain_pitch * pitch_error_derivative;
		  pitch_pid 			 = pitch_p + pitch_i + pitch_d;


		  //PITCH RATE PID CONTROLL
		  pitch_rate_ref = pitch_pid;
		  pitch_rate_error = pitch_rate_ref-ICM20602.gyro_x;
		  pitch_rate_error_sum += pitch_rate_error*0.001;
		  if (sumd_ch.ch1<8725 || arm_flag == 0)     pitch_rate_error_sum = 0.0;
		  //(pitch_rate_error_sum  >  50.0) ? (pitch_rate_error_sum= 50.0) :
		  //(pitch_rate_error_sum  < -50.0) ? (pitch_rate_error_sum=-50.0) : 0;
		  pitch_rate_error_derivative = (ICM20602.gyro_x - ICM20602gyro_x_prev)/0.001;
		  ICM20602gyro_x_prev = ICM20602.gyro_x;
		  pitch_rate_p = pitch_rate_Kp * pitch_rate_error;
		  pitch_rate_i = pitch_rate_Ki * pitch_rate_error_sum;
		  pitch_rate_i = pitch_rate_Kd * pitch_rate_error_derivative;
		  pitch_rate_pid = pitch_rate_p + pitch_rate_i + pitch_rate_d;


		  //ROLL PID CONTROLL
		  roll_ref		   		 = (oneShot125[1] - stick_offset_us) * 0.8f;// -50 ~ 50 deg
		  roll_error 	   		 = roll_ref 	  - ICM20602.angle_roll;
		  roll_error_sum 		+= roll_error     * 0.001;
		  roll_error_derivative  = ICM20602.gyro_y;
		  if (sumd_ch.ch1<8725 || arm_flag == 0)     roll_error_sum = 0.0;
		  (roll_error_sum  >  50.0) ? (roll_error_sum= 50.0) :
		  (roll_error_sum  < -50.0) ? (roll_error_sum=-50.0) : 0;

		  roll_p 		   		 = p_gain_roll    * roll_error;
		  roll_i          		 = i_gain_roll    * roll_error_sum;
		  roll_d 			     = d_gain_roll    * roll_error_derivative;
		  roll_pid 			     = roll_p + roll_i + roll_d;

		  //ROLL RATE PID CONTROLL
		  roll_rate_ref = roll_pid;
		  roll_rate_error = roll_rate_ref-ICM20602.gyro_y;
		  roll_rate_error_sum += roll_rate_error*0.001;
		  if (sumd_ch.ch1<8725 || arm_flag == 0)     roll_rate_error_sum = 0.0;
		  //(roll_rate_error_sum  >  50.0) ? (roll_rate_error_sum= 50.0) :
		  //(roll_rate_error_sum  < -50.0) ? (roll_rate_error_sum=-50.0) : 0;
		  roll_rate_error_derivative = (ICM20602.gyro_y - ICM20602gyro_y_prev)/0.001;
		  ICM20602gyro_y_prev=ICM20602.gyro_y;
		  roll_rate_p = roll_rate_Kp * roll_rate_error;
		  roll_rate_i = roll_rate_Ki * roll_rate_error_sum;
		  roll_rate_d = roll_rate_Kd * roll_rate_error_derivative;
		  roll_rate_pid = roll_rate_p + roll_rate_i + roll_rate_d;



		  //yaw RATE PID CONTROLL
		  yaw_rate_ref = (oneShot125[3] - stick_offset_us) * 8.0f;
		  yaw_rate_error = yaw_rate_ref-ICM20602.gyro_z;
		  yaw_rate_error_sum += yaw_rate_error*0.001;
		  if (sumd_ch.ch1<8725 || arm_flag == 0)     yaw_rate_error_sum = 0.0;
		  //(yaw_rate_error_sum  >  50.0) ? (yaw_rate_error_sum= 50.0) :
		  //(yaw_rate_error_sum  < -50.0) ? (yaw_rate_error_sum=-50.0) : 0;
		  yaw_rate_error_derivative = (ICM20602.gyro_z - ICM20602gyro_z_prev)/0.001;
		  ICM20602gyro_z_prev=ICM20602.gyro_z;
		  yaw_rate_p = yaw_rate_Kp * yaw_rate_error;
		  yaw_rate_i = yaw_rate_Ki * yaw_rate_error_sum;
		  yaw_rate_d = yaw_rate_Kd * yaw_rate_error_derivative;
		  yaw_rate_pid = yaw_rate_p + yaw_rate_i + yaw_rate_d;

		  /**
		   * oneShot125[0] --> Throttle
		   * oneShot125[1] --> Roll
		   * oneShot125[2] --> Pitch
		   * oneShot125[3] --> Yaw
		   * oneShot125[4] --> Arming
		   */

		  /**	 FRT
		   * [1]     [2]
		   *    *   *
		   *      *
		   *    *   *
		   * [4]     [3]
		   *	 BCK
		   */
/*
		  Motor1_pulsewidth = oneShot125[0] - roll_pid  - pitch_pid  - 0.5 *  (oneShot125[3] - stick_offset_us);
		  Motor2_pulsewidth = oneShot125[0] + roll_pid  - pitch_pid  + 0.5 *  (oneShot125[3] - stick_offset_us);
		  Motor3_pulsewidth = oneShot125[0] + roll_pid  + pitch_pid  - 0.5 *  (oneShot125[3] - stick_offset_us);
		  Motor4_pulsewidth = oneShot125[0] - roll_pid  + pitch_pid  + 0.5 *  (oneShot125[3] - stick_offset_us);
*/

			  Motor1_pulsewidth = oneShot125[0]  - pitch_pid - roll_pid - 0.5*(oneShot125[3] - stick_offset_us) ;
			  Motor2_pulsewidth = oneShot125[0]  - pitch_pid + roll_pid + 0.5*(oneShot125[3] - stick_offset_us) ;
			  Motor3_pulsewidth = oneShot125[0]  + pitch_pid + roll_pid - 0.5*(oneShot125[3] - stick_offset_us) ;
			  Motor4_pulsewidth = oneShot125[0]  + pitch_pid - roll_pid + 0.5*(oneShot125[3] - stick_offset_us) ;




	  }
	  /*===============================================================*/
	  /*                Gyro Sensor Processing Part                    */
	  /*===============================================================*/
	  if(ICM20602_DataReady() == 1)
		{

			if( gyro_offset_cnt<5000)
			{
				gyro_offset_cnt++;
				gyro_x_offset +=ICM20602.gyro_x_raw;
				gyro_y_offset +=ICM20602.gyro_y_raw;
				gyro_z_offset +=ICM20602.gyro_z_raw;
			}
			else if(gyro_offset_cnt == 5000)
			{
				gyro_offset_cnt++;
				gyro_x_offset /=5000.0;
				gyro_y_offset /=5000.0;
				gyro_z_offset /=5000.0;
			}
			else
			{


				ICM20602_Get6AxisRawData(&ICM20602.acc_x_raw, &ICM20602.gyro_x_raw);

				//Raw data to deg/s conversion with sensor direction conversion. check AFS_SEL value for Gyro sensor Register!
				ICM20602.gyro_x =   (float)(  ICM20602.gyro_y_raw/16.4f - gyro_y_offset);
				ICM20602.gyro_y =   (float)(-(ICM20602.gyro_x_raw/16.4f - gyro_x_offset));
				ICM20602.gyro_z =   (float)(  ICM20602.gyro_z_raw/16.4f - gyro_z_offset);

				//Complementary filter

				ICM20602.angle_pitch += ICM20602.gyro_x * 0.001; //(sampling frequency: 1kHz)
				ICM20602.angle_roll  += ICM20602.gyro_y * 0.001;
				ICM20602.angle_yaw   += ICM20602.gyro_z * 0.001;

				ICM20602.angle_pitch += ICM20602.angle_roll * sin(ICM20602.gyro_z * 0.001 * DEG2RAD);
				ICM20602.angle_roll -= ICM20602.angle_pitch * sin(ICM20602.gyro_z * 0.001 * DEG2RAD);


				ICM20602.acc_total_vect = sqrt((ICM20602.acc_x_raw*ICM20602.acc_x_raw)
											  +(ICM20602.acc_y_raw*ICM20602.acc_y_raw)
											  +(ICM20602.acc_z_raw*ICM20602.acc_z_raw));


				ICM20602.angle_pitch_acc = asin((float)(ICM20602.acc_x_raw /ICM20602.acc_total_vect))*RAD2DEG;
				ICM20602.angle_roll_acc  = asin((float)(-ICM20602.acc_y_raw)/ICM20602.acc_total_vect)*-RAD2DEG;

				ICM20602.angle_pitch = (ICM20602.angle_pitch * 0.95f + ICM20602.angle_pitch_acc * 0.05f);
				ICM20602.angle_roll  = (ICM20602.angle_roll *  0.95f + ICM20602.angle_roll_acc  * 0.05f);

				//printf("%.2f,%.2f\n\r",ICM20602.angle_pitch_acc,ICM20602.angle_roll_acc);

				//if (printcnt == 1000)
				//{
					//printf("%.2f,%.2f\n\r",-(ICM20602.angle_pitch + 0.8f),-(ICM20602.angle_roll - 1.0f)) ;
					//printf("%.2f,%.2f,%.2f\n\r",ICM20602.gyro_x,ICM20602.gyro_y,ICM20602.gyro_z) ;
					//printf("%.2f,%.2f\n\r",ICM20602.angle_pitch_acc,ICM20602.angle_roll_acc);
					//printcnt =0;
				//}


			}

		}


	  /*===============================================================*/
	  /*                     PID gain Tuning Part                      */
	  /*===============================================================*/

/*
	  if(uart1_rx_flag == 1)
		  {
			  uart1_rx_flag = 0;

			  //print pid gain. roll==pitch
			  //printf("\n\tP_roll\tI_roll\tD_roll\tx10e-3\n\r");


			  switch (uart1_rx_data)
			  {

				  case 'q':
					  p_gain_roll  += 0.025;
					  p_gain_pitch += 0.025;
					  break;

				  case 'a':
					  p_gain_roll  -= 0.025;
					  p_gain_pitch -= 0.025;
					  break;

				  case 'w':
					  i_gain_roll  += 0.0005;
					  i_gain_pitch += 0.0005;
					  break;

				  case 's':
					  i_gain_roll  -= 0.0005;
					  i_gain_pitch -= 0.0005;
					  break;

				  case 'e':
					  d_gain_roll  += 0.025;
					  d_gain_pitch += 0.025;
					  break;

				  case 'd':
					  d_gain_roll  -= 0.1;
					  d_gain_pitch -= 0.1;
					  break;
				  //------------------------------------------------ RATE GAIN-----------
				  case 'r':
					  pitch_rate_Kp  += 0.001;
					  roll_rate_Kp   += 0.001;
					  break;

				  case 'f':
					  pitch_rate_Kp  -= 0.001;
					  roll_rate_Kp   -= 0.001;
					  break;

				  case 't':
					  pitch_rate_Ki  += 0.1;
					  roll_rate_Ki   += 0.1;
					  break;

				  case 'g':
					  pitch_rate_Ki  -= 0.1;
					  roll_rate_Ki   -= 0.1;
					  break;

				  case 'y':
					  pitch_rate_Kd  += 0.001;
					  roll_rate_Kd   += 0.001;
					  break;

				  case 'h':
					  pitch_rate_Kd  -= 0.001;
					  roll_rate_Kd   -= 0.001;
					  break;

			  }
			  printf("P: %.3lf\tI: %.3lf\tD: %.3lf\trateP: %.3lf\trateI: %.3lf\trateD: %.3lf\n\r", p_gain_roll, i_gain_roll, d_gain_roll,pitch_rate_Kp,pitch_rate_Ki,pitch_rate_Kd);

		  }

*/


		  /*===============================================================*/
		  /*                       SUMD Receiver part                      */
		  /*===============================================================*/

		  if((sumd_rx_cplt_flag == 1) && (IsGyroCalibrated() == 1)) //Check whether sumd_rx_buf Receive data is completely filled
		  {
			  sumd_rx_cplt_flag = 0;
			  if(CRC16_check(sumd_rx_buf,  SUMD_BUFFER_LENGTH - 1) == 0) //Check if sumd_rx_buf passed "CRC-16 Sick" Test
			  {
				  SUMD_parsing(sumd_rx_buf, &sumd_ch);
				  ch_data_to_oneShot125(oneShot125, &sumd_ch);

				 // printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n\r",  sumd_ch.ch1, sumd_ch.ch2, sumd_ch.ch3, sumd_ch.ch4, sumd_ch.ch5, sumd_ch.ch6, sumd_ch.ch7, sumd_ch.ch8);
				 //printf("%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t\n\r", oneShot125[0],oneShot125[1],oneShot125[2],oneShot125[3],oneShot125[4],oneShot125[5],oneShot125[6],oneShot125[7]);


				  /*===============================================================*/
				  /*                        Motor Write Part                       */
				  /*===============================================================*/
				  /*PWM Write with pulsewidth, added min/max saturation for motor pulse width, which is 120us to 250us*/

				  //Arming Flag
				  if (sumd_ch.ch5 < 10000 && sumd_ch.ch1<8725 ) //switch on , throttle minimum
				  {
					  arm_flag = 1;

				  }
				  else if (sumd_ch.ch5 > 10000)
				  {
					  arm_flag = 0;
				  }
				  //FAILSAFE
				  if (arm_flag == 0)
				  {
					  Motor1_pulsewidth = 125.0;
					  Motor2_pulsewidth = 125.0;
					  Motor3_pulsewidth = 125.0;
					  Motor4_pulsewidth = 125.0;
				  }



				  PWM_pulsewidth_us(&MOTOR[0], (Motor1_pulsewidth > 250.0) ? 250.0 :
						  	  	  	  	  	   (Motor1_pulsewidth < 125.0) ? 125.0 :
						  	  	  	  	  	    Motor1_pulsewidth);

				  PWM_pulsewidth_us(&MOTOR[1], (Motor2_pulsewidth > 250.0) ? 250.0 :
						  	  	  	  	  	   (Motor2_pulsewidth < 125.0) ? 125.0 :
						  	  	  	  	  	    Motor2_pulsewidth);

				  PWM_pulsewidth_us(&MOTOR[2], (Motor3_pulsewidth > 250.0) ? 250.0 :
						  	  	  	  	  	   (Motor3_pulsewidth < 125.0) ? 125.0 :
						  	  	  	  	  	    Motor3_pulsewidth);

				  PWM_pulsewidth_us(&MOTOR[3], (Motor4_pulsewidth > 250.0) ? 250.0 :
						  	  	  	  	  	   (Motor4_pulsewidth < 125.0) ? 125.0 :
						  	  	  	  	  	    Motor4_pulsewidth);


			  }
		  }








    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void motor_setup(void)
{

	/*PWM Setup------------------------------------------------------*/
	PWM_init(&MOTOR[0], MOTOR_pin[0].port, MOTOR_pin[0].pin);
	PWM_init(&MOTOR[1], MOTOR_pin[1].port, MOTOR_pin[1].pin);
	PWM_init(&MOTOR[2], MOTOR_pin[2].port, MOTOR_pin[2].pin);
	PWM_init(&MOTOR[3], MOTOR_pin[3].port, MOTOR_pin[3].pin);
	HAL_Delay(10);

	PWM_period_us(&MOTOR[0], 500);
	PWM_period_us(&MOTOR[1], 500);
	PWM_period_us(&MOTOR[2], 500);
	PWM_period_us(&MOTOR[3], 500);
	HAL_Delay(10);

	PWM_pulsewidth_us(&MOTOR[0], 125.0);
	PWM_pulsewidth_us(&MOTOR[1], 125.0);
	PWM_pulsewidth_us(&MOTOR[2], 125.0);
	PWM_pulsewidth_us(&MOTOR[3], 125.0);


	PWM_2kHz_init(&MOTOR[0], MOTOR_pin[0].port, MOTOR_pin[0].pin);
	PWM_2kHz_init(&MOTOR[1], MOTOR_pin[1].port, MOTOR_pin[1].pin);
	PWM_2kHz_init(&MOTOR[2], MOTOR_pin[2].port, MOTOR_pin[2].pin);
	PWM_2kHz_init(&MOTOR[3], MOTOR_pin[3].port, MOTOR_pin[3].pin);
	HAL_Delay(10);

	PWM_2kHz(&MOTOR[0]);
	PWM_2kHz(&MOTOR[1]);
	PWM_2kHz(&MOTOR[2]);
	PWM_2kHz(&MOTOR[3]);
	HAL_Delay(10);

	PWM_2kHz_pulsewidth_us(&MOTOR[0], 125.0);
	PWM_2kHz_pulsewidth_us(&MOTOR[1], 125.0);
	PWM_2kHz_pulsewidth_us(&MOTOR[2], 125.0);
	PWM_2kHz_pulsewidth_us(&MOTOR[3], 125.0);




}
uint8_t IsGyroCalibrated(void){
	if (gyro_offset_cnt > 5000){
		return 1;
	}
	else{
		return 0;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
