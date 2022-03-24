/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "HX711.h"
#include "MOTOR.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//long reading1;
//long reading2;
//long reading3;
//long reading4;
//long reading5;
//long reading6;

uint8_t c[4] ={0,};
uint8_t rx_data;

//uint8_t buf[10]="TakeOut!\r\n";

uint8_t springFlag = 0;
uint8_t summerFlag = 0;
uint8_t winterFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void season_Motor(uint8_t c);

//void temp_LED(uint8_t c);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void button_off(void){
	if(springFlag == 1){
		TIM1->CCR3=7000;
		TIM1->CCR4=7000;
		MOTOR1_up();
		HAL_Delay(2000);  //delay
		MOTOR1_stop();   // stop
		springFlag = 0;
	}
	else if(summerFlag == 1){
		TIM1->CCR3=7000;
		TIM1->CCR4=7000;
		MOTOR1_up();
		HAL_Delay(2000);  //delay
		MOTOR1_stop();   // stop
		summerFlag = 0;
	}
	else if(winterFlag == 1){
		TIM1->CCR3=7000;
		TIM1->CCR4=7000;
		MOTOR3_up();
		HAL_Delay(2000);  //delay
		MOTOR3_stop();   // stop
		winterFlag = 0;
	}
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);

}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart3, &rx_data, 1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);		//PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint16_t ccr=0;
	
  while (1)
  {	
//		__HAL_TIM_SET_COMPARE(&htmi1,TIM_CHANHEL_3,ccr);
//		if(c[3]=='u'){
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
//		}
		//========================================
		if(c[0]==0x61){ //97
			if(c[1]==0x62){ //98
				if (c[2] == 0x64) {	
					//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
					TIM1->CCR3=6000;
					TIM1->CCR4=6000;
					//season_Motor(c[3]);
					MOTOR1_down();
					HAL_Delay(500);  //delay
					MOTOR1_stop();   // stop
				}
			}
			if (c[2] == 0x65) {	
				//temp_LED(c[3]);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_Delay(4000);
			}
			 if (c[2] == 0x66){ 
				if(c[3] == 0x02){ 
					//button_off();
					TIM1->CCR3=6000;
					TIM1->CCR4=6000;
					MOTOR1_up();
					HAL_Delay(500);  //delay
					MOTOR1_stop();   // stop
					
				}
			}
		}
		//==========================================
//		TIM1->CCR3=7000;
//		TIM1->CCR4=7000;
//		MOTOR1_down();
//		MOTOR1_down();
//		HAL_Delay(500);
//		MOTOR1_up();
//		HAL_Delay(500);
		
		
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
void season_Motor(uint8_t c) {
		switch(c) {
		case 0x02:		// spring
			springFlag = 1;
			MOTOR1_down();
			HAL_Delay(2000);  //delay
			MOTOR1_stop();   // stop			
		break;
		
		case 0x03:		// summer
			summerFlag = 1;
			MOTOR1_down();
			HAL_Delay(2000);  //delay
			MOTOR1_stop();   // stop
		break;
		
		case 0x04:		// winter
			winterFlag = 1;
			MOTOR3_down();
			HAL_Delay(2000);  //delay
			MOTOR3_stop();   // stop
		break;
	}
}

//void temp_LED(uint8_t c) {
//	long reading1;
//	long reading2;
//	
//	if (springFlag == 0 && summerFlag == 0 && winterFlag == 0) return;
//	switch(c) {
//			case 0x05:		// >28
//				if (summerFlag == 1) {
//					reading1 = meas_weight(CELL3);
//					reading2 = meas_weight(CELL4);
//					if (reading1 > 0 && reading1 < 100)	cell_led_on(CELL3);
//					if (reading2 > 0 && reading2 < 100)	cell_led_on(CELL4);
//					summerFlag = 0;
//				}
//			break;
//			case 0x06:		// >20 <28
//				if (summerFlag == 1) {
//					reading1 = meas_weight(CELL3);
//					reading2 = meas_weight(CELL4);
//					if (reading1 > 100 && reading1 < 500)	cell_led_on(CELL3);
//					if (reading2 > 100 && reading2 < 500)	cell_led_on(CELL4);
//					summerFlag = 0;
//				}
//			break;
//			case 0x07:		// >10 <20
//				if (springFlag == 1) {
//					reading1 = meas_weight(CELL1);
//					reading2 = meas_weight(CELL2);
//					if (reading1 > 500 && reading1 < 800)	cell_led_on(CELL1);
//					if (reading2 > 500 && reading2 < 800)	cell_led_on(CELL2);
//					springFlag = 0;
//				}
//			break;
//			case 0x08:		// >0 <10
//				if (springFlag == 1) {
//					reading1 = meas_weight(CELL1);
//					reading2 = meas_weight(CELL2);
//					if (reading1 > 800 && reading1 < 1100)	cell_led_on(CELL1);
//					if (reading2 > 800 && reading2 < 1100)	cell_led_on(CELL2);
//					springFlag = 0;
//				} 
//				else if (winterFlag) {
//					reading1 = meas_weight(CELL5);
//					reading2 = meas_weight(CELL6);
//					if (reading1 > 1100)	cell_led_on(CELL5);
//					if (reading2 > 1100)	cell_led_on(CELL6);
//					winterFlag = 0;
//				}
//			break;
//			case 0x09:		// <0
//				if (winterFlag) {
//					reading1 = meas_weight(CELL5);
//					reading2 = meas_weight(CELL6);
//					if (reading1 > 1100)	cell_led_on(CELL5);
//					if (reading2 > 1100)	cell_led_on(CELL6);
//					winterFlag = 0;
//				}
//			break;
//	}
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART3){
		c[0]=c[1];
		c[1]=c[2];
		c[2]=c[3];
		c[3]=rx_data;
		HAL_UART_Receive_IT(&huart3,&rx_data,1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
