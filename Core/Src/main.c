/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "oled.h"
#include "u8g2.h"
#include "stdio.h"
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
TaskHandle_t handleOled;
TaskHandle_t handleHC_SR04;
u8g2_t u8g2;
QueueHandle_t queueHC_SR04;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void microDelay(uint32_t time){
	__HAL_TIM_SetCounter(&htim3, 0);
	__HAL_TIM_ENABLE(&htim3);
	while(__HAL_TIM_GET_COUNTER(&htim3) < time){
	}
	__HAL_TIM_DISABLE(&htim3);
}

void taskOledControl(){
	char displayStr[32];
	float distance = 0;
	u8g2Init(&u8g2);
	u8g2_ClearBuffer(&u8g2);
	u8g2_ClearDisplay(&u8g2);
	u8g2_SetFont(&u8g2,u8g2_font_DigitalDiscoThin_tf);
	for(;;){
		if(xQueueReceive(queueHC_SR04, &distance, pdMS_TO_TICKS(500)) == pdPASS){
			snprintf(displayStr, 32, "Dist: %.2fcm",distance);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 20,displayStr);
			u8g2_SendBuffer(&u8g2);
		}
		else{
			snprintf(displayStr, 32, "HC-SR04 Error");
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 20,displayStr);
			u8g2_SendBuffer(&u8g2);
		}
		vTaskDelay(500);
	}
}

void taskMicroWave(){
	for(;;){
		float distance = 0;
		HAL_GPIO_WritePin(GPIOB, HC_TRIG_Pin, 1);
		microDelay(20);
		HAL_GPIO_WritePin(GPIOB, HC_TRIG_Pin, 0);

		int echoTimer = 0;
		__HAL_TIM_SetCounter(&htim3, 0);
		while(HAL_GPIO_ReadPin(GPIOB, HC_ECHO_Pin) == 0);
		microDelay(10);
		__HAL_TIM_ENABLE(&htim3);
		while(HAL_GPIO_ReadPin(GPIOB, HC_ECHO_Pin) == 1);
		echoTimer = __HAL_TIM_GET_COUNTER(&htim3);
		__HAL_TIM_DISABLE(&htim3);
		distance += echoTimer/58.0;

		xQueueSend(queueHC_SR04, &distance, portMAX_DELAY);
		vTaskDelay(100);
	}
	
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
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	xTaskCreate(taskOledControl, "display", 512, NULL, 1, &handleOled);
	xTaskCreate(taskMicroWave, "HC-SR04", 128, NULL, 2, &handleHC_SR04);
	queueHC_SR04 = xQueueCreate(5, 4);
	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Init scheduler */
  //osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*
		float sum = 0;
		//float voiceSpeed = 344;
		float voiceSpeed = 331.4 + 0.607*26;
		float voiceSpeedPreSec = voiceSpeed * 1e-6;
		HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
		HAL_GPIO_WritePin(GPIOB, HC_TRIG_Pin, 1);
		microDelay(20);
		HAL_GPIO_WritePin(GPIOB, HC_TRIG_Pin, 0);

		int echoTimer = 0;
		__HAL_TIM_SetCounter(&htim3, 0);
		while(HAL_GPIO_ReadPin(GPIOB, HC_ECHO_Pin) == 0);
		microDelay(10);
		__HAL_TIM_ENABLE(&htim3);
		while(HAL_GPIO_ReadPin(GPIOB, HC_ECHO_Pin) == 1);
		echoTimer = __HAL_TIM_GET_COUNTER(&htim3);
		__HAL_TIM_DISABLE(&htim3);
		
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, 0);
//		float distance = ((echoTimer * voiceSpeed*1e-6)/2)*100 +2;
		float distance = echoTimer/58.0;
		printf("Distance: %.1fcm\t", distance);
		printf("Echo high: %dus\n", echoTimer);
		
			if(distance <= 10){
			HAL_GPIO_WritePin(GPIOB, LD2_Pin, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOB, LD2_Pin, 0);
		}
		HAL_Delay(500);
		*/
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
