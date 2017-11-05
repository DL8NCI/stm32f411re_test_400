/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <StdIoConnectorRTOS.h>
#include "task.h"
#include "event_groups.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId ReadHIH8000Handle;
osThreadId PrintSomeStatsHandle;
osThreadId ReadMLX90615Handle;
osTimerId myTimer01Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

EventGroupHandle_t EgMainHandle;

uint16_t cntr;
//double T0;
uint8_t T0raw[3];
uint8_t Taraw[3];
uint8_t IRraw[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void TsReadHIH8000(void const * argument);
void TsPrintSomeStats(void const * argument);
void TsReadMLX90615(void const * argument);
void Callback01(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void vAssertCalled( const char *pcFile, uint32_t ulLine );
unsigned long getRunTimeCounterValue(void);
void configureTimerForRunTimeStats(void);
void vApplicationMallocFailedHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  xTimerChangePeriod(myTimer01Handle,1000,100);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ReadHIH8000 */
  osThreadDef(ReadHIH8000, TsReadHIH8000, osPriorityNormal, 0, 256);
  ReadHIH8000Handle = osThreadCreate(osThread(ReadHIH8000), NULL);

  /* definition and creation of PrintSomeStats */
  osThreadDef(PrintSomeStats, TsPrintSomeStats, osPriorityNormal, 0, 256);
  PrintSomeStatsHandle = osThreadCreate(osThread(PrintSomeStats), NULL);

  /* definition and creation of ReadMLX90615 */
  osThreadDef(ReadMLX90615, TsReadMLX90615, osPriorityNormal, 0, 256);
  ReadMLX90615Handle = osThreadCreate(osThread(ReadMLX90615), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  SIOC_InitDiag(&huart1);
  SIOC_Init(&huart2);
  cntr = 0;
  //T0 = 0.0;
  T0raw[0]=0;
  T0raw[1]=0;
  T0raw[2]=0;

  EgMainHandle = xEventGroupCreate();
  xEventGroupClearBits(&EgMainHandle, EG_I2C_CMD_SENT | EG_I2C_DATA_RECEIVED);

  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 960;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

/*
	txBuff = rxBuff;
	HAL_UART_Receive_IT(&huart2, &rxBuff, 1);

	configASSERT( SendCharHandle != NULL );
	vTaskNotifyGiveFromISR( SendCharHandle, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
*/
	}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	SIOC_SendDiagPort("IRQ: TxCplt\r\n");
	SIOC_TxCpltCallback(huart);
	}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	SIOC_SendDiagPort("IRQ: HAL_I2C_MasterTxCpltCallback\r\n");
	if (hi2c->Instance != I2C2) return;

	BaseType_t xHigherPriorityTaskWoken, rc;
	xHigherPriorityTaskWoken = pdFALSE;

	rc = xEventGroupSetBitsFromISR(&EgMainHandle, EG_I2C_CMD_SENT, &xHigherPriorityTaskWoken);
	if (rc!=pdFAIL) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	SIOC_SendDiagPort("IRQ: HAL_I2C_MasterRxCpltCallback\r\n");
	if (hi2c->Instance != I2C2) return;

	BaseType_t xHigherPriorityTaskWoken, rc;
	xHigherPriorityTaskWoken = pdFALSE;

	rc = xEventGroupSetBitsFromISR(&EgMainHandle, EG_I2C_DATA_RECEIVED, &xHigherPriorityTaskWoken);
	if (rc!=pdFAIL) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}


void vAssertCalled( const char *pcFile, uint32_t ulLine ) {
	taskDISABLE_INTERRUPTS();
	SIOC_SendDiagPort(pcFile);
	SIOC_SendDiagPortUInt32(ulLine);
	for( ;; );
	}

unsigned long getRunTimeCounterValue(void) {
	uint32_t c = htim2.Instance->CNT;
	return c;
	}

void configureTimerForRunTimeStats(void) {
	HAL_StatusTypeDef rc = HAL_TIM_Base_Start(&htim2);
	configASSERT( rc == HAL_OK );
	}

void vApplicationMallocFailedHook(void) {
	SIOC_SendDiagPort("malloc-fault\r\n");
	for(;;) { }
	}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName) {
	SIOC_SendDiagPort("Stack overflow in ");
	SIOC_SendDiagPort(pcTaskName);
	for(;;) { }
	}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	for(;;) {
//		HAL_UART_Receive_IT(&huart2, &rxBuff, 1);
//		xTimerChdangePeriod(myTimer01Handle,1000,10);
//		xTimerStart(myTimer01Handle,10);
		osDelay(1);
		}
  /* USER CODE END 5 */ 
}

/* TsReadHIH8000 function */
void TsReadHIH8000(void const * argument)
{
  /* USER CODE BEGIN TsReadHIH8000 */

	uint8_t b[4];
	double humidity, temperature;
	EventBits_t bits;
	HAL_StatusTypeDef rc;

	  /* Infinite loop */
	for(;;) {
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		SIOC_SendDiagPort("TsReadHIH8000 started\r\n");

		xEventGroupClearBits(&EgMainHandle, EG_I2C_CMD_SENT | EG_I2C_DATA_RECEIVED);
		SIOC_SendDiagPort("TsReadHIH8000: xEventGroupClearBits executed\r\n");

		b[0] = 0x00;
		rc = HAL_I2C_Master_Transmit_IT(&hi2c2, 0x4e, &b, 1);
//		rc = HAL_I2C_Master_Transmit(&hi2c2, 0x4e, &b, 1, HAL_MAX_DELAY);
		configASSERT( rc == HAL_OK );
		SIOC_SendDiagPort("TsReadHIH8000: HAL_I2C_Master_Transmit_IT executed\r\n");

		bits = xEventGroupWaitBits(&EgMainHandle, EG_I2C_CMD_SENT, pdTRUE, pdFALSE, 1000);
		SIOC_SendDiagPort("TsReadHIH8000: xEventGroupWaitBits received/timed out\r\n");
		configASSERT((bits & EG_I2C_CMD_SENT)!=0);
		SIOC_SendDiagPort("TsReadHIH8000: xEventGroupWaitBits received\r\n");
		vTaskDelay(100);

		rc = HAL_I2C_Master_Receive_IT(&hi2c2, 0x4f, &b[0], 4);
//		rc = HAL_I2C_Master_Receive(&hi2c2, 0x4f, &b[0], 4, HAL_MAX_DELAY);
		configASSERT( rc == HAL_OK );
//					printf("HAL_I2C_Master_Receive_IT - ok\r\n");
		bits = EG_I2C_DATA_RECEIVED & xEventGroupWaitBits(&EgMainHandle, EG_I2C_DATA_RECEIVED, pdTRUE, pdFALSE, 1000);
		configASSERT(bits!=0);
//		printf("EG_I2C_DATA_RECEIVED - ok\r\n");

						//	printf("HAL8000: %02x %02x %02x %02x\r\n", r[0], r[1], r[2], r[3]);

		humidity = (((uint16_t)b[0] & 0x003f) << 8) | (uint16_t)b[1];
		humidity = humidity/163.820;;

		temperature = ((uint16_t)b[2]<<6) | ((uint16_t)b[3]>>2);
		temperature = temperature*165.0/16382.0 - 40.0;

		printf("RH = %4.1f %%   T = %4.1f deg\r\n", humidity, temperature);

  }
  /* USER CODE END TsReadHIH8000 */
}

/* TsPrintSomeStats function */
void TsPrintSomeStats(void const * argument)
{
  /* USER CODE BEGIN TsPrintSomeStats */
  /* Infinite loop */
	TaskStatus_t *tsa;
	volatile UBaseType_t as, i;
	unsigned long totalRuntime;

	for(;;) {

		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		SIOC_SendDiagPort("TsPrintSomeStats started\r\n");

		as = uxTaskGetNumberOfTasks();
		tsa = pvPortMalloc(as*sizeof(TaskStatus_t));

		as = uxTaskGetSystemState( tsa, as, &totalRuntime);


		printf("cntr = %d   tasks=%lu   total runtime = %10.1f s\r\n", cntr, as, (double)totalRuntime/100000.0);
		cntr++;

		totalRuntime /= 100UL;

		if (totalRuntime == 0) {
			printf("TNr            Task-Name Stk    Curr Prio    Base Prio      Runtime\r\n");
			for (i=0; i<as; i++) {
				printf("%3lu %20s %3hu %12lu %12lu %12lu\r\n", tsa[i].xTaskNumber, tsa[i].pcTaskName, tsa[i].usStackHighWaterMark, tsa[i].uxCurrentPriority, tsa[i].uxBasePriority, tsa[i].ulRunTimeCounter);
				}
			}
		else {
			printf("TNr            Task-Name Stk    Curr Prio    Base Prio Runtme\r\n");
			for (i=0; i<as; i++) {
				printf("%3lu %20s %3hu %12lu %12lu %5.1f%%\r\n", tsa[i].xTaskNumber, tsa[i].pcTaskName, tsa[i].usStackHighWaterMark, tsa[i].uxCurrentPriority, tsa[i].uxBasePriority, (double)tsa[i].ulRunTimeCounter/(double)totalRuntime);
				}
			}



		printf("\r\n");

		vPortFree(tsa);

		osDelay(1);
		}
  /* USER CODE END TsPrintSomeStats */
}

/* TsReadMLX90615 function */
void TsReadMLX90615(void const * argument)
{
  /* USER CODE BEGIN TsReadMLX90615 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    /*
    	for (cmd=0x10; cmd<0x30; cmd++) {

    		rc = HAL_I2C_Master_Transmit(&hi2c2, 0xb6, &cmd, 1, 1000);
    //		printf("I2C-Transmit: rc = %d\r\n", rc);
    		rc = HAL_I2C_Master_Receive(&hi2c2, 0xb7, &T0raw[0], 3, 1000);
    //		printf("I2C-Receive: rc = %d\r\n", rc);

    		printf("%02x: %02x %02x %02x\r\n", cmd, T0raw[0], T0raw[1], T0raw[2]);

    //	uint16_t tmp = T0raw[0];
    //	tmp = tmp + (T0raw[1]<<8);

    //	T0 = (double)tmp * 0.02 - 273.15;
*/


  }
  /* USER CODE END TsReadMLX90615 */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	SIOC_SendDiagPort("Callback01 started\r\n");

	xTaskNotifyGive(PrintSomeStatsHandle);
	xTaskNotifyGive(ReadHIH8000Handle);

  /* USER CODE END Callback01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	vAssertCalled(file,line);
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	vAssertCalled((char*)file,line);
	while(1) { }
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
