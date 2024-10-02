/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "gps.h"
#include "sim.h"
#include "FreeRTOSConfig.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_LENGTH 10      // �?ộ dài của queue
#define QUEUE_ITEM_SIZE 50   // Kích thước tối đa của mỗi chuỗi

#define LEN_COM_CMD 13
uint8_t COM_rx;
uint8_t COM_buffer[100];  // Bộ đệm để lưu chuỗi dữ liệu
uint8_t COM_idx = 0;      // Chỉ số để đánh dấu vị trí trong bộ đệm
bool is_COM_receiving = false; // Cờ để đánh dấu quá trình nhận dữ liệu

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId IDLEHandle;
/* USER CODE BEGIN PV */

osThreadId GPSHandle;
osThreadId COMHandle;
osThreadId OTAHandle;
osThreadId POSTHandle;
osThreadId WARNHandle;

osMutexId simMutexHandle;
osMessageQId gpsQueueHandle;
osMessageQId comQueueHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void IDLE_Task(void const *argument);

/* USER CODE BEGIN PFP */
void GPS_Task(void const *argument);
void COM_Task(void const *argument);
void OTA_Task(void const *argument);
void POST_Task(void const *argument);
void WARN_Task(void const *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		// check ky tu start
		if (COM_rx == 'R') {
			memset(COM_buffer, 0, sizeof(COM_buffer));
			COM_idx = 0;
			is_COM_receiving = true;
		}

		// append data to com buffer
		if (is_COM_receiving) {
			COM_buffer[COM_idx] = COM_rx;
			COM_idx += 1;
		}

		// check ky tu ket thuc
		if (COM_rx == '#') {
			COM_buffer[COM_idx] = '\0';
			is_COM_receiving = false;
			if (COM_idx == LEN_COM_CMD) {
				if (osMessagePut(comQueueHandle, (uint32_t) COM_buffer, 1000) == osOK) {
					printf("Sent value: %s\n", COM_buffer);
				}
			}
		}
		HAL_UART_Receive_IT(&huart1, &COM_rx, 1);

	} else if (huart->Instance == USART2) {

	}
	if (huart->Instance == USART3) {

	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, &COM_rx, 1);

	simInit();
	gpsInit();
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	osMutexDef(simMutex);  // �?ịnh nghĩa Mutex
	simMutexHandle = osMutexCreate(osMutex(simMutex));
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	osMessageQDef(gpsQueue, QUEUE_LENGTH, NULL);
	gpsQueueHandle = osMessageCreate(osMessageQ(gpsQueue), NULL);

	osMessageQDef(comQueue, QUEUE_LENGTH, NULL);
	comQueueHandle = osMessageCreate(osMessageQ(comQueue), NULL);
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of IDLE */
//  osThreadDef(IDLE, IDLE_Task, osPriorityIdle, 0, 128);
//  IDLEHandle = osThreadCreate(osThread(IDLE), NULL);
	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* definition and creation of OTA */
	osThreadDef(OTA, OTA_Task, osPriorityHigh, 0, 5120);
	OTAHandle = osThreadCreate(osThread(OTA), NULL);

	/* definition and creation of WARN */
	osThreadDef(WARN, WARN_Task, osPriorityHigh, 0, 5120);
	WARNHandle = osThreadCreate(osThread(WARN), NULL);

	/* definition and creation of COM */
	osThreadDef(COM, COM_Task, osPriorityHigh, 0, 5120);
	COMHandle = osThreadCreate(osThread(COM), NULL);

	/* definition and creation of POST */
	osThreadDef(POST, POST_Task, osPriorityNormal, 0, 5120);
	POSTHandle = osThreadCreate(osThread(POST), NULL);

	/* definition and creation of GPS */
	osThreadDef(GPS, GPS_Task, osPriorityBelowNormal, 0, 5120);
	GPSHandle = osThreadCreate(osThread(GPS), NULL);

	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 8399;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 99;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN Header_COM_Task */
/**
 * @brief Function implementing the COM thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_COM_Task */
void COM_Task(void const *argument) {
	/* USER CODE BEGIN COM_Task */
	char COM_cmd[LEN_COM_CMD] = "";
	osEvent event;

	/* Infinite loop */
	for (;;) {
		event = osMessageGet(comQueueHandle, 0);
		if (event.status == osEventMessage) {
			// Dữ liệu đã được nhận thành công
			memset(COM_cmd, 0, LEN_COM_CMD);
			strcpy(COM_cmd, (char*) event.value.p);
			// handle COM CMD TODO

			// DATA 1

			// DATA 2

			// DATA 3

			// DATA 4

			// DATA 5

			printf("Data received from queue: %s\n", COM_cmd);
		} else {
			printf("Failed to receive data from queue!\n");
		}
		osDelay(1000);
	}
	/* USER CODE END COM_Task */
}

/* USER CODE BEGIN Header_OTA_Task */
/**
 * @brief Function implementing the OTA thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_OTA_Task */
void OTA_Task(void const *argument) {
	/* USER CODE BEGIN OTA_Task */
	/* Infinite loop */
	for (;;) {
		if (osMutexWait(simMutexHandle, osWaitForever) == osOK) {
			// CALL CHECK NEW VERSION
			osMutexRelease(simMutexHandle);
		}
		osDelay(1800000);
	}
	/* USER CODE END OTA_Task */
}

/* USER CODE BEGIN Header_POST_Task */
/**
 * @brief Function implementing the POST thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_POST_Task */
void POST_Task(void const *argument) {
	/* USER CODE BEGIN POST_Task */
	char gps_data_received[50] = "";
	osEvent event;

	/* Infinite loop */
	for (;;) {
		memset(gps_data_received, 0, sizeof(gps_data_received));
		// SEND GPS DATA
		event = osMessageGet(gpsQueueHandle, 0);
		if (event.status == osEventMessage) {
			// Dữ liệu đã được nhận thành công
			strcpy(gps_data_received, (char*) event.value.p);
			if (osMutexWait(simMutexHandle, osWaitForever) == osOK) {
				updateGPS(gps_data_received);
				osMutexRelease(simMutexHandle);
			}
			printf("Data received from queue: %s\n", gps_data_received);
		} else {
			printf("Failed to receive data from queue!\n");
		}
		// SEND DATA COMPENSATION
		osDelay(500);
	}
	/* USER CODE END POST_Task */
}

void GPS_Task(void const *argument) {
	char gps_data[50] = "";
	for (;;) {
		// get gps data
		getGPSData(gps_data);

		if (osMessagePut(gpsQueueHandle, (uint32_t) gps_data, 1000) == osOK) {
			printf("Sent value: %s\n", gps_data);
		}
		osDelay(1000);
	}
}
/* USER CODE BEGIN Header_WARN_Task */
/**
 * @brief Function implementing the WARN thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_WARN_Task */
void WARN_Task(void const *argument) {
	/* USER CODE BEGIN WARN_Task */
	/* Infinite loop */
	for (;;) {
		osDelay(1000);
	}
	/* USER CODE END WARN_Task */
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_IDLE_Task */
/**
 * @brief  Function implementing the IDLE thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_IDLE_Task */
__weak void IDLE_Task(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1000);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
