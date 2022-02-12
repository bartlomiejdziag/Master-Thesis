/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf.h"
#include "usart.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Led1Task */
osThreadId_t Led1TaskHandle;
const osThreadAttr_t Led1Task_attributes = {
  .name = "Led1Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Led2Task */
osThreadId_t Led2TaskHandle;
const osThreadAttr_t Led2Task_attributes = {
  .name = "Led2Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Led3Task */
osThreadId_t Led3TaskHandle;
const osThreadAttr_t Led3Task_attributes = {
  .name = "Led3Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MutexPrint */
osMutexId_t MutexPrintHandle;
const osMutexAttr_t MutexPrint_attributes = {
  .name = "MutexPrint"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLed1Task(void *argument);
void StartLed2Task(void *argument);
void StartLed3Task(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexPrint */
  MutexPrintHandle = osMutexNew(&MutexPrint_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Led1Task */
  Led1TaskHandle = osThreadNew(StartLed1Task, NULL, &Led1Task_attributes);

  /* creation of Led2Task */
  Led2TaskHandle = osThreadNew(StartLed2Task, NULL, &Led2Task_attributes);

  /* creation of Led3Task */
  Led3TaskHandle = osThreadNew(StartLed3Task, NULL, &Led3Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLed1Task */
/**
 * @brief  Function implementing the Led1Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLed1Task */
void StartLed1Task(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartLed1Task */
	/* Infinite loop */
	for (;;) {
		printf("LED1 Enter\n\r");
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		printf("LED1 Exit\n\r");
		osDelay(500);
	}
  /* USER CODE END StartLed1Task */
}

/* USER CODE BEGIN Header_StartLed2Task */
/**
 * @brief Function implementing the Led2Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLed2Task */
void StartLed2Task(void *argument)
{
  /* USER CODE BEGIN StartLed2Task */

	/* Infinite loop */
	for (;;) {
		printf("LED2 Enter\n\r");
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)) {
			printf("Task LED2 Enter Critical\n\r");
			taskENTER_CRITICAL();
			while (GPIO_PIN_SET == HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)) {

			}
			taskEXIT_CRITICAL();
			printf("Task LED2 Exit Critical\n\r");
		}
		printf("LED2 Exit\n\r");
		osDelay(300);
	}
  /* USER CODE END StartLed2Task */
}

/* USER CODE BEGIN Header_StartLed3Task */
/**
 * @brief Function implementing the Led3Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLed3Task */
void StartLed3Task(void *argument)
{
  /* USER CODE BEGIN StartLed3Task */

	/* Infinite loop */
	for (;;) {
		printf("LED3 Enter\n\r");
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		printf("LED3 Exit\n\r");
		osDelay(1000);
	}
  /* USER CODE END StartLed3Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character) {
	// send char to console etc.
	osMutexAcquire(MutexPrintHandle, osWaitForever);
	HAL_UART_Transmit(&huart3, (uint8_t*) &character, 1, 1000);
	osMutexRelease(MutexPrintHandle);
}
/* USER CODE END Application */

