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
typedef struct {
	uint8_t Value;
	uint8_t Sender;
} Message_t;
/* USER CODE END Variables */
/* Definitions for Led1Task */
osThreadId_t Led1TaskHandle;
const osThreadAttr_t Led1Task_attributes = {
  .name = "Led1Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Led2Task */
osThreadId_t Led2TaskHandle;
const osThreadAttr_t Led2Task_attributes = {
  .name = "Led2Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};
/* Definitions for BtnCntSemaphore */
osSemaphoreId_t BtnCntSemaphoreHandle;
const osSemaphoreAttr_t BtnCntSemaphore_attributes = {
  .name = "BtnCntSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLed1Task(void *argument);
void StartLed2Task(void *argument);

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

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* creation of BtnCntSemaphore */
  BtnCntSemaphoreHandle = osSemaphoreNew(32, 0, &BtnCntSemaphore_attributes);

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
//  MX_LWIP_Init();
  /* USER CODE BEGIN StartLed1Task */
	uint32_t SemaphoreCounter;
	/* Infinite loop */
	for (;;) {
		printf("BelowNormal Task Enter\n\r");

		printf("BelowNormal Semaphore Wait\n\r");
		if (osOK == osSemaphoreAcquire(BtnCntSemaphoreHandle, osWaitForever)) {

		printf("BelowNormal Semaphore Taken\n\r");
		SemaphoreCounter = osSemaphoreGetCount(BtnCntSemaphoreHandle);
		printf("BelowNormal Semaphore Counter Left: %d\n\r", SemaphoreCounter);

		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		}
		printf("BelowNormal Task Exit\n\r");
		osDelay(1000);
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
	uint32_t SemaphoreCounter;

	/* Infinite loop */
	for (;;) {
		printf("Normal Task Enter\n\r");
		printf("Normal Semaphore Wait\n\r");
		if (osOK == osSemaphoreAcquire(BtnCntSemaphoreHandle, osWaitForever)) {

			printf("Normal Semaphore Taken\n\r");
			SemaphoreCounter = osSemaphoreGetCount(BtnCntSemaphoreHandle);
			printf("Normal Semaphore Counter Left: %d\n\r", SemaphoreCounter);

			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			printf("Normal Semaphore Released\n\r");
		}
		printf("Normal Task Exit\n\r");
		osDelay(1000);
	}
  /* USER CODE END StartLed2Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character) {
	// send char to console etc.
	HAL_UART_Transmit(&huart3, (uint8_t*) &character, 1, 1000);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (USER_Btn_Pin == GPIO_Pin) {
		osSemaphoreRelease(BtnCntSemaphoreHandle);
	}
}
/* USER CODE END Application */

