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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Producer1Task */
osThreadId_t Producer1TaskHandle;
const osThreadAttr_t Producer1Task_attributes = {
  .name = "Producer1Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Consumer1Task */
osThreadId_t Consumer1TaskHandle;
const osThreadAttr_t Consumer1Task_attributes = {
  .name = "Consumer1Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Producer2Task */
osThreadId_t Producer2TaskHandle;
const osThreadAttr_t Producer2Task_attributes = {
  .name = "Producer2Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLed1Task(void *argument);
void StartProducer1Task(void *argument);
void StartConsumer1Task(void *argument);
void StartProducer2Task(void *argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(Message_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Led1Task */
  Led1TaskHandle = osThreadNew(StartLed1Task, NULL, &Led1Task_attributes);

  /* creation of Producer1Task */
  Producer1TaskHandle = osThreadNew(StartProducer1Task, NULL, &Producer1Task_attributes);

  /* creation of Consumer1Task */
  Consumer1TaskHandle = osThreadNew(StartConsumer1Task, NULL, &Consumer1Task_attributes);

  /* creation of Producer2Task */
  Producer2TaskHandle = osThreadNew(StartProducer2Task, NULL, &Producer2Task_attributes);

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
  for(;;)
  {
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	printf("LED1\n\r");
    osDelay(1000);
  }
  /* USER CODE END StartLed1Task */
}

/* USER CODE BEGIN Header_StartProducer1Task */
/**
* @brief Function implementing the Producer1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProducer1Task */
void StartProducer1Task(void *argument)
{
  /* USER CODE BEGIN StartProducer1Task */
	Message_t DataToSend = {0, 1};
  /* Infinite loop */
  for(;;)
  {
	if(osOK == osMessageQueuePut(myQueue01Handle, (Message_t*)&DataToSend, 0, osWaitForever)) {
		printf("Producer1Task sent : %d\n\r", DataToSend.Value);
		DataToSend.Value++;
	}
    osDelay(1000);
  }
  /* USER CODE END StartProducer1Task */
}

/* USER CODE BEGIN Header_StartConsumer1Task */
/**
* @brief Function implementing the Consumer1Task thread.
* @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartConsumer1Task */
void StartConsumer1Task(void *argument)
{
  /* USER CODE BEGIN StartConsumer1Task */
	Message_t ReceivedData;
	/* Infinite loop */
	for (;;) {
		if (osOK == osMessageQueueGet(myQueue01Handle, (Message_t*)&ReceivedData, NULL, osWaitForever)) {
			printf("Consumer1Task received %d : %d\n\r", ReceivedData.Sender, ReceivedData.Value);
		}
	}
  /* USER CODE END StartConsumer1Task */
}

/* USER CODE BEGIN Header_StartProducer2Task */
/**
* @brief Function implementing the Producer2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProducer2Task */
void StartProducer2Task(void *argument)
{
  /* USER CODE BEGIN StartProducer2Task */
	Message_t DataToSend = {0, 2};
	/* Infinite loop */
	for (;;) {
		if (osOK == osMessageQueuePut(myQueue01Handle, (Message_t*)&DataToSend, 0, osWaitForever)) {
			printf("Producer2Task sent : %d\n\r", DataToSend.Value);
			DataToSend.Value++;
		}
		osDelay(1000);
	}
  /* USER CODE END StartProducer2Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
  // send char to console etc.
	HAL_UART_Transmit(&huart3, (uint8_t*)&character, 1, 1000);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static Message_t DataToSend = { 0, 3 };

	if (USER_Btn_Pin == GPIO_Pin) {
		if (osOK
				== osMessageQueuePut(myQueue01Handle, (Message_t*) &DataToSend,
						0, 0)) {
			DataToSend.Value++;
		}
	}
}
/* USER CODE END Application */

