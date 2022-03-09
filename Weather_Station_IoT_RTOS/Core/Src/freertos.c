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
#include "i2c.h"
#include "adc.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NORMAL_PRIORITY 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
SemaphoreHandle_t xMutexPrintf;
/* USER CODE END Variables */
/* Definitions for DefaultTask */
osThreadId_t DefaultTaskHandle;
const osThreadAttr_t DefaultTask_attributes = { .name = "DefaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vAnalogTask(void *pvParameters);
void vEthernetTask(void *pvParameters);
void vSen0335Task(void *pvParameters);
void vLCDTask(void *pvParameters);
void vVeml7700Task(void *pvParameters);
void vEthernetTask(void *pvParameters);
void vHeartBeatTask(void *pvParameters);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) {
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	 to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
	 task. It is essential that code added to this hook function never attempts
	 to block in any way (for example, call xQueueReceive() with a block time
	 specified, or call vTaskDelay()). If the application makes use of the
	 vTaskDelete() API function (as this demo application does) then it is also
	 important that vApplicationIdleHook() is permitted to return to its calling
	 function, because it is the responsibility of the idle task to clean up
	 memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void) {
	/* vApplicationMallocFailedHook() will only be called if
	 configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
	 function that will get called if a call to pvPortMalloc() fails.
	 pvPortMalloc() is called internally by the kernel whenever a task, queue,
	 timer or semaphore is created. It is also called by various parts of the
	 demo application. If heap_1.c or heap_2.c are used, then the size of the
	 heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	 FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	 to query the size of free heap space that remains (although it does not
	 provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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
	xMutexPrintf = xSemaphoreCreateMutex();
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
	/* creation of DefaultTask */
	DefaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&DefaultTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	xTaskCreate(vAnalogTask, "AnalogTask", 256, (void*) 1, NORMAL_PRIORITY,NULL);
	xTaskCreate(vHeartBeatTask, "HeartBeatTask", 128, (void*) 1,NORMAL_PRIORITY, NULL);
	xTaskCreate(vSen0335Task, "Sen0335Task", 128, (void*) 1, NORMAL_PRIORITY,NULL);
	xTaskCreate(vLCDTask, "LCDTask", 256, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vVeml7700Task, "Veml7700Task", 128, (void*) 1, NORMAL_PRIORITY,NULL);
	xTaskCreate(vEthernetTask, "EthernetTask", 256, (void*) 1, NORMAL_PRIORITY,NULL);
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the DefaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* init code for LWIP */
//  MX_LWIP_Init();
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Analog Task (VBat, Rain Sensor, Soil moisture sensor) */
void vAnalogTask(void *pvParameters) {

	configASSERT(((uint32_t ) pvParameters) == 1);

	volatile uint16_t AdcValue[3];
	//	AdcValue = pvPortMalloc(32 * sizeof(uint16_t));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) AdcValue, 3);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

	for (;;) {
		for (uint8_t i = 0; i <= 2; i++) {
			printf("ADC_Value%d: %d\n\r", i, AdcValue[i]);
		}
		vTaskDelay(xDelay);
	}
}

/* Weather sensor (temperature, humidity, atmospheric pressure */
void vSen0335Task(void *pvParameters) {

	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
	for (;;) {
		vTaskDelay(xDelay);
	}
}

/* HeartBeat Task */
void vHeartBeatTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
	for (;;) {
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		vTaskDelay(xDelay);
	}
}

/* LCD TFT 2,8" */
void vLCDTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
	for (;;) {
		vTaskDelay(xDelay);
	}
}

/* Digital Ambient Light Sensor */
void vVeml7700Task(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
	for (;;) {
		vTaskDelay(xDelay);
	}
}

/* Ethernet Task. LWIP and MQTT */
void vEthernetTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
	for (;;) {
		vTaskDelay(xDelay);
	}
}

void _putchar(char character) {
	// send char to console etc.
	xSemaphoreTake(xMutexPrintf, portMAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t*) &character, 1, 1000);
	xSemaphoreGive(xMutexPrintf);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (USER_Btn_Pin == GPIO_Pin) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {

	}
}
/* USER CODE END Application */

