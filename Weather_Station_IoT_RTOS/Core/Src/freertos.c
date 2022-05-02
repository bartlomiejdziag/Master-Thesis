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
#include "timers.h"
#include "printf.h"
#include "usart.h"
#include "i2c.h"
#include "adc.h"
#include "semphr.h"
#include "Veml7700.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOW_PRIORITY 1
#define NORMAL_PRIORITY 2
#define HIGH_PRIORITY 3
#define ADC_SAMPLES 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

SemaphoreHandle_t xMutexPrintf, xMutexIdle;
QueueHandle_t xAnalogQueue, xI2CQueue;
TimerHandle_t xTimerIdle;
TimerHandle_t xTimerDelay;
uint32_t IdleTicks;
/* USER CODE END Variables */
/* Definitions for DefaultTask */
osThreadId_t DefaultTaskHandle;
const osThreadAttr_t DefaultTask_attributes = {
  .name = "DefaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void vQueueReceive(QueueHandle_t Queue, uint16_t *val, uint8_t pvItemToQueue) {
	for (uint8_t i = 0; i <= pvItemToQueue; i++) {
		xQueueReceive(Queue, &(val[i]), portMAX_DELAY);
	}
}

static void vQueueSend(QueueHandle_t Queue, uint16_t *val, uint8_t pvItemToQueue) {
	for (uint8_t i = 0; i <= pvItemToQueue; i++) {
		xQueueSend(Queue, &val[i], portMAX_DELAY);
	}
}

static inline uint16_t* xCalcAdc(uint16_t *adc, uint16_t *resault) {
	for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
		if (i != 2) {
			resault[i] = ((adc[i] * 100U) / 4096U);
		} else {
			resault[i] = ((adc[i] * 100U) / 1024U);
		}

	}
	return resault;
}

void vAnalogTask(void *pvParameters);
void vEthernetTask(void *pvParameters);
void vSen0335Task(void *pvParameters);
void vLCDTask(void *pvParameters);
void vVeml7700Task(void *pvParameters);
void vEthernetTask(void *pvParameters);
void vHeartBeatTask(void *pvParameters);
void vTimerIdleCallback(TimerHandle_t xTimer);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) {

	static uint32_t LastTick;

	if (LastTick < osKernelGetTickCount()) {
		xSemaphoreTake(xMutexIdle, portMAX_DELAY);
		IdleTicks++;
		xSemaphoreGive(xMutexIdle);
		LastTick = osKernelGetTickCount();
	}
}
/* USER CODE END 2 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void) {

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
	xMutexIdle = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	xTimerIdle = xTimerCreate("TimerIdle", 1000, pdTRUE, (void*) 0, vTimerIdleCallback);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	xAnalogQueue = xQueueCreate(10, sizeof(uint16_t));
	xI2CQueue = xQueueCreate(10, sizeof(double));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DefaultTask */
	DefaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &DefaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	xTaskCreate(vAnalogTask, "AnalogTask", 256, (void*) 1, HIGH_PRIORITY, NULL);
	xTaskCreate(vHeartBeatTask, "HeartBeatTask", 128, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vSen0335Task, "Sen0335Task", 128, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vLCDTask, "LCDTask", 256, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vVeml7700Task, "Veml7700Task", 256, (void*) 1, HIGH_PRIORITY, NULL);
	xTaskCreate(vEthernetTask, "EthernetTask", 256, (void*) 1, NORMAL_PRIORITY, NULL);
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
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
//  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
	if ( xTimerStart( xTimerIdle, 0 ) != pdPASS) {
		printf("Timer not created\n\r");
	}
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

	uint16_t *AdcRawValue;
	uint16_t *Resault;

	AdcRawValue = pvPortMalloc(ADC_SAMPLES * sizeof(uint16_t)); // Adc will be created on FreeRTOS heap instead of task heap
	Resault = pvPortMalloc(ADC_SAMPLES * sizeof(uint16_t));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) AdcRawValue, 3);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

	for (;;) {;
		vQueueSend(xAnalogQueue, xCalcAdc(AdcRawValue, Resault), ADC_SAMPLES);
	}
	vTaskDelay(xDelay);
}

/* Weather sensor (temperature, humidity, atmospheric pressure */
void vSen0335Task(void *pvParameters) {

	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

	uint16_t *Sen0335Value;
	Sen0335Value = pvPortMalloc(ADC_SAMPLES * sizeof(uint16_t));

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

	uint16_t *AdcValue;
	AdcValue = pvPortMalloc(ADC_SAMPLES * sizeof(uint16_t));
	uint16_t *I2CValue;
	I2CValue = pvPortMalloc(2 * sizeof(uint16_t));

	for (;;) {
		vQueueReceive(xAnalogQueue, AdcValue, 3);
		vQueueReceive(xI2CQueue, I2CValue, 2);

		printf("RainSensor: %d%%\n\r", AdcValue[0]);
		printf("Moisture: %d%%\n\r", AdcValue[1]);
		printf("Battery: %d%%\n\r", AdcValue[2]);
		printf("ALS : %d lx\r\n", I2CValue[0]);
		printf("White : %d\r\n", I2CValue[1]);

		vTaskDelay(xDelay);
	}
}

/* Digital Ambient Light Sensor */
void vVeml7700Task(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
	uint16_t als, white;
	uint16_t calc_values[2];

	VEML7700_TypeDef Veml7700;
	Veml7700_Init(&Veml7700, &hi2c1, VEML7700_ADDR); // gain x2
	Veml7700_Set_Als_Integration_Time(&Veml7700, REG_ALS_CONF_IT_25); //25ms
	VEML7700_Set_PSM(&Veml7700, REG_POWER_SAVING_PSM_2);

	for (;;) {
		Veml7700_Power_On(&Veml7700);
		vTaskDelay(28);

		als = VEML7700_read_als(&Veml7700);
		white = VEML7700_read_white(&Veml7700);

		Veml7700_Shutdown(&Veml7700);

		calc_values[0] = (als * VEML7700_RESOLUTION);
		calc_values[1] = (white * VEML7700_RESOLUTION);

		vQueueSend(xI2CQueue, calc_values, 2);

		vTaskDelay(xDelay);
	}
}

/* Ethernet Task. LWIP and MQTT */
void vEthernetTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

	uint16_t *AdcValue;
	AdcValue = pvPortMalloc(ADC_SAMPLES * sizeof(uint16_t));

	for (;;) {

		vQueueReceive(xAnalogQueue, AdcValue, 3);
		vTaskDelay(xDelay);
	}
}


void _putchar(char character) {
// send char to console etc.
	xSemaphoreTake(xMutexPrintf, portMAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t*) &character, 1, 1000);
	xSemaphoreGive(xMutexPrintf);
}

void vTimerIdleCallback(TimerHandle_t xTimer) {
	configASSERT(xTimer);

	uint32_t IdleTime;
	xSemaphoreTake(xMutexIdle, portMAX_DELAY);
	IdleTime = (IdleTicks * 100) / 1000;
	xSemaphoreGive(xMutexIdle);

	IdleTicks = 0;

	printf("Idle Time: %d%%\n\r", IdleTime);
}

/* USER CODE END Application */

