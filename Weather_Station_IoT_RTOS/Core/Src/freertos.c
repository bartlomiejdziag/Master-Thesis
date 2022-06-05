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
#include <stdarg.h>
#include "timers.h"
#include "printf.h"
#include "usart.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"
#include "semphr.h"

#include "Veml7700.h"
#include "Bme680.h"

#include "TFT_ILI9341.h"
#include "GFX_Color.h"
#include "GFX_EnhancedFonts.h"
#include "EnhancedFonts/times_new_roma_12pts_bold.h"
#include <background.h>
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
#define VEML7700_SAMPLES 2
#define BME680_SAMPLES 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

SemaphoreHandle_t xMutexPrintf, xMutexIdle, xMutexI2C;
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
			resault[i] = ((adc[i] * 100U) / 1024U);
		} else {
			resault[i] = ((adc[i] * 100U) / 256U);
		}
	}
	return resault;
}

static void ConvertValuesToTFT(uint16_t PosX, uint16_t PosY, char const *format, ...)
{
	va_list args;
	char buf[256];

	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
//	ILI9341_ClearDisplay(ILI9341_BLACK, PosX, PosY, 38, 12);
	EF_PutString((const uint8_t*)buf, PosX, PosY, ILI9341_WHITE, BG_COLOR, ILI9341_BLACK);
}

void vAnalogTask(void *pvParameters);
void vEthernetTask(void *pvParameters);
void vBme680Task(void *pvParameters);
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
	xMutexI2C = xSemaphoreCreateMutex();
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
	xTaskCreate(vAnalogTask, "AnalogTask", 256, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vHeartBeatTask, "HeartBeatTask", 128, (void*) 1, LOW_PRIORITY, NULL);
	xTaskCreate(vBme680Task, "Bme680Task", 256, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vLCDTask, "LCDTask", 512, (void*) 1, HIGH_PRIORITY, NULL);
	xTaskCreate(vVeml7700Task, "Veml7700Task", 256, (void*) 1, NORMAL_PRIORITY, NULL);
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
		osDelay(10000);
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

	const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

	for (;;) {
		vQueueSend(xAnalogQueue, xCalcAdc(AdcRawValue, Resault), ADC_SAMPLES);
	}
	vTaskDelay(xDelay);
}

/* Weather sensor (temperature, humidity, atmospheric pressure */
void vBme680Task(void *pvParameters) {

	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

	BME680_TypeDef Bme680;
	BME680_Calib_TypeDef Bme680_calib;

	xSemaphoreTake(xMutexI2C, portMAX_DELAY);
	Bme680_Init(&Bme680, &Bme680_calib, &hi2c1, BME680_ADDR);
	Bme680_Set_Conf(&Bme680, BME680_OSRS_T_OVR_SAMPLING_2, BME680_OSRS_H_OVR_SAMPLING_4, BME680_OSRS_P_OVR_SAMPLING_8, BME680_FILTER_7);
	Bme680_Run_Gas(&Bme680);
	Bme680_Set_Gas_Conf(&Bme680, Bme680.Gas_heat_dur);
	xSemaphoreGive(xMutexI2C);

	for (;;) {
		xSemaphoreTake(xMutexI2C, portMAX_DELAY);
		Bme680_Set_Mode(&Bme680, BME680_MODE_FORCE);
		calc_raw_values(&Bme680, &Bme680_calib);
//		printf("Temperature: %d °C\n\r", (Bme680.Temperature_Calc / 100U));
//		printf("Pressure: %d hPa\n\r", (Bme680.Pressure_Calc / 100U));
//		printf("Humidity: %d %%rH\n\r", (Bme680.Humidity_Calc / 1000U));
//		printf("Gas: %d ohms\n\r", Bme680.Gas_Calc);
		printf("Temperature: %d °C\n\r", (Bme680.Temperature_Raw));
		printf("Pressure: %d hPa\n\r", (Bme680.Pressure_Raw));
		printf("Humidity: %d %%rH\n\r", (Bme680.Humidity_Raw));
//		printf("Gas: %d ohms\n\r", Bme680.Gas_Calc);
		Bme680_Set_Mode(&Bme680, BME680_MODE_SLEEP);
		xSemaphoreGive(xMutexI2C);
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

	const TickType_t xDelay = 250 / portTICK_PERIOD_MS;

	uint16_t *AdcValue;
	AdcValue = pvPortMalloc(ADC_SAMPLES * sizeof(uint16_t));
	uint16_t *I2CValue;
	I2CValue = pvPortMalloc(VEML7700_SAMPLES * sizeof(uint16_t));

	ILI9341_Init(&hspi1);
	EF_SetFont(&timesNewRoman_12ptFontInfo);
//	ILI9341_ClearDisplay(ILI9341_BLACK, 0, 0, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);
	ILI9341_DrawImage(0, 0, background, 320, 240);
//	GFX_Image(0, 0, background, 240, 240);

	EF_PutString((const uint8_t*)"Weather Station", 100, 0, ILI9341_WHITE, BG_TRANSPARENT, ILI9341_BLACK);
//	EF_PutString((const uint8_t*)"Rainsensor:", 0, 90, ILI9341_WHITE, BG_TRANSPARENT, ILI9341_BLACK);
//	EF_PutString((const uint8_t*)"Moisture:", 0, 105, ILI9341_WHITE, BG_TRANSPARENT, ILI9341_BLACK);
//	EF_PutString((const uint8_t*)"Battery:", 0, 120, ILI9341_WHITE, BG_TRANSPARENT, ILI9341_BLACK);
//	EF_PutString((const uint8_t*)"ALS:", 0, 135, ILI9341_WHITE, BG_TRANSPARENT, ILI9341_BLACK);
//	EF_PutString((const uint8_t*)"White:", 0, 150, ILI9341_WHITE, BG_TRANSPARENT, ILI9341_BLACK);
//	EF_PutString((const uint8_t*)"Idle Time:", 0, 165, ILI9341_WHITE, BG_TRANSPARENT, ILI9341_BLACK);

	for (;;) {
		vQueueReceive(xAnalogQueue, AdcValue, 3);
		vQueueReceive(xI2CQueue, I2CValue, 2);

//		ConvertValuesToTFT(100, 90, "%d [%%]", AdcValue[0]);
//		ConvertValuesToTFT(100, 105, "%d [%%]", AdcValue[1]);
//		ConvertValuesToTFT(100, 120, "%d [%%]", AdcValue[2]);
//		ConvertValuesToTFT(100, 135, "%d [lx]", I2CValue[0]);
//		ConvertValuesToTFT(100, 150, "%d", I2CValue[1]);

		vTaskDelay(xDelay);
	}
}

/* Digital Ambient Light Sensor */
void vVeml7700Task(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

	uint16_t als, white;
	uint16_t* calc_values;
	calc_values = pvPortMalloc(VEML7700_SAMPLES * sizeof(uint16_t));

	VEML7700_TypeDef Veml7700;

	xSemaphoreTake(xMutexI2C, portMAX_DELAY);
	Veml7700_Init(&Veml7700, &hi2c1, VEML7700_ADDR); // gain x2
	Veml7700_Set_Als_Integration_Time(&Veml7700, REG_ALS_CONF_IT_25); //25ms
	VEML7700_Set_PSM(&Veml7700, REG_POWER_SAVING_PSM_2);
	xSemaphoreGive(xMutexI2C);

	for (;;) {
		xSemaphoreTake(xMutexI2C, portMAX_DELAY);
		Veml7700_Power_On(&Veml7700);
		xSemaphoreGive(xMutexI2C);
		vTaskDelay(28);

		xSemaphoreTake(xMutexI2C, portMAX_DELAY);
		als = VEML7700_read_als(&Veml7700);
		white = VEML7700_read_white(&Veml7700);

		Veml7700_Shutdown(&Veml7700);
		xSemaphoreGive(xMutexI2C);

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

//	ConvertValuesToTFT(100, 165,"%d [%%]", IdleTime);
}

/* USER CODE END Application */

