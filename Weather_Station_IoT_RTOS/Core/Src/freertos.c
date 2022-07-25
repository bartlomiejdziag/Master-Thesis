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
#include <string.h>
#include <ip_addr.h>
#include "lwip.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/api.h"
#include "MQTT_Interface.h"
#include "semphr.h"
#include "timers.h"
#include "printf.h"
#include "usart.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"

#include "Veml7700.h"
#include "Bme680.h"

#include "TFT_ILI9341.h"
#include "GFX_Color.h"
#include "GFX_EnhancedFonts.h"
#include "EnhancedFonts/times_new_roma_12pts_bold.h"
#include "background.h"
#include "XPT2046.h"

#include "mbedtls.h"
#include "http_client.h"


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

extern TaskHandle_t xTCPHandle;

TaskHandle_t xLCDHandle = NULL;
SemaphoreHandle_t xMutexPrintf, xMutexIdle, xMutexI2C, xTCPSem;
QueueHandle_t xAnalogQueue, xVeml7700Queue, xBME680Queue;
TimerHandle_t xTimerIdle, xTimerDelay;

extern LPTIM_HandleTypeDef hlptim1;

volatile uint32_t DelayTick;

typedef struct {
	uint16_t AdcRawValue[3];
	uint16_t Resault[3];
} Analog_t;

typedef struct {
	uint16_t als;
	uint16_t white;
	uint16_t calc_values[2];
} Veml7700_t;

/* USER CODE END Variables */
/* Definitions for DefaultTask */
osThreadId_t DefaultTaskHandle;
const osThreadAttr_t DefaultTask_attributes = {
  .name = "DefaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vAnalogTask(void *pvParameters);
void vEthernetTask(void *pvParameters);
void vBme680Task(void *pvParameters);
void vLCDTask(void *pvParameters);
void vLCDTouchTask(void *pvParameters);
void vVeml7700Task(void *pvParameters);
void vMQTTTask(void *pvParameters);
void vHTTPTask(void *pvParameters);
void vHeartBeatTask(void *pvParameters);
void vTimerIdleCallback(TimerHandle_t xTimer);
void vTimerDelayCallback(TimerHandle_t xTimer);

static inline uint16_t* xCalcAdc(uint16_t *adc, uint16_t *resault) {
	for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
		if (i != 2) {
			resault[i] = ((adc[i] * 100U) / 1024U);
		} else {
			resault[i] = (uint16_t)((adc[i]) / 2.45);
		}
	}
	return resault;
}

static void ConvertValuesToTFT(uint16_t PosX, uint16_t PosY, char const *format, ...)
{
	va_list args;
	char buf[512];

	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	EF_PutString((const uint8_t*)buf, PosX, PosY, ILI9341_WHITE, BG_COLOR, ILI9341_BLACK);
}

static void Battery_Control(Analog_t *adc, uint8_t *percentage) {
	if (adc->Resault[2] >= 100) {
		*percentage = 25;
	} else {
		*percentage = ((adc->Resault[2] % 25U));
	}

	GFX_DrawFillRectangle(282, 12, *percentage, 11, ILI9341_GREEN);

	if (adc->Resault[2] > 82) {
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */
	HAL_SuspendTick();
	HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 0xFFFF, *ulExpectedIdleTime);
}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */
	HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
	HAL_ResumeTick();
}
/* USER CODE END PREPOSTSLEEP */

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
	xTimerIdle = xTimerCreate("TimerIdle", pdMS_TO_TICKS(1000), pdTRUE, (void*) 0, vTimerIdleCallback);
	xTimerDelay = xTimerCreate("TimerDelay", pdMS_TO_TICKS(1000), pdTRUE, (void*) 0, vTimerDelayCallback);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	xAnalogQueue = xQueueCreate(2, sizeof(Analog_t));
	xVeml7700Queue = xQueueCreate(2, sizeof(Veml7700_t));
	xBME680Queue = xQueueCreate(2, sizeof(BME680_TypeDef));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DefaultTask */
  DefaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &DefaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	xTaskCreate(vAnalogTask, "AnalogTask", 256, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vHeartBeatTask, "HeartBeatTask", 256, (void*) 1, LOW_PRIORITY, NULL);
	xTaskCreate(vBme680Task, "Bme680Task", 512, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vLCDTask, "LCDTask", 512, (void*) 1, HIGH_PRIORITY, &xLCDHandle);
	xTaskCreate(vLCDTouchTask, "LCDTouchTask", 256, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vVeml7700Task, "Veml7700Task", 256, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vMQTTTask, "vMQTTTask", 2048, (void*) 1, HIGH_PRIORITY, NULL);
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
  /* USER CODE BEGIN StartDefaultTask */
	if ( xTimerStart( xTimerIdle, 0 ) != pdPASS) {
		printf("Timer not created\n\r");
	}

	if( xTimerStart(xTimerDelay, 0) != pdPASS) {
		printf("Timer not created\n\r");
	}

	osThreadSuspend(DefaultTaskHandle);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Analog Task (VBat, Rain Sensor, Soil moisture sensor) */
void vAnalogTask(void *pvParameters) {

	configASSERT(((uint32_t ) pvParameters) == 1);

	Analog_t adc = { 0 };

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc.AdcRawValue, 3);

	for (;;) {

		xCalcAdc(adc.AdcRawValue, adc.Resault);
		xQueueSend(xAnalogQueue, &adc, 0);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

/* Weather sensor (temperature, humidity, atmospheric pressure */
void vBme680Task(void *pvParameters) {

	configASSERT(((uint32_t ) pvParameters) == 1);

	BME680_TypeDef Bme680;
	BME680_Calib_TypeDef Bme680_calib;

	xSemaphoreTake(xMutexI2C, portMAX_DELAY);
	Bme680_Init(&Bme680, &Bme680_calib, &hi2c1, BME680_ADDR);
	Bme680_Set_Conf(&Bme680, BME680_OSRS_T_OVR_SAMPLING_2, BME680_OSRS_H_OVR_SAMPLING_4, BME680_OSRS_P_OVR_SAMPLING_8, BME680_FILTER_7);
	Bme680_Set_Gas_Conf(&Bme680, &Bme680_calib ,Bme680.Gas_heat_dur);
	Bme680_Run_Gas(&Bme680);
	xSemaphoreGive(xMutexI2C);

	for (;;) {
		xSemaphoreTake(xMutexI2C, portMAX_DELAY);
		Bme680_Set_Mode(&Bme680, BME680_MODE_FORCE);
		BME680_calc_raw_values(&Bme680, &Bme680_calib);
		Bme680_Set_Mode(&Bme680, BME680_MODE_SLEEP);
		xSemaphoreGive(xMutexI2C);

		xQueueSend(xBME680Queue, &Bme680, 0);

		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

/* HeartBeat Task */
void vHeartBeatTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	for (;;) {
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

/* LCD TFT 2,8" */
void vLCDTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	BaseType_t xStatus;

	uint8_t percentage = 0;
	uint32_t flag;

	Analog_t adc = {0};
	Veml7700_t veml = {0};
	BME680_TypeDef Bme680 = {0};

	ILI9341_Init(&hspi1);
	EF_SetFont(&timesNewRoman_12ptFontInfo);
	ILI9341_DrawImage(0, 0, background, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);

	for (;;) {

		xStatus = xQueueReceive(xAnalogQueue, &adc, pdMS_TO_TICKS(0));
		if (xStatus == pdPASS) {
			if (adc.Resault[0] == 99) {
				EF_PutString((const uint8_t*) "Not Raining", 167, 65, ILI9341_RED, BG_COLOR, ILI9341_BLACK);
			} else {
				EF_PutString((const uint8_t*) "    Raining  ", 167, 65, ILI9341_CYAN, BG_COLOR, ILI9341_BLACK);
			}
			ConvertValuesToTFT(245, 110, "%d", adc.Resault[1]);
		}

		xStatus = xQueueReceive(xVeml7700Queue, &veml, pdMS_TO_TICKS(0));
		if (xStatus == pdPASS) {
			if (veml.calc_values[0] < 100) {
				ConvertValuesToTFT(245, 157, "%d   ", veml.calc_values[0]);
			} else {
				ConvertValuesToTFT(245, 157, "%d", veml.calc_values[0]);
			}
		}

		xStatus = xQueueReceive(xBME680Queue, &Bme680, pdMS_TO_TICKS(0));
		if (xStatus == pdPASS) {
			ConvertValuesToTFT(55, 110, "%.2f  ", Bme680.Temperature_Calc);
			ConvertValuesToTFT(55, 65, "%.2f  ", Bme680.Pressure_Calc);
			ConvertValuesToTFT(55, 160, "%.2f  ", Bme680.Humidity_Calc);
			ConvertValuesToTFT(55, 200, "%.2f  ", Bme680.IAQ_Calc);
		}

		Battery_Control(&adc, &percentage);

		xTaskNotifyWait(0, 0xFFFFFFFF, &flag, 100);

		switch (flag) {
		case 0x01:
			ILI9341_DrawImage(0, 0, background, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);
			EF_PutString((const uint8_t*) "Widget Values", 100, 10, ILI9341_MAGENTA, BG_TRANSPARENT, 0);
			vTaskResume(xTCPHandle);
			break;
		case 0x02:
			ILI9341_DrawImage(0, 0, background, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);
			vTaskSuspend(xTCPHandle);
			break;
		default:
			break;
		}

		if(DelayTick == 1) {
			ILI9341_ClearDisplay(ILI9341_BLACK, 282 + percentage, 12, 1, 11);
			DelayTick = 0;
		}
	}
}

/* LCD Touch */
void vLCDTouchTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	uint8_t counter = 0;

	uint16_t Xread, Yread;
	XPT2046_Init(&hspi2, EXTI0_IRQn);

	for (;;) {
		XPT2046_Task();
		if (XPT2046_IsTouched()) {
			XPT2046_GetTouchPoint(&Xread, &Yread);
			if (Xread > 20 && Yread > 190 && Xread < 110) {
				counter++;
			}

			switch (counter) {
			case 1:
				xTaskNotify(xLCDHandle, 0x01, eSetBits);
				break;
			case 2:
				xTaskNotify(xLCDHandle, 0x02, eSetBits);
			default:
				counter = 0;
				break;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}


/* Digital Ambient Light Sensor */
void vVeml7700Task(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	Veml7700_t Veml_data = {0};
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
		vTaskDelay(pdMS_TO_TICKS(40));

		xSemaphoreTake(xMutexI2C, portMAX_DELAY);
		Veml_data.als = VEML7700_read_als(&Veml7700);
		Veml_data.white = VEML7700_read_white(&Veml7700);

		Veml7700_Shutdown(&Veml7700);
		xSemaphoreGive(xMutexI2C);

		Veml_data.calc_values[0] = (Veml_data.als * VEML7700_RESOLUTION);
		Veml_data.calc_values[1] = (Veml_data.white * VEML7700_RESOLUTION);

		xQueueSend(xVeml7700Queue, &Veml_data, 0);

		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

/* Ethernet Task. LWIP and MQTT */
void vMQTTTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	err_t err = 1;
	extern struct netif gnetif;
	mqtt_client_t static_client = { 0 };

	BaseType_t xStatus;

	Analog_t adc = {0};
	Veml7700_t veml = {0};
	BME680_TypeDef Bme680 = {0};

	MX_LWIP_Init();
	MX_MBEDTLS_Init();
	ethernetif_notify_conn_changed(&gnetif);
	tcpclient_init();

	for (;;) {

		if (err != ERR_OK) {
			err = mqtt_user_connect(&static_client);
//			net_clear();
//			MX_MBEDTLS_Init();
//			net_connect();
		} else {
			xStatus = xQueueReceive(xAnalogQueue, &adc, pdMS_TO_TICKS(0));
			if (xStatus == pdPASS) {
				if (adc.Resault[0] == 99) {
					err = mqtt_user_publish(&static_client, 0, "Not Raining | Ground Moisture: %d%%", adc.Resault[1]);
				} else {
					err = mqtt_user_publish(&static_client, 0, "Raining | Ground Moisture: %d%%", adc.Resault[1]);
				}
			}

			xStatus = xQueueReceive(xVeml7700Queue, &veml, pdMS_TO_TICKS(0));
			if (xStatus == pdPASS) {
				err = mqtt_user_publish(&static_client, 0, "Lux: %d lx", veml.calc_values[0]);
			}

			xStatus = xQueueReceive(xBME680Queue, &Bme680, pdMS_TO_TICKS(0));
			if (xStatus == pdPASS) {
				err = mqtt_user_publish(&static_client, 0,
						"Temperature: %.2f *C | Pressure: %.2f hPa | Humidity: %.2f%% | IAQ: %.2f",
						Bme680.Temperature_Calc,
						(Bme680.Pressure_Calc),
						Bme680.Humidity_Calc, Bme680.IAQ_Calc);
			}
		}

		if (err != ERR_OK) {
			mqtt_disconnect(&static_client);
			mqtt_client_free(&static_client);
		}

		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

void vHTTPTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	tcpinit();
}

void _putchar(char character) {
// send char to console etc.
	xSemaphoreTake(xMutexPrintf, portMAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t*) &character, 1, 1000);
	xSemaphoreGive(xMutexPrintf);
}

/* USER CODE END Application */

