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
#include "lwip.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include <ip_addr.h>
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
#include "background.h"
#include "XPT2046.h"
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

TaskHandle_t xLCDHandle = NULL;
SemaphoreHandle_t xMutexPrintf, xMutexIdle, xMutexI2C;
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
void vEthernetTask(void *pvParameters);
void vHeartBeatTask(void *pvParameters);
void vTimerIdleCallback(TimerHandle_t xTimer);
void vTimerDelayCallback(TimerHandle_t xTimer);
err_t example_do_connect(mqtt_client_t *client);

static int inpub_id;
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
  printf("Incoming publish at topic %s with total length %u\n\r", topic, (unsigned int)tot_len);

  /* Decode topic string into a user defined reference */
  if(strcmp(topic, "print_payload") == 0) {
    inpub_id = 0;
  } else if(topic[0] == 'A') {
    /* All topics starting with 'A' might be handled at the same way */
    inpub_id = 1;
  } else {
    /* For all other topics */
    inpub_id = 2;
  }
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
  printf("Incoming publish payload with length %d, flags %u\n\r", len, (unsigned int)flags);

  if(flags & MQTT_DATA_FLAG_LAST) {
    /* Last fragment of payload received (or whole part if payload fits receive buffer
       See MQTT_VAR_HEADER_BUFFER_LEN)  */

    /* Call function or do action depending on reference, in this case inpub_id */
    if(inpub_id == 0) {
      /* Don't trust the publisher, check zero termination */
      if(data[len-1] == 0) {
        printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
      }
    } else if(inpub_id == 1) {
      /* Call an 'A' function... */
    } else {
      printf("mqtt_incoming_data_cb: Ignoring payload...\n\r");
    }
  } else {
    /* Handle fragmented payload, store in buffer, write to file or whatever */
  }
}

static void mqtt_sub_request_cb(void *arg, err_t result)
{
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
  printf("Subscribe result: %d\n\r", result);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  err_t err;
  if(status == MQTT_CONNECT_ACCEPTED) {
    printf("mqtt_connection_cb: Successfully connected\n\r");

    /* Setup callback for incoming publish requests */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

    /* Subscribe to a topic named "subtopic" with QoS level 1, call mqtt_sub_request_cb with result */
    err = mqtt_subscribe(client, "lwip_test", 1, mqtt_sub_request_cb, arg);

    if(err != ERR_OK) {
      printf("mqtt_subscribe return: %d\n\r", err);
    }
  } else {
    printf("mqtt_connection_cb: Disconnected, reason: %d\n\r", status);

    /* Its more nice to be connected, so try to reconnect */
    example_do_connect(client);
  }
}

/* Called when publish is complete either with sucess or failure */
static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n\r", result);
  }
}

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

err_t example_do_connect(mqtt_client_t *client)
{

    struct mqtt_connect_client_info_t ci;
    err_t err;
    ip_addr_t server;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = "lwip_test";
    ci.client_user = "xxx";
    ci.client_pass = "xxx";

    ip4_addr_set_u32(&server, ipaddr_addr("192.168.1.13"));
    err = mqtt_client_connect(client, &server, MQTT_PORT, mqtt_connection_cb, 0, &ci);

    if (err != ERR_OK) {
        printf("mqtt_connect return %d\n\r", err);
    }

    return err;
}

void example_publish(mqtt_client_t *client, void *arg)
{
  const char *pub_payload= "Test MQTT";
  err_t err;
  u8_t qos = 1; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
  err = mqtt_publish(client, "lwip_test", pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, arg);
  if(err != ERR_OK) {
    printf("Publish err: %d\n\r", err);
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
	xTaskCreate(vHeartBeatTask, "HeartBeatTask", 128, (void*) 1, LOW_PRIORITY, NULL);
	xTaskCreate(vBme680Task, "Bme680Task", 512, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vLCDTask, "LCDTask", 512, (void*) 1, HIGH_PRIORITY, &xLCDHandle);
	xTaskCreate(vLCDTouchTask, "LCDTouchTask", 256, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vVeml7700Task, "Veml7700Task", 256, (void*) 1, NORMAL_PRIORITY, NULL);
	xTaskCreate(vEthernetTask, "EthernetTask", 512, (void*) 1, HIGH_PRIORITY, NULL);
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

	uint8_t percentage;
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
			ConvertValuesToTFT(55, 65, "%.2f  ", (Bme680.Pressure_Calc / 100.0f) + 10.0f);
			ConvertValuesToTFT(55, 160, "%.2f  ", Bme680.Humidity_Calc);
			ConvertValuesToTFT(55, 200, "%.2f  ", Bme680.IAQ_Calc);
		}

		if (adc.Resault[2] > 82) {
			if (adc.Resault[2] >= 100) {
				percentage = 24;
			} else {
				percentage = ((adc.Resault[2] % 25U));
			}
			GFX_DrawFillRectangle(282, 12, percentage, 11, ILI9341_GREEN);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		}

		xTaskNotifyWait(0, 0xFFFFFFFF, &flag, 100);

		switch (flag) {
		case 0x01:
			ILI9341_DrawImage(0, 0, background, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);
			EF_PutString((const uint8_t*) "Widget Values", 100, 10, ILI9341_MAGENTA, BG_TRANSPARENT, 0);
			break;
		case 0x02:
			ILI9341_DrawImage(0, 0, background, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);
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
void vEthernetTask(void *pvParameters) {
	configASSERT(((uint32_t ) pvParameters) == 1);

	err_t err = 1;
	extern struct netif gnetif;
	mqtt_client_t static_client = { 0 };

	MX_LWIP_Init();
	ethernetif_notify_conn_changed(&gnetif);

	for (;;) {

		if (err != ERR_OK) {
			err = example_do_connect(&static_client);
		} else {
			example_publish(&static_client, 0);
		}

		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}


void _putchar(char character) {
// send char to console etc.
	xSemaphoreTake(xMutexPrintf, portMAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t*) &character, 1, 1000);
	xSemaphoreGive(xMutexPrintf);
}

/* USER CODE END Application */

