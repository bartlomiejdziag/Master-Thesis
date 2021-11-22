/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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
#include <string.h>
#include "lwip.h"
#include "lwip/api.h"
#include "MQTTClient.h"
#include "MQTTInterface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BROKER_IP "192.168.1.227"
#define MQTT_PORT 1883
#define MQTT_BUFSIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern struct netif gnetif; // Extern gnetif

osThreadId mqttClientSubTaskHandle; // MQTT client task handle
osThreadId mqttClientPubTaskHandle; // MQTT client task handle

Network net; // MQTT network
MQTTClient mqttClient; // MQTT Client

uint8_t sndBuffer[MQTT_BUFSIZE]; // MQTT SEND BUFFER
uint8_t rcvBuffer[MQTT_BUFSIZE]; // MQTT RECEIVE BUFFER
uint8_t msgBuffer[MQTT_BUFSIZE]; // MQTT message BUFFER

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MqttClientSubTask(void const * argument); // MQTT client subscribe task function
void MqttClientPubTask(void const * argument); // MQTT client publish task function
int MqttConnectBroker(void);
void MqttMessageArrived(MessageData * msg); // MQTT message callback function
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask,
		signed char *pcTaskName) {
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Turn on red led when detects stack overflow
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
	osThreadDef(mqttClientSubTask, MqttClientSubTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE); // Subscribe task
	osThreadDef(mqttClientPubTask, MqttClientPubTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE); // Publish task
	mqttClientSubTaskHandle = osThreadCreate(osThread(mqttClientSubTask), NULL);
	mqttClientPubTaskHandle = osThreadCreate(osThread(mqttClientPubTask), NULL);

	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); // Toggle running led
		osDelay(500);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void MqttClientSubTask(void const *argument) {
	while (1) {
		// waiting for valid ip address
		if (gnetif.ip_addr.addr == 0 || gnetif.netmask.addr == 0 || gnetif.gw.addr == 0) { // system has no valid ip address
			osDelay(1000);
			continue; // immediately leave while
		} else {
			printf("DHCP/Static IP ok\n");
			break;
		}
	}

	while (1) {
		if (!mqttClient.isconnected) {
			// Try to connect to the broker
			MQTTDisconnect(&mqttClient);
			MqttConnectBroker();
			osDelay(1000);
		} else {
			MQTTYield(&mqttClient, 1000); // Handle timer
			osDelay(100);
		}
	}
}

void MqttClientPubTask(void const * argument) {
	const char *str = "MQTT message from STM32";
	MQTTMessage message;

	while(1){
		if(mqttClient.isconnected) {
			message.payload = (void*)str;
			message.payloadlen = strlen(str);

			MQTTPublish(&mqttClient, "test", &message); // Publish a message
		}
		osDelay(1000);
	}
}

int MqttConnectBroker() {
	int ret;

	NewNetwork(&net);
	ret = ConnectNetwork(&net, BROKER_IP, MQTT_PORT);
	if (ret != MQTT_SUCCESS) {
		printf("ConnectNetwork failed.\n");
		return -1;
	}

	MQTTClientInit(&mqttClient, &net, 1000, sndBuffer, sizeof(sndBuffer),
			rcvBuffer, sizeof(rcvBuffer));

	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	data.willFlag = 0;
	data.MQTTVersion = 3;
	data.clientID.cstring = "STM32F4";
	data.username.cstring = "STM32F4";
	data.password.cstring = "";
	data.keepAliveInterval = 60;
	data.cleansession = 1;

	ret = MQTTConnect(&mqttClient, &data);
	if (ret != MQTT_SUCCESS) {
		printf("MQTTConnect failed.\n");
		return ret;
	}

	ret = MQTTSubscribe(&mqttClient, "test", QOS0, MqttMessageArrived);
	if (ret != MQTT_SUCCESS) {
		printf("MQTTSubscribe failed.\n");
		return ret;
	}

	printf("MQTT_ConnectBroker OK\n");
	return MQTT_SUCCESS;
}

void MqttMessageArrived(MessageData *msg) {
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //toggle pin when new message arrived

	MQTTMessage *message = msg->message;
	memset(msgBuffer, 0, sizeof(msgBuffer));
	memcpy(msgBuffer, message->payload, message->payloadlen);

	printf("MQTT MSG[%d]:%s\n", (int) message->payloadlen, msgBuffer);
}



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
