/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  REQ = 0,
  RESP = 1
} packet_type;

struct time_packet
{
  uint8_t head; //0xAE
  uint8_t type; //0:REQ, 1:RESP
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t dummy[247]; //you may add more information
  uint8_t tail; //0xEA
};
//256 bytes

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVER_IP1  192 //server ip address
#define SERVER_IP2  168
#define SERVER_IP3  1
#define SERVER_IP4  227
#define SERVER_PORT	1234 //server listen port
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern struct netif gnetif; //extern gnetif
osThreadId tcpClientTaskHandle;  //tcp client task handle
ip_addr_t server_addr; //server address
struct time_packet packet; //256 bytes time_packet structure

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartTcpClientTask(void const *argument); //tcp client task function
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); //turn on red led when detects stack overflow
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
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
  osThreadDef(tcpClientTask, StartTcpClientTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  tcpClientTaskHandle = osThreadCreate(osThread(tcpClientTask), NULL); //run tcp client task

  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); //toggle running led
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartTcpClientTask(void const *argument)
{
  err_t err;
  struct netconn *conn;
  struct netbuf *buf;
  void *data;

  u16_t len; //buffer length
  u16_t nRead; //read buffer index
  u16_t nWritten; //write buffer index

  LWIP_UNUSED_ARG(argument);

  while (1)
  {
    if (gnetif.ip_addr.addr == 0 || gnetif.netmask.addr == 0 || gnetif.gw.addr == 0) //system has no valid ip address
    {
      osDelay(1000);
      continue;
    }
    else //valid ip address
    {
      osDelay(100); //request interval
    }

    nRead = 0; //clear indexes
    nWritten = 0;

    conn = netconn_new(NETCONN_TCP); //new tcp netconn

    if (conn != NULL)
    {
      IP4_ADDR(&server_addr, SERVER_IP1, SERVER_IP2, SERVER_IP3, SERVER_IP4); //server ip
      err = netconn_connect(conn, &server_addr, SERVER_PORT); //connect to the server

      if (err != ERR_OK)
      {
        netconn_delete(conn); //free memory
        continue;
      }

      memset(&packet, 0, sizeof(struct time_packet));
      packet.head = 0xAE; //head
      packet.type = REQ; //request type
      packet.tail = 0xEA; //tail

      do
      {
        if (netconn_write_partly(
            conn, //connection
            (const void*) (&packet + nWritten), //buffer pointer
            (sizeof(struct time_packet) - nWritten), //buffer length
            NETCONN_NOFLAG, //no copy
            (size_t*) &len) != ERR_OK) //written len
        {
          netconn_close(conn); //close session
          netconn_delete(conn); //free memory
          continue;
        }
        else
        {
          nWritten += len;
        }
      }
      while (nWritten < sizeof(struct time_packet)); //send request

      while (netconn_recv(conn, &buf) == ERR_OK) //receive the response
      {
        do
        {
          netbuf_data(buf, &data, &len); //receive data pointer & length

          memcpy(&packet + nRead, data, len);
          nRead += len;
        }
        while (netbuf_next(buf) >= 0); //check buffer empty
        netbuf_delete(buf); //clear buffer
      }

      if (nRead == sizeof(struct time_packet) && packet.type == RESP) //if received length is valid
      {
        printf("%04d-%02d-%02d %02d:%02d:%02d\n", packet.year + 2000, packet.month, packet.day, packet.hour, packet.minute, packet.second); //print time information
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //toggle data led
      }

      netconn_close(conn); //close session
      netconn_delete(conn); //free memory
    }
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
