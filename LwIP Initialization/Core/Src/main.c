/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c);
/* Direct printf to output somewhere */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#ifndef __UUID_H
#define __UUID_H
//#define STM32_UUID ((uint32_t *)0x1FF0F420)
#define STM32_UUID ((uint32_t *)UID_BASE)
#endif //__UUID_H


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void ethernetif_notify_conn_changed(struct netif *netif);
//static void MX_NVIC_Init(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
//  HAL_I2C_MspInit(&hi2c1);
//  I2C_ClearBusyFlagErratum(&hi2c1);
//  HAL_GPIO_WritePin(WAKE_GPIO_Port, WAKE_Pin, GPIO_PIN_SET);
//  HAL_Delay(100);
  HAL_GPIO_WritePin(WAKE_GPIO_Port, WAKE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

     char uart2Data[25] = "Connected to UART Three\r\n";
        /*
         * Output to uart2
         * use screen or putty or whatever terminal software
         * 8N1 115200
         */
        HAL_UART_Transmit(&huart3, (uint8_t *)&uart2Data,sizeof(uart2Data), 0xFFFF);

      	printf("\r\n");

      	printf("Scanning I2C bus:\r\n");
     	HAL_StatusTypeDef result;
      	uint8_t i;
      	for (i=1; i<128; i++)
      	{
      	  /*
      	   * the HAL wants a left aligned i2c address
      	   * &hi2c1 is the handle
      	   * (uint16_t)(i<<1) is the i2c address left aligned
      	   * retries 2
      	   * timeout 2
      	   */
      	  result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
      	  if (result == HAL_BUSY)
      	  {
//      		I2C_ClearBusyFlagErratum(&hi2c1);
      		printf("HAL_BUSY\r\n"); // No ACK received at that address
      	  }
      	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
      	  {
//      		I2C_ClearBusyFlagErratum(&hi2c1);
      		  printf("."); // No ACK received at that address
      	  }
      	  if (result == HAL_OK)
      	  {
      		  printf("0x%X", i); // Received an ACK at that address
      	  }

      	  result = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 2, 2);
      	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
      	  {
      		  printf("."); // No ACK received at that address
      	  }
      	  if (result == HAL_OK)
      	  {
      		  printf("0x%X", i); // Received an ACK at that address
      	  }
      	}
      	printf("\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  MX_LWIP_Process();
	  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	  HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;

}

void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *i2c) {
	static uint8_t resetTried = 0;
	if (resetTried == 1) {
		return;
	}

	uint32_t SDA_PIN = sda_Pin;
	uint32_t SCL_PIN = scl_Pin;
	GPIO_InitTypeDef GPIO_InitStruct;

	// 1. Clear PE bit.
	__HAL_I2C_DISABLE(i2c);

	//  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).

	HAL_I2C_DeInit(i2c);
	GPIO_InitStruct.Pin = SCL_PIN | SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_Init(scl_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_Init(sda_GPIO_Port, &GPIO_InitStruct);

	// 3. Check SCL and SDA High level in GPIOx_IDR.

	HAL_GPIO_WritePin(scl_GPIO_Port, SCL_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(sda_GPIO_Port, SDA_PIN, GPIO_PIN_SET);

	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(scl_GPIO_Port, SCL_PIN)) {
		asm("nop");
	}

	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sda_GPIO_Port, SDA_PIN)) {
		asm("nop");
	}

	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(sda_GPIO_Port, SDA_PIN, GPIO_PIN_RESET);

	//  5. Check SDA Low level in GPIOx_IDR.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(sda_GPIO_Port, SDA_PIN)) {
		asm("nop");
	}

	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(scl_GPIO_Port, SCL_PIN, GPIO_PIN_RESET);

	//  7. Check SCL Low level in GPIOx_IDR.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(scl_GPIO_Port, SCL_PIN)) {
		asm("nop");
	}

	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(scl_GPIO_Port, SCL_PIN, GPIO_PIN_SET);

	// 9. Check SCL High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(scl_GPIO_Port, SCL_PIN)) {
		asm("nop");
	}

	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(sda_GPIO_Port, SDA_PIN, GPIO_PIN_SET);

	// 11. Check SDA High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sda_GPIO_Port, SDA_PIN)) {
		asm("nop");
	}

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

	GPIO_InitStruct.Pin = SCL_PIN;
	HAL_GPIO_Init(scl_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SDA_PIN;
	HAL_GPIO_Init(sda_GPIO_Port, &GPIO_InitStruct);

	// 13. Set SWRST bit in I2Cx_CR1 register.
	i2c->Instance->CR1 |= 0x8000;

	asm("nop");

	// 14. Clear SWRST bit in I2Cx_CR1 register.
	i2c->Instance->CR1 &= ~0x8000;

	asm("nop");

	// 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
	i2c->Instance->CR1 |= 0x0001;

	// Call initialization function.
	HAL_I2C_Init(i2c);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

