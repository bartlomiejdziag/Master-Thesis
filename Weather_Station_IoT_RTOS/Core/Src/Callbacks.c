#include "FreeRTOS.h"
#include "main.h"

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

