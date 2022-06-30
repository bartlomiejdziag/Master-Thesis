#include "main.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"
#include "XPT2046.h"

extern volatile uint32_t DelayTick;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == TOUCH_IRQ_Pin) {
		XPT2046_IRQ();
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

void vTimerIdleCallback(TimerHandle_t xTimer) {
	configASSERT(xTimer);

}

void vTimerDelayCallback(TimerHandle_t xTimer) {
	configASSERT(xTimer);
	DelayTick = 1;
}
