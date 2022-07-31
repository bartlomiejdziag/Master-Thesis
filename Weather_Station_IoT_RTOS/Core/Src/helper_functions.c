#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "queue.h"
#include "Bme680.h"
#include "lwip/apps/mqtt_priv.h"
#include "helper_functions.h"
#include "MQTT_Interface.h"
#include "TFT_ILI9341.h"
#include "GFX_Color.h"
#include "GFX_EnhancedFonts.h"
#include "printf.h"

extern QueueHandle_t xAnalogQueue, xVeml7700Queue, xBME680Queue, xWeatherQueue;

uint16_t* xCalcAdc(uint16_t *adc, uint16_t *resault) {
	for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
		if (i != 2) {
			resault[i] = ((adc[i] * 100U) / 1024U);
		} else {
			resault[i] = (uint16_t)((adc[i]) / 2.45);
		}
	}
	return resault;
}

void ConvertValuesToTFT(uint16_t PosX, uint16_t PosY, char const *format, ...)
{
	va_list args;
	char buf[512];

	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	EF_PutString((const uint8_t*)buf, PosX, PosY, ILI9341_WHITE, BG_COLOR, ILI9341_BLACK);
}

void Battery_Control(Analog_t *adc, uint32_t *percentage) {
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
