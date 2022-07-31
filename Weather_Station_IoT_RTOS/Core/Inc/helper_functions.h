#ifndef INC_HELPER_FUNCTIONS_H_
#define INC_HELPER_FUNCTIONS_H_

#define LOW_PRIORITY 1
#define NORMAL_PRIORITY 2
#define HIGH_PRIORITY 3

#define ADC_SAMPLES 3
#define VEML7700_SAMPLES 2
#define BME680_SAMPLES 4

typedef struct {
	uint16_t AdcRawValue[3];
	uint16_t Resault[3];
} Analog_t;

typedef struct {
	uint16_t als;
	uint16_t white;
	uint16_t calc_values[2];
} Veml7700_t;

uint16_t* xCalcAdc(uint16_t *adc, uint16_t *resault);
void ConvertValuesToTFT(uint16_t PosX, uint16_t PosY, char const *format, ...);
void Battery_Control(Analog_t *adc, uint32_t *percentage);

#endif /* INC_HELPER_FUNCTIONS_H_ */
