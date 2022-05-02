#include "main.h"
#include "i2c.h"
#include "Veml7700.h"

static uint16_t read_register(VEML7700_TypeDef *VEML7700, uint8_t Register) {
	uint8_t Value[2] = { 0 };

	int err = HAL_I2C_Mem_Read(VEML7700->veml7700_i2c, VEML7700->write_addr,
			Register, sizeof(Register), &Value[0], 2, VEML7700_I2C_TIMEOUT);
	if (err != HAL_OK) {
		return 0;
	}
	return (Value[1] << 8) | Value[0];
}

static uint32_t write_register(VEML7700_TypeDef *VEML7700, uint8_t Register,
		uint16_t value) {
	uint8_t Value[3] = { Register, value & 0xFF, value >> 8 };

	return HAL_I2C_Master_Transmit(VEML7700->veml7700_i2c, VEML7700->write_addr,
			&Value[0], 3, VEML7700_I2C_TIMEOUT);
}

uint32_t Veml7700_Init(VEML7700_TypeDef *VEML7700, I2C_HandleTypeDef *i2c,
		uint8_t Address) {

	VEML7700->veml7700_i2c = i2c;
	VEML7700->read_addr = (Address << 1) | 0x01;
	VEML7700->write_addr = (Address << 1);

	return write_register(VEML7700, VEML7700_CONF, 0);
}

uint32_t Veml7700_Power_On(VEML7700_TypeDef *VEML7700) {
	// Read current config and turn on the sensor
	uint16_t Power_config = read_register(VEML7700, VEML7700_CONF);
	Power_config &= ~REG_ALS_CONF_SHUTDOWN;

	return write_register(VEML7700, VEML7700_CONF, Power_config);
}

uint32_t Veml7700_Shutdown(VEML7700_TypeDef *VEML7700) {
	// Read current config and shutdown the sensor
	uint16_t Power_config = read_register(VEML7700, VEML7700_CONF);
	Power_config |= REG_ALS_CONF_SHUTDOWN;

	return write_register(VEML7700, VEML7700_CONF, Power_config);
}

uint32_t Veml7700_Set_Als_Integration_Time(VEML7700_TypeDef *VEML7700,
		uint16_t it) {
	uint16_t config = read_register(VEML7700, VEML7700_CONF);
	config &= ~REG_ALS_CONF_IT_CLEAR;
	config |= it;
	return write_register(VEML7700, VEML7700_CONF, config);
}

uint16_t VEML7700_Get_Als_Integration_Time(VEML7700_TypeDef *VEML7700) {
	uint16_t config = read_register(VEML7700, VEML7700_CONF);
	return (config & REG_ALS_CONF_IT_CLEAR) >> 6;
}

uint32_t VEML7700_Set_Als_Gain(VEML7700_TypeDef *VEML7700, uint16_t gain) {
	uint16_t config = read_register(VEML7700, VEML7700_CONF);
	// Clear all gain bits
	config &= ~REG_ALS_CONF_GAIN_1_4;
	config |= gain;
	return write_register(VEML7700, VEML7700_CONF, gain);
}

uint16_t VEML7700_get_als_gain(VEML7700_TypeDef *VEML7700) {
	uint16_t config = read_register(VEML7700, VEML7700_CONF);
	return (config & REG_ALS_CONF_GAIN_1_4) >> 11;
}

uint32_t VEML7700_Set_PSM(VEML7700_TypeDef *VEML7700, uint16_t psm_mode) {
	uint16_t config = read_register(VEML7700, VEML7700_PWR_SAVE);
	config &= REG_POWER_SAVING_PSM_4;
	config |= psm_mode;
	return write_register(VEML7700, VEML7700_PWR_SAVE, psm_mode);
}

uint16_t VEML7700_read_als(VEML7700_TypeDef *VEML7700) {
	return read_register(VEML7700, VEML7700_ALS_OUTPUT_DATA);
}

uint16_t VEML7700_read_white(VEML7700_TypeDef *VEML7700) {
	return read_register(VEML7700, VEML7700_WHITE_OUTPUT_DATA);
}

