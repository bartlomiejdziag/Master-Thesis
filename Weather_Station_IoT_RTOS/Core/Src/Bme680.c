#include "main.h"
#include "FreeRTOS.h"
#include "i2c.h"
#include "Bme680.h"


static uint16_t read_register8(BME680_TypeDef *BME680, uint8_t Register) {
	uint8_t Value = 0;

	int err = HAL_I2C_Mem_Read(BME680->bme680_i2c, BME680->write_addr, Register,
			sizeof(Register), &Value, sizeof(Value), BME680_I2C_TIMEOUT);
	if (err != HAL_OK) {
		return 0;
	}
	return Value;
}

static uint32_t write_register8(BME680_TypeDef *BME680, uint8_t Register,
		uint8_t value) {
	uint8_t Value[2] = { Register, value };

	return HAL_I2C_Master_Transmit(BME680->bme680_i2c, BME680->write_addr,
			&Value[0], sizeof(Value), BME680_I2C_TIMEOUT);
}

static uint32_t Bme680_Set_Register(BME680_TypeDef *BME680, uint8_t Msk, uint8_t Reg_addr, uint8_t Bit_addr) {
	uint8_t rslt;
	uint8_t config = read_register8(BME680, Reg_addr);
	config &= Msk;
	config |= Bit_addr;
	return rslt = write_register8(BME680, Reg_addr, config);
}

static uint8_t BME680_Calc_Heater_Dur(uint16_t dur) {
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xfc0) {
		durval = 0xff; /* Max duration*/
	} else {
		while (dur > 0x3F) {
			dur = dur / 4;
			factor += 1;
		}
		durval = (uint8_t) (dur + (factor * 64));
	}

	return durval;
}

uint8_t Bme680_Init(BME680_TypeDef *BME680, I2C_HandleTypeDef *i2c,
		uint8_t Address) {

	uint32_t rslt;

	BME680->bme680_i2c = i2c;
	BME680->write_addr = (Address << 1);
	BME680->Gas_heat_dur = 100;
	BME680->Gas_heat_temp = 320;

	rslt = Bme680_Reset(BME680);
	vTaskDelay(5 / portTICK_PERIOD_MS); // For sure it should be there?

	if (rslt == HAL_OK) {

		BME680->chipID = read_register8(BME680, BME680_CHIPID_REGISTER);

		if (BME680->chipID != BME680_CHIPID) {
			return BME680_ERROR;
		} else {
			return BME680_OK;
		}
	} else {
		return BME680_ERROR;
	}
}

uint32_t Bme680_Reset(BME680_TypeDef *BME680) {
	return write_register8(BME680, BME680_RESET_REGISTER, BME680_RESET);
}

uint32_t Bme680_Set_Mode(BME680_TypeDef *BME680, uint8_t Mode) {
	uint8_t config = read_register8(BME680, BME680_CTRL_MEAS_REGISTER);
	config &= BME680_OSP_OST_MSK;
	config |= Mode;
	return write_register8(BME680, BME680_CTRL_MEAS_REGISTER, config);
}

uint32_t Bme680_Set_Conf(BME680_TypeDef *BME680, uint8_t os_t, uint8_t os_h, uint8_t os_p, uint8_t filter) {
	uint8_t rslt;
	uint8_t config = read_register8(BME680, BME680_CTRL_MEAS_REGISTER);
	config &= (BME680_MODE_MSK);

	if (config == BME680_MODE_FORCE) {
		Bme680_Reset(BME680);
	}
	rslt = Bme680_Set_Register(BME680, BME680_OSH_MSK, BME680_CTRL_HUM_REGISTER, os_h);
	rslt = Bme680_Set_Register(BME680, BME680_OST_MSK, BME680_CTRL_MEAS_REGISTER, os_t);
	rslt = Bme680_Set_Register(BME680, BME680_OSP_OST_MSK, BME680_CTRL_MEAS_REGISTER, os_p);
	rslt = Bme680_Set_Register(BME680, BME680_FILTER_MSK, BME680_CONFIG_REGISTER, filter);

	return rslt;
}

uint32_t Bme680_Run_Gas(BME680_TypeDef *BME680) {
	uint8_t rslt;
	return rslt = Bme680_Set_Register(BME680, BME680_RUN_GAS_MSK, BME680_CTRL_GAS_1_REGISTER, BME680_RUN_GAS);
}

uint32_t Bme680_Set_Gas_Conf(BME680_TypeDef *BME680, uint16_t heat_dur ) {
	uint8_t rslt;
	uint8_t calc_dur;
	calc_dur = BME680_Calc_Heater_Dur(heat_dur);
	rslt = Bme680_Set_Register(BME680, BME680_NBCONV_MSK, BME680_CTRL_GAS_1_REGISTER, BME680_HEATER_PROF_SET_POINT_0);
	rslt = Bme680_Set_Register(BME680, 0xFF, BME680_GAS_WAIT_0_REGISTER, calc_dur);
//	rslt = Bme680_Set_Register(BME680, 0x00, BME680_RES_HEAT_0_REGISTER, BME680_GAS_WAIT_100MS);
	return rslt;
}

uint16_t Bme680_Read_Humidity(BME680_TypeDef *BME680) {
	uint8_t Value[2] = { 0 };
	Value[0] = read_register8(BME680, BME680_HUM_LSB_REGISTER);
	Value[1] = read_register8(BME680, BME680_HUM_MSB_REGISTER);
	BME680->Humidity_Raw = ((Value[1] << 8) | Value[0]);
	return BME680->Humidity_Raw;
}

uint32_t Bme680_Read_Temperature(BME680_TypeDef *BME680) {
	uint8_t Value[3] = { 0 };
	Value[0] = (read_register8(BME680, BME680_TEMP_XLSB_REGISTER) & 0x0F);
	Value[1] = read_register8(BME680, BME680_TEMP_LSB_REGISTER);
	Value[2] = read_register8(BME680, BME680_TEMP_MSB_REGISTER);
	BME680->Temperature_Raw = (((Value[2] << 16) | (Value[1] << 8) | Value[0]) & 0x00FFFFFF);
	return BME680->Temperature_Raw;
}

uint32_t Bme680_Read_Pressure(BME680_TypeDef *BME680) {
	uint8_t Value[3] = { 0 };
	Value[0] = (read_register8(BME680, BME680_PRESS_XLSB_REGISTER) & 0x0F);
	Value[1] = read_register8(BME680, BME680_PRESS_LSB_REGISTER);
	Value[2] = read_register8(BME680, BME680_PRESS_MSB_REGISTER);
	BME680->Pressure_Raw = (((Value[2] << 16) | (Value[1] << 8) | Value[0]) & 0x00FFFFFF);
	return BME680->Pressure_Raw;
}
