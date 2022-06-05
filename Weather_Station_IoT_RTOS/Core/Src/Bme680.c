#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"
#include "Bme680.h"

static uint8_t read_register8(BME680_TypeDef *BME680, uint8_t Register) {
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

static int8_t get_calib_data(BME680_TypeDef *BME680, BME680_Calib_TypeDef *dev) {

	uint8_t coeff_array[BME680_COEFF_SIZE] = { 0 };
	uint8_t temp_var = 0; /* Temporary variable */
	uint8_t i, j;
	/* Append the first half in the array */
	for (i = 0; i < 25; i++) {
		coeff_array[i] = read_register8(BME680, BME680_COEFF_ADDR1 + i);
	}
	/* Append the second half in the same array */
	for (j = 0; j < 16; j++) {
		coeff_array[i + j] = read_register8(BME680, BME680_COEFF_ADDR2 + j);
	}

	/* Temperature related coefficients */
	dev->par_t1 = (uint16_t) (BME680_CONCAT_BYTES(
			coeff_array[BME680_T1_MSB_REG], coeff_array[BME680_T1_LSB_REG]));
	dev->par_t2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T2_MSB_REG],
			coeff_array[BME680_T2_LSB_REG]));
	dev->par_t3 = (int8_t) (coeff_array[BME680_T3_REG]);

	/* Pressure related coefficients */
	dev->par_p1 = (uint16_t) (BME680_CONCAT_BYTES(
			coeff_array[BME680_P1_MSB_REG], coeff_array[BME680_P1_LSB_REG]));
	dev->par_p2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P2_MSB_REG],
			coeff_array[BME680_P2_LSB_REG]));
	dev->par_p3 = (int8_t) coeff_array[BME680_P3_REG];
	dev->par_p4 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P4_MSB_REG],
			coeff_array[BME680_P4_LSB_REG]));
	dev->par_p5 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P5_MSB_REG],
			coeff_array[BME680_P5_LSB_REG]));
	dev->par_p6 = (int8_t) (coeff_array[BME680_P6_REG]);
	dev->par_p7 = (int8_t) (coeff_array[BME680_P7_REG]);
	dev->par_p8 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P8_MSB_REG],
			coeff_array[BME680_P8_LSB_REG]));
	dev->par_p9 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P9_MSB_REG],
			coeff_array[BME680_P9_LSB_REG]));
	dev->par_p10 = (uint8_t) (coeff_array[BME680_P10_REG]);

	/* Humidity related coefficients */
	dev->par_h1 = (uint16_t) (((uint16_t) coeff_array[BME680_H1_MSB_REG]
			<< BME680_HUM_REG_SHIFT_VAL)
			| (coeff_array[BME680_H1_LSB_REG] & BME680_BIT_H1_DATA_MSK));
	dev->par_h2 = (uint16_t) (((uint16_t) coeff_array[BME680_H2_MSB_REG]
			<< BME680_HUM_REG_SHIFT_VAL)
			| ((coeff_array[BME680_H2_LSB_REG]) >> BME680_HUM_REG_SHIFT_VAL));
	dev->par_h3 = (int8_t) coeff_array[BME680_H3_REG];
	dev->par_h4 = (int8_t) coeff_array[BME680_H4_REG];
	dev->par_h5 = (int8_t) coeff_array[BME680_H5_REG];
	dev->par_h6 = (uint8_t) coeff_array[BME680_H6_REG];
	dev->par_h7 = (int8_t) coeff_array[BME680_H7_REG];

	/* Gas heater related coefficients */
	dev->par_gh1 = (int8_t) coeff_array[BME680_GH1_REG];
	dev->par_gh2 = (int16_t) (BME680_CONCAT_BYTES(
			coeff_array[BME680_GH2_MSB_REG], coeff_array[BME680_GH2_LSB_REG]));
	dev->par_gh3 = (int8_t) coeff_array[BME680_GH3_REG];

	/* Other coefficients */
	temp_var = read_register8(BME680, BME680_ADDR_RES_HEAT_RANGE_ADDR);
	dev->res_heat_range = ((temp_var & BME680_RHRANGE_MSK) / 16);

	temp_var = read_register8(BME680, BME680_ADDR_RES_HEAT_VAL_ADDR);
	dev->res_heat_val = (int8_t) temp_var;

	temp_var = read_register8(BME680, BME680_ADDR_RANGE_SW_ERR_ADDR);
	dev->range_sw_err = ((int8_t) temp_var & (int8_t) BME680_RSERROR_MSK) / 16;

	return BME680_OK;
}

static int16_t calc_temperature(uint32_t temp_adc, BME680_Calib_TypeDef *dev) {
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int16_t calc_temp;

	var1 = ((int32_t) temp_adc >> 3) - ((int32_t) dev->par_t1 << 1);
	var2 = (var1 * (int32_t) dev->par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t) dev->par_t3 << 4)) >> 14;
	dev->t_fine = (int32_t) (var2 + var3);
	calc_temp = (int16_t) (((dev->t_fine * 5) + 128) >> 8);

	return calc_temp;
}

static uint32_t calc_pressure(uint32_t pres_adc, BME680_Calib_TypeDef *dev) {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t pressure_comp;

	var1 = (((int32_t)dev->t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
		(int32_t)dev->par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)dev->par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)dev->par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
		((int32_t)dev->par_p3 << 5)) >> 3) +
		(((int32_t)dev->par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)dev->par_p1) >> 15;
	pressure_comp = 1048576 - pres_adc;
	pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
	if (pressure_comp >= BME680_MAX_OVERFLOW_VAL)
		pressure_comp = ((pressure_comp / var1) << 1);
	else
		pressure_comp = ((pressure_comp << 1) / var1);
	var1 = ((int32_t)dev->par_p9 * (int32_t)(((pressure_comp >> 3) *
		(pressure_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(pressure_comp >> 2) *
		(int32_t)dev->par_p8) >> 13;
	var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
		(int32_t)(pressure_comp >> 8) *
		(int32_t)dev->par_p10) >> 17;

	pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
		((int32_t)dev->par_p7 << 7)) >> 4);

	return (uint32_t)pressure_comp;

}

static uint32_t calc_humidity(uint16_t hum_adc, BME680_Calib_TypeDef *dev) {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t var6;
	int32_t temp_scaled;
	int32_t calc_hum;

	temp_scaled = (((int32_t) dev->t_fine * 5) + 128) >> 8;
	var1 = (int32_t) (hum_adc - ((int32_t) ((int32_t) dev->par_h1 * 16)))
		- (((temp_scaled * (int32_t) dev->par_h3) / ((int32_t) 100)) >> 1);
	var2 = ((int32_t) dev->par_h2
		* (((temp_scaled * (int32_t) dev->par_h4) / ((int32_t) 100))
			+ (((temp_scaled * ((temp_scaled * (int32_t) dev->par_h5) / ((int32_t) 100))) >> 6)
				/ ((int32_t) 100)) + (int32_t) (1 << 14))) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t) dev->par_h6 << 7;
	var4 = ((var4) + ((temp_scaled * (int32_t) dev->par_h7) / ((int32_t) 100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

	if (calc_hum > 100000) /* Cap at 100%rH */
		calc_hum = 100000;
	else if (calc_hum < 0)
		calc_hum = 0;

	return (uint32_t) calc_hum;
}

static uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, BME680_Calib_TypeDef *dev) {
	int64_t var1;
	uint64_t var2;
	int64_t var3;
	uint32_t calc_gas_res;
	/**Look up table 1 for the possible gas range values */
	uint32_t lookupTable1[16] = { UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
		UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
		UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
		UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647) };
	/**Look up table 2 for the possible gas range values */
	uint32_t lookupTable2[16] = { UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
		UINT32_C(255744255), UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016),
		UINT32_C(8000000), UINT32_C(4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000),
		UINT32_C(250000), UINT32_C(125000) };

	var1 = (int64_t) ((1340 + (5 * (int64_t) dev->range_sw_err)) *
		((int64_t) lookupTable1[gas_range])) >> 16;
	var2 = (((int64_t) ((int64_t) gas_res_adc << 15) - (int64_t) (16777216)) + var1);
	var3 = (((int64_t) lookupTable2[gas_range] * (int64_t) var1) >> 9);
	calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);

	return calc_gas_res;
}

static uint8_t calc_heater_res(uint16_t temp, BME680_Calib_TypeDef *dev) {
	uint8_t heatr_res;
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t heatr_res_x100;

	if (temp > 400) /* Cap temperature */
		temp = 400;

	var1 = (((int32_t) dev->amb_temp * dev->par_gh3) / 1000) * 256;
	var2 = (dev->par_gh1 + 784) * (((((dev->par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
	var3 = var1 + (var2 / 2);
	var4 = (var3 / (dev->res_heat_range + 4));
	var5 = (131 * dev->res_heat_val) + 65536;
	heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
	heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

	return heatr_res;
}

void calc_raw_values(BME680_TypeDef *BME680, BME680_Calib_TypeDef *dev) {

	BME680->Temperature_Raw = Bme680_Read_Temperature(BME680);
	BME680->Pressure_Raw = Bme680_Read_Pressure(BME680);
	BME680->Humidity_Raw = Bme680_Read_Humidity(BME680);
	BME680->Gas_Raw = Bme680_Read_Gas_Data(BME680);

	BME680->Temperature_Calc = calc_temperature(BME680->Temperature_Raw, dev);
	BME680->Pressure_Calc = calc_pressure(BME680->Pressure_Raw, dev);
	BME680->Humidity_Calc = calc_humidity(BME680->Humidity_Raw, dev);
	BME680->Gas_Calc = calc_gas_resistance(BME680->Gas_Raw, BME680->Gas_Range, dev);
}

uint8_t Bme680_Init(BME680_TypeDef *BME680, BME680_Calib_TypeDef *calib, I2C_HandleTypeDef *i2c, uint8_t Address) {

	uint32_t rslt;

	BME680->bme680_i2c = i2c;
	BME680->write_addr = (Address << 1);
	BME680->Gas_heat_dur = 100;
	BME680->Gas_heat_temp = 320;
	calib->amb_temp = 25;

	rslt = Bme680_Reset(BME680);
	vTaskDelay(5 / portTICK_PERIOD_MS); // For sure it should be there?

	if (rslt == HAL_OK) {

		BME680->chipID = read_register8(BME680, BME680_CHIPID_REGISTER);

		if (BME680->chipID != BME680_CHIPID) {
			return BME680_ERROR;
		} else {
			get_calib_data(BME680, calib);
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
	Value[0] = (read_register8(BME680, BME680_TEMP_XLSB_REGISTER) & BME680_TEMP_XLSB_MASK);
	Value[1] = read_register8(BME680, BME680_TEMP_LSB_REGISTER);
	Value[2] = read_register8(BME680, BME680_TEMP_MSB_REGISTER);
	BME680->Temperature_Raw = (((Value[2] << 16) | (Value[1] << 8) | Value[0]));
	return BME680->Temperature_Raw;
}

uint32_t Bme680_Read_Pressure(BME680_TypeDef *BME680) {
	uint8_t Value[3] = { 0 };
	Value[0] = (read_register8(BME680, BME680_PRESS_XLSB_REGISTER) & BME680_PRESS_XLSB_MASK);
	Value[1] = read_register8(BME680, BME680_PRESS_LSB_REGISTER);
	Value[2] = read_register8(BME680, BME680_PRESS_MSB_REGISTER);
	BME680->Pressure_Raw = (((Value[2] << 16) | (Value[1] << 8) | Value[0]) & BME680_20BIT_MASK);
	return BME680->Pressure_Raw;
}

uint16_t Bme680_Read_Gas_Data(BME680_TypeDef *BME680) {
	uint8_t Value[2] = { 0 };
	Value[0] = (read_register8(BME680, BME680_GAS_R_LSB_REGISTER) & BME680_GAS_LSB_MASK);
	Value[1] = read_register8(BME680, BME680_GAS_R_MSB_REGISTER);
	BME680->Gas_Raw = (((Value[1] << 2) | Value[0]) & BME680_GAS_RAW_MASK);
	return BME680->Gas_Raw;
}

uint32_t Bme680_Read_Gas_Range(BME680_TypeDef *BME680) {
	return BME680->Gas_Range = (read_register8(BME680, BME680_GAS_R_LSB_REGISTER) & BME680_GAS_RANGE_MSK);
}
