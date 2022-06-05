#ifndef INC_BME680_H_
#define INC_BME680_H_

#define BME680_ADDR 0x76
#define BME680_CHIPID 0x61
#define BME680_COEFF_SIZE 41

#define BME680_CONCAT_BYTES(MSB, LSB) (((uint16_t)MSB << 8) | (uint16_t)LSB)
#define BME680_MAX_OVERFLOW_VAL (0x40000000)
// BME680 REGISTERS

#define BME680_STATUS_REGISTER 		0x73
#define BME680_RESET_REGISTER 		0xE0
#define BME680_CHIPID_REGISTER 		0xD0
#define BME680_CONFIG_REGISTER 		0x75
#define BME680_CTRL_MEAS_REGISTER 	0x74
#define BME680_CTRL_HUM_REGISTER 	0x72
#define BME680_CTRL_GAS_1_REGISTER 	0x71
#define BME680_CTRL_GAS_0_REGISTER 	0x70
#define BME680_GAS_WAIT_0_REGISTER 	0x64
#define BME680_RES_HEAT_0_REGISTER 	0x63
#define BME680_IDAC_HEAT_0_REGISTER 0x59
#define BME680_GAS_R_LSB_REGISTER 	0x2B
#define BME680_GAS_R_MSB_REGISTER 	0x2A
#define BME680_HUM_LSB_REGISTER 	0x26
#define BME680_HUM_MSB_REGISTER 	0x25
#define BME680_TEMP_XLSB_REGISTER 	0x24
#define BME680_TEMP_LSB_REGISTER 	0x23
#define BME680_TEMP_MSB_REGISTER 	0x22
#define BME680_PRESS_XLSB_REGISTER 	0x21
#define BME680_PRESS_LSB_REGISTER 	0x20
#define BME680_PRESS_MSB_REGISTER 	0x1F
#define BME680_EAS_STATUS_0_REGISTER 0x1D

/** Coefficient's address */
#define BME680_COEFF_ADDR1	(0x89)
#define BME680_COEFF_ADDR2	(0xe1)

// BME680 Control bits
#define BME680_RESET 0xB6
#define BME680_MODE_SLEEP (0x00)
#define BME680_MODE_FORCE (0x01)
#define BME680_RUN_GAS (0x01 << 4)

// SPI 3 WIRE
#define BME680_SPI_3W_ENABLE (0x01)
#define BME680_SPI_3W_DISABLE (0x00)

// Humidity oversampling
#define BME680_OSRS_H_OVR_SAMPLING_1 (0x01) // x1
#define BME680_OSRS_H_OVR_SAMPLING_2 (0x02) // x2
#define BME680_OSRS_H_OVR_SAMPLING_4 (0x03) // x4
#define BME680_OSRS_H_OVR_SAMPLING_8 (0x04) // x8
#define BME680_OSRS_H_OVR_SAMPLING_16 (0x05) // x16

// Temperature oversampling
#define BME680_OSRS_T_OVR_SAMPLING_1 (0x01 << 5) // x1
#define BME680_OSRS_T_OVR_SAMPLING_2 (0x02 << 5) // x2
#define BME680_OSRS_T_OVR_SAMPLING_4 (0x03 << 5) // x4
#define BME680_OSRS_T_OVR_SAMPLING_8 (0x04 << 5) // x8
#define BME680_OSRS_T_OVR_SAMPLING_16 (0x05 << 5) // x16

// Pressure oversampling
#define BME680_OSRS_P_OVR_SAMPLING_1 (0x01 << 2) // x1
#define BME680_OSRS_P_OVR_SAMPLING_2 (0x02 << 2) // x2
#define BME680_OSRS_P_OVR_SAMPLING_4 (0x03 << 2) // x4
#define BME680_OSRS_P_OVR_SAMPLING_8 (0x04 << 2) // x8
#define BME680_OSRS_P_OVR_SAMPLING_16 (0x05 << 2) // x16

// Filter coefficient
#define BME680_FILTER_0 (0x00 << 2)
#define BME680_FILTER_1 (0x01 << 2)
#define BME680_FILTER_3 (0x02 << 2)
#define BME680_FILTER_7 (0x03 << 2)
#define BME680_FILTER_15 (0x04 << 2)
#define BME680_FILTER_31 (0x05 << 2)
#define BME680_FILTER_63 (0x06 << 2)
#define BME680_FILTER_127 (0x07 << 2)

// Gas conf
#define BME680_HEATER_PROF_SET_POINT_0 (0x00)
#define BME680_HEATER_PROF_SET_POINT_1 (0x01)
#define BME680_HEATER_PROF_SET_POINT_2 (0x02)
#define BME680_HEATER_PROF_SET_POINT_3 (0x03)
#define BME680_HEATER_PROF_SET_POINT_4 (0x04)
#define BME680_HEATER_PROF_SET_POINT_5 (0x05)
#define BME680_HEATER_PROF_SET_POINT_6 (0x06)
#define BME680_HEATER_PROF_SET_POINT_7 (0x07)
#define BME680_HEATER_PROF_SET_POINT_8 (0x08)
#define BME680_HEATER_PROF_SET_POINT_9 (0x09)

/* Mask macros */
#define BME680_20BIT_MASK (0x00FFFFFF)
#define BME680_GAS_LSB_MASK (0xC0)
#define BME680_GAS_RAW_MASK (0x1FF)
#define BME680_TEMP_XLSB_MASK (0xF0)
#define BME680_PRESS_XLSB_MASK (0xF0)

/* Mask for pressure and temperature oversampling */
#define BME680_OSP_OST_MSK 					 	  (0xFC)

/* Mask for number of conversions */
#define BME680_NBCONV_MSK                         (0X0f)

/* Mask for IIR filter */
#define BME680_FILTER_MSK                         (0X1c)

/* Mask for ODR[3] */
#define BME680_ODR3_MSK                           (0x80)

/* Mask for ODR[2:0] */
#define BME680_ODR20_MSK                          (0xe0)

/* Mask for temperature oversampling */
#define BME680_OST_MSK                            (0Xe0)

/* Mask for pressure oversampling */
#define BME680_OSP_MSK                            (0X1c)

/* Mask for humidity oversampling */
#define BME680_OSH_MSK                            (0X07)

/* Mask for heater control */
#define BME680_HCTRL_MSK                          (0x08)

/* Mask for run gas */
#define BME680_RUN_GAS_MSK                        (0x30)

/* Mask for operation mode */
#define BME680_MODE_MSK                           (0x03)

/* Mask for res heat range */
#define BME680_RHRANGE_MSK                        (0x30)

/* Mask for range switching error */
#define BME680_RSERROR_MSK                        (0xf0)

/* Mask for new data */
#define BME680_NEW_DATA_MSK                       (0x80)

/* Mask for gas index */
#define BME680_GAS_INDEX_MSK                      (0x0f)

/* Mask for gas range */
#define BME680_GAS_RANGE_MSK                      (0x0f)

/* Mask for gas measurement valid */
#define BME680_GASM_VALID_MSK                     (0x20)

/* Mask for heater stability */
#define BME680_HEAT_STAB_MSK                      (0x10)

/* Mask for SPI memory page */
#define BME680_MEM_PAGE_MSK                       (0x10)

/* Mask for reading a register in SPI */
#define BME680_SPI_RD_MSK                         (0x80)

/* Mask for writing a register in SPI */
#define BME680_SPI_WR_MSK                         (0x7f)

/* Mask for the H1 calibration coefficient */
#define BME680_BIT_H1_DATA_MSK                    (0x0f)

/* ODR/Standby time macros */

/* Standby time of 0.59ms */
#define BME680_ODR_0_59_MS	(0x00)

/* Standby time of 62.5ms */
#define BME680_ODR_62_5_MS (0x01)

/* Standby time of 125ms */
#define BME680_ODR_125_MS  (0x02)

/* Standby time of 250ms */
#define BME680_ODR_250_MS	(0x03)

/* Standby time of 500ms */
#define BME680_ODR_500_MS	(0x04)

/* Standby time of 1s */
#define BME680_ODR_1000_MS	(0x05)

/* Standby time of 10ms */
#define BME680_ODR_10_MS	(0x06)

/* Standby time of 20ms */
#define BME680_ODR_20_MS	(0x07)

/* No standby time */
#define BME680_ODR_NONE	(0x08)

#define BME680_I2C_TIMEOUT 200

/** Ambient humidity shift value for compensation */
#define BME680_HUM_REG_SHIFT_VAL	(4)

/** Other coefficient's address */
#define BME680_ADDR_RES_HEAT_VAL_ADDR	(0x00)
#define BME680_ADDR_RES_HEAT_RANGE_ADDR	(0x02)
#define BME680_ADDR_RANGE_SW_ERR_ADDR	(0x04)
#define BME680_ADDR_SENS_CONF_START	(0x5A)
#define BME680_ADDR_GAS_CONF_START	(0x64)

/** Array Index to Field data mapping for Calibration Data*/
#define BME680_T2_LSB_REG	(1)
#define BME680_T2_MSB_REG	(2)
#define BME680_T3_REG		(3)
#define BME680_P1_LSB_REG	(5)
#define BME680_P1_MSB_REG	(6)
#define BME680_P2_LSB_REG	(7)
#define BME680_P2_MSB_REG	(8)
#define BME680_P3_REG		(9)
#define BME680_P4_LSB_REG	(11)
#define BME680_P4_MSB_REG	(12)
#define BME680_P5_LSB_REG	(13)
#define BME680_P5_MSB_REG	(14)
#define BME680_P7_REG		(15)
#define BME680_P6_REG		(16)
#define BME680_P8_LSB_REG	(19)
#define BME680_P8_MSB_REG	(20)
#define BME680_P9_LSB_REG	(21)
#define BME680_P9_MSB_REG	(22)
#define BME680_P10_REG		(23)
#define BME680_H2_MSB_REG	(25)
#define BME680_H2_LSB_REG	(26)
#define BME680_H1_LSB_REG	(26)
#define BME680_H1_MSB_REG	(27)
#define BME680_H3_REG		(28)
#define BME680_H4_REG		(29)
#define BME680_H5_REG		(30)
#define BME680_H6_REG		(31)
#define BME680_H7_REG		(32)
#define BME680_T1_LSB_REG	(33)
#define BME680_T1_MSB_REG	(34)
#define BME680_GH2_LSB_REG	(35)
#define BME680_GH2_MSB_REG	(36)
#define BME680_GH1_REG		(37)
#define BME680_GH3_REG		(38)

typedef struct {
	I2C_HandleTypeDef *bme680_i2c;
	uint8_t write_addr;
	uint8_t chipID;

	uint32_t Gas_Raw;
	uint32_t Temperature_Raw;
	uint32_t Pressure_Raw;
	uint16_t Humidity_Raw;

	uint16_t Temperature_Calc;
	uint32_t Pressure_Calc;
	uint32_t Gas_Calc;
	uint32_t Humidity_Calc;

	uint16_t Gas_heat_temp;
	uint16_t Gas_heat_dur;
	uint8_t Gas_Range;
} BME680_TypeDef;

typedef struct bme680_calib_data {
	/*! Variables to store calibrated humidity data */
	uint16_t par_h1;
	uint16_t par_h2;
	int8_t par_h3;
	int8_t par_h4;
	int8_t par_h5;
	uint8_t par_h6;
	int8_t par_h7;
	/*! Variables to store calibrated gas data */
	int8_t par_gh1;
	int16_t par_gh2;
	int8_t par_gh3;
	/*! Variables to store calibrated temperature data */
	uint16_t par_t1;
	int16_t par_t2;
	int8_t par_t3;
	/*! Variables to store calibrated pressure data */
	uint16_t par_p1;
	int16_t par_p2;
	int8_t par_p3;
	int16_t par_p4;
	int16_t par_p5;
	int8_t par_p6;
	int8_t par_p7;
	int16_t par_p8;
	int16_t par_p9;
	uint8_t par_p10;
	/*! Variable to store t_fine size */
	int32_t t_fine;
	/*! Variable to store heater resistance range */
	uint8_t res_heat_range;
	/*! Variable to store heater resistance value */
	int8_t res_heat_val;
	/*! Variable to store error range */
	int8_t range_sw_err;
	/*! Ambient temperature in Degree C */
	int8_t amb_temp;
} BME680_Calib_TypeDef;

enum {
	BME680_OK = 0,
	BME680_ERROR = 1,
};

uint8_t Bme680_Init(BME680_TypeDef *BME680, BME680_Calib_TypeDef *calib, I2C_HandleTypeDef *i2c, uint8_t Address);
uint32_t Bme680_Reset(BME680_TypeDef *BME680);
uint32_t Bme680_Set_Mode(BME680_TypeDef *BME680, uint8_t Mode);
uint32_t Bme680_Set_Conf(BME680_TypeDef *BME680, uint8_t os_t, uint8_t os_h, uint8_t os_p, uint8_t filter);
uint32_t Bme680_Run_Gas(BME680_TypeDef *BME680);
uint32_t Bme680_Set_Gas_Conf(BME680_TypeDef *BME680, uint16_t heat_dur);
uint16_t Bme680_Read_Humidity(BME680_TypeDef *BME680);
uint32_t Bme680_Read_Temperature(BME680_TypeDef *BME680);
uint32_t Bme680_Read_Pressure(BME680_TypeDef *BME680);
uint16_t Bme680_Read_Gas_Data(BME680_TypeDef *BME680);
uint32_t Bme680_Read_Gas_Range(BME680_TypeDef *BME680);
void calc_raw_values(BME680_TypeDef *BM680, BME680_Calib_TypeDef *dev);

#endif /* INC_BME680_H_ */
