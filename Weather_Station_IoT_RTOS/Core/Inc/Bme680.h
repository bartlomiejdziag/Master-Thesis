#ifndef INC_BME680_H_
#define INC_BME680_H_

#define BME680_ADDR 0x76

// BME680 REGISTERS

#define BME680_STATUS_REGISTER 		0x73
#define BME680_RESET_REGISTER 		0xE0
#define BME680_CHIPID_REGISTER 		0xD0
#define BME680_CONFIG_REGISTER 		0x75
#define BME680_CTRL_MEAS_REGISTER 	0x74
#define BME680_CTRL_HUM_REGISTER 	0x72
#define BME680_CTRL_GAS_1_REGISTER 	0x71
#define BME680_CTRL_GAS_0_REGISTER 	0x70
#define BME680_GAS_WAIT_0_REGISTER 	0x6D
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


#endif /* INC_BME680_H_ */
