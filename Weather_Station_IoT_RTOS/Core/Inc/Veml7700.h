/*
 * Veml7700.h
 *
 *  Created on: Apr 25, 2022
 *      Author: dzion
 */

#ifndef INC_VEML7700_H_
#define INC_VEML7700_H_

#define VEML7700_ADDR 0x10
#define VEML7700_RESOLUTION 0.1152  // Check documentation for more details

// VEML REGISTERS
#define VEML7700_CONF 0x00U
#define VEML7700_HIGH_TH_WINDOWS 0x01U
#define VEML7700_LOW_TH_WINDOWS 0x02U
#define VEML7700_PWR_SAVE 0x03U
#define VEML7700_ALS_OUTPUT_DATA 0x04U
#define VEML7700_WHITE_OUTPUT_DATA 0x05U
#define VEML7700_INT 0x06U

#define VEML7700_I2C_TIMEOUT 200

/* CONFIGURATION REGISTER */

// ALS_GAIN
#define REG_ALS_CONF_GAIN_1		(0x00 << 11) // x1 (default)
#define REG_ALS_CONF_GAIN_2		(0x01 << 11) // x2
#define REG_ALS_CONF_GAIN_1_8	(0x02 << 11) // x(1/8)
#define REG_ALS_CONF_GAIN_1_4	(0x03 << 11) // x(1/4)

// ALS integration times (ms)

#define REG_ALS_CONF_IT_25 		(0x0C << 6) // 25ms
#define REG_ALS_CONF_IT_50 		(0x08 << 6) // 50ms
#define REG_ALS_CONF_IT_100 	(0x00 << 6) // 100ms
#define REG_ALS_CONF_IT_200 	(0x01 << 6) // 200ms
#define REG_ALS_CONF_IT_400 	(0x02 << 6) // 400ms
#define REG_ALS_CONF_IT_800 	(0x03 << 6) // 800ms

// ALS integration times - all bits

#define REG_ALS_CONF_IT_CLEAR   (0x0f << 6)

// ALS persistent protect number

#define REG_ALS_CONF_PERS_1     (0x00 << 4)
#define REG_ALS_CONF_PERS_2     (0x01 << 4)
#define REG_ALS_CONF_PERS_4     (0x02 << 4)
#define REG_ALS_CONF_PERS_8     (0x03 << 4)

// ALS interrupt enable

#define REG_ALS_CONF_IT_ENABLE  (0x01 << 1)

// ALS shutdown setting

#define REG_ALS_CONF_SHUTDOWN   0x01

/* POWER SAVING REGISTER */

#define REG_POWER_SAVING_PSM_1  (0x00 << 1)
#define REG_POWER_SAVING_PSM_2  (0x01 << 1)
#define REG_POWER_SAVING_PSM_3  (0x02 << 1)
#define REG_POWER_SAVING_PSM_4  (0x03 << 1)
#define REG_POWER_SAVING_ENABLE  0x01


typedef struct {
	I2C_HandleTypeDef *veml7700_i2c;
	uint8_t read_addr;
	uint8_t write_addr;
} VEML7700_TypeDef;

enum {
	VEML7700_OK = 0, VEML7700_ERROR = 1,
};

// Functions declaration
uint32_t Veml7700_Init(VEML7700_TypeDef *VEML7700, I2C_HandleTypeDef *i2c,
		uint8_t Address);
uint32_t Veml7700_Power_On(VEML7700_TypeDef *VEML7700);
uint32_t Veml7700_Shutdown(VEML7700_TypeDef *VEML7700);
uint32_t Veml7700_Set_Als_Integration_Time(VEML7700_TypeDef *VEML7700, uint16_t it);
uint16_t VEML7700_Get_Als_Integration_Time(VEML7700_TypeDef *VEML7700);
uint32_t VEML7700_Set_Als_Gain(VEML7700_TypeDef *VEML7700, uint16_t gain);
uint16_t VEML7700_get_als_gain(VEML7700_TypeDef *VEML7700);
uint16_t VEML7700_read_als(VEML7700_TypeDef *VEML7700);
uint16_t VEML7700_read_white(VEML7700_TypeDef *VEML7700);
uint32_t VEML7700_Set_PSM(VEML7700_TypeDef *VEML7700, uint16_t psm_mode);
#endif /* INC_VEML7700_H_ */
