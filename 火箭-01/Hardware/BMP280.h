#ifndef __BMP280_H
#define __BMP280_H

#include <stdint.h>

#define BMP280_CHIP_ID                     0x58U
#define BMP280_I2C_ADDRESS_LOW             0x76U
#define BMP280_I2C_ADDRESS_HIGH            0x77U

typedef enum
{
	BMP280_RESULT_OK = 0,
	BMP280_RESULT_INVALID_ARGUMENT,
	BMP280_RESULT_NOT_INITIALIZED,
	BMP280_RESULT_BUS_ERROR,
	BMP280_RESULT_DEVICE_ID_MISMATCH,
	BMP280_RESULT_RESET_TIMEOUT,
	BMP280_RESULT_CALIBRATION_ERROR,
	BMP280_RESULT_CONFIG_ERROR,
	BMP280_RESULT_DATA_INVALID
} BMP280_Result;

typedef struct
{
	int32_t temperature_centi_c;
	/* Bosch Q24.8格式，除以256得到Pa，适合高精度高度计算。 */
	uint32_t pressure_q24_8;
	uint32_t pressure_pa;
	int32_t raw_temperature;
	int32_t raw_pressure;
	uint8_t valid;
} BMP280_Data;

BMP280_Result BMP280_Init(void);
BMP280_Result BMP280_ReadChipId(uint8_t *chip_id);
BMP280_Result BMP280_Read(BMP280_Data *data);
uint8_t BMP280_IsInitialized(void);
uint8_t BMP280_GetI2CAddress(void);
BMP280_Result BMP280_GetLastInitResult(void);
uint8_t BMP280_GetKnownAddressAckMask(void);

#endif
