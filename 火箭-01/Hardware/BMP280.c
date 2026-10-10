#include <stdint.h>

#include "stm32f10x.h"
#include "Delay.h"
#include "RocketConfig.h"
#include "BMP280.h"

/*
 * Pressure module uses the H7 software-I2C connector configured in
 * RocketConfig.h: H7 SCL=PB6, SDA=PB7.
 */

/*============================== 私有宏 ==============================*/

#define BMP280_REG_CALIB_START             0x88U
#define BMP280_REG_CHIP_ID                 0xD0U
#define BMP280_REG_RESET                   0xE0U
#define BMP280_REG_STATUS                  0xF3U
#define BMP280_REG_CTRL_MEAS               0xF4U
#define BMP280_REG_CONFIG                  0xF5U
#define BMP280_REG_PRESS_MSB               0xF7U

#define BMP280_RESET_COMMAND               0xB6U
#define BMP280_STATUS_IM_UPDATE            0x01U
#define BMP280_RAW_SKIPPED                 0x00080000L

#define BMP280_CALIBRATION_LENGTH          24U
#define BMP280_DATA_LENGTH                 6U
#define BMP280_I2C_TIMEOUT_LOOPS           100000UL
#define BMP280_RESET_TIMEOUT_MS            100U
#define BMP280_POWER_ON_DELAY_MS           3U
#define BMP280_USE_SOFTWARE_I2C            1U
#define BMP280_SOFT_I2C_HALF_PERIOD_US     5U
#define BMP280_SOFT_I2C_STRETCH_LOOPS      1000U

/*
 * High-accuracy altitude profile: temperature x2, pressure x16,
 * IIR coefficient 4, normal mode and 0.5ms standby. Pressure updates
 * at approximately 26Hz; the moderate filter avoids excessive flight lag.
 */
#define BMP280_CONFIG_VALUE                0x08U
#define BMP280_CTRL_MEAS_VALUE             0x57U

#define BMP280_I2C_ERROR_MASK              (I2C_SR1_BERR | \
											I2C_SR1_ARLO | \
											I2C_SR1_AF | \
											I2C_SR1_OVR)

/*============================== 私有类型 ==============================*/

typedef struct
{
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
} BMP280_Calibration;

/*============================== 私有变量 ==============================*/

static BMP280_Calibration s_calibration;
static uint8_t s_i2c_address = 0U;
static uint8_t s_initialized = 0U;
static BMP280_Result s_last_init_result = BMP280_RESULT_NOT_INITIALIZED;
static uint8_t s_known_address_ack_mask = 0U;

/*============================== 私有函数 ==============================*/

#if BMP280_USE_SOFTWARE_I2C
static void BMP280_SoftI2C_Delay(void)
{
	Delay_us(BMP280_SOFT_I2C_HALF_PERIOD_US);
}

static void BMP280_SoftI2C_SdaHigh(void)
{
	GPIO_SetBits(ROCKET_PRESSURE_I2C_GPIO,
				 ROCKET_PRESSURE_I2C_SDA_PIN);
}

static void BMP280_SoftI2C_SdaLow(void)
{
	GPIO_ResetBits(ROCKET_PRESSURE_I2C_GPIO,
				   ROCKET_PRESSURE_I2C_SDA_PIN);
}

static void BMP280_SoftI2C_SclLow(void)
{
	GPIO_ResetBits(ROCKET_PRESSURE_I2C_GPIO,
				   ROCKET_PRESSURE_I2C_SCL_PIN);
}

static uint8_t BMP280_SoftI2C_SclHigh(void)
{
	uint16_t timeout = BMP280_SOFT_I2C_STRETCH_LOOPS;

	GPIO_SetBits(ROCKET_PRESSURE_I2C_GPIO,
				 ROCKET_PRESSURE_I2C_SCL_PIN);
	while(GPIO_ReadInputDataBit(ROCKET_PRESSURE_I2C_GPIO,
									 ROCKET_PRESSURE_I2C_SCL_PIN) == Bit_RESET)
	{
		if(timeout == 0U)
		{
			return 0U;
		}
		timeout--;
	}
	return 1U;
}

static uint8_t BMP280_SoftI2C_Start(void)
{
	BMP280_SoftI2C_SdaHigh();
	if(BMP280_SoftI2C_SclHigh() == 0U)
	{
		return 0U;
	}
	BMP280_SoftI2C_Delay();
	BMP280_SoftI2C_SdaLow();
	BMP280_SoftI2C_Delay();
	BMP280_SoftI2C_SclLow();
	return 1U;
}

static void BMP280_SoftI2C_Stop(void)
{
	BMP280_SoftI2C_SdaLow();
	BMP280_SoftI2C_Delay();
	(void)BMP280_SoftI2C_SclHigh();
	BMP280_SoftI2C_Delay();
	BMP280_SoftI2C_SdaHigh();
	BMP280_SoftI2C_Delay();
}

static uint8_t BMP280_SoftI2C_WriteByte(uint8_t value)
{
	uint8_t bit_index;
	uint8_t acknowledged;

	for(bit_index = 0U; bit_index < 8U; bit_index++)
	{
		if((value & 0x80U) != 0U)
		{
			BMP280_SoftI2C_SdaHigh();
		}
		else
		{
			BMP280_SoftI2C_SdaLow();
		}
		BMP280_SoftI2C_Delay();
		if(BMP280_SoftI2C_SclHigh() == 0U)
		{
			return 0U;
		}
		BMP280_SoftI2C_Delay();
		BMP280_SoftI2C_SclLow();
		value <<= 1;
	}

	BMP280_SoftI2C_SdaHigh();
	BMP280_SoftI2C_Delay();
	if(BMP280_SoftI2C_SclHigh() == 0U)
	{
		return 0U;
	}
	BMP280_SoftI2C_Delay();
	acknowledged =
		(GPIO_ReadInputDataBit(ROCKET_PRESSURE_I2C_GPIO,
							   ROCKET_PRESSURE_I2C_SDA_PIN) == Bit_RESET) ?
		1U : 0U;
	BMP280_SoftI2C_SclLow();
	return acknowledged;
}

static uint8_t BMP280_SoftI2C_ReadByte(uint8_t acknowledge)
{
	uint8_t bit_index;
	uint8_t value = 0U;

	BMP280_SoftI2C_SdaHigh();
	for(bit_index = 0U; bit_index < 8U; bit_index++)
	{
		value <<= 1;
		BMP280_SoftI2C_Delay();
		if(BMP280_SoftI2C_SclHigh() == 0U)
		{
			return value;
		}
		if(GPIO_ReadInputDataBit(ROCKET_PRESSURE_I2C_GPIO,
								 ROCKET_PRESSURE_I2C_SDA_PIN) != Bit_RESET)
		{
			value |= 0x01U;
		}
		BMP280_SoftI2C_Delay();
		BMP280_SoftI2C_SclLow();
	}

	if(acknowledge != 0U)
	{
		BMP280_SoftI2C_SdaLow();
	}
	else
	{
		BMP280_SoftI2C_SdaHigh();
	}
	BMP280_SoftI2C_Delay();
	(void)BMP280_SoftI2C_SclHigh();
	BMP280_SoftI2C_Delay();
	BMP280_SoftI2C_SclLow();
	BMP280_SoftI2C_SdaHigh();
	return value;
}

static void BMP280_SoftI2C_RecoverBus(void)
{
	uint8_t pulse;

	BMP280_SoftI2C_SdaHigh();
	for(pulse = 0U; pulse < 9U; pulse++)
	{
		BMP280_SoftI2C_SclLow();
		BMP280_SoftI2C_Delay();
		(void)BMP280_SoftI2C_SclHigh();
		BMP280_SoftI2C_Delay();
	}
	BMP280_SoftI2C_Stop();
}
#endif

static uint16_t BMP280_ReadLeU16(const uint8_t *data)
{
	return (uint16_t)((uint16_t)data[0] |
					  ((uint16_t)data[1] << 8));
}

static int16_t BMP280_ReadLeI16(const uint8_t *data)
{
	return (int16_t)BMP280_ReadLeU16(data);
}

#if !BMP280_USE_SOFTWARE_I2C
static void BMP280_I2C_ClearAddress(void)
{
	volatile uint32_t discard;

	discard = ROCKET_PRESSURE_I2C->SR1;
	discard = ROCKET_PRESSURE_I2C->SR2;
	(void)discard;
}

static void BMP280_I2C_ClearErrors(void)
{
	ROCKET_PRESSURE_I2C->SR1 &=
		(uint16_t)(~BMP280_I2C_ERROR_MASK);
}

static uint8_t BMP280_I2C_WaitSr1(uint16_t mask)
{
	uint32_t timeout = BMP280_I2C_TIMEOUT_LOOPS;

	while((ROCKET_PRESSURE_I2C->SR1 & mask) == 0U)
	{
		if((ROCKET_PRESSURE_I2C->SR1 & BMP280_I2C_ERROR_MASK) != 0U)
		{
			return 0U;
		}
		if(timeout == 0U)
		{
			return 0U;
		}
		timeout--;
	}

	return 1U;
}

static uint8_t BMP280_I2C_WaitBusIdle(void)
{
	uint32_t timeout = BMP280_I2C_TIMEOUT_LOOPS;

	while((ROCKET_PRESSURE_I2C->SR2 & I2C_SR2_BUSY) != 0U)
	{
		if(timeout == 0U)
		{
			return 0U;
		}
		timeout--;
	}

	return 1U;
}

static void BMP280_I2C_RestoreReceiveMode(void)
{
	I2C_NACKPositionConfig(ROCKET_PRESSURE_I2C,
						   I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(ROCKET_PRESSURE_I2C, ENABLE);
}

static void BMP280_I2C_Abort(void)
{
	if((ROCKET_PRESSURE_I2C->SR2 & I2C_SR2_MSL) != 0U)
	{
		I2C_GenerateSTOP(ROCKET_PRESSURE_I2C, ENABLE);
	}

	BMP280_I2C_ClearErrors();
	I2C_Cmd(ROCKET_PRESSURE_I2C, DISABLE);
	I2C_Cmd(ROCKET_PRESSURE_I2C, ENABLE);
	BMP280_I2C_RestoreReceiveMode();
}

static uint8_t BMP280_I2C_StartAddress(uint8_t address,
									  uint8_t direction)
{
	I2C_GenerateSTART(ROCKET_PRESSURE_I2C, ENABLE);
	if(BMP280_I2C_WaitSr1(I2C_SR1_SB) == 0U)
	{
		return 0U;
	}

	I2C_Send7bitAddress(ROCKET_PRESSURE_I2C,
					   (uint8_t)(address << 1),
					   direction);
	if(BMP280_I2C_WaitSr1(I2C_SR1_ADDR) == 0U)
	{
		return 0U;
	}

	return 1U;
}
#endif

static BMP280_Result BMP280_I2C_WriteRegister(uint8_t address,
											 uint8_t reg,
											 uint8_t value)
{
#if BMP280_USE_SOFTWARE_I2C
	if((BMP280_SoftI2C_Start() == 0U) ||
	   (BMP280_SoftI2C_WriteByte((uint8_t)(address << 1)) == 0U) ||
	   (BMP280_SoftI2C_WriteByte(reg) == 0U) ||
	   (BMP280_SoftI2C_WriteByte(value) == 0U))
	{
		BMP280_SoftI2C_Stop();
		return BMP280_RESULT_BUS_ERROR;
	}
	BMP280_SoftI2C_Stop();
	return BMP280_RESULT_OK;
#else
	if(BMP280_I2C_WaitBusIdle() == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}

	if(BMP280_I2C_StartAddress(address,
							   I2C_Direction_Transmitter) == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}
	BMP280_I2C_ClearAddress();

	if(BMP280_I2C_WaitSr1(I2C_SR1_TXE) == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}
	I2C_SendData(ROCKET_PRESSURE_I2C, reg);
	if(BMP280_I2C_WaitSr1(I2C_SR1_BTF) == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}

	I2C_SendData(ROCKET_PRESSURE_I2C, value);
	if(BMP280_I2C_WaitSr1(I2C_SR1_BTF) == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}

	I2C_GenerateSTOP(ROCKET_PRESSURE_I2C, ENABLE);
	return BMP280_RESULT_OK;
#endif
}

static BMP280_Result BMP280_I2C_ReadRegisters(uint8_t address,
											 uint8_t reg,
											 uint8_t *data,
											 uint16_t length)
{
	uint16_t index;
#if !BMP280_USE_SOFTWARE_I2C
	uint16_t remaining;
#endif

	if((data == 0) || (length == 0U))
	{
		return BMP280_RESULT_INVALID_ARGUMENT;
	}
#if BMP280_USE_SOFTWARE_I2C
	if((BMP280_SoftI2C_Start() == 0U) ||
	   (BMP280_SoftI2C_WriteByte((uint8_t)(address << 1)) == 0U) ||
	   (BMP280_SoftI2C_WriteByte(reg) == 0U) ||
	   (BMP280_SoftI2C_Start() == 0U) ||
	   (BMP280_SoftI2C_WriteByte((uint8_t)((address << 1) | 0x01U)) == 0U))
	{
		BMP280_SoftI2C_Stop();
		return BMP280_RESULT_BUS_ERROR;
	}

	for(index = 0U; index < length; index++)
	{
		data[index] = BMP280_SoftI2C_ReadByte(
			(index + 1U < length) ? 1U : 0U);
	}
	BMP280_SoftI2C_Stop();
	return BMP280_RESULT_OK;
#else
	if(BMP280_I2C_WaitBusIdle() == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}

	if(BMP280_I2C_StartAddress(address,
							   I2C_Direction_Transmitter) == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}
	BMP280_I2C_ClearAddress();
	if(BMP280_I2C_WaitSr1(I2C_SR1_TXE) == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}
	I2C_SendData(ROCKET_PRESSURE_I2C, reg);
	if(BMP280_I2C_WaitSr1(I2C_SR1_BTF) == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}

	if(BMP280_I2C_StartAddress(address,
							   I2C_Direction_Receiver) == 0U)
	{
		BMP280_I2C_Abort();
		return BMP280_RESULT_BUS_ERROR;
	}

	index = 0U;
	remaining = length;
	if(remaining == 1U)
	{
		I2C_AcknowledgeConfig(ROCKET_PRESSURE_I2C, DISABLE);
		BMP280_I2C_ClearAddress();
		I2C_GenerateSTOP(ROCKET_PRESSURE_I2C, ENABLE);
		if(BMP280_I2C_WaitSr1(I2C_SR1_RXNE) == 0U)
		{
			BMP280_I2C_Abort();
			return BMP280_RESULT_BUS_ERROR;
		}
		data[index] = (uint8_t)I2C_ReceiveData(ROCKET_PRESSURE_I2C);
	}
	else if(remaining == 2U)
	{
		I2C_NACKPositionConfig(ROCKET_PRESSURE_I2C,
							   I2C_NACKPosition_Next);
		I2C_AcknowledgeConfig(ROCKET_PRESSURE_I2C, DISABLE);
		BMP280_I2C_ClearAddress();
		if(BMP280_I2C_WaitSr1(I2C_SR1_BTF) == 0U)
		{
			BMP280_I2C_Abort();
			return BMP280_RESULT_BUS_ERROR;
		}
		I2C_GenerateSTOP(ROCKET_PRESSURE_I2C, ENABLE);
		data[index++] =
			(uint8_t)I2C_ReceiveData(ROCKET_PRESSURE_I2C);
		data[index] =
			(uint8_t)I2C_ReceiveData(ROCKET_PRESSURE_I2C);
	}
	else
	{
		I2C_AcknowledgeConfig(ROCKET_PRESSURE_I2C, ENABLE);
		BMP280_I2C_ClearAddress();
		while(remaining > 3U)
		{
			if(BMP280_I2C_WaitSr1(I2C_SR1_BTF) == 0U)
			{
				BMP280_I2C_Abort();
				return BMP280_RESULT_BUS_ERROR;
			}
			data[index++] =
				(uint8_t)I2C_ReceiveData(ROCKET_PRESSURE_I2C);
			remaining--;
		}

		if(BMP280_I2C_WaitSr1(I2C_SR1_BTF) == 0U)
		{
			BMP280_I2C_Abort();
			return BMP280_RESULT_BUS_ERROR;
		}
		I2C_AcknowledgeConfig(ROCKET_PRESSURE_I2C, DISABLE);
		data[index++] =
			(uint8_t)I2C_ReceiveData(ROCKET_PRESSURE_I2C);

		if(BMP280_I2C_WaitSr1(I2C_SR1_BTF) == 0U)
		{
			BMP280_I2C_Abort();
			return BMP280_RESULT_BUS_ERROR;
		}
		I2C_GenerateSTOP(ROCKET_PRESSURE_I2C, ENABLE);
		data[index++] =
			(uint8_t)I2C_ReceiveData(ROCKET_PRESSURE_I2C);
		data[index] =
			(uint8_t)I2C_ReceiveData(ROCKET_PRESSURE_I2C);
	}

	BMP280_I2C_RestoreReceiveMode();
	return BMP280_RESULT_OK;
#endif
}

static void BMP280_I2C_Init(void)
{
	GPIO_InitTypeDef gpio_init;
#if !BMP280_USE_SOFTWARE_I2C
	I2C_InitTypeDef i2c_init;
#endif

	RCC_APB2PeriphClockCmd(ROCKET_PRESSURE_I2C_GPIO_CLOCK, ENABLE);

#if BMP280_USE_SOFTWARE_I2C
	RCC_APB1PeriphClockCmd(ROCKET_PRESSURE_I2C_CLOCK, ENABLE);
	I2C_Cmd(ROCKET_PRESSURE_I2C, DISABLE);
	GPIO_SetBits(ROCKET_PRESSURE_I2C_GPIO,
				 ROCKET_PRESSURE_I2C_SCL_PIN |
				 ROCKET_PRESSURE_I2C_SDA_PIN);
	GPIO_StructInit(&gpio_init);
	gpio_init.GPIO_Pin = ROCKET_PRESSURE_I2C_SCL_PIN |
						 ROCKET_PRESSURE_I2C_SDA_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ROCKET_PRESSURE_I2C_GPIO, &gpio_init);
	BMP280_SoftI2C_RecoverBus();
#else
	RCC_APB1PeriphClockCmd(ROCKET_PRESSURE_I2C_CLOCK, ENABLE);

	GPIO_StructInit(&gpio_init);
	gpio_init.GPIO_Pin = ROCKET_PRESSURE_I2C_SCL_PIN |
						 ROCKET_PRESSURE_I2C_SDA_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ROCKET_PRESSURE_I2C_GPIO, &gpio_init);

	I2C_DeInit(ROCKET_PRESSURE_I2C);
	I2C_StructInit(&i2c_init);
	i2c_init.I2C_Mode = I2C_Mode_I2C;
	i2c_init.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_init.I2C_OwnAddress1 = 0U;
	i2c_init.I2C_Ack = I2C_Ack_Enable;
	i2c_init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c_init.I2C_ClockSpeed = ROCKET_PRESSURE_I2C_SPEED_HZ;
	I2C_Init(ROCKET_PRESSURE_I2C, &i2c_init);
	I2C_Cmd(ROCKET_PRESSURE_I2C, ENABLE);
	BMP280_I2C_RestoreReceiveMode();
#endif
}

static uint8_t BMP280_I2C_ProbeAddress(uint8_t address)
{
	BMP280_I2C_Init();
	Delay_ms(1U);
#if BMP280_USE_SOFTWARE_I2C
	if((BMP280_SoftI2C_Start() == 0U) ||
	   (BMP280_SoftI2C_WriteByte((uint8_t)(address << 1)) == 0U))
	{
		BMP280_SoftI2C_Stop();
		return 0U;
	}
	BMP280_SoftI2C_Stop();
	return 1U;
#else
	if(BMP280_I2C_WaitBusIdle() == 0U)
	{
		BMP280_I2C_Abort();
		return 0U;
	}
	if(BMP280_I2C_StartAddress(address,
								   I2C_Direction_Transmitter) == 0U)
	{
		BMP280_I2C_Abort();
		return 0U;
	}

	BMP280_I2C_ClearAddress();
	I2C_GenerateSTOP(ROCKET_PRESSURE_I2C, ENABLE);
	return 1U;
#endif
}

static void BMP280_ScanKnownAddresses(void)
{
	s_known_address_ack_mask = 0U;
	if(BMP280_I2C_ProbeAddress(0x38U) != 0U)
	{
		s_known_address_ack_mask |= 0x01U;
	}
	if(BMP280_I2C_ProbeAddress(BMP280_I2C_ADDRESS_LOW) != 0U)
	{
		s_known_address_ack_mask |= 0x02U;
	}
	if(BMP280_I2C_ProbeAddress(BMP280_I2C_ADDRESS_HIGH) != 0U)
	{
		s_known_address_ack_mask |= 0x04U;
	}
}

static BMP280_Result BMP280_DetectAddress(void)
{
	BMP280_Result low_result;
	BMP280_Result high_result;
	uint8_t chip_id;

	low_result = BMP280_I2C_ReadRegisters(BMP280_I2C_ADDRESS_LOW,
										 BMP280_REG_CHIP_ID,
										 &chip_id,
										 1U);
	if((low_result == BMP280_RESULT_OK) &&
	   (chip_id == BMP280_CHIP_ID))
	{
		s_i2c_address = BMP280_I2C_ADDRESS_LOW;
		return BMP280_RESULT_OK;
	}

	/* Fully recover I2C1 after the expected NACK at 0x76. */
	BMP280_I2C_Init();
	Delay_ms(1U);

	high_result = BMP280_I2C_ReadRegisters(BMP280_I2C_ADDRESS_HIGH,
										  BMP280_REG_CHIP_ID,
										  &chip_id,
										  1U);
	if((high_result == BMP280_RESULT_OK) &&
	   (chip_id == BMP280_CHIP_ID))
	{
		s_i2c_address = BMP280_I2C_ADDRESS_HIGH;
		return BMP280_RESULT_OK;
	}

	if((low_result != BMP280_RESULT_OK) &&
	   (high_result != BMP280_RESULT_OK))
	{
		return BMP280_RESULT_BUS_ERROR;
	}

	return BMP280_RESULT_DEVICE_ID_MISMATCH;
}

static BMP280_Result BMP280_WaitNvmReady(void)
{
	BMP280_Result result;
	uint8_t status;
	uint32_t elapsed_ms;

	for(elapsed_ms = 0U;
		elapsed_ms < BMP280_RESET_TIMEOUT_MS;
		elapsed_ms++)
	{
		result = BMP280_I2C_ReadRegisters(s_i2c_address,
										 BMP280_REG_STATUS,
										 &status,
										 1U);
		if(result != BMP280_RESULT_OK)
		{
			return result;
		}
		if((status & BMP280_STATUS_IM_UPDATE) == 0U)
		{
			return BMP280_RESULT_OK;
		}
		Delay_ms(1U);
	}

	return BMP280_RESULT_RESET_TIMEOUT;
}

static BMP280_Result BMP280_ReadCalibration(void)
{
	BMP280_Result result;
	uint8_t data[BMP280_CALIBRATION_LENGTH];

	result = BMP280_I2C_ReadRegisters(s_i2c_address,
									 BMP280_REG_CALIB_START,
									 data,
									 BMP280_CALIBRATION_LENGTH);
	if(result != BMP280_RESULT_OK)
	{
		return result;
	}

	s_calibration.dig_t1 = BMP280_ReadLeU16(&data[0]);
	s_calibration.dig_t2 = BMP280_ReadLeI16(&data[2]);
	s_calibration.dig_t3 = BMP280_ReadLeI16(&data[4]);
	s_calibration.dig_p1 = BMP280_ReadLeU16(&data[6]);
	s_calibration.dig_p2 = BMP280_ReadLeI16(&data[8]);
	s_calibration.dig_p3 = BMP280_ReadLeI16(&data[10]);
	s_calibration.dig_p4 = BMP280_ReadLeI16(&data[12]);
	s_calibration.dig_p5 = BMP280_ReadLeI16(&data[14]);
	s_calibration.dig_p6 = BMP280_ReadLeI16(&data[16]);
	s_calibration.dig_p7 = BMP280_ReadLeI16(&data[18]);
	s_calibration.dig_p8 = BMP280_ReadLeI16(&data[20]);
	s_calibration.dig_p9 = BMP280_ReadLeI16(&data[22]);

	if((s_calibration.dig_t1 == 0U) ||
	   (s_calibration.dig_t1 == 0xFFFFU) ||
	   (s_calibration.dig_p1 == 0U) ||
	   (s_calibration.dig_p1 == 0xFFFFU))
	{
		return BMP280_RESULT_CALIBRATION_ERROR;
	}

	return BMP280_RESULT_OK;
}

static int32_t BMP280_CompensateTemperature(int32_t adc_temperature,
											int32_t *t_fine)
{
	int32_t var1;
	int32_t var2;

	var1 =
		((((adc_temperature >> 3) -
		   ((int32_t)s_calibration.dig_t1 << 1))) *
		 (int32_t)s_calibration.dig_t2) >> 11;
	var2 =
		((((((adc_temperature >> 4) -
			 (int32_t)s_calibration.dig_t1) *
			((adc_temperature >> 4) -
			 (int32_t)s_calibration.dig_t1)) >> 12) *
		  (int32_t)s_calibration.dig_t3) >> 14);
	*t_fine = var1 + var2;
	return (*t_fine * 5 + 128) >> 8;
}

static uint32_t BMP280_CompensatePressure(int32_t adc_pressure,
										 int32_t t_fine)
{
	int64_t var1;
	int64_t var2;
	int64_t pressure;

	var1 = (int64_t)t_fine - 128000;
	var2 = var1 * var1 * (int64_t)s_calibration.dig_p6;
	var2 += (var1 * (int64_t)s_calibration.dig_p5) << 17;
	var2 += ((int64_t)s_calibration.dig_p4) << 35;
	var1 =
		((var1 * var1 * (int64_t)s_calibration.dig_p3) >> 8) +
		((var1 * (int64_t)s_calibration.dig_p2) << 12);
	var1 =
		(((((int64_t)1 << 47) + var1) *
		  (int64_t)s_calibration.dig_p1) >> 33);
	if(var1 == 0)
	{
		return 0U;
	}

	pressure = 1048576 - adc_pressure;
	pressure = (((pressure << 31) - var2) * 3125) / var1;
	var1 =
		((int64_t)s_calibration.dig_p9 *
		 (pressure >> 13) * (pressure >> 13)) >> 25;
	var2 =
		((int64_t)s_calibration.dig_p8 * pressure) >> 19;
	pressure =
		((pressure + var1 + var2) >> 8) +
		((int64_t)s_calibration.dig_p7 << 4);

	return (uint32_t)pressure;
}

static BMP280_Result BMP280_Configure(void)
{
	BMP280_Result result;
	uint8_t value;

	/* config在normal模式下可能被忽略，因此先保持sleep再写入。 */
	result = BMP280_I2C_WriteRegister(s_i2c_address,
									 BMP280_REG_CTRL_MEAS,
									 0x00U);
	if(result != BMP280_RESULT_OK)
	{
		return result;
	}
	result = BMP280_I2C_WriteRegister(s_i2c_address,
									 BMP280_REG_CONFIG,
									 BMP280_CONFIG_VALUE);
	if(result != BMP280_RESULT_OK)
	{
		return result;
	}
	result = BMP280_I2C_WriteRegister(s_i2c_address,
									 BMP280_REG_CTRL_MEAS,
									 BMP280_CTRL_MEAS_VALUE);
	if(result != BMP280_RESULT_OK)
	{
		return result;
	}

	result = BMP280_I2C_ReadRegisters(s_i2c_address,
									 BMP280_REG_CONFIG,
									 &value,
									 1U);
	if((result != BMP280_RESULT_OK) ||
	   ((value & 0xFDU) != BMP280_CONFIG_VALUE))
	{
		return (result == BMP280_RESULT_OK) ?
			   BMP280_RESULT_CONFIG_ERROR : result;
	}
	result = BMP280_I2C_ReadRegisters(s_i2c_address,
									 BMP280_REG_CTRL_MEAS,
									 &value,
									 1U);
	if((result != BMP280_RESULT_OK) ||
	   (value != BMP280_CTRL_MEAS_VALUE))
	{
		return (result == BMP280_RESULT_OK) ?
			   BMP280_RESULT_CONFIG_ERROR : result;
	}

	return BMP280_RESULT_OK;
}

static BMP280_Result BMP280_ResetDevice(void)
{
	BMP280_Result result;

	result = BMP280_I2C_WriteRegister(s_i2c_address,
									 BMP280_REG_RESET,
									 BMP280_RESET_COMMAND);
	if(result != BMP280_RESULT_OK)
	{
		return result;
	}
	Delay_ms(2U);
	return BMP280_WaitNvmReady();
}

/*============================== 公开函数 ==============================*/

BMP280_Result BMP280_Init(void)
{
	BMP280_Result result;

	s_initialized = 0U;
	s_i2c_address = 0U;
	s_last_init_result = BMP280_RESULT_NOT_INITIALIZED;
	BMP280_I2C_Init();
	Delay_ms(BMP280_POWER_ON_DELAY_MS);
	BMP280_ScanKnownAddresses();
	BMP280_I2C_Init();
	Delay_ms(1U);

	result = BMP280_DetectAddress();
	if(result != BMP280_RESULT_OK)
	{
		s_last_init_result = result;
		return result;
	}
	result = BMP280_ResetDevice();
	if(result != BMP280_RESULT_OK)
	{
		s_last_init_result = result;
		return result;
	}
	result = BMP280_ReadCalibration();
	if(result != BMP280_RESULT_OK)
	{
		s_last_init_result = result;
		return result;
	}
	result = BMP280_Configure();
	if(result != BMP280_RESULT_OK)
	{
		s_last_init_result = result;
		return result;
	}

	s_initialized = 1U;
	s_last_init_result = BMP280_RESULT_OK;
	return BMP280_RESULT_OK;
}

BMP280_Result BMP280_ReadChipId(uint8_t *chip_id)
{
	if(chip_id == 0)
	{
		return BMP280_RESULT_INVALID_ARGUMENT;
	}
	if(s_i2c_address == 0U)
	{
		return BMP280_RESULT_NOT_INITIALIZED;
	}

	return BMP280_I2C_ReadRegisters(s_i2c_address,
									BMP280_REG_CHIP_ID,
									chip_id,
									1U);
}

BMP280_Result BMP280_Read(BMP280_Data *data)
{
	BMP280_Result result;
	uint8_t raw[BMP280_DATA_LENGTH];
	int32_t t_fine;
	uint32_t pressure_q24_8;

	if(data == 0)
	{
		return BMP280_RESULT_INVALID_ARGUMENT;
	}

	data->temperature_centi_c = 0;
	data->pressure_q24_8 = 0U;
	data->pressure_pa = 0U;
	data->raw_temperature = 0;
	data->raw_pressure = 0;
	data->valid = 0U;
	if(s_initialized == 0U)
	{
		return BMP280_RESULT_NOT_INITIALIZED;
	}

	/*
	 * 数据手册要求0xF7..0xFC单次突发读取，以确保压力和温度来自
	 * 同一测量周期。
	 */
	result = BMP280_I2C_ReadRegisters(s_i2c_address,
									 BMP280_REG_PRESS_MSB,
									 raw,
									 BMP280_DATA_LENGTH);
	if(result != BMP280_RESULT_OK)
	{
		if(result == BMP280_RESULT_BUS_ERROR)
		{
			s_initialized = 0U;
		}
		return result;
	}

	data->raw_pressure =
		((int32_t)raw[0] << 12) |
		((int32_t)raw[1] << 4) |
		((int32_t)raw[2] >> 4);
	data->raw_temperature =
		((int32_t)raw[3] << 12) |
		((int32_t)raw[4] << 4) |
		((int32_t)raw[5] >> 4);
	if((data->raw_pressure == BMP280_RAW_SKIPPED) ||
	   (data->raw_temperature == BMP280_RAW_SKIPPED))
	{
		return BMP280_RESULT_DATA_INVALID;
	}

	data->temperature_centi_c =
		BMP280_CompensateTemperature(data->raw_temperature, &t_fine);
	pressure_q24_8 =
		BMP280_CompensatePressure(data->raw_pressure, t_fine);
	if(pressure_q24_8 == 0U)
	{
		return BMP280_RESULT_DATA_INVALID;
	}
	data->pressure_q24_8 = pressure_q24_8;
	data->pressure_pa = (pressure_q24_8 + 128U) >> 8;
	data->valid = 1U;
	return BMP280_RESULT_OK;
}

uint8_t BMP280_IsInitialized(void)
{
	return s_initialized;
}

uint8_t BMP280_GetI2CAddress(void)
{
	return s_i2c_address;
}

BMP280_Result BMP280_GetLastInitResult(void)
{
	return s_last_init_result;
}

uint8_t BMP280_GetKnownAddressAckMask(void)
{
	return s_known_address_ack_mask;
}
