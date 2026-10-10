#include <math.h>
#include <stdint.h>

#include "stm32f10x.h"
#include "Delay.h"
#include "RocketConfig.h"
#include "FullFlowTestConfig.h"
#include "lsm6dsv16x_reg.h"
#include "IMU.h"

/*============================== 私有宏 ==============================*/

#define IMU_SPI_TIMEOUT_LOOPS           100000U
#define IMU_RESET_TIMEOUT_MS            100U
#define IMU_BOOT_TIME_MS                10U
#define IMU_FIFO_WATERMARK              3U
#define IMU_FIFO_MAX_REALTIME_ENTRIES   9U
#define IMU_FIFO_WORD_RATE_HZ           360U
#define IMU_SAMPLE_STALE_TIME_MS        30U
#define IMU_BUS_ERROR_LIMIT             3U

#define IMU_RAD_TO_DEG                  57.29577951308232f
#define IMU_MDPS_TO_DPS                 0.001f
#define IMU_QUATERNION_NORM_EPSILON     0.02f

/*============================== 私有变量 ==============================*/

static stmdev_ctx_t s_imu_ctx;
static uint8_t s_imu_initialized = 0U;
static uint8_t s_spi_pins_mapped = 0U;
static uint8_t s_bus_error_count = 0U;
static IMU_Status s_imu_status = IMU_STATUS_NOT_INITIALIZED;
static IMU_FaultReason s_last_fault = IMU_FAULT_NONE;
static uint16_t s_last_fifo_level = 0U;
static uint16_t s_init_count = 0U;
static uint16_t s_total_bus_errors = 0U;
static uint16_t s_fifo_reset_count = 0U;
static uint8_t s_have_attitude = 0U;
static uint8_t s_have_gyro = 0U;
static uint8_t s_have_accel = 0U;
static uint32_t s_last_attitude_ms = 0U;
static uint32_t s_last_gyro_ms = 0U;
static uint32_t s_last_accel_ms = 0U;
static float s_roll_deg = 0.0f;
static float s_pitch_deg = 0.0f;
static float s_yaw_deg = 0.0f;
static float s_roll_rate_dps = 0.0f;
static float s_pitch_rate_dps = 0.0f;
static float s_yaw_rate_dps = 0.0f;
static float s_accel_x_mg = 0.0f;
static float s_accel_y_mg = 0.0f;
static float s_accel_z_mg = 0.0f;

/*============================== 私有函数 ==============================*/

static void IMU_CS_Low(void)
{
	GPIO_ResetBits(ROCKET_IMU_CS_GPIO, ROCKET_IMU_CS_PIN);
}

static void IMU_CS_High(void)
{
	GPIO_SetBits(ROCKET_IMU_CS_GPIO, ROCKET_IMU_CS_PIN);
}

static void IMU_SPI_Recover(SPI_TypeDef *spi)
{
	volatile uint16_t discard;

	IMU_CS_High();
	if(spi == 0)
	{
		return;
	}

	/*
	 * 超时后结束当前事务，并按DR后SR的顺序清除潜在OVR。
	 * 重新使能SPI，使下一次有界事务可重新开始。
	 */
	SPI_Cmd(spi, DISABLE);
	discard = spi->DR;
	discard = spi->SR;
	(void)discard;
	SPI_Cmd(spi, ENABLE);
}

static uint8_t IMU_SPI_WaitFlag(SPI_TypeDef *spi,
								uint16_t flag,
								FlagStatus expected)
{
	uint32_t timeout = IMU_SPI_TIMEOUT_LOOPS;

	while(SPI_I2S_GetFlagStatus(spi, flag) != expected)
	{
		if(timeout == 0U)
		{
			return 0U;
		}
		timeout--;
	}

	return 1U;
}

static uint8_t IMU_SPI_TransferByte(SPI_TypeDef *spi,
								   uint8_t tx_data,
								   uint8_t *rx_data)
{
	if((spi == 0) || (rx_data == 0))
	{
		return 0U;
	}

	if(IMU_SPI_WaitFlag(spi, SPI_I2S_FLAG_TXE, SET) == 0U)
	{
		return 0U;
	}

	SPI_I2S_SendData(spi, tx_data);
	if(IMU_SPI_WaitFlag(spi, SPI_I2S_FLAG_RXNE, SET) == 0U)
	{
		return 0U;
	}

	*rx_data = (uint8_t)SPI_I2S_ReceiveData(spi);
	return 1U;
}

static int32_t IMU_PlatformWrite(void *handle,
								 uint8_t reg,
								 const uint8_t *buffer,
								 uint16_t length)
{
	SPI_TypeDef *spi = (SPI_TypeDef *)handle;
	uint8_t command;
	uint8_t discard;
	uint16_t index;
	uint8_t success = 1U;

	if((spi == 0) || ((buffer == 0) && (length != 0U)))
	{
		return -1;
	}

	command = reg & 0x7FU;
	IMU_CS_Low();

	if(IMU_SPI_TransferByte(spi, command, &discard) == 0U)
	{
		success = 0U;
	}

	for(index = 0U; (index < length) && (success != 0U); index++)
	{
		if(IMU_SPI_TransferByte(spi, buffer[index], &discard) == 0U)
		{
			success = 0U;
		}
	}

	if((success != 0U) &&
	   (IMU_SPI_WaitFlag(spi, SPI_I2S_FLAG_BSY, RESET) == 0U))
	{
		success = 0U;
	}

	if(success != 0U)
	{
		IMU_CS_High();
	}
	else
	{
		IMU_SPI_Recover(spi);
	}
	return (success != 0U) ? 0 : -1;
}

static int32_t IMU_PlatformRead(void *handle,
								uint8_t reg,
								uint8_t *buffer,
								uint16_t length)
{
	SPI_TypeDef *spi = (SPI_TypeDef *)handle;
	uint8_t command;
	uint8_t discard;
	uint16_t index;
	uint8_t success = 1U;

	if((spi == 0) || ((buffer == 0) && (length != 0U)))
	{
		return -1;
	}

	command = reg | 0x80U;
	IMU_CS_Low();

	if(IMU_SPI_TransferByte(spi, command, &discard) == 0U)
	{
		success = 0U;
	}

	for(index = 0U; (index < length) && (success != 0U); index++)
	{
		if(IMU_SPI_TransferByte(spi, 0x00U, &buffer[index]) == 0U)
		{
			success = 0U;
		}
	}

	if((success != 0U) &&
	   (IMU_SPI_WaitFlag(spi, SPI_I2S_FLAG_BSY, RESET) == 0U))
	{
		success = 0U;
	}

	if(success != 0U)
	{
		IMU_CS_High();
	}
	else
	{
		IMU_SPI_Recover(spi);
	}
	return (success != 0U) ? 0 : -1;
}

static void IMU_SPI_Init(void)
{
	GPIO_InitTypeDef gpio_init;
	SPI_InitTypeDef spi_init;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
						  ROCKET_IMU_CS_GPIO_CLOCK |
						  ROCKET_IMU_SPI_GPIO_CLOCK |
						  ROCKET_IMU_SPI_CLOCK,
						  ENABLE);

	if(s_spi_pins_mapped == 0U)
	{
		/*
		 * 普通重映射必须先完成；SWJ配置最后写入，避免后续MAPR读改写
		 * 破坏只写的SWJ_CFG字段。JTAG关闭后仍保留SWD。重试初始化
		 * 时不可再次执行普通MAPR读改写。
		 */
		GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		s_spi_pins_mapped = 1U;
	}

	/* 先预装高电平，再切换PA11为输出，避免片选初始化毛刺。 */
	IMU_CS_High();
	GPIO_StructInit(&gpio_init);
	gpio_init.GPIO_Pin = ROCKET_IMU_CS_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ROCKET_IMU_CS_GPIO, &gpio_init);

	gpio_init.GPIO_Pin = ROCKET_IMU_SPI_SCK_PIN |
						 ROCKET_IMU_SPI_MOSI_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ROCKET_IMU_SPI_GPIO, &gpio_init);

	gpio_init.GPIO_Pin = ROCKET_IMU_SPI_MISO_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ROCKET_IMU_SPI_GPIO, &gpio_init);

	SPI_I2S_DeInit(ROCKET_IMU_SPI);
	SPI_StructInit(&spi_init);
	spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi_init.SPI_Mode = SPI_Mode_Master;
	spi_init.SPI_DataSize = SPI_DataSize_8b;
	spi_init.SPI_CPOL = SPI_CPOL_High;
	spi_init.SPI_CPHA = SPI_CPHA_2Edge;
	spi_init.SPI_NSS = SPI_NSS_Soft;
	spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
	spi_init.SPI_CRCPolynomial = 7U;
	SPI_Init(ROCKET_IMU_SPI, &spi_init);
	SPI_NSSInternalSoftwareConfig(ROCKET_IMU_SPI,
								  SPI_NSSInternalSoft_Set);
	SPI_Cmd(ROCKET_IMU_SPI, ENABLE);
}

static int16_t IMU_ReadLeI16(const uint8_t *bytes)
{
	uint16_t value;

	value = (uint16_t)bytes[0];
	value |= (uint16_t)((uint16_t)bytes[1] << 8);
	return (int16_t)value;
}

static uint32_t IMU_FifoEntryTimeMs(uint32_t now_ms,
								   uint16_t entries_after)
{
	uint32_t age_ms;

	age_ms =
		((uint32_t)entries_after * 1000U +
		 (IMU_FIFO_WORD_RATE_HZ - 1U)) /
		IMU_FIFO_WORD_RATE_HZ;
	return now_ms - age_ms;
}

static float IMU_HalfToFloat(uint16_t value)
{
	union
	{
		float number;
		uint32_t bits;
	} conversion;

	conversion.bits = lsm6dsv16x_from_f16_to_f32(value);
	return conversion.number;
}

static uint8_t IMU_SflpToQuaternion(const uint8_t raw[6], float quaternion[4])
{
	float sum_squared;
	float norm;
	uint16_t half_value;
	uint8_t index;

	for(index = 0U; index < 3U; index++)
	{
		half_value = (uint16_t)raw[index * 2U];
		half_value |= (uint16_t)((uint16_t)raw[index * 2U + 1U] << 8);
		if((half_value & 0x7C00U) == 0x7C00U)
		{
			return 0U;
		}
		quaternion[index] = IMU_HalfToFloat(half_value);
	}

	sum_squared = quaternion[0] * quaternion[0] +
				  quaternion[1] * quaternion[1] +
				  quaternion[2] * quaternion[2];

	if(sum_squared > (1.0f + IMU_QUATERNION_NORM_EPSILON))
	{
		return 0U;
	}

	if(sum_squared > 1.0f)
	{
		norm = sqrtf(sum_squared);
		quaternion[0] /= norm;
		quaternion[1] /= norm;
		quaternion[2] /= norm;
		sum_squared = 1.0f;
	}

	quaternion[3] = sqrtf(1.0f - sum_squared);
	return 1U;
}

static void IMU_QuaternionToEuler(const float quaternion[4],
								  float *roll_deg,
								  float *pitch_deg,
								  float *yaw_deg)
{
	float twice_x;
	float twice_y;
	float twice_z;
	float matrix_00;
	float matrix_10;
	float matrix_20;
	float matrix_21;
	float matrix_22;
	float pitch_sine;

	twice_x = 2.0f * quaternion[0];
	twice_y = 2.0f * quaternion[1];
	twice_z = 2.0f * quaternion[2];

	matrix_00 = 1.0f -
				(twice_y * quaternion[1] + twice_z * quaternion[2]);
	matrix_10 = twice_x * quaternion[1] + twice_z * quaternion[3];
	matrix_20 = twice_x * quaternion[2] - twice_y * quaternion[3];
	matrix_21 = twice_y * quaternion[2] + twice_x * quaternion[3];
	matrix_22 = 1.0f -
				(twice_x * quaternion[0] + twice_y * quaternion[1]);

	pitch_sine = -matrix_20;
	if(pitch_sine > 1.0f)
	{
		pitch_sine = 1.0f;
	}
	else if(pitch_sine < -1.0f)
	{
		pitch_sine = -1.0f;
	}

	/* 本工程约定X轴为roll、Y轴为pitch；实装后必须逐轴验证符号。 */
	*roll_deg = atan2f(-matrix_21, matrix_22) * IMU_RAD_TO_DEG;
	*pitch_deg = asinf(pitch_sine) * IMU_RAD_TO_DEG;
	*yaw_deg = atan2f(matrix_10, matrix_00) * IMU_RAD_TO_DEG;
}

static float IMU_WrapAngleDeg(float angle_deg)
{
	while(angle_deg > 180.0f)
	{
		angle_deg -= 360.0f;
	}
	while(angle_deg <= -180.0f)
	{
		angle_deg += 360.0f;
	}
	return angle_deg;
}

static void IMU_ClearCache(void)
{
	s_have_attitude = 0U;
	s_have_gyro = 0U;
	s_have_accel = 0U;
	s_last_attitude_ms = 0U;
	s_last_gyro_ms = 0U;
	s_last_accel_ms = 0U;
	s_roll_deg = 0.0f;
	s_pitch_deg = 0.0f;
	s_yaw_deg = 0.0f;
	s_roll_rate_dps = 0.0f;
	s_pitch_rate_dps = 0.0f;
	s_yaw_rate_dps = 0.0f;
	s_accel_x_mg = 0.0f;
	s_accel_y_mg = 0.0f;
	s_accel_z_mg = 0.0f;
}

static void IMU_RecordBusFailure(uint8_t force_reinit,
								 IMU_FaultReason reason)
{
	s_last_fault = reason;
	if(s_bus_error_count < IMU_BUS_ERROR_LIMIT)
	{
		s_bus_error_count++;
	}
	if(s_total_bus_errors < 65535U)
	{
		s_total_bus_errors++;
	}

	if((force_reinit != 0U) ||
	   (s_bus_error_count >= IMU_BUS_ERROR_LIMIT))
	{
		s_imu_initialized = 0U;
		IMU_ClearCache();
	}

	s_imu_status = IMU_STATUS_BUS_ERROR;
}

static uint8_t IMU_ResetFifo(void)
{
	if(lsm6dsv16x_fifo_mode_set(&s_imu_ctx,
								LSM6DSV16X_BYPASS_MODE) != 0)
	{
		return 0U;
	}

	IMU_ClearCache();
	if(lsm6dsv16x_fifo_mode_set(&s_imu_ctx,
								LSM6DSV16X_STREAM_MODE) != 0)
	{
		return 0U;
	}

	return 1U;
}

/*
 * 返回1表示配置匹配，0表示读回值不匹配，-1表示总线读失败。
 * 初始化只有在全部关键配置均通过读回校验后才宣告成功。
 */
static int8_t IMU_VerifyConfiguration(void)
{
	lsm6dsv16x_spi_mode_t spi_mode;
	lsm6dsv16x_ui_i2c_i3c_mode_t i2c_i3c_mode;
	lsm6dsv16x_xl_full_scale_t xl_full_scale;
	lsm6dsv16x_gy_full_scale_t gy_full_scale;
	lsm6dsv16x_data_rate_t xl_data_rate;
	lsm6dsv16x_data_rate_t gy_data_rate;
	lsm6dsv16x_fifo_xl_batch_t fifo_xl_batch;
	lsm6dsv16x_fifo_gy_batch_t fifo_gy_batch;
	lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
	lsm6dsv16x_fifo_mode_t fifo_mode;
	lsm6dsv16x_sflp_data_rate_t sflp_data_rate;
	uint8_t value;

	if(lsm6dsv16x_spi_mode_get(&s_imu_ctx, &spi_mode) != 0)
	{
		return -1;
	}
	if(spi_mode != LSM6DSV16X_SPI_4_WIRE)
	{
		return 0;
	}

	if(lsm6dsv16x_ui_i2c_i3c_mode_get(&s_imu_ctx,
									  &i2c_i3c_mode) != 0)
	{
		return -1;
	}
	if(i2c_i3c_mode != LSM6DSV16X_I2C_I3C_DISABLE)
	{
		return 0;
	}

	if(lsm6dsv16x_auto_increment_get(&s_imu_ctx, &value) != 0)
	{
		return -1;
	}
	if(value != PROPERTY_ENABLE)
	{
		return 0;
	}

	if(lsm6dsv16x_block_data_update_get(&s_imu_ctx, &value) != 0)
	{
		return -1;
	}
	if(value != PROPERTY_ENABLE)
	{
		return 0;
	}

	if(lsm6dsv16x_xl_full_scale_get(&s_imu_ctx, &xl_full_scale) != 0)
	{
		return -1;
	}
	if(xl_full_scale != LSM6DSV16X_16g)
	{
		return 0;
	}

	if(lsm6dsv16x_gy_full_scale_get(&s_imu_ctx, &gy_full_scale) != 0)
	{
		return -1;
	}
	if(gy_full_scale != LSM6DSV16X_2000dps)
	{
		return 0;
	}

	if(lsm6dsv16x_fifo_watermark_get(&s_imu_ctx, &value) != 0)
	{
		return -1;
	}
	if(value != IMU_FIFO_WATERMARK)
	{
		return 0;
	}

	if(lsm6dsv16x_fifo_xl_batch_get(&s_imu_ctx, &fifo_xl_batch) != 0)
	{
		return -1;
	}
	if(fifo_xl_batch != LSM6DSV16X_XL_BATCHED_AT_120Hz)
	{
		return 0;
	}

	if(lsm6dsv16x_fifo_gy_batch_get(&s_imu_ctx, &fifo_gy_batch) != 0)
	{
		return -1;
	}
	if(fifo_gy_batch != LSM6DSV16X_GY_BATCHED_AT_120Hz)
	{
		return 0;
	}

	if(lsm6dsv16x_fifo_sflp_batch_get(&s_imu_ctx, &fifo_sflp) != 0)
	{
		return -1;
	}
	if((fifo_sflp.game_rotation != 1U) ||
	   (fifo_sflp.gravity != 0U) ||
	   (fifo_sflp.gbias != 0U))
	{
		return 0;
	}

	if(lsm6dsv16x_xl_data_rate_get(&s_imu_ctx, &xl_data_rate) != 0)
	{
		return -1;
	}
	if(xl_data_rate != LSM6DSV16X_ODR_AT_120Hz)
	{
		return 0;
	}

	if(lsm6dsv16x_gy_data_rate_get(&s_imu_ctx, &gy_data_rate) != 0)
	{
		return -1;
	}
	if(gy_data_rate != LSM6DSV16X_ODR_AT_120Hz)
	{
		return 0;
	}

	if(lsm6dsv16x_sflp_data_rate_get(&s_imu_ctx, &sflp_data_rate) != 0)
	{
		return -1;
	}
	if(sflp_data_rate != LSM6DSV16X_SFLP_120Hz)
	{
		return 0;
	}

	if(lsm6dsv16x_sflp_game_rotation_get(&s_imu_ctx, &value) != 0)
	{
		return -1;
	}
	if(value != PROPERTY_ENABLE)
	{
		return 0;
	}

	if(lsm6dsv16x_fifo_mode_get(&s_imu_ctx, &fifo_mode) != 0)
	{
		return -1;
	}
	if(fifo_mode != LSM6DSV16X_STREAM_MODE)
	{
		return 0;
	}

	return 1;
}

static uint8_t IMU_FillOutput(IMU_Data *data, uint32_t now_ms)
{
	if((s_have_attitude == 0U) ||
	   (s_have_gyro == 0U) ||
	   (s_have_accel == 0U) ||
	   ((uint32_t)(now_ms - s_last_attitude_ms) >=
		IMU_SAMPLE_STALE_TIME_MS) ||
	   ((uint32_t)(now_ms - s_last_gyro_ms) >=
		IMU_SAMPLE_STALE_TIME_MS) ||
	   ((uint32_t)(now_ms - s_last_accel_ms) >=
		IMU_SAMPLE_STALE_TIME_MS))
	{
		data->valid = 0U;
		return 0U;
	}

	/* Board is mounted face-down: 180 degrees about the board X axis. */
	data->roll_deg =
		IMU_WrapAngleDeg(s_roll_deg + ROCKET_IMU_MOUNT_ROLL_OFFSET_DEG) *
		ROCKET_ROLL_ANGLE_SIGN;
	data->pitch_deg = s_pitch_deg * ROCKET_PITCH_ANGLE_SIGN;
	data->yaw_deg = s_yaw_deg;
	data->roll_rate_dps =
		(s_roll_rate_dps * ROCKET_IMU_MOUNT_X_SIGN -
		 ROCKET_ROLL_RATE_BIAS_DPS) *
		ROCKET_ROLL_RATE_SIGN;
	data->pitch_rate_dps =
		(s_pitch_rate_dps * ROCKET_IMU_MOUNT_Y_SIGN -
		 ROCKET_PITCH_RATE_BIAS_DPS) *
		ROCKET_PITCH_RATE_SIGN;
	data->yaw_rate_dps = s_yaw_rate_dps * ROCKET_IMU_MOUNT_Z_SIGN;
	data->accel_x_mg = s_accel_x_mg * ROCKET_IMU_MOUNT_X_SIGN;
	data->accel_y_mg = s_accel_y_mg * ROCKET_IMU_MOUNT_Y_SIGN;
	data->accel_z_mg = s_accel_z_mg * ROCKET_IMU_MOUNT_Z_SIGN;
	data->valid = 1U;
	return 1U;
}

/*============================== 公开函数 ==============================*/

uint8_t IMU_Init(void)
{
	lsm6dsv16x_reset_t reset_state;
	lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
	int8_t config_status;
	uint8_t device_id;
	uint32_t elapsed_ms;

	if(s_init_count < 65535U)
	{
		s_init_count++;
	}
	s_imu_initialized = 0U;
	s_bus_error_count = 0U;
	s_imu_status = IMU_STATUS_NOT_INITIALIZED;
	IMU_ClearCache();
	IMU_SPI_Init();

	s_imu_ctx.write_reg = IMU_PlatformWrite;
	s_imu_ctx.read_reg = IMU_PlatformRead;
	s_imu_ctx.mdelay = Delay_ms;
	s_imu_ctx.handle = (void *)ROCKET_IMU_SPI;

	Delay_ms(IMU_BOOT_TIME_MS);

	if(lsm6dsv16x_mem_bank_set(&s_imu_ctx,
							   LSM6DSV16X_MAIN_MEM_BANK) != 0)
	{
		s_last_fault = IMU_FAULT_INIT_BUS_ERROR;
		s_imu_status = IMU_STATUS_BUS_ERROR;
		return 0U;
	}

	if(lsm6dsv16x_device_id_get(&s_imu_ctx, &device_id) != 0)
	{
		s_last_fault = IMU_FAULT_INIT_BUS_ERROR;
		s_imu_status = IMU_STATUS_BUS_ERROR;
		return 0U;
	}
	if(device_id != LSM6DSV16X_ID)
	{
		s_last_fault = IMU_FAULT_DEVICE_ID;
		s_imu_status = IMU_STATUS_DEVICE_ID_ERROR;
		return 0U;
	}

	if(lsm6dsv16x_reset_set(&s_imu_ctx,
						   LSM6DSV16X_RESTORE_CTRL_REGS) != 0)
	{
		s_last_fault = IMU_FAULT_INIT_BUS_ERROR;
		s_imu_status = IMU_STATUS_BUS_ERROR;
		return 0U;
	}

	reset_state = LSM6DSV16X_RESTORE_CTRL_REGS;
	for(elapsed_ms = 0U; elapsed_ms < IMU_RESET_TIMEOUT_MS; elapsed_ms++)
	{
		if(lsm6dsv16x_reset_get(&s_imu_ctx, &reset_state) != 0)
		{
			s_last_fault = IMU_FAULT_INIT_BUS_ERROR;
			s_imu_status = IMU_STATUS_BUS_ERROR;
			return 0U;
		}
		if(reset_state == LSM6DSV16X_READY)
		{
			break;
		}
		Delay_ms(1U);
	}
	if(reset_state != LSM6DSV16X_READY)
	{
		s_last_fault = IMU_FAULT_RESET_TIMEOUT;
		s_imu_status = IMU_STATUS_RESET_TIMEOUT;
		return 0U;
	}

	if((lsm6dsv16x_spi_mode_set(&s_imu_ctx,
								LSM6DSV16X_SPI_4_WIRE) != 0) ||
	   (lsm6dsv16x_ui_i2c_i3c_mode_set(
			&s_imu_ctx,
			LSM6DSV16X_I2C_I3C_DISABLE) != 0) ||
	   (lsm6dsv16x_auto_increment_set(
			&s_imu_ctx,
			PROPERTY_ENABLE) != 0) ||
	   (lsm6dsv16x_block_data_update_set(
			&s_imu_ctx,
			PROPERTY_ENABLE) != 0) ||
	   (lsm6dsv16x_xl_full_scale_set(
			&s_imu_ctx,
			LSM6DSV16X_16g) != 0) ||
	   (lsm6dsv16x_gy_full_scale_set(
			&s_imu_ctx,
			LSM6DSV16X_2000dps) != 0) ||
	   (lsm6dsv16x_fifo_mode_set(
			&s_imu_ctx,
			LSM6DSV16X_BYPASS_MODE) != 0) ||
	   (lsm6dsv16x_fifo_watermark_set(
			&s_imu_ctx,
			IMU_FIFO_WATERMARK) != 0) ||
	   (lsm6dsv16x_fifo_xl_batch_set(
			&s_imu_ctx,
			LSM6DSV16X_XL_BATCHED_AT_120Hz) != 0) ||
	   (lsm6dsv16x_fifo_gy_batch_set(
			&s_imu_ctx,
			LSM6DSV16X_GY_BATCHED_AT_120Hz) != 0))
	{
		s_last_fault = IMU_FAULT_INIT_BUS_ERROR;
		s_imu_status = IMU_STATUS_BUS_ERROR;
		return 0U;
	}

	fifo_sflp.game_rotation = 1U;
	fifo_sflp.gravity = 0U;
	fifo_sflp.gbias = 0U;
	if(lsm6dsv16x_fifo_sflp_batch_set(&s_imu_ctx, fifo_sflp) != 0)
	{
		s_last_fault = IMU_FAULT_INIT_BUS_ERROR;
		s_imu_status = IMU_STATUS_BUS_ERROR;
		return 0U;
	}

	if((lsm6dsv16x_xl_data_rate_set(
			&s_imu_ctx,
			LSM6DSV16X_ODR_AT_120Hz) != 0) ||
	   (lsm6dsv16x_gy_data_rate_set(
			&s_imu_ctx,
			LSM6DSV16X_ODR_AT_120Hz) != 0) ||
	   (lsm6dsv16x_sflp_data_rate_set(
			&s_imu_ctx,
			LSM6DSV16X_SFLP_120Hz) != 0) ||
	   (lsm6dsv16x_sflp_game_rotation_set(
			&s_imu_ctx,
			PROPERTY_ENABLE) != 0) ||
	   (lsm6dsv16x_fifo_mode_set(
			&s_imu_ctx,
			LSM6DSV16X_STREAM_MODE) != 0))
	{
		s_last_fault = IMU_FAULT_INIT_BUS_ERROR;
		s_imu_status = IMU_STATUS_BUS_ERROR;
		return 0U;
	}

	config_status = IMU_VerifyConfiguration();
	if(config_status < 0)
	{
		s_last_fault = IMU_FAULT_INIT_BUS_ERROR;
		s_imu_status = IMU_STATUS_BUS_ERROR;
		return 0U;
	}
	if(config_status == 0)
	{
		s_last_fault = IMU_FAULT_CONFIG;
		s_imu_status = IMU_STATUS_CONFIG_ERROR;
		return 0U;
	}

	s_imu_initialized = 1U;
	s_bus_error_count = 0U;
	s_imu_status = IMU_STATUS_READY;
	return 1U;
}

uint8_t IMU_Read(IMU_Data *data, uint32_t now_ms)
{
	lsm6dsv16x_fifo_status_t fifo_status;
	lsm6dsv16x_fifo_out_raw_t fifo_data;
	float quaternion[4];
	uint16_t entries_to_read;
	uint16_t index;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint8_t output_valid;

	if(data == 0)
	{
		return 0U;
	}

	data->roll_deg = 0.0f;
	data->pitch_deg = 0.0f;
	data->yaw_deg = 0.0f;
	data->roll_rate_dps = 0.0f;
	data->pitch_rate_dps = 0.0f;
	data->yaw_rate_dps = 0.0f;
	data->accel_x_mg = 0.0f;
	data->accel_y_mg = 0.0f;
	data->accel_z_mg = 0.0f;
	data->valid = 0U;
	if(s_imu_initialized == 0U)
	{
		return 0U;
	}

	if(lsm6dsv16x_fifo_status_get(&s_imu_ctx, &fifo_status) != 0)
	{
		IMU_RecordBusFailure(0U, IMU_FAULT_FIFO_STATUS_READ);
		return 0U;
	}
	s_last_fifo_level = fifo_status.fifo_level;

	if((fifo_status.fifo_ovr != 0U) ||
	   (fifo_status.fifo_full != 0U) ||
	   (fifo_status.fifo_level > IMU_FIFO_MAX_REALTIME_ENTRIES))
	{
		s_last_fault = IMU_FAULT_FIFO_OVERFLOW;
		if(s_fifo_reset_count < 65535U)
		{
			s_fifo_reset_count++;
		}
		if(IMU_ResetFifo() == 0U)
		{
			IMU_RecordBusFailure(1U, IMU_FAULT_FIFO_RESET_FAILED);
			return 0U;
		}

		s_bus_error_count = 0U;
		s_imu_status = IMU_STATUS_FIFO_RESET;
		return 0U;
	}

	entries_to_read = fifo_status.fifo_level;
	for(index = 0U; index < entries_to_read; index++)
	{
		if(lsm6dsv16x_fifo_out_raw_get(&s_imu_ctx, &fifo_data) != 0)
		{
			IMU_RecordBusFailure(0U, IMU_FAULT_FIFO_ENTRY_READ);
			return 0U;
		}

		switch(fifo_data.tag)
		{
			case LSM6DSV16X_XL_NC_TAG:
				accel_x = IMU_ReadLeI16(&fifo_data.data[0]);
				accel_y = IMU_ReadLeI16(&fifo_data.data[2]);
				accel_z = IMU_ReadLeI16(&fifo_data.data[4]);
				s_accel_x_mg = lsm6dsv16x_from_fs16_to_mg(accel_x);
				s_accel_y_mg = lsm6dsv16x_from_fs16_to_mg(accel_y);
				s_accel_z_mg = lsm6dsv16x_from_fs16_to_mg(accel_z);
				s_have_accel = 1U;
				s_last_accel_ms =
					IMU_FifoEntryTimeMs(
						now_ms,
						(uint16_t)(entries_to_read - index - 1U));
				break;

			case LSM6DSV16X_GY_NC_TAG:
				gyro_x = IMU_ReadLeI16(&fifo_data.data[0]);
				gyro_y = IMU_ReadLeI16(&fifo_data.data[2]);
				gyro_z = IMU_ReadLeI16(&fifo_data.data[4]);
				s_roll_rate_dps =
					lsm6dsv16x_from_fs2000_to_mdps(gyro_x) *
					IMU_MDPS_TO_DPS;
				s_pitch_rate_dps =
					lsm6dsv16x_from_fs2000_to_mdps(gyro_y) *
					IMU_MDPS_TO_DPS;
				s_yaw_rate_dps =
					lsm6dsv16x_from_fs2000_to_mdps(gyro_z) *
					IMU_MDPS_TO_DPS;
				s_have_gyro = 1U;
				s_last_gyro_ms =
					IMU_FifoEntryTimeMs(
						now_ms,
						(uint16_t)(entries_to_read - index - 1U));
				break;

			case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
				if(IMU_SflpToQuaternion(fifo_data.data, quaternion) != 0U)
				{
					IMU_QuaternionToEuler(quaternion,
										 &s_roll_deg,
										 &s_pitch_deg,
										 &s_yaw_deg);
					s_have_attitude = 1U;
					s_last_attitude_ms =
						IMU_FifoEntryTimeMs(
							now_ms,
							(uint16_t)(entries_to_read - index - 1U));
				}
				break;

			default:
				break;
		}
	}

	/*
	 * 120Hz传感器与100Hz任务存在相位差，短暂无新条目时复用缓存；
	 * 任一数据流超过30ms未更新则返回失败，避免任务抖动时按调用
	 * 次数误判新鲜度。
	 */
	output_valid = IMU_FillOutput(data, now_ms);
	s_bus_error_count = 0U;
	s_imu_status = (output_valid != 0U) ?
				   IMU_STATUS_READY :
				   IMU_STATUS_DATA_STALE;
	return output_valid;
}

uint8_t IMU_IsInitialized(void)
{
	return s_imu_initialized;
}

IMU_Status IMU_GetStatus(void)
{
	return s_imu_status;
}

void IMU_GetDiagnostics(IMU_Diagnostics *diagnostics)
{
	if(diagnostics == 0)
	{
		return;
	}

	diagnostics->status = s_imu_status;
	diagnostics->last_fault = s_last_fault;
	diagnostics->consecutive_bus_errors = s_bus_error_count;
	diagnostics->fifo_level = s_last_fifo_level;
	diagnostics->init_count = s_init_count;
	diagnostics->total_bus_errors = s_total_bus_errors;
	diagnostics->fifo_reset_count = s_fifo_reset_count;
}
