#include <stdint.h>

#include "stm32f10x.h"
#include "DebugConfig.h"
#include "RocketConfig.h"
#include "FullFlowTestConfig.h"
#include "W25Q64.h"
#include "FlightControl.h"
#include "FlightLog.h"
#include "Rocket.h"

#if (DEBUG_FLIGHT_LOG_ENABLE != 0) && \
	(DEBUG_FLIGHT_LOG_ENABLE != 1)
#error "DEBUG_FLIGHT_LOG_ENABLE must be 0 or 1"
#endif

#if DEBUG_FLIGHT_LOG_ENABLE

#if (DEBUG_FLIGHT_LOG_FREQUENCY_HZ == 0U) || \
	(DEBUG_FLIGHT_LOG_FREQUENCY_HZ > 1000U)
#error "Flight log frequency must be in range 1..1000Hz"
#endif

#if ((1000U % DEBUG_FLIGHT_LOG_FREQUENCY_HZ) != 0U)
#error "Flight log frequency must divide 1000 exactly"
#endif

#if ((DEBUG_FLIGHT_LOG_START_ADDRESS % W25Q64_SECTOR_SIZE_BYTES) != 0U)
#error "Flight log start address must be aligned to 4KB"
#endif

#define FLIGHT_LOG_PERIOD_MS \
	(1000U / DEBUG_FLIGHT_LOG_FREQUENCY_HZ)
#define FLIGHT_LOG_MAX_RECORDS \
	((DEBUG_FLIGHT_LOG_DURATION_MS + FLIGHT_LOG_PERIOD_MS - 1U) / \
	 FLIGHT_LOG_PERIOD_MS)
#define FLIGHT_LOG_USED_BYTES \
	(FLIGHT_LOG_HEADER_SIZE_BYTES + \
	 (FLIGHT_LOG_MAX_RECORDS * FLIGHT_LOG_RECORD_SIZE_BYTES))
#define FLIGHT_LOG_ERASE_BYTES \
	(((FLIGHT_LOG_USED_BYTES + W25Q64_SECTOR_SIZE_BYTES - 1U) / \
	  W25Q64_SECTOR_SIZE_BYTES) * W25Q64_SECTOR_SIZE_BYTES)
#define FLIGHT_LOG_DATA_ADDRESS \
	(DEBUG_FLIGHT_LOG_START_ADDRESS + FLIGHT_LOG_HEADER_SIZE_BYTES)

#if (DEBUG_FLIGHT_LOG_DURATION_MS < FLIGHT_LOG_PERIOD_MS)
#error "Flight log duration is shorter than one record period"
#endif

#if ((DEBUG_FLIGHT_LOG_START_ADDRESS + FLIGHT_LOG_ERASE_BYTES) > \
	W25Q64_SIZE_BYTES)
#error "Flight log region exceeds W25Q64 capacity"
#endif

#endif

/*============================== 私有变量 ==============================*/

static FlightLog_Status s_log_status = FLIGHT_LOG_STATUS_DISABLED;
static uint32_t s_record_count = 0U;

#if DEBUG_FLIGHT_LOG_ENABLE
static uint32_t s_last_record_ms = 0U;
static uint32_t s_previous_rate_ms = 0U;
static float s_previous_roll_rate_dps = 0.0f;
static float s_previous_pitch_rate_dps = 0.0f;
static float s_previous_yaw_rate_dps = 0.0f;
static uint8_t s_first_record = 1U;
static uint8_t s_have_previous_rate = 0U;
#endif

/*============================== 私有函数 ==============================*/

#if DEBUG_FLIGHT_LOG_ENABLE
static void FlightLog_PutU16(uint8_t *buffer, uint16_t value)
{
	buffer[0] = (uint8_t)value;
	buffer[1] = (uint8_t)(value >> 8);
}

static void FlightLog_PutU32(uint8_t *buffer, uint32_t value)
{
	buffer[0] = (uint8_t)value;
	buffer[1] = (uint8_t)(value >> 8);
	buffer[2] = (uint8_t)(value >> 16);
	buffer[3] = (uint8_t)(value >> 24);
}

static int16_t FlightLog_FloatToI16(float value, float scale)
{
	float scaled;

	scaled = value * scale;
	if(scaled > 32767.0f)
	{
		return 32767;
	}
	if(scaled < -32768.0f)
	{
		return -32768;
	}

	return (int16_t)((scaled >= 0.0f) ?
					 (scaled + 0.5f) :
					 (scaled - 0.5f));
}

static int32_t FlightLog_FloatToI32(float value, float scale)
{
	float scaled;

	scaled = value * scale;
	if(scaled > 2147483520.0f)
	{
		return 2147483647;
	}
	if(scaled < -2147483520.0f)
	{
		return (-2147483647 - 1);
	}

	return (int32_t)((scaled >= 0.0f) ?
					 (scaled + 0.5f) :
					 (scaled - 0.5f));
}

static W25Q64_Result FlightLog_WriteHeader(void)
{
	uint8_t header[FLIGHT_LOG_HEADER_SIZE_BYTES];
	uint16_t index;

	for(index = 0U; index < sizeof(header); index++)
	{
		header[index] = 0U;
	}

	header[0] = 'R';
	header[1] = 'L';
	header[2] = 'O';
	header[3] = 'G';
	FlightLog_PutU16(&header[4], FLIGHT_LOG_FORMAT_VERSION);
	FlightLog_PutU16(&header[6],
					 (uint16_t)FLIGHT_LOG_HEADER_SIZE_BYTES);
	FlightLog_PutU16(&header[8],
					 (uint16_t)FLIGHT_LOG_RECORD_SIZE_BYTES);
	FlightLog_PutU16(&header[10],
					 (uint16_t)DEBUG_FLIGHT_LOG_FREQUENCY_HZ);
	FlightLog_PutU32(&header[12], DEBUG_FLIGHT_LOG_DURATION_MS);
	FlightLog_PutU32(&header[16], FLIGHT_LOG_MAX_RECORDS);
	FlightLog_PutU32(&header[20], FLIGHT_LOG_DATA_ADDRESS);
	FlightLog_PutU32(&header[24], DEBUG_FLIGHT_LOG_START_ADDRESS);
	FlightLog_PutU32(&header[28], FLIGHT_LOG_ERASE_BYTES);

	/* v9 configuration snapshot, stored once per flight. */
	header[32] = 'C';
	header[33] = 'F';
	header[34] = 'G';
	header[35] = '9';
	FlightLog_PutU32(&header[36],
		(uint32_t)FlightLog_FloatToI32(ROCKET_ALTITUDE_TARGET_M, 100.0f));
	FlightLog_PutU32(&header[40],
		(uint32_t)FlightLog_FloatToI32(ROCKET_ALTITUDE_STAGE_MASS_KG, 10000.0f));
	FlightLog_PutU32(&header[44],
		(uint32_t)FlightLog_FloatToI32(
			ROCKET_ALTITUDE_REFERENCE_DIAMETER_M, 1000000.0f));
	FlightLog_PutU32(&header[48],
		(uint32_t)FlightLog_FloatToI32(
			ROCKET_ALTITUDE_AIR_DENSITY_KG_M3, 1000000.0f));
	FlightLog_PutU32(&header[52],
		(uint32_t)FlightLog_FloatToI32(ROCKET_ALTITUDE_BASE_CD, 1000000.0f));
	FlightLog_PutU32(&header[56],
		(uint32_t)FlightLog_FloatToI32(ROCKET_ALTITUDE_FIN_DRAG_K, 1000000.0f));
	FlightLog_PutU32(&header[60],
		(uint32_t)FlightLog_FloatToI32(ROCKET_ALTITUDE_FIN_US_PER_DEG, 10000.0f));
	FlightLog_PutU16(&header[64],
		(uint16_t)FlightLog_FloatToI16(ROCKET_ALTITUDE_SERVO_LIMIT_DEG, 100.0f));
	FlightLog_PutU16(&header[66],
		(uint16_t)FlightLog_FloatToI16(ROCKET_ALTITUDE_ROLL_MIX_SIGN, 100.0f));
	FlightLog_PutU16(&header[68],
		(uint16_t)FlightLog_FloatToI16(ROCKET_ALTITUDE_PITCH_MIX_SIGN, 100.0f));
	FlightLog_PutU16(&header[70],
		(uint16_t)FlightLog_FloatToI16(ROCKET_IMU_MOUNT_ROLL_OFFSET_DEG, 100.0f));
	FlightLog_PutU16(&header[72],
		(uint16_t)FlightLog_FloatToI16(ROCKET_TARGET_ROLL_DEG, 100.0f));
	FlightLog_PutU16(&header[74],
		(uint16_t)FlightLog_FloatToI16(ROCKET_TARGET_PITCH_DEG, 100.0f));
	FlightLog_PutU16(&header[76],
		(uint16_t)FlightLog_FloatToI16(
			ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN, 100.0f));
	FlightLog_PutU32(&header[78], ROCKET_IGNITION_DELAY_MS);
	FlightLog_PutU32(&header[82], ROCKET_IGNITION_PULSE_MS);
	FlightLog_PutU32(&header[86], ROCKET_FORCE_DEPLOY_TIME_MS);
	FlightLog_PutU16(&header[90], ROCKET_IMU_PERIOD_MS);
	FlightLog_PutU16(&header[92], ROCKET_BAROMETER_PERIOD_MS);
	FlightLog_PutU16(&header[94], ROCKET_RATE_LOOP_PERIOD_MS);
	FlightLog_PutU16(&header[96], ROCKET_ANGLE_LOOP_PERIOD_MS);
	header[98] = ROCKET_FULL_FLOW_ALTITUDE_MIX_ENABLE;
	header[99] = ROCKET_ALTITUDE_SERVO_ENABLE;
	FlightLog_PutU32(&header[100],
		(uint32_t)FlightLog_FloatToI32(
			ROCKET_ALTITUDE_COST_HEIGHT, 1000000.0f));
	FlightLog_PutU32(&header[104],
		(uint32_t)FlightLog_FloatToI32(
			ROCKET_ALTITUDE_COST_DEFLECTION, 1000000000.0f));
	FlightLog_PutU32(&header[108],
		(uint32_t)FlightLog_FloatToI32(
			ROCKET_ALTITUDE_COST_DEFLECTION_CHANGE, 1000000000.0f));
	FlightLog_PutU16(&header[112],
		(uint16_t)FlightLog_FloatToI16(
			ROCKET_ALTITUDE_MAX_DEFLECTION_STEP_DEG, 100.0f));
	header[114] = ROCKET_ALTITUDE_HEIGHT_PRIORITY_ENABLE;

	return W25Q64_Write(DEBUG_FLIGHT_LOG_START_ADDRESS,
						header,
						sizeof(header));
}

static W25Q64_Result FlightLog_EraseRegion(void)
{
	W25Q64_Result result;
	uint32_t address;
	uint32_t end_address;

	end_address =
		DEBUG_FLIGHT_LOG_START_ADDRESS + FLIGHT_LOG_ERASE_BYTES;
	for(address = DEBUG_FLIGHT_LOG_START_ADDRESS;
		address < end_address;
		address += W25Q64_SECTOR_SIZE_BYTES)
	{
		result = W25Q64_EraseSector4K(address);
		if(result != W25Q64_RESULT_OK)
		{
			return result;
		}
	}

	return W25Q64_RESULT_OK;
}

static void FlightLog_BuildRecord(uint8_t record[FLIGHT_LOG_RECORD_SIZE_BYTES],
								  uint32_t now_ms,
								  const IMU_Data *imu,
								  const BMP280_Data *barometer,
								  float altitude_m,
								  uint8_t altitude_zero_ready,
								  const AltitudeControl_State *altitude_control,
								  uint32_t last_valid_imu_ms,
								  uint32_t last_valid_barometer_ms,
								  uint8_t ignition_active,
								  uint8_t parachute_deployed)
{
	IMU_Diagnostics diagnostics;
	FlightControl_Telemetry control_telemetry;
	float dt_s = 0.0f;
	float roll_accel_dps2 = 0.0f;
	float pitch_accel_dps2 = 0.0f;
	float yaw_accel_dps2 = 0.0f;
	uint8_t index;

	for(index = 0U; index < FLIGHT_LOG_RECORD_SIZE_BYTES; index++)
	{
		record[index] = 0U;
	}

	if((imu != 0) && (imu->valid != 0U))
	{
		if((s_have_previous_rate != 0U) &&
		   (now_ms != s_previous_rate_ms))
		{
			dt_s =
				(float)(uint32_t)(now_ms - s_previous_rate_ms) /
				1000.0f;
			roll_accel_dps2 =
				(imu->roll_rate_dps - s_previous_roll_rate_dps) / dt_s;
			pitch_accel_dps2 =
				(imu->pitch_rate_dps - s_previous_pitch_rate_dps) / dt_s;
			yaw_accel_dps2 =
				(imu->yaw_rate_dps - s_previous_yaw_rate_dps) / dt_s;
		}

		s_previous_rate_ms = now_ms;
		s_previous_roll_rate_dps = imu->roll_rate_dps;
		s_previous_pitch_rate_dps = imu->pitch_rate_dps;
		s_previous_yaw_rate_dps = imu->yaw_rate_dps;
		s_have_previous_rate = 1U;

		FlightLog_PutU16(&record[4],
						 (uint16_t)FlightLog_FloatToI16(
							roll_accel_dps2, 1.0f));
		FlightLog_PutU16(&record[6],
						 (uint16_t)FlightLog_FloatToI16(
							pitch_accel_dps2, 1.0f));
		FlightLog_PutU16(&record[8],
						 (uint16_t)FlightLog_FloatToI16(
							yaw_accel_dps2, 1.0f));
		FlightLog_PutU16(&record[10],
						 (uint16_t)FlightLog_FloatToI16(
							imu->accel_x_mg, 1.0f));
		FlightLog_PutU16(&record[12],
						 (uint16_t)FlightLog_FloatToI16(
							imu->accel_y_mg, 1.0f));
		FlightLog_PutU16(&record[14],
						 (uint16_t)FlightLog_FloatToI16(
							imu->accel_z_mg, 1.0f));
		FlightLog_PutU16(&record[16],
						 (uint16_t)FlightLog_FloatToI16(
							imu->roll_deg, 100.0f));
		FlightLog_PutU16(&record[18],
						 (uint16_t)FlightLog_FloatToI16(
							imu->pitch_deg, 100.0f));
		FlightLog_PutU16(&record[20],
						 (uint16_t)FlightLog_FloatToI16(
							imu->yaw_deg, 100.0f));
		record[24] = 1U;
	}
	else
	{
		s_have_previous_rate = 0U;
	}

	if((barometer != 0) && (barometer->valid != 0U))
	{
		record[25] = 1U;
		FlightLog_PutU32(&record[32], barometer->pressure_q24_8);
		FlightLog_PutU16(&record[40],
						 (uint16_t)barometer->temperature_centi_c);
		if(altitude_zero_ready != 0U)
		{
			FlightLog_PutU32(
				&record[36],
				(uint32_t)FlightLog_FloatToI32(altitude_m, 100.0f));
		}
	}

	FlightLog_PutU32(&record[0], now_ms);
	record[22] = (ignition_active != 0U) ? 1U : 0U;
	record[23] = (parachute_deployed != 0U) ? 1U : 0U;
	record[42] = (altitude_zero_ready != 0U) ? 1U : 0U;
	record[43] = FLIGHT_LOG_BAROMETER_MARKER;
	FlightLog_PutU32(&record[26], s_record_count);
	FlightLog_PutU16(&record[30], FLIGHT_LOG_RECORD_MARKER);

	IMU_GetDiagnostics(&diagnostics);
	record[44] = (uint8_t)diagnostics.status;
	record[45] = (uint8_t)diagnostics.last_fault;
	record[46] = diagnostics.consecutive_bus_errors;
	record[47] = FLIGHT_LOG_IMU_DIAGNOSTIC_MARKER;
	FlightLog_PutU16(&record[48], diagnostics.fifo_level);
	FlightLog_PutU16(&record[50], diagnostics.init_count);
	FlightLog_PutU16(&record[52], diagnostics.total_bus_errors);
	FlightLog_PutU16(&record[54], diagnostics.fifo_reset_count);

	if(altitude_control != 0)
	{
		FlightLog_PutU16(
			&record[56],
			(uint16_t)FlightLog_FloatToI16(
				altitude_control->vertical_speed_mps, 100.0f));
		FlightLog_PutU32(
			&record[58],
			(uint32_t)FlightLog_FloatToI32(
				altitude_control->predicted_apogee_m, 100.0f));
		FlightLog_PutU16(
			&record[62],
			(uint16_t)FlightLog_FloatToI16(
				altitude_control->suggested_deflection_deg, 100.0f));
		record[64] = altitude_control->estimator_valid;
		record[65] = altitude_control->prediction_valid;
		record[66] = altitude_control->control_allowed;
		FlightLog_PutU16(
			&record[68],
			(uint16_t)FlightLog_FloatToI16(
				altitude_control->barometer_vertical_speed_mps, 100.0f));
		FlightLog_PutU16(
			&record[70],
			(uint16_t)FlightLog_FloatToI16(
				altitude_control->imu_vertical_speed_mps, 100.0f));
		FlightLog_PutU16(
			&record[72],
			(uint16_t)FlightLog_FloatToI16(
				altitude_control->vertical_accel_mps2, 100.0f));
		record[74] = altitude_control->imu_estimator_valid;
	}
	record[67] = FLIGHT_LOG_ALTITUDE_CONTROL_MARKER;
	record[75] = (uint8_t)BMP280_GetLastInitResult();
	record[76] = BMP280_GetI2CAddress();
	record[77] = BMP280_IsInitialized();
	record[78] = BMP280_GetKnownAddressAckMask();
	record[79] = (uint8_t)Rocket_GetState();

	FlightControl_GetTelemetry(&control_telemetry);
	if(imu != 0)
	{
		FlightLog_PutU16(&record[80],
			(uint16_t)FlightLog_FloatToI16(imu->roll_rate_dps, 10.0f));
		FlightLog_PutU16(&record[82],
			(uint16_t)FlightLog_FloatToI16(imu->pitch_rate_dps, 10.0f));
		FlightLog_PutU16(&record[84],
			(uint16_t)FlightLog_FloatToI16(imu->yaw_rate_dps, 10.0f));
	}
	FlightLog_PutU16(&record[86],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.roll_rate_target_dps, 100.0f));
	FlightLog_PutU16(&record[88],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.pitch_rate_target_dps, 100.0f));
	FlightLog_PutU16(&record[90],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.roll_pid_output_us, 100.0f));
	FlightLog_PutU16(&record[92],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.pitch_pid_output_us, 100.0f));
	FlightLog_PutU16(&record[94],
		(uint16_t)ROCKET_ROLL_PWM_TIMER->CCR2);
	FlightLog_PutU16(&record[96],
		(uint16_t)ROCKET_PITCH_PWM_TIMER->CCR1);
	FlightLog_PutU16(&record[98],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.applied_altitude_deflection_deg, 100.0f));
	FlightLog_PutU32(&record[100], last_valid_imu_ms);
	FlightLog_PutU32(&record[104], last_valid_barometer_ms);
	record[108] = control_telemetry.running;
	record[109] = ROCKET_FULL_FLOW_ALTITUDE_MIX_ENABLE;
	record[110] = ROCKET_ALTITUDE_SERVO_ENABLE;
	record[111] = FLIGHT_LOG_CONTROL_TELEMETRY_MARKER;
	if(altitude_control != 0)
	{
		FlightLog_PutU32(&record[112],
			(uint32_t)FlightLog_FloatToI32(
				altitude_control->height_error_cost, 100.0f));
		FlightLog_PutU32(&record[116],
			(uint32_t)FlightLog_FloatToI32(
				altitude_control->deflection_cost, 1000000.0f));
		FlightLog_PutU32(&record[120],
			(uint32_t)FlightLog_FloatToI32(
				altitude_control->deflection_change_cost, 1000000.0f));
	}
	record[127] = FLIGHT_LOG_COST_TELEMETRY_MARKER;
	FlightLog_PutU16(&record[128],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.requested_altitude_deflection_deg, 100.0f));
	FlightLog_PutU16(&record[130],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.applied_altitude_deflection_deg, 100.0f));
	FlightLog_PutU16(&record[132],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.allocated_roll_pid_output_us, 100.0f));
	FlightLog_PutU16(&record[134],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.allocated_pitch_pid_output_us, 100.0f));
	FlightLog_PutU16(&record[136],
		(uint16_t)FlightLog_FloatToI16(
			control_telemetry.applied_altitude_offset_us, 10.0f));
	record[138] = control_telemetry.altitude_limit_reason;
	record[139] = ROCKET_ALTITUDE_HEIGHT_PRIORITY_ENABLE;
	record[143] = FLIGHT_LOG_HEIGHT_PRIORITY_MARKER;
}
#endif

/*============================== 公开函数 ==============================*/

FlightLog_Status FlightLog_Init(void)
{
#if DEBUG_FLIGHT_LOG_ENABLE
	W25Q64_Result result;

	s_log_status = FLIGHT_LOG_STATUS_FLASH_ERROR;
	s_record_count = 0U;
	s_last_record_ms = 0U;
	s_previous_rate_ms = 0U;
	s_first_record = 1U;
	s_have_previous_rate = 0U;

	result = W25Q64_Init();
	if(result == W25Q64_RESULT_OK)
	{
		result = FlightLog_EraseRegion();
	}
	if(result == W25Q64_RESULT_OK)
	{
		result = FlightLog_WriteHeader();
	}
	if(result == W25Q64_RESULT_OK)
	{
		s_log_status = FLIGHT_LOG_STATUS_READY;
	}
#else
	s_log_status = FLIGHT_LOG_STATUS_DISABLED;
	s_record_count = 0U;
#endif

	return s_log_status;
}

void FlightLog_Record(uint32_t now_ms,
					  const IMU_Data *imu,
					  const BMP280_Data *barometer,
					  float altitude_m,
					  uint8_t altitude_zero_ready,
					  const AltitudeControl_State *altitude_control,
					  uint32_t last_valid_imu_ms,
					  uint32_t last_valid_barometer_ms,
					  uint8_t ignition_active,
					  uint8_t parachute_deployed)
{
#if DEBUG_FLIGHT_LOG_ENABLE
	uint8_t record[FLIGHT_LOG_RECORD_SIZE_BYTES];
	W25Q64_Result result;
	uint32_t address;

	if(s_log_status != FLIGHT_LOG_STATUS_READY)
	{
		return;
	}
	if(s_record_count >= FLIGHT_LOG_MAX_RECORDS)
	{
		s_log_status = FLIGHT_LOG_STATUS_FULL;
		return;
	}
	if((s_first_record == 0U) &&
	   ((uint32_t)(now_ms - s_last_record_ms) < FLIGHT_LOG_PERIOD_MS))
	{
		return;
	}

	s_first_record = 0U;
	s_last_record_ms = now_ms;
	FlightLog_BuildRecord(record,
						  now_ms,
						  imu,
						  barometer,
						  altitude_m,
						  altitude_zero_ready,
						  altitude_control,
						  last_valid_imu_ms,
						  last_valid_barometer_ms,
						  ignition_active,
						  parachute_deployed);
	address =
		FLIGHT_LOG_DATA_ADDRESS +
		(s_record_count * FLIGHT_LOG_RECORD_SIZE_BYTES);
	result = W25Q64_Write(address, record, sizeof(record));
	if(result != W25Q64_RESULT_OK)
	{
		s_log_status = FLIGHT_LOG_STATUS_FLASH_ERROR;
		return;
	}

	s_record_count++;
	if(s_record_count >= FLIGHT_LOG_MAX_RECORDS)
	{
		s_log_status = FLIGHT_LOG_STATUS_FULL;
	}
#else
	(void)now_ms;
	(void)imu;
	(void)barometer;
	(void)altitude_m;
	(void)altitude_zero_ready;
	(void)altitude_control;
	(void)last_valid_imu_ms;
	(void)last_valid_barometer_ms;
	(void)ignition_active;
	(void)parachute_deployed;
#endif
}

FlightLog_Status FlightLog_GetStatus(void)
{
	return s_log_status;
}

uint32_t FlightLog_GetRecordCount(void)
{
	return s_record_count;
}
