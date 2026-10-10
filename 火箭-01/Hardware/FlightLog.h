#ifndef __FLIGHT_LOG_H
#define __FLIGHT_LOG_H

#include <stdint.h>
#include "IMU.h"
#include "BMP280.h"
#include "AltitudeControl.h"

#define FLIGHT_LOG_FORMAT_VERSION           9U
#define FLIGHT_LOG_HEADER_SIZE_BYTES        256UL
#define FLIGHT_LOG_RECORD_SIZE_BYTES        144UL
#define FLIGHT_LOG_RECORD_MARKER            0xA55AU
#define FLIGHT_LOG_BAROMETER_MARKER         0xB2U
#define FLIGHT_LOG_IMU_DIAGNOSTIC_MARKER    0xD3U
#define FLIGHT_LOG_ALTITUDE_CONTROL_MARKER  0xA4U
#define FLIGHT_LOG_CONTROL_TELEMETRY_MARKER 0xC6U
#define FLIGHT_LOG_COST_TELEMETRY_MARKER    0xC7U
#define FLIGHT_LOG_ALLOCATOR_MARKER         0xC8U
#define FLIGHT_LOG_HEIGHT_PRIORITY_MARKER   0xC9U

/*
 * v9 records are 144 bytes, little-endian. Bytes 0..127 retain the complete
 * v5 layout. Bytes 80..111 add angular rates, rate targets, PID outputs,
 * actual timer PWM values, applied altitude deflection, sensor timestamps,
 * controller state and the 0xC6 extension marker. Bytes 112..123 add the
 * three selected altitude optimization cost terms; byte 127 is 0xC7.
 * Bytes 128..139 add the height-priority allocator telemetry; byte 143
 * is the 0xC9 extension marker.
 */

typedef enum
{
	FLIGHT_LOG_STATUS_DISABLED = 0,
	FLIGHT_LOG_STATUS_READY,
	FLIGHT_LOG_STATUS_FULL,
	FLIGHT_LOG_STATUS_FLASH_ERROR
} FlightLog_Status;

FlightLog_Status FlightLog_Init(void);
void FlightLog_Record(uint32_t now_ms,
					  const IMU_Data *imu,
					  const BMP280_Data *barometer,
					  float altitude_m,
					  uint8_t altitude_zero_ready,
					  const AltitudeControl_State *altitude_control,
					  uint32_t last_valid_imu_ms,
					  uint32_t last_valid_barometer_ms,
					  uint8_t ignition_active,
					  uint8_t parachute_deployed);
FlightLog_Status FlightLog_GetStatus(void);
uint32_t FlightLog_GetRecordCount(void);

#endif
