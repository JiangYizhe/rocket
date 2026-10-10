#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#include <stdint.h>
#include "IMU.h"

#define FLIGHT_CONTROL_ATTITUDE_LIMIT_ROLL     0x01U
#define FLIGHT_CONTROL_ATTITUDE_LIMIT_PITCH    0x02U

typedef struct
{
	float roll_rate_target_dps;
	float pitch_rate_target_dps;
	float roll_pid_output_us;
	float pitch_pid_output_us;
	float requested_altitude_deflection_deg;
	float attitude_scaled_deflection_deg;
	float applied_altitude_deflection_deg;
	float altitude_attitude_scale;
	float roll_altitude_headroom_us;
	float pitch_altitude_headroom_us;
	float applied_altitude_offset_us;
	float allocated_roll_pid_output_us;
	float allocated_pitch_pid_output_us;
	uint8_t altitude_limit_reason;
	uint8_t running;
} FlightControl_Telemetry;

void FlightControl_Init(void);
void FlightControl_Stop(void);
void FlightControl_Update(const IMU_Data *imu,
						  uint32_t now_ms,
						  float altitude_deflection_deg);
void FlightControl_GetTelemetry(FlightControl_Telemetry *telemetry);

#endif
