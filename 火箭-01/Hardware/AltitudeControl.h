#ifndef __ALTITUDE_CONTROL_H
#define __ALTITUDE_CONTROL_H

#include <stdint.h>
#include "IMU.h"

typedef struct
{
	float altitude_m;
	float vertical_speed_mps;
	float barometer_vertical_speed_mps;
	float imu_vertical_speed_mps;
	float vertical_accel_mps2;
	float predicted_apogee_m;
	float suggested_deflection_deg;
	float height_error_cost;
	float deflection_cost;
	float deflection_change_cost;
	uint8_t estimator_valid;
	uint8_t imu_estimator_valid;
	uint8_t prediction_valid;
	uint8_t control_allowed;
} AltitudeControl_State;

void AltitudeControl_Init(void);
void AltitudeControl_Update(uint32_t now_ms,
							const IMU_Data *imu,
							uint32_t imu_sample_ms,
							float altitude_m,
							uint8_t altitude_zero_ready,
							uint8_t barometer_valid,
							uint32_t barometer_sample_ms,
							uint8_t flight_control_enabled,
							uint8_t parachute_deployed);
const AltitudeControl_State *AltitudeControl_GetState(void);

#endif
