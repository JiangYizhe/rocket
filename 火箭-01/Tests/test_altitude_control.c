#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "RocketConfig.h"
#include "AltitudeControl.h"

static unsigned int s_checks = 0U;
static unsigned int s_failures = 0U;

#define CHECK(condition) \
	do \
	{ \
		s_checks++; \
		if(!(condition)) \
		{ \
			printf("FAIL line %d: %s\n", __LINE__, #condition); \
			s_failures++; \
		} \
	} while(0)

static IMU_Data Test_MakeImu(void)
{
	IMU_Data imu = {0};

	imu.accel_z_mg = 5000.0f;
	imu.valid = 1U;
	return imu;
}

static void Test_UpdateSamples(uint32_t now_ms,
							   IMU_Data *imu,
							   uint32_t imu_sample_ms,
							   float altitude_m,
							   uint32_t barometer_sample_ms,
							   uint8_t altitude_zero_ready,
							   uint8_t flight_control_enabled,
							   uint8_t parachute_deployed)
{
	AltitudeControl_Update(now_ms,
						   imu,
						   imu_sample_ms,
						   altitude_m,
						   altitude_zero_ready,
						   1U,
						   barometer_sample_ms,
						   flight_control_enabled,
						   parachute_deployed);
}

static void Test_Update(uint32_t now_ms,
						IMU_Data *imu,
						float altitude_m,
						uint8_t altitude_zero_ready,
						uint8_t flight_control_enabled,
						uint8_t parachute_deployed)
{
	Test_UpdateSamples(now_ms,
					   imu,
					   now_ms,
					   altitude_m,
					   now_ms,
					   altitude_zero_ready,
					   flight_control_enabled,
					   parachute_deployed);
}

static void Test_ControlGateAndPersistentFusion(void)
{
	IMU_Data imu = Test_MakeImu();
	const AltitudeControl_State *state;
	float first_prediction_m;
	uint32_t start_ms = ROCKET_ALTITUDE_CONTROL_START_MS;
	uint32_t index;

	imu.accel_z_mg = 10000.0f;
	AltitudeControl_Init();
	Test_Update(start_ms, &imu, 0.0f, 1U, 0U, 0U);
	for(index = 1U; index < 4U; index++)
	{
		Test_UpdateSamples(start_ms + index * 10U,
			&imu, start_ms + index * 10U,
			0.0f, start_ms, 1U, 0U, 0U);
	}
	Test_UpdateSamples(start_ms + 40U, &imu, start_ms + 40U,
		0.4f, start_ms + 40U, 1U, 0U, 0U);
	state = AltitudeControl_GetState();
	CHECK(state->estimator_valid != 0U);
	CHECK(state->control_allowed == 0U);
	CHECK(state->prediction_valid == 0U);

	for(index = 5U; index < 8U; index++)
	{
		Test_UpdateSamples(start_ms + index * 10U,
			&imu, start_ms + index * 10U,
			0.4f, start_ms + 40U, 1U, 0U, 0U);
	}
	Test_UpdateSamples(start_ms + 80U, &imu, start_ms + 80U,
		0.8f, start_ms + 80U, 1U, 0U, 0U);
	Test_UpdateSamples(start_ms + 90U, &imu, start_ms + 90U,
		0.8f, start_ms + 80U, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK(state->control_allowed != 0U);
	CHECK(state->prediction_valid != 0U);
	CHECK(fabsf(state->suggested_deflection_deg - 3.0f) < 0.0001f);
	CHECK(state->predicted_apogee_m > state->altitude_m);
	first_prediction_m = state->predicted_apogee_m;

	/* The command is intentionally retained between 40 ms decisions. */
	Test_UpdateSamples(start_ms + 100U, &imu, start_ms + 100U,
		0.8f, start_ms + 80U, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK(fabsf(state->predicted_apogee_m - first_prediction_m) < 0.0001f);

	Test_UpdateSamples(start_ms + 110U, &imu, start_ms + 110U,
		0.8f, start_ms + 80U, 1U, 1U, 0U);
	Test_UpdateSamples(start_ms + 120U, &imu, start_ms + 120U,
		1.2f, start_ms + 120U, 1U, 1U, 0U);
	Test_UpdateSamples(start_ms + 130U, &imu, start_ms + 130U,
		1.2f, start_ms + 120U, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK(fabsf(state->vertical_speed_mps -
				  state->imu_vertical_speed_mps) > 0.001f);
	CHECK(fabsf(state->predicted_apogee_m - first_prediction_m) > 0.01f);
	CHECK(state->suggested_deflection_deg >= 0.0f);
	CHECK(state->suggested_deflection_deg <=
		ROCKET_ALTITUDE_SERVO_LIMIT_DEG);
	CHECK(fabsf(fmodf(state->suggested_deflection_deg, 3.0f)) < 0.0001f);
}

static void Test_InvalidInputClearsCommand(void)
{
	IMU_Data imu = Test_MakeImu();
	const AltitudeControl_State *state;
	uint32_t start_ms = ROCKET_ALTITUDE_CONTROL_START_MS;

	AltitudeControl_Init();
	Test_Update(start_ms, &imu, 0.0f, 1U, 1U, 0U);
	Test_Update(start_ms + 100U, &imu, 1.0f, 1U, 1U, 0U);
	imu.valid = 0U;
	Test_Update(start_ms + 110U, &imu, 1.0f, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK(state->estimator_valid == 0U);
	CHECK(state->control_allowed == 0U);
	CHECK(state->prediction_valid == 0U);
	CHECK(state->suggested_deflection_deg == 0.0f);
}

static void Test_DeflectionRampsAndReportsCosts(void)
{
	IMU_Data imu = Test_MakeImu();
	const AltitudeControl_State *state;
	uint32_t start_ms = ROCKET_ALTITUDE_CONTROL_START_MS;
	uint32_t barometer_sample_ms;
	float altitude_m;
	float elapsed_s;
	uint32_t index;

	imu.accel_z_mg = 16000.0f;
	AltitudeControl_Init();
	Test_Update(start_ms, &imu, 70.0f, 1U, 0U, 0U);
	barometer_sample_ms = start_ms;
	altitude_m = 70.0f;
	for(index = 1U; index <= 30U; index++)
	{
		if((index % 4U) == 0U)
		{
			barometer_sample_ms = start_ms + index * 10U;
			elapsed_s = (float)(index * 10U) / 1000.0f;
			altitude_m = 70.0f +
				0.5f * 147.09975f * elapsed_s * elapsed_s;
		}
		Test_UpdateSamples(start_ms + index * 10U,
			&imu, start_ms + index * 10U,
			altitude_m, barometer_sample_ms, 1U, 0U, 0U);
	}
	Test_UpdateSamples(start_ms + 300U, &imu, start_ms + 300U,
		altitude_m, barometer_sample_ms, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK(state->prediction_valid != 0U);
	CHECK(fabsf(state->suggested_deflection_deg - 3.0f) < 0.0001f);
	CHECK(state->height_error_cost > 0.0f);
	CHECK(state->deflection_cost > 0.0f);
	CHECK(state->deflection_change_cost > 0.0f);

	for(index = 31U; index <= 34U; index++)
	{
		if((index % 4U) == 0U)
		{
			barometer_sample_ms = start_ms + index * 10U;
			elapsed_s = (float)(index * 10U) / 1000.0f;
			altitude_m = 70.0f +
				0.5f * 147.09975f * elapsed_s * elapsed_s;
		}
		Test_UpdateSamples(start_ms + index * 10U,
			&imu, start_ms + index * 10U,
			altitude_m, barometer_sample_ms, 1U, 1U, 0U);
	}
	state = AltitudeControl_GetState();
	CHECK(fabsf(state->suggested_deflection_deg - 6.0f) < 0.0001f);

	for(index = 35U; index <= 74U; index++)
	{
		if((index % 4U) == 0U)
		{
			barometer_sample_ms = start_ms + index * 10U;
			elapsed_s = (float)(index * 10U) / 1000.0f;
			altitude_m = 70.0f +
				0.5f * 147.09975f * elapsed_s * elapsed_s;
		}
		Test_UpdateSamples(start_ms + index * 10U,
			&imu, start_ms + index * 10U,
			altitude_m, barometer_sample_ms, 1U, 1U, 0U);
	}
	state = AltitudeControl_GetState();
	CHECK(fabsf(state->suggested_deflection_deg - 30.0f) < 0.0001f);
}

static void Test_NewSamplesAndBarometerAnomalyGate(void)
{
	IMU_Data imu = Test_MakeImu();
	const AltitudeControl_State *state;
	float imu_speed_mps;
	float baro_speed_mps;
	float held_deflection_deg;
	uint32_t start_ms = ROCKET_ALTITUDE_CONTROL_START_MS;

	imu.accel_z_mg = 10000.0f;
	AltitudeControl_Init();
	Test_Update(start_ms, &imu, 10.0f, 1U, 0U, 0U);
	Test_UpdateSamples(start_ms + 10U, &imu, start_ms + 10U,
		10.0f, start_ms, 1U, 0U, 0U);
	state = AltitudeControl_GetState();
	imu_speed_mps = state->imu_vertical_speed_mps;
	Test_UpdateSamples(start_ms + 15U, &imu, start_ms + 10U,
		10.0f, start_ms, 1U, 0U, 0U);
	state = AltitudeControl_GetState();
	CHECK(fabsf(state->imu_vertical_speed_mps - imu_speed_mps) < 0.0001f);

	Test_UpdateSamples(start_ms + 40U, &imu, start_ms + 40U,
		10.4f, start_ms + 40U, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	baro_speed_mps = state->barometer_vertical_speed_mps;
	CHECK(baro_speed_mps > 0.0f);
	CHECK(state->control_allowed == 0U);
	CHECK(state->prediction_valid == 0U);

	Test_UpdateSamples(start_ms + 80U, &imu, start_ms + 80U,
		10.8f, start_ms + 80U, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	baro_speed_mps = state->barometer_vertical_speed_mps;
	held_deflection_deg = state->suggested_deflection_deg;
	CHECK(state->control_allowed != 0U);

	/* A one-sample pressure jump is rejected and not used for prediction. */
	Test_UpdateSamples(start_ms + 120U, &imu, start_ms + 120U,
		4.0f, start_ms + 120U, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK(fabsf(state->barometer_vertical_speed_mps - baro_speed_mps) <
		0.0001f);
	CHECK(fabsf(state->altitude_m - 10.8f) < 0.0001f);
	CHECK(state->control_allowed == 0U);
	CHECK(state->prediction_valid == 0U);
	CHECK(fabsf(state->suggested_deflection_deg - held_deflection_deg) <
		0.0001f);

	Test_UpdateSamples(start_ms + 160U, &imu, start_ms + 160U,
		4.4f, start_ms + 160U, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK(state->prediction_valid == 0U);
	Test_UpdateSamples(start_ms + 200U, &imu, start_ms + 200U,
		4.8f, start_ms + 200U, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK(state->prediction_valid == 0U);
	Test_UpdateSamples(start_ms + 240U, &imu, start_ms + 240U,
		5.2f, start_ms + 240U, 1U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK(state->control_allowed != 0U);
	CHECK(state->prediction_valid != 0U);
	CHECK(fabsf(state->vertical_speed_mps -
				  state->imu_vertical_speed_mps) < 0.0001f);
}

int main(void)
{
	Test_ControlGateAndPersistentFusion();
	Test_InvalidInputClearsCommand();
	Test_DeflectionRampsAndReportsCosts();
	Test_NewSamplesAndBarometerAnomalyGate();

	if(s_failures != 0U)
	{
		printf("FAILED: %u/%u checks failed\n", s_failures, s_checks);
		return 1;
	}

	printf("PASS: %u checks\n", s_checks);
	return 0;
}
