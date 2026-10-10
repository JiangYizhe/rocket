#include <math.h>
#include <stdio.h>

#include "FlightControl.h"

static unsigned int s_checks = 0U;
static unsigned int s_failures = 0U;
static float s_roll_command_us = 0.0f;
static float s_pitch_command_us = 0.0f;

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

void Servo_CenterControl(void)
{
	s_roll_command_us = 0.0f;
	s_pitch_command_us = 0.0f;
}

void Servo_SetMixedControlUs(float roll_offset_us,
							 float pitch_offset_us,
							 float altitude_deflection_deg)
{
	(void)altitude_deflection_deg;
	s_roll_command_us = roll_offset_us;
	s_pitch_command_us = pitch_offset_us;
}

static IMU_Data Test_MakeImu(void)
{
	IMU_Data imu = {0};
	imu.valid = 1U;
	return imu;
}

static void Test_FullAltitudeAuthorityWhenLevel(void)
{
	IMU_Data imu = Test_MakeImu();
	FlightControl_Telemetry telemetry;

	FlightControl_Init();
	FlightControl_Update(&imu, 8000U, 30.0f);
	FlightControl_GetTelemetry(&telemetry);

	CHECK(fabsf(telemetry.altitude_attitude_scale - 1.0f) < 0.0001f);
	CHECK(fabsf(telemetry.applied_altitude_deflection_deg - 30.0f) < 0.001f);
	CHECK(fabsf(telemetry.allocated_roll_pid_output_us) < 0.001f);
	CHECK(fabsf(telemetry.allocated_pitch_pid_output_us) < 0.001f);
	CHECK(telemetry.altitude_limit_reason == 0U);
	CHECK(fabsf(s_roll_command_us - 166.668f) < 0.01f);
	CHECK(fabsf(s_pitch_command_us + 166.668f) < 0.01f);
}

static void Test_HeightPriorityLimitsAttitudePid(void)
{
	IMU_Data imu = Test_MakeImu();
	FlightControl_Telemetry telemetry;

	imu.roll_deg = -20.0f;
	FlightControl_Init();
	FlightControl_Update(&imu, 8000U, 30.0f);
	FlightControl_GetTelemetry(&telemetry);

	CHECK(fabsf(telemetry.roll_pid_output_us - 80.0f) < 0.01f);
	CHECK(fabsf(telemetry.altitude_attitude_scale - 1.0f) < 0.0001f);
	CHECK(fabsf(telemetry.applied_altitude_deflection_deg - 30.0f) < 0.001f);
	CHECK(fabsf(telemetry.allocated_roll_pid_output_us - 80.0f) < 0.01f);
	CHECK(telemetry.altitude_limit_reason == 0U);
	CHECK(fabsf(s_roll_command_us - 246.668f) < 0.01f);
}

static void Test_LargeAttitudeErrorDoesNotReduceAltitude(void)
{
	IMU_Data imu = Test_MakeImu();
	FlightControl_Telemetry telemetry;

	imu.roll_deg = -30.0f;
	FlightControl_Init();
	FlightControl_Update(&imu, 8000U, 30.0f);
	FlightControl_GetTelemetry(&telemetry);

	CHECK(telemetry.altitude_attitude_scale == 1.0f);
	CHECK(fabsf(telemetry.applied_altitude_deflection_deg - 30.0f) < 0.001f);
	CHECK(telemetry.altitude_limit_reason == 0U);
}

static void Test_PidHeadroomHasPriority(void)
{
	IMU_Data imu = Test_MakeImu();
	FlightControl_Telemetry telemetry;

	imu.roll_rate_dps = -300.0f;
	FlightControl_Init();
	FlightControl_Update(&imu, 8000U, 30.0f);
	FlightControl_GetTelemetry(&telemetry);

	CHECK(fabsf(telemetry.roll_pid_output_us - 300.0f) < 0.01f);
	CHECK(fabsf(telemetry.allocated_roll_pid_output_us - 300.0f) < 0.01f);
	CHECK(fabsf(telemetry.applied_altitude_deflection_deg) < 0.001f);
	CHECK((telemetry.altitude_limit_reason &
		FLIGHT_CONTROL_ATTITUDE_LIMIT_ROLL) != 0U);
	CHECK(fabsf(s_roll_command_us - 300.0f) < 0.01f);
}

int main(void)
{
	Test_FullAltitudeAuthorityWhenLevel();
	Test_HeightPriorityLimitsAttitudePid();
	Test_LargeAttitudeErrorDoesNotReduceAltitude();
	Test_PidHeadroomHasPriority();

	if(s_failures != 0U)
	{
		printf("FAILED: %u/%u checks failed\n", s_failures, s_checks);
		return 1;
	}

	printf("PASS: %u checks\n", s_checks);
	return 0;
}
