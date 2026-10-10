#include <math.h>

#include "RocketConfig.h"
#include "FullFlowTestConfig.h"
#include "Ignition.h"
#include "Servo.h"
#include "IMU.h"
#include "BMP280.h"
#include "FlightControl.h"
#include "FlightLog.h"
#include "RocketState.h"
#include "AltitudeControl.h"
#if ROCKET_REMOTE_LAUNCH_ENABLE
#include "RemoteLaunch.h"
#include "Buzzer.h"
#endif
#include "Rocket.h"

#if ROCKET_IMU_RETRY_STOP_MARGIN_MS >= ROCKET_PID_START_DELAY_MS
#error "ROCKET_IMU_RETRY_STOP_MARGIN_MS must be less than PID start delay"
#endif

#if (ROCKET_BAROMETER_ENABLE != 0) && (ROCKET_BAROMETER_ENABLE != 1)
#error "ROCKET_BAROMETER_ENABLE must be 0 or 1"
#endif

#if ROCKET_BAROMETER_ZERO_SAMPLES == 0U
#error "ROCKET_BAROMETER_ZERO_SAMPLES must be greater than zero"
#endif

/*============================== 私有变量 ==============================*/

static Rocket_StateMachine s_state_machine;
static IMU_Data s_imu_data;
static BMP280_Data s_barometer_data;
static uint32_t s_last_imu_ms = 0U;
static uint32_t s_last_imu_retry_ms = 0U;
static uint32_t s_last_valid_imu_ms = 0U;
#if ROCKET_BAROMETER_ENABLE
static uint32_t s_last_barometer_ms = 0U;
static uint32_t s_last_barometer_retry_ms = 0U;
static uint32_t s_last_valid_barometer_ms = 0U;
static uint64_t s_barometer_zero_sum_q24_8 = 0U;
static uint32_t s_barometer_reference_q24_8 = 0U;
static uint16_t s_barometer_zero_count = 0U;
#endif
static float s_altitude_m = 0.0f;
static uint8_t s_altitude_zero_ready = 0U;
static uint8_t s_control_running = 0U;
static uint8_t s_parachute_deployed = 0U;

/*============================== 私有函数 ==============================*/

static void Rocket_UpdateControl(uint32_t now_ms,
							 uint8_t sample_attempted,
							 uint8_t sample_valid)
{
	uint8_t pid_enabled;
	const AltitudeControl_State *altitude_control;

	if(s_state_machine.deployed != 0U)
	{
		if(s_parachute_deployed == 0U)
		{
			Servo_DeployParachute();
			s_parachute_deployed = 1U;
		}

		if(s_control_running != 0U)
		{
			FlightControl_Stop();
			s_control_running = 0U;
		}
		return;
	}

	pid_enabled = RocketState_IsPidEnabled(&s_state_machine, now_ms);
	if(pid_enabled == 0U)
	{
		if(s_control_running != 0U)
		{
			FlightControl_Stop();
			s_control_running = 0U;
		}
		return;
	}

	if((sample_attempted != 0U) && (sample_valid != 0U))
	{
		altitude_control = AltitudeControl_GetState();
		FlightControl_Update(&s_imu_data,
							 now_ms,
							 altitude_control->suggested_deflection_deg);
		s_control_running = 1U;
	}
	else if((sample_attempted != 0U) && (s_control_running != 0U))
	{
		FlightControl_Stop();
		s_control_running = 0U;
	}
}

#if ROCKET_BAROMETER_ENABLE
static float Rocket_CalculateAltitudeM(uint32_t pressure_q24_8,
									  uint32_t reference_q24_8)
{
	float pressure_ratio;

	if((pressure_q24_8 == 0U) || (reference_q24_8 == 0U))
	{
		return 0.0f;
	}

	pressure_ratio =
		(float)pressure_q24_8 / (float)reference_q24_8;
	return 44330.0f *
		   (1.0f - powf(pressure_ratio, 0.19029495f));
}
#endif

static void Rocket_ResetBarometerZero(void)
{
#if ROCKET_BAROMETER_ENABLE
	s_barometer_zero_sum_q24_8 = 0U;
	s_barometer_reference_q24_8 = 0U;
	s_barometer_zero_count = 0U;
#endif
	s_altitude_m = 0.0f;
	s_altitude_zero_ready = 0U;
}

static void Rocket_UpdateBarometer(uint32_t now_ms)
{
#if ROCKET_BAROMETER_ENABLE
	BMP280_Result result;

	if((BMP280_IsInitialized() == 0U) &&
	   ((uint32_t)(now_ms - s_last_barometer_retry_ms) >=
		ROCKET_BAROMETER_RETRY_PERIOD_MS))
	{
		s_last_barometer_retry_ms = now_ms;
		s_last_barometer_ms = now_ms;
		result = BMP280_Init();
		if((result == BMP280_RESULT_OK) &&
		   (s_altitude_zero_ready == 0U))
		{
			Rocket_ResetBarometerZero();
		}
	}

	if((BMP280_IsInitialized() == 0U) ||
	   ((uint32_t)(now_ms - s_last_barometer_ms) <
		ROCKET_BAROMETER_PERIOD_MS))
	{
		return;
	}

	s_last_barometer_ms = now_ms;
	result = BMP280_Read(&s_barometer_data);
	if(result != BMP280_RESULT_OK)
	{
		return;
	}
	s_last_valid_barometer_ms = now_ms;

	if(s_altitude_zero_ready == 0U)
	{
		s_barometer_zero_sum_q24_8 +=
			s_barometer_data.pressure_q24_8;
		s_barometer_zero_count++;
		s_altitude_m = 0.0f;
		if(s_barometer_zero_count >= ROCKET_BAROMETER_ZERO_SAMPLES)
		{
			s_barometer_reference_q24_8 =
				(uint32_t)(s_barometer_zero_sum_q24_8 /
						   s_barometer_zero_count);
			s_altitude_zero_ready = 1U;
		}
		return;
	}

	s_altitude_m =
		Rocket_CalculateAltitudeM(
			s_barometer_data.pressure_q24_8,
			s_barometer_reference_q24_8);
#else
	(void)now_ms;
#endif
}

/*============================== 公开函数 ==============================*/

void Rocket_Init(void)
{
	s_imu_data.roll_deg = 0.0f;
	s_imu_data.pitch_deg = 0.0f;
	s_imu_data.yaw_deg = 0.0f;
	s_imu_data.roll_rate_dps = 0.0f;
	s_imu_data.pitch_rate_dps = 0.0f;
	s_imu_data.yaw_rate_dps = 0.0f;
	s_imu_data.accel_x_mg = 0.0f;
	s_imu_data.accel_y_mg = 0.0f;
	s_imu_data.accel_z_mg = 0.0f;
	s_imu_data.valid = 0U;
	s_barometer_data.temperature_centi_c = 0;
	s_barometer_data.pressure_q24_8 = 0U;
	s_barometer_data.pressure_pa = 0U;
	s_barometer_data.raw_temperature = 0;
	s_barometer_data.raw_pressure = 0;
	s_barometer_data.valid = 0U;
	s_last_imu_ms = 0U;
	s_last_imu_retry_ms = 0U;
	s_last_valid_imu_ms = 0U;
#if ROCKET_BAROMETER_ENABLE
	s_last_barometer_ms = 0U;
	s_last_barometer_retry_ms = 0U;
	s_last_valid_barometer_ms = 0U;
#endif
	Rocket_ResetBarometerZero();
	s_control_running = 0U;
	s_parachute_deployed = 0U;

	Servo_Init();
#if ROCKET_REMOTE_LAUNCH_ENABLE
	Buzzer_Init();
	RemoteLaunch_Init();
#endif
	FlightControl_Init();
	AltitudeControl_Init();
	RocketState_Init(&s_state_machine);
	(void)IMU_Init();
#if ROCKET_BAROMETER_ENABLE
	(void)BMP280_Init();
#endif
}

void Rocket_Task(uint32_t now_ms)
{
	uint8_t sample_attempted = 0U;
	uint8_t sample_valid = 0U;
	uint8_t imu_retry_attempted = 0U;
	uint8_t ignition_active;
	uint8_t flight_control_enabled;
	uint8_t barometer_height_valid = 0U;
	uint8_t launch_started;
	uint32_t mission_ms;
	const AltitudeControl_State *altitude_control;

	launch_started = RocketState_HasLaunchStarted(&s_state_machine);
	mission_ms = RocketState_GetMissionElapsedMs(&s_state_machine, now_ms);
	if((IMU_IsInitialized() == 0U) &&
	   ((launch_started == 0U) ||
		(mission_ms <
		 (ROCKET_PID_START_DELAY_MS -
		  ROCKET_IMU_RETRY_STOP_MARGIN_MS))) &&
	   ((uint32_t)(now_ms - s_last_imu_retry_ms) >=
		ROCKET_IMU_RETRY_PERIOD_MS))
	{
		s_last_imu_retry_ms = now_ms;
		s_last_imu_ms = now_ms;
		imu_retry_attempted = 1U;
		(void)IMU_Init();
	}

	if((imu_retry_attempted == 0U) &&
	   ((uint32_t)(now_ms - s_last_imu_ms) >= ROCKET_IMU_PERIOD_MS))
	{
		s_last_imu_ms = now_ms;
		sample_attempted = 1U;
		sample_valid = IMU_Read(&s_imu_data, now_ms);
		if(sample_valid != 0U)
		{
			s_last_valid_imu_ms = now_ms;
		}
	}

	Rocket_UpdateBarometer(now_ms);
	altitude_control = AltitudeControl_GetState();
#if ROCKET_BAROMETER_ENABLE
	barometer_height_valid =
		(uint8_t)((s_altitude_zero_ready != 0U) &&
				  (s_barometer_data.valid != 0U) &&
				  ((uint32_t)(now_ms - s_last_valid_barometer_ms) <=
				   ROCKET_ALTITUDE_BARO_STALE_MS));
#endif
	RocketState_Update(&s_state_machine,
						now_ms,
						sample_attempted,
						sample_valid,
						s_imu_data.roll_deg,
						s_imu_data.pitch_deg,
						s_imu_data.accel_z_mg,
						s_altitude_m,
						altitude_control->vertical_speed_mps,
						s_altitude_zero_ready,
						barometer_height_valid,
						altitude_control->estimator_valid,
						altitude_control->imu_vertical_speed_mps,
						altitude_control->imu_estimator_valid);
#if ROCKET_REMOTE_LAUNCH_ENABLE
	RemoteLaunch_Task(now_ms,
					  RocketState_IsLaunchReady(&s_state_machine));
	if(RemoteLaunch_ConsumeRequest() != 0U)
	{
		if(RocketState_StartLaunch(
			&s_state_machine,
			(uint32_t)(now_ms - RemoteLaunch_GetLaunchAgeMs())) != 0U)
		{
			/* Do not carry pad-wait velocity/filter history into the flight. */
			AltitudeControl_Init();
		}
	}
	Buzzer_Task(now_ms,
				RemoteLaunch_IsReady(),
				RocketState_HasLaunchStarted(&s_state_machine));
#endif
	flight_control_enabled =
		RocketState_IsPidEnabled(&s_state_machine, now_ms);

	AltitudeControl_Update(now_ms,
						   &s_imu_data,
						   s_last_valid_imu_ms,
						   s_altitude_m,
						   s_altitude_zero_ready,
						   s_barometer_data.valid,
						   s_last_valid_barometer_ms,
						   flight_control_enabled,
						   s_parachute_deployed);
	Rocket_UpdateControl(now_ms, sample_attempted, sample_valid);
	ignition_active =
		RocketState_IsIgnitionAllowed(&s_state_machine, now_ms);
	Ignition_Set(ignition_active);
	/* Preserve the complete 20-second log window even after a long pad wait. */
	if(RocketState_HasLaunchStarted(&s_state_machine) != 0U)
	{
		FlightLog_Record(now_ms,
						 &s_imu_data,
						 &s_barometer_data,
						 s_altitude_m,
						 s_altitude_zero_ready,
						 AltitudeControl_GetState(),
						 s_last_valid_imu_ms,
						 s_last_valid_barometer_ms,
						 ignition_active,
						 s_parachute_deployed);
	}
}

Rocket_State Rocket_GetState(void)
{
	return s_state_machine.state;
}

uint8_t Rocket_HasImuFault(void)
{
	if((IMU_IsInitialized() == 0U) ||
	   (IMU_GetStatus() != IMU_STATUS_READY) ||
	   (s_imu_data.valid == 0U))
	{
		return 1U;
	}

	return 0U;
}

uint8_t Rocket_HasBarometerFault(void)
{
#if ROCKET_BAROMETER_ENABLE
	if((BMP280_IsInitialized() == 0U) ||
	   (s_barometer_data.valid == 0U))
	{
		return 1U;
	}
#endif

	return 0U;
}

void Rocket_DeployOutputInit(void)
{
	Servo_Init();
}

void Rocket_DeployOutputSet(void)
{
	Servo_DeployParachute();
}
