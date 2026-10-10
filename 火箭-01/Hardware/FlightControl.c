#include "RocketConfig.h"
#include "FullFlowTestConfig.h"
#include "PID.h"
#include "ControlMath.h"
#include "Servo.h"
#include "FlightControl.h"

/*============================== 私有变量 ==============================*/

static PID_Controller s_roll_rate_pid;
static PID_Controller s_pitch_rate_pid;
static float s_roll_rate_target_dps = 0.0f;
static float s_pitch_rate_target_dps = 0.0f;
static float s_roll_pid_output_us = 0.0f;
static float s_pitch_pid_output_us = 0.0f;
static float s_requested_altitude_deflection_deg = 0.0f;
static float s_attitude_scaled_deflection_deg = 0.0f;
static float s_applied_altitude_deflection_deg = 0.0f;
static float s_altitude_attitude_scale = 0.0f;
static float s_roll_altitude_headroom_us = 0.0f;
static float s_pitch_altitude_headroom_us = 0.0f;
static float s_applied_altitude_offset_us = 0.0f;
static float s_allocated_roll_pid_output_us = 0.0f;
static float s_allocated_pitch_pid_output_us = 0.0f;
static uint8_t s_altitude_limit_reason = 0U;
static uint32_t s_last_angle_ms = 0U;
static uint32_t s_last_rate_ms = 0U;
static uint8_t s_running = 0U;

/*============================== 私有函数 ==============================*/

static void FlightControl_UpdateAngleLoop(const IMU_Data *imu)
{
	s_roll_rate_target_dps = ControlMath_AngleToRate(
		ROCKET_TARGET_ROLL_DEG,
		imu->roll_deg,
		ROCKET_ROLL_ANGLE_KP,
		ROCKET_RATE_TARGET_LIMIT_DPS);
	s_pitch_rate_target_dps = ControlMath_AngleToRate(
		ROCKET_TARGET_PITCH_DEG,
		imu->pitch_deg,
		ROCKET_PITCH_ANGLE_KP,
		ROCKET_RATE_TARGET_LIMIT_DPS);
}

#if ROCKET_ALTITUDE_HEIGHT_PRIORITY_ENABLE
static float FlightControl_AllocatePidAfterAltitude(float pid_output_us,
												 float altitude_offset_us,
												 float altitude_mix_sign,
												 uint8_t limit_reason)
{
	float minimum_pid_us;
	float maximum_pid_us;
	float allocated_pid_us;

	minimum_pid_us =
		-ROCKET_CONTROL_LIMIT_US -
		altitude_offset_us * altitude_mix_sign;
	maximum_pid_us =
		ROCKET_CONTROL_LIMIT_US -
		altitude_offset_us * altitude_mix_sign;
	allocated_pid_us = ControlMath_Clamp(
		pid_output_us, minimum_pid_us, maximum_pid_us);
	if((allocated_pid_us < (pid_output_us - 0.01f)) ||
	   (allocated_pid_us > (pid_output_us + 0.01f)))
	{
		s_altitude_limit_reason |= limit_reason;
	}

	return allocated_pid_us;
}
#endif

static void FlightControl_UpdateRateLoop(const IMU_Data *imu,
										 float dt_s,
										 float altitude_deflection_deg)
{
	float roll_output_us;
	float pitch_output_us;
#if ROCKET_FULL_FLOW_ALTITUDE_MIX_ENABLE
	float altitude_offset_us;
	float requested_altitude_offset_us;
	float weighted_altitude_limit_us;
	float roll_available_us;
	float pitch_available_us;
#endif

	roll_output_us = PID_Update(&s_roll_rate_pid,
							   s_roll_rate_target_dps,
							   imu->roll_rate_dps,
							   dt_s);
	pitch_output_us = PID_Update(&s_pitch_rate_pid,
								s_pitch_rate_target_dps,
								imu->pitch_rate_dps,
								   dt_s);
	s_roll_pid_output_us = roll_output_us;
	s_pitch_pid_output_us = pitch_output_us;
	s_requested_altitude_deflection_deg = 0.0f;
	s_attitude_scaled_deflection_deg = 0.0f;
	s_applied_altitude_deflection_deg = 0.0f;
	s_altitude_attitude_scale = 0.0f;
	s_roll_altitude_headroom_us = 0.0f;
	s_pitch_altitude_headroom_us = 0.0f;
	s_applied_altitude_offset_us = 0.0f;
	s_allocated_roll_pid_output_us = roll_output_us;
	s_allocated_pitch_pid_output_us = pitch_output_us;
	s_altitude_limit_reason = 0U;
#if ROCKET_FULL_FLOW_ALTITUDE_MIX_ENABLE
	altitude_deflection_deg =
		ControlMath_Clamp(altitude_deflection_deg,
						  0.0f,
						  ROCKET_ALTITUDE_SERVO_LIMIT_DEG);
	s_requested_altitude_deflection_deg = altitude_deflection_deg;
	s_altitude_attitude_scale = 1.0f;
	s_attitude_scaled_deflection_deg = altitude_deflection_deg;
	requested_altitude_offset_us =
		altitude_deflection_deg * ROCKET_ALTITUDE_FIN_US_PER_DEG;
	weighted_altitude_limit_us =
		ROCKET_CONTROL_LIMIT_US *
		ROCKET_CONTROL_HEIGHT_WEIGHT /
		(ROCKET_CONTROL_ATTITUDE_WEIGHT + ROCKET_CONTROL_HEIGHT_WEIGHT);
	altitude_offset_us = ControlMath_Clamp(
		requested_altitude_offset_us,
		0.0f,
		weighted_altitude_limit_us);
#if ROCKET_ALTITUDE_HEIGHT_PRIORITY_ENABLE
	s_applied_altitude_offset_us = altitude_offset_us;
	s_applied_altitude_deflection_deg = altitude_deflection_deg;
	s_roll_altitude_headroom_us =
		ROCKET_CONTROL_LIMIT_US - altitude_offset_us;
	s_pitch_altitude_headroom_us =
		ROCKET_CONTROL_LIMIT_US - altitude_offset_us;
	s_allocated_roll_pid_output_us =
		FlightControl_AllocatePidAfterAltitude(
			roll_output_us,
			altitude_offset_us,
			ROCKET_ALTITUDE_ROLL_MIX_SIGN,
			FLIGHT_CONTROL_ATTITUDE_LIMIT_ROLL);
	s_allocated_pitch_pid_output_us =
		FlightControl_AllocatePidAfterAltitude(
			pitch_output_us,
			altitude_offset_us,
			ROCKET_ALTITUDE_PITCH_MIX_SIGN,
			FLIGHT_CONTROL_ATTITUDE_LIMIT_PITCH);
	roll_output_us = s_allocated_roll_pid_output_us +
		altitude_offset_us * ROCKET_ALTITUDE_ROLL_MIX_SIGN;
	pitch_output_us = s_allocated_pitch_pid_output_us +
		altitude_offset_us * ROCKET_ALTITUDE_PITCH_MIX_SIGN;
#else
	/*
	 * Attitude-first weighted allocation.  Keep both PID commands intact and
	 * use only the remaining actuator headroom for the altitude common mode.
	 */
	if(ROCKET_ALTITUDE_ROLL_MIX_SIGN >= 0.0f)
	{
		roll_available_us = ROCKET_CONTROL_LIMIT_US - roll_output_us;
	}
	else
	{
		roll_available_us = ROCKET_CONTROL_LIMIT_US + roll_output_us;
	}
	if(ROCKET_ALTITUDE_PITCH_MIX_SIGN >= 0.0f)
	{
		pitch_available_us = ROCKET_CONTROL_LIMIT_US - pitch_output_us;
	}
	else
	{
		pitch_available_us = ROCKET_CONTROL_LIMIT_US + pitch_output_us;
	}
	roll_available_us = ControlMath_Clamp(
		roll_available_us, 0.0f, weighted_altitude_limit_us);
	pitch_available_us = ControlMath_Clamp(
		pitch_available_us, 0.0f, weighted_altitude_limit_us);
	if(altitude_offset_us > roll_available_us)
	{
		altitude_offset_us = roll_available_us;
		s_altitude_limit_reason |= FLIGHT_CONTROL_ATTITUDE_LIMIT_ROLL;
	}
	if(altitude_offset_us > pitch_available_us)
	{
		altitude_offset_us = pitch_available_us;
		s_altitude_limit_reason |= FLIGHT_CONTROL_ATTITUDE_LIMIT_PITCH;
	}
	s_roll_altitude_headroom_us = roll_available_us;
	s_pitch_altitude_headroom_us = pitch_available_us;
	s_applied_altitude_offset_us = altitude_offset_us;
	s_applied_altitude_deflection_deg =
		altitude_offset_us / ROCKET_ALTITUDE_FIN_US_PER_DEG;
	if(requested_altitude_offset_us > 0.0001f)
	{
		s_altitude_attitude_scale =
			altitude_offset_us / requested_altitude_offset_us;
	}
	else
	{
		s_altitude_attitude_scale = 1.0f;
	}
	s_attitude_scaled_deflection_deg =
		s_applied_altitude_deflection_deg;
	roll_output_us +=
		altitude_offset_us * ROCKET_ALTITUDE_ROLL_MIX_SIGN;
	pitch_output_us +=
		altitude_offset_us * ROCKET_ALTITUDE_PITCH_MIX_SIGN;
#endif
	altitude_deflection_deg = 0.0f;
#elif ROCKET_ALTITUDE_SERVO_ENABLE
	s_applied_altitude_deflection_deg =
		ControlMath_Clamp(altitude_deflection_deg,
						  0.0f,
						  ROCKET_ALTITUDE_SERVO_LIMIT_DEG);
#endif
	Servo_SetMixedControlUs(roll_output_us,
							 pitch_output_us,
							 altitude_deflection_deg);
}

/*============================== 公开函数 ==============================*/

void FlightControl_Init(void)
{
	PID_Init(&s_roll_rate_pid,
			 ROCKET_ROLL_RATE_KP,
			 ROCKET_ROLL_RATE_KI,
			 ROCKET_ROLL_RATE_KD,
			 ROCKET_RATE_INTEGRAL_LIMIT_US,
			 ROCKET_CONTROL_LIMIT_US,
			 ROCKET_RATE_D_FILTER_ALPHA);
	PID_Init(&s_pitch_rate_pid,
			 ROCKET_PITCH_RATE_KP,
			 ROCKET_PITCH_RATE_KI,
			 ROCKET_PITCH_RATE_KD,
			 ROCKET_RATE_INTEGRAL_LIMIT_US,
			 ROCKET_CONTROL_LIMIT_US,
			 ROCKET_RATE_D_FILTER_ALPHA);
	FlightControl_Stop();
}

void FlightControl_Stop(void)
{
	PID_Reset(&s_roll_rate_pid);
	PID_Reset(&s_pitch_rate_pid);
	s_roll_rate_target_dps = 0.0f;
	s_pitch_rate_target_dps = 0.0f;
	s_roll_pid_output_us = 0.0f;
	s_pitch_pid_output_us = 0.0f;
	s_requested_altitude_deflection_deg = 0.0f;
	s_attitude_scaled_deflection_deg = 0.0f;
	s_applied_altitude_deflection_deg = 0.0f;
	s_altitude_attitude_scale = 0.0f;
	s_roll_altitude_headroom_us = 0.0f;
	s_pitch_altitude_headroom_us = 0.0f;
	s_applied_altitude_offset_us = 0.0f;
	s_allocated_roll_pid_output_us = 0.0f;
	s_allocated_pitch_pid_output_us = 0.0f;
	s_altitude_limit_reason = 0U;
	s_running = 0U;
	Servo_CenterControl();
}

void FlightControl_Update(const IMU_Data *imu,
						  uint32_t now_ms,
						  float altitude_deflection_deg)
{
	uint32_t rate_elapsed_ms;

	if(imu->valid == 0U)
	{
		FlightControl_Stop();
		return;
	}

	if(s_running == 0U)
	{
		PID_Reset(&s_roll_rate_pid);
		PID_Reset(&s_pitch_rate_pid);
		s_last_angle_ms = now_ms;
		s_last_rate_ms = now_ms;
		s_running = 1U;
		FlightControl_UpdateAngleLoop(imu);
		FlightControl_UpdateRateLoop(imu,
									 (float)ROCKET_RATE_LOOP_PERIOD_MS / 1000.0f,
									 altitude_deflection_deg);
		return;
	}

	if((uint32_t)(now_ms - s_last_angle_ms) >= ROCKET_ANGLE_LOOP_PERIOD_MS)
	{
		s_last_angle_ms = now_ms;
		FlightControl_UpdateAngleLoop(imu);
	}

	rate_elapsed_ms = (uint32_t)(now_ms - s_last_rate_ms);
	if(rate_elapsed_ms >= ROCKET_RATE_LOOP_PERIOD_MS)
	{
		s_last_rate_ms = now_ms;
		FlightControl_UpdateRateLoop(imu,
									 (float)rate_elapsed_ms / 1000.0f,
									 altitude_deflection_deg);
	}
}

void FlightControl_GetTelemetry(FlightControl_Telemetry *telemetry)
{
	if(telemetry == 0)
	{
		return;
	}

	telemetry->roll_rate_target_dps = s_roll_rate_target_dps;
	telemetry->pitch_rate_target_dps = s_pitch_rate_target_dps;
	telemetry->roll_pid_output_us = s_roll_pid_output_us;
	telemetry->pitch_pid_output_us = s_pitch_pid_output_us;
	telemetry->requested_altitude_deflection_deg =
		s_requested_altitude_deflection_deg;
	telemetry->attitude_scaled_deflection_deg =
		s_attitude_scaled_deflection_deg;
	telemetry->applied_altitude_deflection_deg =
		s_applied_altitude_deflection_deg;
	telemetry->altitude_attitude_scale = s_altitude_attitude_scale;
	telemetry->roll_altitude_headroom_us =
		s_roll_altitude_headroom_us;
	telemetry->pitch_altitude_headroom_us =
		s_pitch_altitude_headroom_us;
	telemetry->applied_altitude_offset_us =
		s_applied_altitude_offset_us;
	telemetry->allocated_roll_pid_output_us =
		s_allocated_roll_pid_output_us;
	telemetry->allocated_pitch_pid_output_us =
		s_allocated_pitch_pid_output_us;
	telemetry->altitude_limit_reason = s_altitude_limit_reason;
	telemetry->running = s_running;
}
