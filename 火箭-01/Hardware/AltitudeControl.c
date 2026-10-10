#include <math.h>
#include <stdint.h>

#include "RocketConfig.h"
#include "FullFlowTestConfig.h"
#include "ControlMath.h"
#include "AltitudeControl.h"

#define ALTITUDE_CONTROL_CANDIDATE_COUNT 11U

#if ROCKET_ALTITUDE_BARO_RECOVERY_SAMPLES == 0U
#error "ROCKET_ALTITUDE_BARO_RECOVERY_SAMPLES must be greater than zero"
#endif

static const float s_candidate_deflection_deg[ALTITUDE_CONTROL_CANDIDATE_COUNT] =
{
	0.0f, 3.0f, 6.0f, 9.0f, 12.0f, 15.0f,
	18.0f, 21.0f, 24.0f, 27.0f, 30.0f
};

static AltitudeControl_State s_state;
static uint8_t s_have_previous_altitude = 0U;
static uint32_t s_previous_altitude_ms = 0U;
static float s_previous_altitude_m = 0.0f;
static float s_filtered_vertical_speed_mps = 0.0f;
static float s_baro_vertical_speed_mps = 0.0f;
static float s_imu_vertical_speed_mps = 0.0f;
static float s_vertical_accel_mps2 = 0.0f;
static uint8_t s_have_imu_velocity = 0U;
static uint32_t s_previous_imu_sample_ms = 0U;
static float s_prediction_altitude_m = 0.0f;
static uint8_t s_have_prediction_altitude = 0U;
static uint8_t s_barometer_estimator_valid = 0U;
static uint8_t s_barometer_recovery_count = 0U;
static float s_last_deflection_deg = 0.0f;
static uint8_t s_have_control_update = 0U;
static uint32_t s_last_control_update_ms = 0U;
static uint8_t s_initial_deflection_sent = 0U;

static float AltitudeControl_DegToRad(float value_deg)
{
	return value_deg * 0.017453292519943295f;
}

static float AltitudeControl_ReferenceAreaM2(void)
{
	float diameter_m = ROCKET_ALTITUDE_REFERENCE_DIAMETER_M;
	return 0.7853981633974483f * diameter_m * diameter_m;
}

static float AltitudeControl_DragCoefficient(float deflection_deg)
{
	float deflection_rad;
	float coefficient;

	deflection_rad = AltitudeControl_DegToRad(deflection_deg);
	coefficient =
		ROCKET_ALTITUDE_BASE_CD +
		ROCKET_ALTITUDE_FIN_DRAG_K * deflection_rad * deflection_rad;
	return ControlMath_Clamp(coefficient,
							 ROCKET_ALTITUDE_CD_MIN,
							 ROCKET_ALTITUDE_CD_MAX);
}

static float AltitudeControl_PredictApogee(float altitude_m,
										  float vertical_speed_mps,
										  float deflection_deg)
{
	float area_m2;
	float drag_gain;
	float drag_cd;
	float remaining_height_m;

	if(vertical_speed_mps <= 0.0f)
	{
		return altitude_m;
	}

	area_m2 = AltitudeControl_ReferenceAreaM2();
	drag_cd = AltitudeControl_DragCoefficient(deflection_deg);
	drag_gain =
		(ROCKET_ALTITUDE_AIR_DENSITY_KG_M3 * drag_cd * area_m2) /
		(2.0f * ROCKET_ALTITUDE_STAGE_MASS_KG);

	/*
	 * Closed-form solution for dv/dt = -g - k*v^2.  This replaces the
	 * former 500-step integration for each of the 16 fin candidates.
	 */
	if(drag_gain <= 0.000001f)
	{
		remaining_height_m =
			(vertical_speed_mps * vertical_speed_mps) /
			(2.0f * ROCKET_ALTITUDE_GRAVITY_MPS2);
	}
	else
	{
		remaining_height_m =
			logf(1.0f +
				 (drag_gain * vertical_speed_mps * vertical_speed_mps) /
				 ROCKET_ALTITUDE_GRAVITY_MPS2) /
			(2.0f * drag_gain);
	}

	return altitude_m + remaining_height_m;
}

static float AltitudeControl_CalculateVerticalSpecificMg(const IMU_Data *imu)
{
	float roll_rad;
	float pitch_rad;
	float sin_roll;
	float cos_roll;
	float sin_pitch;
	float cos_pitch;
	float vertical_specific_mg;

	roll_rad = AltitudeControl_DegToRad(imu->roll_deg);
	pitch_rad = AltitudeControl_DegToRad(imu->pitch_deg);
	sin_roll = sinf(roll_rad);
	cos_roll = cosf(roll_rad);
	sin_pitch = sinf(pitch_rad);
	cos_pitch = cosf(pitch_rad);

	vertical_specific_mg =
		-imu->accel_x_mg * sin_pitch +
		 imu->accel_y_mg * sin_roll * cos_pitch +
		 imu->accel_z_mg * cos_roll * cos_pitch;

	return vertical_specific_mg * ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN;
}

static uint8_t AltitudeControl_UpdateImuVelocity(uint32_t now_ms,
										 const IMU_Data *imu,
										 uint32_t imu_sample_ms)
{
	uint32_t elapsed_ms;
	float dt_s;
	float vertical_specific_mg;

	if((imu == 0) || (imu->valid == 0U))
	{
		s_have_imu_velocity = 0U;
		s_imu_vertical_speed_mps = 0.0f;
		s_vertical_accel_mps2 = 0.0f;
		return 0U;
	}

	if((uint32_t)(now_ms - imu_sample_ms) >
	   ROCKET_ALTITUDE_IMU_VELOCITY_MAX_DT_MS)
	{
		s_have_imu_velocity = 0U;
		s_imu_vertical_speed_mps = 0.0f;
		s_vertical_accel_mps2 = 0.0f;
		return 0U;
	}

	if((s_have_imu_velocity != 0U) &&
	   (imu_sample_ms == s_previous_imu_sample_ms))
	{
		return 1U;
	}

	vertical_specific_mg = AltitudeControl_CalculateVerticalSpecificMg(imu);
	s_vertical_accel_mps2 =
		(vertical_specific_mg -
		 ROCKET_ALTITUDE_ACCEL_1G_MG -
		 ROCKET_ALTITUDE_ACCEL_VERTICAL_BIAS_MG) *
		(ROCKET_ALTITUDE_GRAVITY_MPS2 / 1000.0f);

	if(s_have_imu_velocity == 0U)
	{
		s_have_imu_velocity = 1U;
		s_previous_imu_sample_ms = imu_sample_ms;
		s_imu_vertical_speed_mps = s_filtered_vertical_speed_mps;
		return 1U;
	}

	elapsed_ms = (uint32_t)(imu_sample_ms - s_previous_imu_sample_ms);
	if(elapsed_ms == 0U)
	{
		return 1U;
	}

	if(elapsed_ms > ROCKET_ALTITUDE_IMU_VELOCITY_MAX_DT_MS)
	{
		s_previous_imu_sample_ms = imu_sample_ms;
		s_imu_vertical_speed_mps = s_filtered_vertical_speed_mps;
		return 0U;
	}

	dt_s = (float)elapsed_ms / 1000.0f;
	s_previous_imu_sample_ms = imu_sample_ms;
	s_imu_vertical_speed_mps += s_vertical_accel_mps2 * dt_s;
	s_filtered_vertical_speed_mps += s_vertical_accel_mps2 * dt_s;
	s_imu_vertical_speed_mps =
		ControlMath_Clamp(s_imu_vertical_speed_mps,
						  -ROCKET_ALTITUDE_IMU_SPEED_LIMIT_MPS,
						  ROCKET_ALTITUDE_IMU_SPEED_LIMIT_MPS);
	s_filtered_vertical_speed_mps =
		ControlMath_Clamp(s_filtered_vertical_speed_mps,
						  -ROCKET_ALTITUDE_IMU_SPEED_LIMIT_MPS,
						  ROCKET_ALTITUDE_IMU_SPEED_LIMIT_MPS);
	return 1U;
}

static uint8_t AltitudeControl_UpdateVelocity(uint32_t now_ms,
										 const IMU_Data *imu,
										 uint32_t imu_sample_ms,
										 float altitude_m,
										 uint8_t barometer_valid,
										 uint32_t barometer_sample_ms)
{
	uint32_t elapsed_ms;
	float dt_s;
	float raw_speed_mps;
	uint8_t imu_velocity_valid;
	uint8_t baro_velocity_valid = 0U;
	uint8_t barometer_usable;
	uint8_t barometer_recovered = 0U;

	imu_velocity_valid = AltitudeControl_UpdateImuVelocity(
		now_ms, imu, imu_sample_ms);
	barometer_usable =
		(uint8_t)((barometer_valid != 0U) &&
		((uint32_t)(now_ms - barometer_sample_ms) <=
		 ROCKET_ALTITUDE_BARO_STALE_MS));
	if(barometer_usable == 0U)
	{
		s_barometer_estimator_valid = 0U;
		s_barometer_recovery_count = 0U;
		return imu_velocity_valid;
	}

	if(s_have_previous_altitude == 0U)
	{
		s_have_previous_altitude = 1U;
		s_previous_altitude_ms = barometer_sample_ms;
		s_previous_altitude_m = altitude_m;
		s_prediction_altitude_m = altitude_m;
		s_have_prediction_altitude = 1U;
		s_barometer_recovery_count = 1U;
		s_barometer_estimator_valid =
			(uint8_t)(ROCKET_ALTITUDE_BARO_RECOVERY_SAMPLES <= 1U);
		s_baro_vertical_speed_mps = 0.0f;
		if(imu_velocity_valid != 0U)
		{
			s_filtered_vertical_speed_mps = s_imu_vertical_speed_mps;
			return 1U;
		}
		s_filtered_vertical_speed_mps = 0.0f;
		return 0U;
	}

	if(barometer_sample_ms == s_previous_altitude_ms)
	{
		return imu_velocity_valid;
	}

	elapsed_ms = (uint32_t)(barometer_sample_ms - s_previous_altitude_ms);
	if((elapsed_ms >= ROCKET_ALTITUDE_VELOCITY_MIN_DT_MS) &&
	   (elapsed_ms <= ROCKET_ALTITUDE_VELOCITY_MAX_DT_MS))
	{
		dt_s = (float)elapsed_ms / 1000.0f;
		raw_speed_mps = (altitude_m - s_previous_altitude_m) / dt_s;
		if((imu_velocity_valid != 0U) &&
		   (fabsf(raw_speed_mps - s_imu_vertical_speed_mps) <=
			ROCKET_ALTITUDE_BARO_IMU_SPEED_GATE_MPS))
		{
			/* Only accepted pressure samples may move the apogee input. */
			s_baro_vertical_speed_mps =
				ROCKET_ALTITUDE_VELOCITY_FILTER_ALPHA * raw_speed_mps +
				(1.0f - ROCKET_ALTITUDE_VELOCITY_FILTER_ALPHA) *
				s_baro_vertical_speed_mps;
			s_prediction_altitude_m = altitude_m;
			s_have_prediction_altitude = 1U;
			if(s_barometer_recovery_count <
			   ROCKET_ALTITUDE_BARO_RECOVERY_SAMPLES)
			{
				s_barometer_recovery_count++;
			}
			if(s_barometer_recovery_count >=
			   ROCKET_ALTITUDE_BARO_RECOVERY_SAMPLES)
			{
				if(s_barometer_estimator_valid == 0U)
				{
					barometer_recovered = 1U;
				}
				s_barometer_estimator_valid = 1U;
				baro_velocity_valid = 1U;
			}
			else
			{
				s_barometer_estimator_valid = 0U;
			}
		}
		else
		{
			s_barometer_estimator_valid = 0U;
			s_barometer_recovery_count = 0U;
		}
		s_previous_altitude_ms = barometer_sample_ms;
		s_previous_altitude_m = altitude_m;
	}
	else if(elapsed_ms > ROCKET_ALTITUDE_VELOCITY_MAX_DT_MS)
	{
		s_previous_altitude_ms = barometer_sample_ms;
		s_previous_altitude_m = altitude_m;
		s_barometer_recovery_count = 1U;
		s_barometer_estimator_valid = 0U;
		s_baro_vertical_speed_mps = 0.0f;
	}
	else
	{
		s_previous_altitude_ms = barometer_sample_ms;
		s_previous_altitude_m = altitude_m;
		s_barometer_estimator_valid = 0U;
		s_barometer_recovery_count = 0U;
	}

	if((imu_velocity_valid != 0U) && (baro_velocity_valid != 0U))
	{
		if(barometer_recovered != 0U)
		{
			/* Discard pressure-corrupted fusion history after recovery. */
			s_filtered_vertical_speed_mps = s_imu_vertical_speed_mps;
		}
		else
		{
			/* Keep the fused estimate between barometer updates. */
			s_filtered_vertical_speed_mps +=
				ROCKET_ALTITUDE_BARO_SPEED_CORRECTION *
				(s_baro_vertical_speed_mps - s_filtered_vertical_speed_mps);
		}
		s_filtered_vertical_speed_mps =
			ControlMath_Clamp(s_filtered_vertical_speed_mps,
							 -ROCKET_ALTITUDE_IMU_SPEED_LIMIT_MPS,
							 ROCKET_ALTITUDE_IMU_SPEED_LIMIT_MPS);
		return 1U;
	}

	if(imu_velocity_valid != 0U)
	{
		return 1U;
	}

	if(baro_velocity_valid != 0U)
	{
		s_filtered_vertical_speed_mps = s_baro_vertical_speed_mps;
		return 1U;
	}

	return 0U;
}

static void AltitudeControl_SelectDeflection(void)
{
	float best_cost = 3.402823466e+38f;
	float best_apogee;
	float best_deflection = 0.0f;
	float zero_apogee_m;
	float apogee_m;
	float error_m;
	float deflection_delta;
	float height_error_cost;
	float deflection_cost;
	float deflection_change_cost;
	float cost;
	uint8_t index;

	zero_apogee_m =
		AltitudeControl_PredictApogee(s_state.altitude_m,
								  s_state.vertical_speed_mps,
								  0.0f);
	if(zero_apogee_m <=
	   (ROCKET_ALTITUDE_TARGET_M + ROCKET_ALTITUDE_DEADBAND_M))
	{
		best_deflection = ControlMath_Clamp(
			s_last_deflection_deg -
			ROCKET_ALTITUDE_MAX_DEFLECTION_STEP_DEG,
			0.0f,
			ROCKET_ALTITUDE_SERVO_LIMIT_DEG);
		best_apogee = AltitudeControl_PredictApogee(
			s_state.altitude_m,
			s_state.vertical_speed_mps,
			best_deflection);
		error_m = best_apogee - ROCKET_ALTITUDE_TARGET_M;
		deflection_delta = best_deflection - s_last_deflection_deg;
		s_state.predicted_apogee_m = best_apogee;
		s_state.suggested_deflection_deg = best_deflection;
		s_state.height_error_cost =
			ROCKET_ALTITUDE_COST_HEIGHT * error_m * error_m;
		s_state.deflection_cost =
			ROCKET_ALTITUDE_COST_DEFLECTION *
			best_deflection * best_deflection;
		s_state.deflection_change_cost =
			ROCKET_ALTITUDE_COST_DEFLECTION_CHANGE *
			deflection_delta * deflection_delta;
		s_state.prediction_valid = 1U;
		s_last_deflection_deg = best_deflection;
		return;
	}

	best_apogee = zero_apogee_m;
	for(index = 0U; index < ALTITUDE_CONTROL_CANDIDATE_COUNT; index++)
	{
		deflection_delta =
			s_candidate_deflection_deg[index] - s_last_deflection_deg;
		if(fabsf(deflection_delta) >
		   ROCKET_ALTITUDE_MAX_DEFLECTION_STEP_DEG)
		{
			continue;
		}

		apogee_m =
			AltitudeControl_PredictApogee(
				s_state.altitude_m,
				s_state.vertical_speed_mps,
				s_candidate_deflection_deg[index]);
		error_m = apogee_m - ROCKET_ALTITUDE_TARGET_M;
		height_error_cost =
			ROCKET_ALTITUDE_COST_HEIGHT * error_m * error_m;
		deflection_cost =
			ROCKET_ALTITUDE_COST_DEFLECTION *
			s_candidate_deflection_deg[index] *
			s_candidate_deflection_deg[index];
		deflection_change_cost =
			ROCKET_ALTITUDE_COST_DEFLECTION_CHANGE *
			deflection_delta * deflection_delta;
		cost = height_error_cost +
			deflection_cost +
			deflection_change_cost;

		if(cost < best_cost)
		{
			best_cost = cost;
			best_apogee = apogee_m;
			best_deflection = s_candidate_deflection_deg[index];
			s_state.height_error_cost = height_error_cost;
			s_state.deflection_cost = deflection_cost;
			s_state.deflection_change_cost = deflection_change_cost;
		}
	}

	s_state.predicted_apogee_m = best_apogee;
	s_state.suggested_deflection_deg = best_deflection;
	s_state.prediction_valid = 1U;
	s_last_deflection_deg = best_deflection;
}

static void AltitudeControl_ForceInitialDeflection(void)
{
	float deflection_deg = 0.0f;
	float error_m;
	float deflection_delta;

	s_state.predicted_apogee_m = AltitudeControl_PredictApogee(
		s_state.altitude_m,
		s_state.vertical_speed_mps,
		deflection_deg);
	error_m =
		s_state.predicted_apogee_m - ROCKET_ALTITUDE_TARGET_M;
	deflection_delta = deflection_deg - s_last_deflection_deg;
	s_state.suggested_deflection_deg = deflection_deg;
	s_state.height_error_cost =
		ROCKET_ALTITUDE_COST_HEIGHT * error_m * error_m;
	s_state.deflection_cost =
		ROCKET_ALTITUDE_COST_DEFLECTION *
		deflection_deg * deflection_deg;
	s_state.deflection_change_cost =
		ROCKET_ALTITUDE_COST_DEFLECTION_CHANGE *
		deflection_delta * deflection_delta;
	s_state.prediction_valid = 1U;
	s_last_deflection_deg = deflection_deg;
}

void AltitudeControl_Init(void)
{
	s_state.altitude_m = 0.0f;
	s_state.vertical_speed_mps = 0.0f;
	s_state.barometer_vertical_speed_mps = 0.0f;
	s_state.imu_vertical_speed_mps = 0.0f;
	s_state.vertical_accel_mps2 = 0.0f;
	s_state.predicted_apogee_m = 0.0f;
	s_state.suggested_deflection_deg = 0.0f;
	s_state.height_error_cost = 0.0f;
	s_state.deflection_cost = 0.0f;
	s_state.deflection_change_cost = 0.0f;
	s_state.estimator_valid = 0U;
	s_state.imu_estimator_valid = 0U;
	s_state.prediction_valid = 0U;
	s_state.control_allowed = 0U;
	s_have_previous_altitude = 0U;
	s_previous_altitude_ms = 0U;
	s_previous_altitude_m = 0.0f;
	s_filtered_vertical_speed_mps = 0.0f;
	s_baro_vertical_speed_mps = 0.0f;
	s_imu_vertical_speed_mps = 0.0f;
	s_vertical_accel_mps2 = 0.0f;
	s_have_imu_velocity = 0U;
	s_previous_imu_sample_ms = 0U;
	s_prediction_altitude_m = 0.0f;
	s_have_prediction_altitude = 0U;
	s_barometer_estimator_valid = 0U;
	s_barometer_recovery_count = 0U;
	s_last_deflection_deg = 0.0f;
	s_have_control_update = 0U;
	s_last_control_update_ms = 0U;
	s_initial_deflection_sent = 0U;
}

void AltitudeControl_Update(uint32_t now_ms,
							const IMU_Data *imu,
							uint32_t imu_sample_ms,
							float altitude_m,
							uint8_t altitude_zero_ready,
							uint8_t barometer_valid,
							uint32_t barometer_sample_ms,
							uint8_t flight_control_enabled,
							uint8_t parachute_deployed)
{
	uint8_t velocity_valid;

	s_state.altitude_m = altitude_m;
	s_state.control_allowed = 0U;

	if((altitude_zero_ready == 0U) ||
	   (imu == 0) ||
	   (imu->valid == 0U) ||
	   (parachute_deployed != 0U))
	{
		s_state.estimator_valid = 0U;
		s_state.imu_estimator_valid = 0U;
		s_state.vertical_speed_mps = 0.0f;
		s_state.barometer_vertical_speed_mps = 0.0f;
		s_state.imu_vertical_speed_mps = 0.0f;
		s_state.vertical_accel_mps2 = 0.0f;
		s_have_previous_altitude = 0U;
		s_have_imu_velocity = 0U;
		s_baro_vertical_speed_mps = 0.0f;
		s_imu_vertical_speed_mps = 0.0f;
		s_vertical_accel_mps2 = 0.0f;
		s_have_prediction_altitude = 0U;
		s_barometer_estimator_valid = 0U;
		s_barometer_recovery_count = 0U;
		s_last_deflection_deg = 0.0f;
		s_have_control_update = 0U;
		s_state.predicted_apogee_m = 0.0f;
		s_state.suggested_deflection_deg = 0.0f;
		s_state.prediction_valid = 0U;
		s_state.height_error_cost = 0.0f;
		s_state.deflection_cost = 0.0f;
		s_state.deflection_change_cost = 0.0f;

		return;
	}

	velocity_valid = AltitudeControl_UpdateVelocity(now_ms,
													  imu,
													  imu_sample_ms,
													  altitude_m,
													  barometer_valid,
													  barometer_sample_ms);
	if(s_have_prediction_altitude != 0U)
	{
		s_state.altitude_m = s_prediction_altitude_m;
	}
	s_state.vertical_speed_mps = s_filtered_vertical_speed_mps;
	s_state.barometer_vertical_speed_mps = s_baro_vertical_speed_mps;
	s_state.imu_vertical_speed_mps = s_imu_vertical_speed_mps;
	s_state.vertical_accel_mps2 = s_vertical_accel_mps2;
	s_state.estimator_valid = velocity_valid;
	s_state.imu_estimator_valid = s_have_imu_velocity;
	if(velocity_valid == 0U)
	{
		return;
	}

	if((flight_control_enabled == 0U) ||
	   (now_ms < ROCKET_ALTITUDE_CONTROL_START_MS) ||
	   (s_state.vertical_speed_mps < ROCKET_ALTITUDE_MIN_CONTROL_SPEED_MPS))
	{
		s_last_deflection_deg = 0.0f;
		s_have_control_update = 0U;
		s_state.predicted_apogee_m = 0.0f;
		s_state.suggested_deflection_deg = 0.0f;
		s_state.prediction_valid = 0U;
		s_state.height_error_cost = 0.0f;
		s_state.deflection_cost = 0.0f;
		s_state.deflection_change_cost = 0.0f;
		return;
	}

	if(s_barometer_estimator_valid == 0U)
	{
		/* Hold the last safe command while pressure data re-qualifies. */
		s_state.predicted_apogee_m = 0.0f;
		s_state.prediction_valid = 0U;
		s_state.height_error_cost = 0.0f;
		s_state.deflection_cost = 0.0f;
		s_state.deflection_change_cost = 0.0f;
		return;
	}

	s_state.control_allowed = 1U;
	if((s_have_control_update == 0U) ||
	   ((uint32_t)(now_ms - s_last_control_update_ms) >=
		ROCKET_ALTITUDE_CONTROL_PERIOD_MS))
	{
		s_last_control_update_ms = now_ms;
		if(s_initial_deflection_sent == 0U)
		{
			AltitudeControl_ForceInitialDeflection();
			s_initial_deflection_sent = 1U;
		}
		else
		{
			AltitudeControl_SelectDeflection();
		}
		s_have_control_update = 1U;
	}
}

const AltitudeControl_State *AltitudeControl_GetState(void)
{
	return &s_state;
}
