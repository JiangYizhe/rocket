#include "RocketState.h"
#include "RocketConfig.h"
#include "FullFlowTestConfig.h"
#include "ControlMath.h"

#if (ROCKET_REMOTE_LAUNCH_ENABLE != 0) && \
	(ROCKET_REMOTE_LAUNCH_ENABLE != 1)
#error "ROCKET_REMOTE_LAUNCH_ENABLE must be 0 or 1"
#endif

#if (ROCKET_LEGACY_AUTO_LAUNCH_ENABLE != 0) && \
	(ROCKET_LEGACY_AUTO_LAUNCH_ENABLE != 1)
#error "ROCKET_LEGACY_AUTO_LAUNCH_ENABLE must be 0 or 1"
#endif

#if (ROCKET_REMOTE_LAUNCH_ENABLE + ROCKET_LEGACY_AUTO_LAUNCH_ENABLE) != 1
#error "Select exactly one launch source"
#endif

#if ROCKET_IMU_IGNITION_GOOD_SAMPLES == 0U
#error "ROCKET_IMU_IGNITION_GOOD_SAMPLES must be greater than zero"
#endif

#if ROCKET_IMU_IGNITION_GOOD_SAMPLES > 255U
#error "ROCKET_IMU_IGNITION_GOOD_SAMPLES must fit in uint8_t"
#endif

#if ROCKET_IGNITION_START_GRACE_MS >= ROCKET_IGNITION_PULSE_MS
#error "Ignition start grace must be shorter than the ignition pulse"
#endif

#if ROCKET_BURNOUT_CONFIRM_MS < ROCKET_IMU_PERIOD_MS
#error "Burnout confirm time must be at least one IMU sample period"
#endif

#if (ROCKET_SENSOR_FAIL_DEPLOY_ENABLE != 0) && \
	(ROCKET_SENSOR_FAIL_DEPLOY_ENABLE != 1)
#error "ROCKET_SENSOR_FAIL_DEPLOY_ENABLE must be 0 or 1"
#endif

#if ROCKET_ALTITUDE_DESCENT_GAP_MS >= \
	ROCKET_ALTITUDE_LOW_DESCENT_CONFIRM_MS
#error "Descent gap tolerance must be shorter than descent confirmation"
#endif

/*============================== 私有函数 ==============================*/

static float RocketState_AbsFloat(float value)
{
	return (value < 0.0f) ? -value : value;
}

static uint8_t RocketState_IsTiltExceeded(float roll_deg, float pitch_deg)
{
	if(RocketState_AbsFloat(
		ControlMath_AngleErrorDeg(ROCKET_TARGET_ROLL_DEG, roll_deg)) >
	   ROCKET_CRITICAL_ANGLE_DEG)
	{
		return 1U;
	}

	if(RocketState_AbsFloat(
		ControlMath_AngleErrorDeg(ROCKET_TARGET_PITCH_DEG, pitch_deg)) >
	   ROCKET_CRITICAL_ANGLE_DEG)
	{
		return 1U;
	}

	return 0U;
}

static void RocketState_UpdateImuHealth(Rocket_StateMachine *machine,
										uint8_t sample_attempted,
										uint8_t sample_valid)
{
	if(sample_attempted == 0U)
	{
		return;
	}

	if(sample_valid != 0U)
	{
		machine->imu_sample_valid = 1U;
		machine->imu_fail_count = 0U;
		machine->imu_healthy = 1U;
		if(machine->imu_good_count <
		   ROCKET_IMU_IGNITION_GOOD_SAMPLES)
		{
			machine->imu_good_count++;
		}
		return;
	}

	if(machine->imu_fail_count < ROCKET_IMU_FAIL_LIMIT)
	{
		machine->imu_fail_count++;
	}

	/*
	 * Initial arming still requires consecutive valid samples.  Once armed,
	 * tolerate up to two isolated/short invalid samples so readiness and the
	 * buzzer do not chatter on a single transient sensor read failure.
	 */
	if(machine->imu_good_count < ROCKET_IMU_IGNITION_GOOD_SAMPLES)
	{
		machine->imu_sample_valid = 0U;
		machine->imu_good_count = 0U;
	}
	if(machine->imu_fail_count >= ROCKET_IMU_FAIL_LIMIT)
	{
		machine->imu_sample_valid = 0U;
		machine->imu_good_count = 0U;
		machine->imu_healthy = 0U;
	}
}

#if ROCKET_TILT_DEPLOY_ENABLE
static void RocketState_UpdateTilt(Rocket_StateMachine *machine,
								   uint32_t now_ms,
								   uint8_t sample_attempted,
								   uint8_t sample_valid,
								   float roll_deg,
								   float pitch_deg)
{
	uint8_t tilt_exceeded;

	if((now_ms < ROCKET_TILT_CHECK_TIME_MS) || (sample_attempted == 0U))
	{
		return;
	}

	if(sample_valid == 0U)
	{
		machine->tilt_pending = 0U;
		machine->tilt_last_sample_ms = 0U;
		return;
	}

	if((machine->tilt_pending != 0U) &&
	   ((uint32_t)(now_ms - machine->tilt_last_sample_ms) >
		ROCKET_TILT_MAX_SAMPLE_GAP_MS))
	{
		machine->tilt_pending = 0U;
	}
	machine->tilt_last_sample_ms = now_ms;

	tilt_exceeded = RocketState_IsTiltExceeded(roll_deg, pitch_deg);
	if(tilt_exceeded == 0U)
	{
		machine->tilt_pending = 0U;
		return;
	}

	if(machine->tilt_pending == 0U)
	{
		machine->tilt_pending = 1U;
		machine->tilt_start_ms = now_ms;
		return;
	}

	if((uint32_t)(now_ms - machine->tilt_start_ms) >= ROCKET_TILT_CONFIRM_MS)
	{
		machine->deployed = 1U;
	}
}
#endif

#if ROCKET_ALTITUDE_DEPLOY_ENABLE
static void RocketState_UpdateAltitudeDeploy(Rocket_StateMachine *machine,
											 uint32_t now_ms,
											 float altitude_m,
											 float vertical_speed_mps,
											 uint8_t altitude_zero_ready,
											 uint8_t barometer_height_valid,
											 uint8_t vertical_speed_valid,
											 float imu_vertical_speed_mps,
											 uint8_t imu_vertical_speed_valid)
{
	uint8_t height_usable;
	uint8_t normal_descent_usable;
	float descent_speed_mps;
	uint8_t descent_speed_valid;

	height_usable =
		(uint8_t)((altitude_zero_ready != 0U) &&
				  (barometer_height_valid != 0U));
	normal_descent_usable =
		(uint8_t)((height_usable != 0U) &&
				  (vertical_speed_valid != 0U));

#if ROCKET_SENSOR_FAIL_DEPLOY_ENABLE
	/* 最终保险：正常高度/速度通道和独立IMU速度都失效时，11 s强制开伞。 */
	if((now_ms >= ROCKET_FORCE_DEPLOY_TIME_MS) &&
	   (normal_descent_usable == 0U) &&
	   (imu_vertical_speed_valid == 0U))
	{
		machine->deployed = 1U;
		return;
	}
#endif

	if(height_usable == 0U)
	{
		machine->altitude_high_pending = 0U;
	}
	else if(machine->altitude_deploy_armed == 0U)
	{
		/* 高度持续超过65 m才认定为高弹道，避免单次气压尖峰误判。 */
		if(altitude_m > ROCKET_ALTITUDE_HIGH_FLIGHT_M)
		{
			if(machine->altitude_high_pending == 0U)
			{
				machine->altitude_high_pending = 1U;
				machine->altitude_high_start_ms = now_ms;
			}
			else if((uint32_t)(now_ms - machine->altitude_high_start_ms) >=
					ROCKET_ALTITUDE_HIGH_CONFIRM_MS)
			{
				machine->altitude_deploy_armed = 1U;
				machine->altitude_descent_pending = 0U;
				machine->altitude_descent_last_negative_ms = 0U;
			}
		}
		else
		{
			machine->altitude_high_pending = 0U;
		}
	}

	/* 高弹道且高度通道正常：下降到55 m时开伞。 */
	if((machine->altitude_deploy_armed != 0U) &&
	   (normal_descent_usable != 0U))
	{
		machine->altitude_descent_pending = 0U;
		machine->altitude_descent_last_negative_ms = 0U;
		if((vertical_speed_mps < 0.0f) &&
		   (altitude_m <= ROCKET_ALTITUDE_DEPLOY_M))
		{
			machine->deployed = 1U;
		}
		return;
	}

	/* 低弹道使用融合速度；高度通道失效时改用独立IMU速度。 */
	if(normal_descent_usable != 0U)
	{
		descent_speed_mps = vertical_speed_mps;
		descent_speed_valid = 1U;
	}
	else
	{
		descent_speed_mps = imu_vertical_speed_mps;
		descent_speed_valid = imu_vertical_speed_valid;
	}

	/* 二级点火脉冲结束后确认下降；短暂抖动或丢样允许60 ms间隙。 */
	if(now_ms < ROCKET_BURNOUT_DETECT_START_MS)
	{
		machine->altitude_descent_pending = 0U;
		machine->altitude_descent_last_negative_ms = 0U;
		return;
	}

	if((descent_speed_valid == 0U) ||
	   (descent_speed_mps >= 0.0f))
	{
		if((machine->altitude_descent_pending != 0U) &&
		   ((uint32_t)(now_ms -
			machine->altitude_descent_last_negative_ms) <=
			ROCKET_ALTITUDE_DESCENT_GAP_MS))
		{
			return;
		}
		machine->altitude_descent_pending = 0U;
		machine->altitude_descent_last_negative_ms = 0U;
		return;
	}

	if(machine->altitude_descent_pending == 0U)
	{
		machine->altitude_descent_pending = 1U;
		machine->altitude_descent_start_ms = now_ms;
		machine->altitude_descent_last_negative_ms = now_ms;
		return;
	}
	machine->altitude_descent_last_negative_ms = now_ms;

	if((uint32_t)(now_ms - machine->altitude_descent_start_ms) >=
	   ROCKET_ALTITUDE_LOW_DESCENT_CONFIRM_MS)
	{
		machine->deployed = 1U;
	}
}
#endif

static void RocketState_UpdateIgnition(Rocket_StateMachine *machine,
									   uint32_t now_ms,
									   float roll_deg,
									   float pitch_deg)
{
	uint32_t ignition_end_ms;

	ignition_end_ms =
		ROCKET_IGNITION_DELAY_MS + ROCKET_IGNITION_PULSE_MS;

	if(machine->ignition_active != 0U)
	{
		if((now_ms >= ignition_end_ms) ||
		   (machine->deployed != 0U) ||
		   (machine->imu_sample_valid == 0U) ||
		   (machine->tilt_pending != 0U) ||
		   (RocketState_IsTiltExceeded(roll_deg, pitch_deg) != 0U))
		{
			machine->ignition_active = 0U;
		}
		return;
	}

	if((machine->ignition_decided != 0U) ||
	   (now_ms < ROCKET_IGNITION_DELAY_MS))
	{
		return;
	}

	/*
	 * 点火窗口入口只决策一次。入口条件不满足、点火后任一联锁
	 * 失效，均禁止本次上电周期内迟到或断续重新点火。
	 */
	machine->ignition_decided = 1U;
	if((now_ms >
		(ROCKET_IGNITION_DELAY_MS +
		 ROCKET_IGNITION_START_GRACE_MS)) ||
	   (now_ms >= ignition_end_ms) ||
	   (machine->deployed != 0U) ||
	   (machine->imu_sample_valid == 0U) ||
	   (machine->imu_good_count <
		ROCKET_IMU_IGNITION_GOOD_SAMPLES) ||
	   (machine->tilt_pending != 0U) ||
	   (RocketState_IsTiltExceeded(roll_deg, pitch_deg) != 0U))
	{
		return;
	}

	machine->ignition_active = 1U;
}

static void RocketState_UpdateBurnout(Rocket_StateMachine *machine,
									  uint32_t now_ms,
									  uint8_t sample_attempted,
									  uint8_t sample_valid,
									  float accel_z_mg)
{
	float accel_z_abs_mg;

	if(machine->burnout_detected != 0U)
	{
		return;
	}

	if(machine->deployed != 0U)
	{
		machine->burnout_pending = 0U;
		return;
	}

	if((now_ms < ROCKET_BURNOUT_DETECT_START_MS) ||
	   (sample_attempted == 0U))
	{
		return;
	}

	if(sample_valid == 0U)
	{
		machine->burnout_pending = 0U;
		return;
	}

	accel_z_abs_mg = RocketState_AbsFloat(accel_z_mg);
	if(accel_z_abs_mg > ROCKET_BURNOUT_ACCEL_Z_ABS_THRESHOLD_MG)
	{
		machine->burnout_pending = 0U;
		return;
	}

	if(machine->burnout_pending == 0U)
	{
		machine->burnout_pending = 1U;
		machine->burnout_start_ms = now_ms;
		return;
	}

	if((uint32_t)(now_ms - machine->burnout_start_ms) >=
	   ROCKET_BURNOUT_CONFIRM_MS)
	{
		machine->burnout_detected = 1U;
	}
}

static void RocketState_UpdatePhase(Rocket_StateMachine *machine, uint32_t now_ms)
{
	uint32_t ignition_end_ms;

	ignition_end_ms =
		ROCKET_IGNITION_DELAY_MS + ROCKET_IGNITION_PULSE_MS;

	if(machine->deployed != 0U)
	{
		machine->state = ROCKET_STATE_DEPLOYED;
	}
	else if(now_ms < ROCKET_IGNITION_DELAY_MS)
	{
		machine->state = ROCKET_STATE_WAIT_PID;
	}
	else if(now_ms < ignition_end_ms)
	{
		machine->state = ROCKET_STATE_IGNITION;
	}
	else if(machine->burnout_detected == 0U)
	{
		machine->state = ROCKET_STATE_WAIT_PID;
	}
	else
	{
		machine->state = ROCKET_STATE_FLIGHT;
	}
}

/*============================== 公开函数 ==============================*/

void RocketState_Init(Rocket_StateMachine *machine)
{
	machine->state = ROCKET_STATE_INIT;
	machine->tilt_start_ms = 0U;
	machine->tilt_last_sample_ms = 0U;
	machine->tilt_pending = 0U;
	machine->deployed = 0U;
	machine->imu_healthy = 0U;
	machine->imu_sample_valid = 0U;
	machine->imu_fail_count = 0U;
	machine->imu_good_count = 0U;
	machine->ignition_decided = 0U;
	machine->ignition_active = 0U;
	machine->launch_start_ms = 0U;
#if ROCKET_LEGACY_AUTO_LAUNCH_ENABLE
	machine->launch_started = 1U;
#else
	machine->launch_started = 0U;
#endif
	machine->burnout_start_ms = 0U;
	machine->burnout_pending = 0U;
	machine->burnout_detected = 0U;
	machine->altitude_deploy_armed = 0U;
	machine->altitude_high_start_ms = 0U;
	machine->altitude_descent_start_ms = 0U;
	machine->altitude_descent_last_negative_ms = 0U;
	machine->altitude_high_pending = 0U;
	machine->altitude_descent_pending = 0U;
}

uint8_t RocketState_StartLaunch(Rocket_StateMachine *machine, uint32_t now_ms)
{
	if((machine == 0) || (RocketState_IsLaunchReady(machine) == 0U))
	{
		return 0U;
	}

	machine->launch_start_ms = now_ms;
	machine->launch_started = 1U;
	machine->ignition_decided = 0U;
	machine->ignition_active = 0U;
	machine->burnout_pending = 0U;
	machine->burnout_detected = 0U;
	machine->altitude_deploy_armed = 0U;
	machine->altitude_high_pending = 0U;
	machine->altitude_descent_pending = 0U;
	return 1U;
}

uint8_t RocketState_IsLaunchReady(const Rocket_StateMachine *machine)
{
	if((machine == 0) ||
	   (machine->launch_started != 0U) ||
	   (machine->deployed != 0U) ||
	   (machine->imu_healthy == 0U) ||
	   (machine->imu_sample_valid == 0U) ||
	   (machine->imu_good_count < ROCKET_IMU_IGNITION_GOOD_SAMPLES))
	{
		return 0U;
	}
	return 1U;
}

uint8_t RocketState_HasLaunchStarted(const Rocket_StateMachine *machine)
{
	return (machine != 0) ? machine->launch_started : 0U;
}

uint32_t RocketState_GetMissionElapsedMs(const Rocket_StateMachine *machine,
										 uint32_t now_ms)
{
	if((machine == 0) || (machine->launch_started == 0U))
	{
		return 0U;
	}
	return (uint32_t)(now_ms - machine->launch_start_ms);
}

void RocketState_Update(Rocket_StateMachine *machine,
						uint32_t now_ms,
						uint8_t sample_attempted,
						uint8_t sample_valid,
						float roll_deg,
						float pitch_deg,
						float accel_z_mg,
						float altitude_m,
						float vertical_speed_mps,
						uint8_t altitude_zero_ready,
						uint8_t barometer_height_valid,
						uint8_t vertical_speed_valid,
						float imu_vertical_speed_mps,
						uint8_t imu_vertical_speed_valid)
{
	uint32_t mission_ms;

	RocketState_UpdateImuHealth(machine, sample_attempted, sample_valid);
	if(machine->launch_started == 0U)
	{
		machine->state = ROCKET_STATE_WAIT_PID;
		return;
	}
	mission_ms = RocketState_GetMissionElapsedMs(machine, now_ms);

	if(machine->deployed == 0U)
	{
#if ROCKET_TILT_DEPLOY_ENABLE
		RocketState_UpdateTilt(machine,
						   mission_ms,
						   sample_attempted,
						   sample_valid,
						   roll_deg,
						   pitch_deg);
#endif

#if ROCKET_ALTITUDE_DEPLOY_ENABLE
		RocketState_UpdateAltitudeDeploy(machine,
									 mission_ms,
									 altitude_m,
									 vertical_speed_mps,
									 altitude_zero_ready,
									 barometer_height_valid,
									 vertical_speed_valid,
									 imu_vertical_speed_mps,
									 imu_vertical_speed_valid);
#endif

#if ROCKET_TIME_DEPLOY_ENABLE
		if(mission_ms >= ROCKET_FORCE_DEPLOY_TIME_MS)
		{
			machine->deployed = 1U;
		}
#endif
	}

	RocketState_UpdateIgnition(machine,
							   mission_ms,
							   roll_deg,
							   pitch_deg);
	RocketState_UpdateBurnout(machine,
							  mission_ms,
							  sample_attempted,
							  sample_valid,
							  accel_z_mg);
	RocketState_UpdatePhase(machine, mission_ms);
}

uint8_t RocketState_IsPidEnabled(const Rocket_StateMachine *machine, uint32_t now_ms)
{
	if(machine->deployed != 0U)
	{
		return 0U;
	}

	if(machine->imu_healthy == 0U)
	{
		return 0U;
	}

	if(machine->launch_started == 0U)
	{
		return 0U;
	}
	return ((uint32_t)(now_ms - machine->launch_start_ms) >=
			ROCKET_PID_START_DELAY_MS) ? 1U : 0U;
}

uint8_t RocketState_IsIgnitionAllowed(const Rocket_StateMachine *machine,
									  uint32_t now_ms)
{
	uint32_t mission_ms;

	if((machine == 0) ||
	   (machine->launch_started == 0U) ||
	   (machine->deployed != 0U) ||
	   (machine->imu_healthy == 0U) ||
	   (machine->imu_sample_valid == 0U) ||
	   (machine->ignition_active == 0U))
	{
		return 0U;
	}
	mission_ms = RocketState_GetMissionElapsedMs(machine, now_ms);

	if(mission_ms < ROCKET_IGNITION_DELAY_MS)
	{
		return 0U;
	}

	if(mission_ms >= (ROCKET_IGNITION_DELAY_MS + ROCKET_IGNITION_PULSE_MS))
	{
		return 0U;
	}

	return 1U;
}
