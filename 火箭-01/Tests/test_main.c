#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "PID.h"
#include "ControlMath.h"
#include "RocketConfig.h"
#include "RocketState.h"
#include "AltitudeControl.h"

static unsigned int s_check_count = 0U;
static unsigned int s_failure_count = 0U;

#define CHECK_TRUE(condition) Test_CheckTrue((condition), #condition, __LINE__)
#define CHECK_FLOAT(actual, expected, tolerance) \
	Test_CheckFloat((actual), (expected), (tolerance), #actual, __LINE__)

static void Test_CheckTrue(int condition, const char *expression, int line)
{
	s_check_count++;
	if(condition == 0)
	{
		s_failure_count++;
		printf("FAIL line %d: %s\n", line, expression);
	}
}

static void Test_CheckFloat(float actual,
							float expected,
							float tolerance,
							const char *expression,
							int line)
{
	s_check_count++;
	if(fabsf(actual - expected) > tolerance)
	{
		s_failure_count++;
		printf("FAIL line %d: %s = %.6f, expected %.6f\n",
			   line,
			   expression,
			   (double)actual,
			   (double)expected);
	}
}

static void Test_PidBasic(void)
{
	PID_Controller pid;
	float previous_output;

	PID_Init(&pid, 2.0f, 0.0f, 0.0f, 80.0f, 300.0f, 0.2f);
	CHECK_FLOAT(PID_Update(&pid, 0.0f, 0.0f, 0.01f), 0.0f, 0.0001f);
	CHECK_FLOAT(PID_Update(&pid, 10.0f, 0.0f, 0.01f), 20.0f, 0.0001f);
	CHECK_FLOAT(PID_Update(&pid, -10.0f, 0.0f, 0.01f), -20.0f, 0.0001f);
	CHECK_FLOAT(PID_Update(&pid, 1000.0f, 0.0f, 0.01f), 300.0f, 0.0001f);
	CHECK_FLOAT(PID_Update(&pid, -1000.0f, 0.0f, 0.01f), -300.0f, 0.0001f);

	previous_output = pid.output;
	CHECK_FLOAT(PID_Update(&pid, 10.0f, 0.0f, 0.0f), previous_output, 0.0001f);
}

static void Test_PidIntegralAndAntiWindup(void)
{
	PID_Controller pid;
	unsigned int i;

	PID_Init(&pid, 0.0f, 1.0f, 0.0f, 80.0f, 300.0f, 0.2f);
	for(i = 0U; i < 20U; i++)
	{
		(void)PID_Update(&pid, 100.0f, 0.0f, 1.0f);
	}
	CHECK_FLOAT(pid.integral, 80.0f, 0.0001f);
	CHECK_FLOAT(pid.output, 80.0f, 0.0001f);

	PID_Reset(&pid);
	for(i = 0U; i < 20U; i++)
	{
		(void)PID_Update(&pid, -100.0f, 0.0f, 1.0f);
	}
	CHECK_FLOAT(pid.integral, -80.0f, 0.0001f);
	CHECK_FLOAT(pid.output, -80.0f, 0.0001f);

	PID_Init(&pid, 10.0f, 1.0f, 0.0f, 80.0f, 300.0f, 0.2f);
	for(i = 0U; i < 100U; i++)
	{
		(void)PID_Update(&pid, 100.0f, 0.0f, 1.0f);
	}
	CHECK_FLOAT(pid.output, 300.0f, 0.0001f);
	CHECK_FLOAT(pid.integral, 0.0f, 0.0001f);
	CHECK_FLOAT(PID_Update(&pid, 0.0f, 0.0f, 1.0f), 0.0f, 0.0001f);
}

static void Test_PidDerivativeAndReset(void)
{
	PID_Controller pid;

	PID_Init(&pid, 0.0f, 0.0f, 1.0f, 80.0f, 1000.0f, 0.2f);
	CHECK_FLOAT(PID_Update(&pid, 0.0f, 0.0f, 0.1f), 0.0f, 0.0001f);
	CHECK_FLOAT(PID_Update(&pid, 0.0f, -10.0f, 0.1f), 20.0f, 0.0001f);
	CHECK_FLOAT(PID_Update(&pid, 0.0f, -10.0f, 0.1f), 16.0f, 0.0001f);

	PID_Reset(&pid);
	CHECK_FLOAT(PID_Update(&pid, 0.0f, 100.0f, 0.1f), 0.0f, 0.0001f);
	CHECK_FLOAT(pid.integral, 0.0f, 0.0001f);
}

static void Test_ControlMath(void)
{
	CHECK_FLOAT(ControlMath_AngleErrorDeg(180.0f, -177.0f),
				-3.0f,
				0.0001f);
	CHECK_FLOAT(ControlMath_AngleErrorDeg(-180.0f, 177.0f),
				3.0f,
				0.0001f);
	CHECK_FLOAT(ControlMath_AngleToRate(0.0f, -10.0f, 2.0f, 90.0f),
				20.0f,
				0.0001f);
	CHECK_FLOAT(ControlMath_AngleToRate(0.0f, 100.0f, 2.0f, 90.0f),
				-90.0f,
				0.0001f);
	CHECK_FLOAT(ControlMath_AngleToRate(0.0f, -100.0f, 2.0f, 90.0f),
				90.0f,
				0.0001f);

	CHECK_TRUE(ControlMath_OffsetToPulseUs(500.0f, 1.0f, 1500U, 1000U, 2000U, 300.0f) == 1800U);
	CHECK_TRUE(ControlMath_OffsetToPulseUs(-500.0f, 1.0f, 1500U, 1000U, 2000U, 300.0f) == 1200U);
	CHECK_TRUE(ControlMath_OffsetToPulseUs(300.0f, -1.0f, 1500U, 1000U, 2000U, 300.0f) == 1200U);
	CHECK_TRUE(ControlMath_OffsetToPulseUs(1000.0f, 1.0f, 1500U, 1000U, 2000U, 1000.0f) == 2000U);
	CHECK_TRUE(ControlMath_OffsetToPulseUs(-1000.0f, 1.0f, 1500U, 1000U, 2000U, 1000.0f) == 1000U);
}

static void Test_QualifyImuForIgnition(Rocket_StateMachine *machine)
{
	uint32_t start_ms;
	unsigned int index;

	start_ms =
		ROCKET_IGNITION_DELAY_MS -
		(ROCKET_IMU_IGNITION_GOOD_SAMPLES * ROCKET_IMU_PERIOD_MS);
	for(index = 0U;
		index < ROCKET_IMU_IGNITION_GOOD_SAMPLES;
		index++)
	{
		RocketState_Update(machine,
						   start_ms +
						   ((uint32_t)index * ROCKET_IMU_PERIOD_MS),
						   1U,
						   1U,
						   0.0f,
						   0.0f);
	}
}

static void Test_StateTimeBoundaries(void)
{
	Rocket_StateMachine machine;
	uint32_t ignition_end_ms;

	RocketState_Init(&machine);
	CHECK_TRUE(machine.state == ROCKET_STATE_INIT);
	CHECK_TRUE(RocketState_IsPidEnabled(
		&machine,
		ROCKET_PID_START_DELAY_MS) == 0U);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		ROCKET_IGNITION_DELAY_MS) == 0U);

	Test_QualifyImuForIgnition(&machine);
	CHECK_TRUE(machine.state == ROCKET_STATE_WAIT_PID);
	CHECK_TRUE(machine.imu_good_count ==
			   ROCKET_IMU_IGNITION_GOOD_SAMPLES);
	CHECK_TRUE(RocketState_IsPidEnabled(
		&machine,
		ROCKET_PID_START_DELAY_MS - 1U) == 0U);

	RocketState_Update(&machine,
					   ROCKET_IGNITION_DELAY_MS,
					   0U,
					   0U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.state == ROCKET_STATE_IGNITION);
	CHECK_TRUE(RocketState_IsPidEnabled(
		&machine,
		ROCKET_IGNITION_DELAY_MS) == 1U);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		ROCKET_IGNITION_DELAY_MS) == 1U);

	ignition_end_ms =
		ROCKET_IGNITION_DELAY_MS + ROCKET_IGNITION_PULSE_MS;
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		ignition_end_ms - 1U) == 1U);
	RocketState_Update(&machine,
					   ignition_end_ms,
					   1U,
					   1U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.state == ROCKET_STATE_FLIGHT);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		ignition_end_ms) == 0U);

	RocketState_Update(&machine,
					   ROCKET_FORCE_DEPLOY_TIME_MS - 1U,
					   1U,
					   1U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.state == ROCKET_STATE_FLIGHT);
	CHECK_TRUE(machine.deployed == 0U);

	RocketState_Update(&machine,
					   ROCKET_FORCE_DEPLOY_TIME_MS,
					   1U,
					   0U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.state == ROCKET_STATE_DEPLOYED);
	CHECK_TRUE(machine.deployed == 1U);
	CHECK_TRUE(RocketState_IsPidEnabled(
		&machine,
		ROCKET_FORCE_DEPLOY_TIME_MS) == 0U);
}

static void Test_TiltDeploy(void)
{
	Rocket_StateMachine machine;
	uint32_t now_ms;

	RocketState_Init(&machine);
	RocketState_Update(&machine,
					   ROCKET_TILT_CHECK_TIME_MS,
					   1U,
					   1U,
					   ROCKET_CRITICAL_ANGLE_DEG,
					   -ROCKET_CRITICAL_ANGLE_DEG);
	CHECK_TRUE(machine.tilt_pending == 0U);
	CHECK_TRUE(machine.deployed == 0U);

	RocketState_Update(&machine,
					   ROCKET_TILT_CHECK_TIME_MS + 10U,
					   1U,
					   1U,
					   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
					   0.0f);
	CHECK_TRUE(machine.tilt_pending == 1U);
	for(now_ms = ROCKET_TILT_CHECK_TIME_MS + 20U;
		now_ms < (ROCKET_TILT_CHECK_TIME_MS +
				  10U +
				  ROCKET_TILT_CONFIRM_MS);
		now_ms += 10U)
	{
		RocketState_Update(&machine,
						   now_ms,
						   1U,
						   1U,
						   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
						   0.0f);
		CHECK_TRUE(machine.deployed == 0U);
	}
	RocketState_Update(&machine,
					   ROCKET_TILT_CHECK_TIME_MS +
					   10U +
					   ROCKET_TILT_CONFIRM_MS,
					   1U,
					   1U,
					   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
					   0.0f);
	CHECK_TRUE(machine.deployed == 1U);
	CHECK_TRUE(machine.state == ROCKET_STATE_DEPLOYED);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		ROCKET_IGNITION_DELAY_MS) == 0U);

	RocketState_Init(&machine);
	RocketState_Update(&machine,
					   ROCKET_TILT_CHECK_TIME_MS,
					   1U,
					   1U,
					   0.0f,
					   -ROCKET_CRITICAL_ANGLE_DEG - 1.0f);
	for(now_ms = ROCKET_TILT_CHECK_TIME_MS + ROCKET_IMU_PERIOD_MS;
		now_ms <= (ROCKET_TILT_CHECK_TIME_MS +
				   ROCKET_TILT_CONFIRM_MS);
		now_ms += ROCKET_IMU_PERIOD_MS)
	{
		RocketState_Update(&machine,
						   now_ms,
						   1U,
						   1U,
						   0.0f,
						   -ROCKET_CRITICAL_ANGLE_DEG - 1.0f);
	}
	CHECK_TRUE(machine.deployed == 1U);
}

static void Test_TiltContinuity(void)
{
	Rocket_StateMachine machine;
	uint32_t tilt_ms;
	uint32_t now_ms;

	tilt_ms = ROCKET_TILT_CHECK_TIME_MS;
	RocketState_Init(&machine);
	RocketState_Update(&machine,
					   tilt_ms,
					   1U,
					   1U,
					   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
					   0.0f);
	RocketState_Update(&machine,
					   tilt_ms + 50U,
					   1U,
					   1U,
					   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
					   0.0f);
	CHECK_TRUE(machine.tilt_pending == 1U);
	CHECK_TRUE(machine.tilt_start_ms == (tilt_ms + 50U));
	CHECK_TRUE(machine.deployed == 0U);

	RocketState_Update(&machine,
					   tilt_ms + 60U,
					   1U,
					   1U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.tilt_pending == 0U);

	RocketState_Update(&machine,
					   tilt_ms + 70U,
					   1U,
					   1U,
					   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
					   0.0f);
	for(now_ms = tilt_ms + 80U;
		now_ms <= (tilt_ms + 70U + ROCKET_TILT_CONFIRM_MS);
		now_ms += ROCKET_IMU_PERIOD_MS)
	{
		RocketState_Update(&machine,
						   now_ms,
						   1U,
						   1U,
						   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
						   0.0f);
	}
	CHECK_TRUE(machine.deployed == 1U);

	RocketState_Init(&machine);
	RocketState_Update(&machine,
					   tilt_ms,
					   1U,
					   1U,
					   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
					   0.0f);
	RocketState_Update(&machine,
					   tilt_ms + 10U,
					   1U,
					   0U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.tilt_pending == 0U);
	RocketState_Update(&machine,
					   tilt_ms + 20U,
					   1U,
					   1U,
					   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
					   0.0f);
	for(now_ms = tilt_ms + 30U;
		now_ms <= (tilt_ms + 20U + ROCKET_TILT_CONFIRM_MS);
		now_ms += ROCKET_IMU_PERIOD_MS)
	{
		RocketState_Update(&machine,
						   now_ms,
						   1U,
						   1U,
						   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
						   0.0f);
	}
	CHECK_TRUE(machine.deployed == 1U);
}

static void Test_ImuFaultAndRecovery(void)
{
	Rocket_StateMachine machine;
	unsigned int i;

	RocketState_Init(&machine);
	CHECK_TRUE(machine.imu_healthy == 0U);
	RocketState_Update(&machine, 100U, 1U, 1U, 0.0f, 0.0f);
	CHECK_TRUE(machine.imu_healthy == 1U);

	RocketState_Update(&machine, 110U, 1U, 0U, 0.0f, 0.0f);
	CHECK_TRUE(machine.imu_healthy == 1U);
	RocketState_Update(&machine, 120U, 1U, 0U, 0.0f, 0.0f);
	CHECK_TRUE(machine.imu_healthy == 1U);
	RocketState_Update(&machine, 130U, 1U, 0U, 0.0f, 0.0f);
	CHECK_TRUE(machine.imu_healthy == 0U);
	CHECK_TRUE(machine.imu_fail_count == ROCKET_IMU_FAIL_LIMIT);

	for(i = 0U; i < 300U; i++)
	{
		RocketState_Update(&machine, 140U + i, 1U, 0U, 0.0f, 0.0f);
	}
	CHECK_TRUE(machine.imu_fail_count == ROCKET_IMU_FAIL_LIMIT);

	RocketState_Update(&machine, 1000U, 1U, 1U, 0.0f, 0.0f);
	CHECK_TRUE(machine.imu_healthy == 1U);
	CHECK_TRUE(machine.imu_fail_count == 0U);

	RocketState_Update(&machine,
					   ROCKET_FORCE_DEPLOY_TIME_MS,
					   1U,
					   0U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.deployed == 1U);
	CHECK_TRUE(machine.state == ROCKET_STATE_DEPLOYED);
}

static void Test_IgnitionInterlocks(void)
{
	Rocket_StateMachine machine;
	uint32_t failure_ms;

	RocketState_Init(&machine);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		ROCKET_IGNITION_DELAY_MS) == 0U);

	Test_QualifyImuForIgnition(&machine);
	RocketState_Update(&machine,
					   ROCKET_IGNITION_DELAY_MS,
					   0U,
					   0U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		ROCKET_IGNITION_DELAY_MS) == 1U);

	failure_ms =
		ROCKET_IGNITION_DELAY_MS + ROCKET_IMU_PERIOD_MS;
	RocketState_Update(&machine,
					   failure_ms,
					   1U,
					   0U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.imu_healthy == 1U);
	CHECK_TRUE(machine.ignition_active == 0U);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		failure_ms) == 0U);

	/* 一次有效样本可恢复控制健康，但本次点火不得重新开始。 */
	RocketState_Update(&machine,
					   failure_ms + ROCKET_IMU_PERIOD_MS,
					   1U,
					   1U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.imu_healthy == 1U);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		failure_ms + ROCKET_IMU_PERIOD_MS) == 0U);

	RocketState_Init(&machine);
	Test_QualifyImuForIgnition(&machine);
	RocketState_Update(&machine,
					   ROCKET_IGNITION_DELAY_MS,
					   0U,
					   0U,
					   0.0f,
					   0.0f);
	machine.deployed = 1U;
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		ROCKET_IGNITION_DELAY_MS) == 0U);

	/* 危险倾角在窗口入口直接否决点火，不等待开伞确认。 */
	RocketState_Init(&machine);
	Test_QualifyImuForIgnition(&machine);
	RocketState_Update(&machine,
					   ROCKET_IGNITION_DELAY_MS,
					   1U,
					   1U,
					   ROCKET_CRITICAL_ANGLE_DEG + 1.0f,
					   0.0f);
	CHECK_TRUE(machine.ignition_decided == 1U);
	CHECK_TRUE(machine.ignition_active == 0U);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		ROCKET_IGNITION_DELAY_MS) == 0U);

	/* 超过入口宽限才首次运行任务时，不允许迟到点火。 */
	RocketState_Init(&machine);
	Test_QualifyImuForIgnition(&machine);
	RocketState_Update(&machine,
					   ROCKET_IGNITION_DELAY_MS +
					   ROCKET_IGNITION_START_GRACE_MS + 1U,
					   0U,
					   0U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.ignition_active == 0U);

	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		0,
		ROCKET_IGNITION_DELAY_MS) == 0U);
}

static void Test_LateFirstTask(void)
{
	Rocket_StateMachine machine;
	uint32_t late_ms;

	late_ms =
		ROCKET_IGNITION_DELAY_MS + ROCKET_IGNITION_PULSE_MS + 100U;
	RocketState_Init(&machine);
	RocketState_Update(&machine, late_ms, 0U, 0U, 0.0f, 0.0f);
	CHECK_TRUE(machine.state == ROCKET_STATE_FLIGHT);
	CHECK_TRUE(machine.deployed == 0U);
	CHECK_TRUE(RocketState_IsIgnitionAllowed(
		&machine,
		late_ms) == 0U);

	RocketState_Init(&machine);
	RocketState_Update(&machine,
					   ROCKET_FORCE_DEPLOY_TIME_MS,
					   0U,
					   0U,
					   0.0f,
					   0.0f);
	CHECK_TRUE(machine.state == ROCKET_STATE_DEPLOYED);
	CHECK_TRUE(machine.deployed == 1U);
}

static void Test_AltitudeControlPredictionGate(void)
{
	IMU_Data imu;
	const AltitudeControl_State *state;
	uint32_t start_ms;

	imu.roll_deg = 0.0f;
	imu.pitch_deg = 0.0f;
	imu.yaw_deg = 0.0f;
	imu.roll_rate_dps = 0.0f;
	imu.pitch_rate_dps = 0.0f;
	imu.yaw_rate_dps = 0.0f;
	imu.accel_x_mg = 0.0f;
	imu.accel_y_mg = 0.0f;
	imu.accel_z_mg = 5000.0f;
	imu.valid = 1U;

	AltitudeControl_Init();
	AltitudeControl_Update(ROCKET_ALTITUDE_CONTROL_START_MS,
						   &imu,
						   ROCKET_ALTITUDE_CONTROL_START_MS,
						   0.0f,
						   0U,
						   1U,
						   ROCKET_ALTITUDE_CONTROL_START_MS,
						   0U,
						   0U);
	state = AltitudeControl_GetState();
	CHECK_TRUE(state->estimator_valid == 0U);
	CHECK_TRUE(state->prediction_valid == 0U);

	start_ms = ROCKET_ALTITUDE_CONTROL_START_MS;
	AltitudeControl_Update(start_ms, &imu, start_ms,
		0.0f, 1U, 1U, start_ms, 0U, 0U);
	AltitudeControl_Update(start_ms + 100U, &imu, start_ms + 100U,
		1.0f, 1U, 1U, start_ms + 100U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK_TRUE(state->estimator_valid == 1U);
	CHECK_TRUE(state->control_allowed == 1U);
	CHECK_TRUE(state->prediction_valid == 1U);
	CHECK_TRUE(state->predicted_apogee_m > state->altitude_m);
	CHECK_FLOAT(state->suggested_deflection_deg, 0.0f, 0.0001f);

	/* Height control remains allowed at large attitude error. */
	imu.roll_deg = 60.0f;
	imu.pitch_deg = 45.0f;
	AltitudeControl_Update(start_ms + 200U, &imu, start_ms + 200U,
		2.0f, 1U, 1U, start_ms + 200U, 1U, 0U);
	state = AltitudeControl_GetState();
	CHECK_TRUE(state->control_allowed == 1U);
	CHECK_TRUE(state->prediction_valid == 1U);
}

int main(void)
{
	Test_PidBasic();
	Test_PidIntegralAndAntiWindup();
	Test_PidDerivativeAndReset();
	Test_ControlMath();
	Test_StateTimeBoundaries();
	Test_TiltDeploy();
	Test_TiltContinuity();
	Test_ImuFaultAndRecovery();
	Test_IgnitionInterlocks();
	Test_LateFirstTask();
	Test_AltitudeControlPredictionGate();

	if(s_failure_count != 0U)
	{
		printf("FAILED: %u/%u checks failed\n", s_failure_count, s_check_count);
		return 1;
	}

	printf("PASS: %u checks\n", s_check_count);
	return 0;
}
