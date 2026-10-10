#include <stdint.h>
#include <stdio.h>

#include "RocketConfig.h"
#include "FullFlowTestConfig.h"
#include "RocketState.h"

static unsigned int s_failures = 0U;

#define CHECK(condition) \
	do { if(!(condition)) { \
		s_failures++; \
		printf("FAIL line %d: %s\n", __LINE__, #condition); \
	} } while(0)

static void UpdateHealthy(Rocket_StateMachine *machine, uint32_t now_ms)
{
	RocketState_Update(machine,
					  now_ms,
					  1U,
					  1U,
					  1.0f,
					  0.0f,
					  -1000.0f);
}

int main(void)
{
	Rocket_StateMachine machine;
	Rocket_StateMachine face_up_machine;
	uint32_t now_ms;

	RocketState_Init(&machine);
	for(now_ms = 0U; now_ms < 100U; now_ms += 10U)
	{
		UpdateHealthy(&machine, now_ms);
	}
	CHECK(machine.imu_healthy != 0U);
	CHECK(machine.deployed == 0U);

	/* After the mount transform, face-up is near 180 and must be rejected. */
	RocketState_Init(&face_up_machine);
	for(now_ms = 0U;
		now_ms <= ROCKET_IGNITION_DELAY_MS;
		now_ms += 10U)
	{
		RocketState_Update(&face_up_machine,
						  now_ms,
						  1U,
						  1U,
						  179.0f,
						  0.0f,
						  1000.0f);
	}
	CHECK(RocketState_IsIgnitionAllowed(
		&face_up_machine, ROCKET_IGNITION_DELAY_MS) == 0U);

	UpdateHealthy(&machine, ROCKET_IGNITION_DELAY_MS - 10U);
	CHECK(RocketState_IsIgnitionAllowed(
		&machine, ROCKET_IGNITION_DELAY_MS - 10U) == 0U);
	UpdateHealthy(&machine, ROCKET_IGNITION_DELAY_MS);
	CHECK(machine.state == ROCKET_STATE_IGNITION);
	CHECK(RocketState_IsIgnitionAllowed(
		&machine, ROCKET_IGNITION_DELAY_MS) != 0U);
	UpdateHealthy(&machine, ROCKET_BURNOUT_DETECT_START_MS - 10U);
	CHECK(RocketState_IsIgnitionAllowed(
		&machine, ROCKET_BURNOUT_DETECT_START_MS - 10U) != 0U);
	UpdateHealthy(&machine, ROCKET_BURNOUT_DETECT_START_MS);
	CHECK(RocketState_IsIgnitionAllowed(
		&machine, ROCKET_BURNOUT_DETECT_START_MS) == 0U);

	for(now_ms = ROCKET_BURNOUT_DETECT_START_MS + 10U;
		now_ms <= (ROCKET_BURNOUT_DETECT_START_MS +
				   ROCKET_BURNOUT_CONFIRM_MS + 10U);
		now_ms += 10U)
	{
		UpdateHealthy(&machine, now_ms);
	}
	CHECK(machine.burnout_detected != 0U);
	CHECK(machine.state == ROCKET_STATE_FLIGHT);
	CHECK(RocketState_IsPidEnabled(&machine, now_ms - 10U) != 0U);

	UpdateHealthy(&machine, ROCKET_FORCE_DEPLOY_TIME_MS - 10U);
	CHECK(machine.deployed == 0U);
	UpdateHealthy(&machine, ROCKET_FORCE_DEPLOY_TIME_MS);
	CHECK(machine.deployed != 0U);
	CHECK(machine.state == ROCKET_STATE_DEPLOYED);
	CHECK(RocketState_IsPidEnabled(
		&machine, ROCKET_FORCE_DEPLOY_TIME_MS) == 0U);

	if(s_failures != 0U)
	{
		printf("FAILED: %u checks\n", s_failures);
		return 1;
	}
	printf("PASS: full flow bench sequence\n");
	return 0;
}
