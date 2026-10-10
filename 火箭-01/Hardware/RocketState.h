#ifndef __ROCKET_STATE_H
#define __ROCKET_STATE_H

#include <stdint.h>

typedef enum
{
	ROCKET_STATE_INIT = 0,
	ROCKET_STATE_WAIT_PID,
	ROCKET_STATE_PID_CONTROL,
	ROCKET_STATE_IGNITION,
	ROCKET_STATE_FLIGHT,
	ROCKET_STATE_DEPLOYED
} Rocket_State;

typedef struct
{
	Rocket_State state;
	uint32_t tilt_start_ms;
	uint32_t tilt_last_sample_ms;
	uint8_t tilt_pending;
	uint8_t deployed;
	uint8_t imu_healthy;
	uint8_t imu_sample_valid;
	uint8_t imu_fail_count;
	uint8_t imu_good_count;
	uint8_t ignition_decided;
	uint8_t ignition_active;
	uint32_t launch_start_ms;
	uint8_t launch_started;
	uint32_t burnout_start_ms;
	uint8_t burnout_pending;
	uint8_t burnout_detected;
	uint8_t altitude_deploy_armed;
	uint32_t altitude_high_start_ms;
	uint32_t altitude_descent_start_ms;
	uint32_t altitude_descent_last_negative_ms;
	uint8_t altitude_high_pending;
	uint8_t altitude_descent_pending;
} Rocket_StateMachine;

void RocketState_Init(Rocket_StateMachine *machine);
uint8_t RocketState_StartLaunch(Rocket_StateMachine *machine, uint32_t now_ms);
uint8_t RocketState_IsLaunchReady(const Rocket_StateMachine *machine);
uint8_t RocketState_HasLaunchStarted(const Rocket_StateMachine *machine);
uint32_t RocketState_GetMissionElapsedMs(const Rocket_StateMachine *machine,
										 uint32_t now_ms);
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
						uint8_t imu_vertical_speed_valid);
uint8_t RocketState_IsPidEnabled(const Rocket_StateMachine *machine, uint32_t now_ms);
uint8_t RocketState_IsIgnitionAllowed(const Rocket_StateMachine *machine,
									  uint32_t now_ms);

#endif
