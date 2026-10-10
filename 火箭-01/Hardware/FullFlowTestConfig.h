#ifndef __FULL_FLOW_TEST_CONFIG_H
#define __FULL_FLOW_TEST_CONFIG_H

/* Bench-only complete sequence. Never connect a real igniter. */
#define ROCKET_FULL_FLOW_BENCH_TEST_ENABLE  1

#if ROCKET_FULL_FLOW_BENCH_TEST_ENABLE
/*
 * The PCB is installed face-down (180 degrees about X). Apply the 180-degree
 * transform once at the IMU output, then all interlocks/controllers use the
 * normal body-frame target of zero degrees. Do not also move the target to
 * 180 degrees, because that would cancel the mounting transform.
 */
#undef ROCKET_IMU_MOUNT_ROLL_OFFSET_DEG
#define ROCKET_IMU_MOUNT_ROLL_OFFSET_DEG    180.0f
#undef ROCKET_TARGET_ROLL_DEG
#define ROCKET_TARGET_ROLL_DEG              0.0f

/* After the mounting transform, installed attitude and vertical gravity are positive. */
#undef ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN
#define ROCKET_ALTITUDE_ACCEL_VERTICAL_SIGN 1.0f

/* Mix the altitude controller output into roll and pitch on the bench. */
#define ROCKET_FULL_FLOW_ALTITUDE_MIX_ENABLE 1
#endif

#endif
